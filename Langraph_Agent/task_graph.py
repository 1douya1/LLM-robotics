import asyncio
import json
from dataclasses import dataclass, asdict
from typing import Any, Dict, List, Optional
import re

from langgraph.graph import StateGraph, END
from langgraph.checkpoint.memory import MemorySaver


@dataclass
class StepResult:
    step: str
    success: bool
    status: str
    error: str
    duration_sec: float
    raw: Any

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def _build_tool_map(tools: List[Any]) -> Dict[str, Any]:
    return {getattr(t, "name", ""): t for t in tools}


async def _call_tool_safe(tool: Any, args: Dict[str, Any]) -> Dict[str, Any]:
    try:
        if hasattr(tool, "ainvoke"):
            return await tool.ainvoke(args)
        if hasattr(tool, "invoke"):
            return tool.invoke(args)
        return {"ok": False, "status": "tool_invoke_missing", "error": "Tool has no invoke/ainvoke"}
    except Exception as e:
        return {"ok": False, "status": "exception", "error": str(e)}


async def _call_tool_with_retry(tool: Any, args: Dict[str, Any], step_name: str, reporter=None) -> Dict[str, Any]:
    """
    Tool call with retry mechanism:
    1. Execute first attempt by default
    2. Auto-retry once if failed
    3. Ask user whether to continue if both attempts failed
    """
    # First attempt
    await _maybe_report(reporter, {
        "type": "tool_attempt", 
        "step": step_name, 
        "attempt": 1,
        "message": f"Starting {step_name} (attempt 1)",
        "params": _visible_params(args)
    })
    
    result1 = await _call_tool_safe(tool, args)
    success1 = bool(result1.get("ok") or result1.get("success"))
    
    await _maybe_report(reporter, {
        "type": "tool_result", 
        "step": step_name, 
        "attempt": 1,
        "success": success1,
        "message": f"{step_name} attempt 1: {'success' if success1 else 'failed'}",
        "result": {"success": success1, "status": result1.get("status"), "error": result1.get("error")}
    })
    
    if success1:
        return result1
    
    # First attempt failed, auto-retry
    await _maybe_report(reporter, {
        "type": "auto_retry", 
        "step": step_name, 
        "attempt": 2,
        "message": f"{step_name} failed, auto-retrying (attempt 2)"
    })
    
    result2 = await _call_tool_safe(tool, args)
    success2 = bool(result2.get("ok") or result2.get("success"))
    
    await _maybe_report(reporter, {
        "type": "tool_result", 
        "step": step_name, 
        "attempt": 2,
        "success": success2,
        "message": f"{step_name} attempt 2: {'success' if success2 else 'failed'}",
        "result": {"success": success2, "status": result2.get("status"), "error": result2.get("error")}
    })
    
    if success2:
        return result2
    
    # Both attempts failed, ask user
    await _maybe_report(reporter, {
        "type": "user_decision_needed", 
        "step": step_name,
        "message": f"{step_name} failed twice consecutively, user decision needed"
    })
    
    continue_exec = _ask_yes_no(f"{step_name} has failed twice. Continue with attempt 3?")
    
    if continue_exec:
        await _maybe_report(reporter, {
            "type": "user_approved_retry", 
            "step": step_name, 
            "attempt": 3,
            "message": f"User approved to continue, executing {step_name} (attempt 3)"
        })
        
        result3 = await _call_tool_safe(tool, args)
        success3 = bool(result3.get("ok") or result3.get("success"))
        
        await _maybe_report(reporter, {
            "type": "tool_result", 
            "step": step_name, 
            "attempt": 3,
            "success": success3,
            "message": f"{step_name} attempt 3: {'success' if success3 else 'failed'}",
            "result": {"success": success3, "status": result3.get("status"), "error": result3.get("error")}
        })
        
        return result3
    else:
        await _maybe_report(reporter, {
            "type": "user_skipped", 
            "step": step_name,
            "message": f"User chose to skip {step_name}"
        })
        
        return {"ok": False, "status": "user_skipped", "error": "User chose to skip execution", "user_decision": "skip"}


def _ask_yes_no(prompt: str) -> bool:
    while True:
        ans = input(f"{prompt} (yes/no): ").strip().lower()
        if ans in ("y", "yes"): return True
        if ans in ("n", "no"): return False
        print("Please enter yes or no")


def _visible_params(params: Dict[str, Any]) -> Dict[str, Any]:
    """Filter parameters that are allowed to be displayed/passed to users (avoiding coordinates/velocity etc.)."""
    if not params:
        return {}
    allow_keys = {"object_id", "pour_execute", "return_to_origin", "plan_only"}
    return {k: v for k, v in params.items() if k in allow_keys}


async def _maybe_report(reporter, event: Dict[str, Any]):
    if reporter is None:
        # 控制台精简打印
        msg = event.get("message") or ""
        prefix = f"[{event.get('type','event')}]"
        print(prefix, msg)
        return
    try:
        # 兼容同步/异步
        if asyncio.iscoroutinefunction(reporter):
            await reporter(event)
        else:
            reporter(event)
    except Exception:
        pass


def build_main_graph(tools: List[Any], reporter=None):
    """Build deterministic task graph: pick -> move_to_pour -> place -> return (with template selection and event reporting)."""
    tool_map = _build_tool_map(tools)

    required_tool_names = [
        "mcp_MTC_SERVER_pick_container",
        "mcp_MTC_SERVER_move_to_pour_position",
        "mcp_MTC_SERVER_place_container",
        "mcp_MTC_SERVER_return_to_home",
    ]

    for name in required_tool_names:
        if name not in tool_map:
            raise RuntimeError(f"Missing MCP tool: {name}")

    # Available for fallback when needed
    fallback_pour_tool = tool_map.get("mcp_MTC_SERVER_pour_to_target")

    def initial_state() -> Dict[str, Any]:
        return {
            "plan": ["pick", "move_to_pour", "place", "return"],
            "results": {},
            "events": [],
            # Parse results
            "object_id": None,            # Source cup
            "target_object_id": None,     # Target cup (optional)
            "do_pour": True,              # Whether to execute pouring
            "plan_template": "P2",       # P1/P2/P3
            "user_prompt": "",
            # Retry statistics
            "retry_counts": {},           # Record retry counts for each step
        }

    def _extract_object_ids(text: str) -> List[str]:
        return re.findall(r"object[_-]?\d+|object", text)

    async def resolve_and_choose_plan(state: Dict[str, Any]):
        text = (state.get("user_prompt") or "").strip().lower()
        ids = _extract_object_ids(text)

        src = ids[0] if ids else None
        dst = None

        # 尝试解析目的地：dst:xxx / to xxx / 给xxx
        m1 = re.search(r"dst[:：]\s*(object[_-]?\d+|object)", text)
        m2 = re.search(r"to\s+(object[_-]?\d+|object)", text)
        m3 = re.search(r"倒给|给\s*(object[_-]?\d+|object)", text)
        if m1:
            dst = m1.group(1)
        elif m2:
            dst = m2.group(1)
        elif m3:
            dst = m3.group(1)
        elif len(ids) >= 2:
            # 若出现两个ID，默认第一个为src，第二个为dst
            dst = ids[1]

        # Identify no-pour/safety mode
        no_pour = any(k in text for k in ["不倒水", "no pour", "nopour", "避险", "handover"])  # P3

        # Select template
        if no_pour:
            tpl = "P3"; do_pour = False
        elif dst:
            tpl = "P1"; do_pour = True
        else:
            tpl = "P2"; do_pour = True

        state["object_id"] = src
        state["target_object_id"] = dst
        state["do_pour"] = do_pour
        state["plan_template"] = tpl

        # 新增：执行计划预告
        steps_desc = []
        if src:
            steps_desc.append(f"1. Pick container from {src}")
        else:
            steps_desc.append("1. Pick container from default position")
        
        if dst:
            steps_desc.append(f"2. Move to {dst} and {'pour' if do_pour else 'approach without pouring'}")
        else:
            steps_desc.append(f"2. Move to pouring position and {'execute pouring' if do_pour else 'approach without pouring'}")
        
        steps_desc.append("3. Place container back to original position")
        steps_desc.append("4. Return robot arm to home position")
        
        plan_msg = f"🎯 Execution Plan ({tpl} template): " + " → ".join([s.split('. ', 1)[1] for s in steps_desc])
        
        await _maybe_report(reporter, {
            "type": "execution_plan",
            "message": plan_msg,
            "template": tpl,
            "steps": steps_desc,
            "estimated_duration": "~60-90 seconds"
        })

        # Event
        evt = {
            "type": "plan_selected",
            "message": f"Template={tpl} Source={src or 'unspecified'} Target={dst or 'default'} Pour={'yes' if do_pour else 'no'}",
            "template": tpl,
            "source": src,
            "target": dst,
            "do_pour": do_pour,
        }
        state["events"].append(evt)
        await _maybe_report(reporter, evt)
        return state

    async def do_pick(state: Dict[str, Any]):
        # 步骤开始提示
        src_info = state.get("object_id") or "default position"
        await _maybe_report(reporter, {
            "type": "step_starting",
            "step": "pick",
            "message": f"🔧 Starting Step 1: Pick container from {src_info}",
            "details": "Moving robot arm to grasp the container"
        })
        
        tool = tool_map["mcp_MTC_SERVER_pick_container"]
        args: Dict[str, Any] = {}
        if state.get("object_id"):
            args["object_id"] = state["object_id"]
        
        # 使用新的重试机制
        result = await _call_tool_with_retry(tool, args, "pick", reporter)
        
        sr = StepResult(
            step="pick",
            success=bool(result.get("ok") or result.get("success")),
            status=str(result.get("status", "")),
            error=str(result.get("error", "")),
            duration_sec=float(result.get("duration_sec", 0.0) or 0.0),
            raw=result,
        )
        state["results"]["pick"] = sr.to_dict()
        return state

    async def do_move_to_pour(state: Dict[str, Any]):
        # 步骤开始提示
        dst_info = state.get("target_object_id") or "pouring position"
        pour_action = "execute pouring" if state.get("do_pour", True) else "approach without pouring"
        await _maybe_report(reporter, {
            "type": "step_starting",
            "step": "move_to_pour",
            "message": f"🔧 Starting Step 2: Move to {dst_info} and {pour_action}",
            "details": "Moving to target position and performing pouring motion"
        })
        
        tool = tool_map["mcp_MTC_SERVER_move_to_pour_position"]
        args: Dict[str, Any] = {}
        if state.get("target_object_id"):
            args["object_id"] = state["target_object_id"]
        # 是否执行倾倒
        args["pour_execute"] = bool(state.get("do_pour", True))
        
        # 使用新的重试机制
        result = await _call_tool_with_retry(tool, args, "move_to_pour", reporter)
        
        sr = StepResult(
            step="move_to_pour",
            success=bool(result.get("ok") or result.get("success")),
            status=str(result.get("status", "")),
            error=str(result.get("error", "")),
            duration_sec=float(result.get("duration_sec", 0.0) or 0.0),
            raw=result,
        )
        state["results"]["move_to_pour"] = sr.to_dict()

        # Fallback: if move_to_pour failed and pouring is allowed, try standalone pour_to_target
        if not sr.success and state.get("do_pour", True) and fallback_pour_tool is not None:
            await _maybe_report(reporter, {"type": "fallback_attempt", "step": "move_to_pour", "tool": "mcp_MTC_SERVER_pour_to_target", "message": "move_to_pour failed, trying fallback to pour_to_target"})
            fb_res = await _call_tool_with_retry(fallback_pour_tool, {}, "pour_to_target_fallback", reporter)
            fb_ok = bool(fb_res.get("ok") or fb_res.get("success"))
            # If fallback succeeds, update main result
            if fb_ok:
                sr.success = True
                sr.status = "fallback_success"
                sr.raw["fallback_result"] = fb_res
                state["results"]["move_to_pour"] = sr.to_dict()
        return state

    async def do_place(state: Dict[str, Any]):
        # 步骤开始提示
        src_info = state.get("object_id") or "original"
        await _maybe_report(reporter, {
            "type": "step_starting",
            "step": "place",
            "message": f"🔧 Starting Step 3: Place container back to {src_info} position",
            "details": "Returning container to its original location"
        })
        
        tool = tool_map["mcp_MTC_SERVER_place_container"]
        args: Dict[str, Any] = {"return_to_origin": True}
        if state.get("object_id"):
            args["object_id"] = state["object_id"]
        
        # 使用新的重试机制
        result = await _call_tool_with_retry(tool, args, "place", reporter)
        
        sr = StepResult(
            step="place",
            success=bool(result.get("ok") or result.get("success")),
            status=str(result.get("status", "")),
            error=str(result.get("error", "")),
            duration_sec=float(result.get("duration_sec", 0.0) or 0.0),
            raw=result,
        )
        state["results"]["place"] = sr.to_dict()
        return state

    async def do_return(state: Dict[str, Any]):
        # 步骤开始提示
        await _maybe_report(reporter, {
            "type": "step_starting",
            "step": "return",
            "message": "🔧 Starting Step 4: Return robot arm to home position",
            "details": "Moving robot arm back to safe home position"
        })
        
        tool = tool_map["mcp_MTC_SERVER_return_to_home"]
        args: Dict[str, Any] = {}
        
        # 使用新的重试机制
        result = await _call_tool_with_retry(tool, args, "return", reporter)
        
        sr = StepResult(
            step="return",
            success=bool(result.get("ok") or result.get("success")),
            status=str(result.get("status", "")),
            error=str(result.get("error", "")),
            duration_sec=float(result.get("duration_sec", 0.0) or 0.0),
            raw=result,
        )
        state["results"]["return"] = sr.to_dict()
        return state

    graph = StateGraph(dict)

    # Add nodes (confirmation gates removed)
    graph.add_node("resolve_and_choose_plan", resolve_and_choose_plan)
    graph.add_node("do_pick", do_pick)
    graph.add_node("do_move_to_pour", do_move_to_pour)
    graph.add_node("do_place", do_place)
    graph.add_node("do_return", do_return)

    # Set flow (direct execution, no confirmation gates)
    graph.set_entry_point("resolve_and_choose_plan")
    graph.add_edge("resolve_and_choose_plan", "do_pick")
    graph.add_edge("do_pick", "do_move_to_pour")
    graph.add_edge("do_move_to_pour", "do_place")
    graph.add_edge("do_place", "do_return")
    graph.add_edge("do_return", END)

    memory = MemorySaver()
    app = graph.compile(checkpointer=memory)

    return app, memory, initial_state


async def run_main_sequence_cli(tools: List[Any], user_prompt: str = "", reporter=None) -> Dict[str, Any]:
    app, _memory, initial_state = build_main_graph(tools, reporter=reporter)

    state = initial_state()
    state["user_prompt"] = user_prompt

    final_state = await app.ainvoke(state)

    print("\n=== Execution Results (Step by Step) ===")
    for step in ["pick", "move_to_pour", "place", "return"]:
        if step in final_state.get("results", {}):
            print(json.dumps(final_state["results"][step], ensure_ascii=False, indent=2))

    summary = {
        "template": final_state.get("plan_template"),
        "do_pour": final_state.get("do_pour"),
        "source": final_state.get("object_id"),
        "target": final_state.get("target_object_id"),
        "success": all(
            final_state["results"].get(s, {}).get("success", False)
            for s in ["pick", "move_to_pour", "place", "return"]
        ),
        "results": final_state.get("results", {}),
        "events": final_state.get("events", []),
    }

    print("\n=== Summary ===")
    print(json.dumps(summary, ensure_ascii=False, indent=2))

    return summary