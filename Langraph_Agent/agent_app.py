import asyncio, json, os, sys
from langgraph.prebuilt import create_react_agent
from langchain_anthropic import ChatAnthropic
from langchain_mcp_adapters.client import MultiServerMCPClient

# ==================== API Key 配置区域 ====================
# 从环境变量读取API Key（安全实践）
# 使用方式：export ANTHROPIC_API_KEY="your-key"
# 或创建 .env 文件（已添加到 .gitignore）
ANTHROPIC_API_KEY = os.getenv("ANTHROPIC_API_KEY")

if not ANTHROPIC_API_KEY:
    print("警告: 未找到 ANTHROPIC_API_KEY 环境变量")
    print("请设置: export ANTHROPIC_API_KEY='your-api-key'")
# ========================================================

SYSTEM_PROMPT = """
You are a robot operation agent.
- Main workflow is solidified by deterministic Task Graph (pick → move_to_pour → place → return), with confirmation gates before high-risk actions.
- You are only allowed to use MCP tools; strictly forbidden to generate coordinates/poses/velocity parameters yourself.
- Limited ReAct is only used for lightweight reasoning in Resolve/ChoosePlan (parsing object_id / selecting templates) and result interpretation; actual execution is driven by Task Graph.
- Common tools: pick / move_to_pour / place / return (and state/scene related tools).
- Output should be concise and structured, avoiding exposure of internal stack traces.
"""

from task_graph import run_main_sequence_cli  # 新增：导入主流程

def create_claude_model():
    """创建Claude Sonnet 4模型实例"""
    if ANTHROPIC_API_KEY:
        return ChatAnthropic(model="claude-sonnet-4-20250514", api_key=ANTHROPIC_API_KEY, temperature=0)
    else:
        # 使用环境变量 ANTHROPIC_API_KEY
        return ChatAnthropic(model="claude-sonnet-4-20250514", temperature=0)

async def load_tools_from_mcp(config_path="mcp_server.json"):
    with open(config_path, "r") as f:
        servers = json.load(f)
    client = MultiServerMCPClient(servers)
    tools = await client.get_tools()
    return tools

async def console_reporter(event: dict):
    t = event.get("type", "event")
    step = event.get("step")
    tool = event.get("tool")
    msg = event.get("message", "")
    attempt = event.get("attempt")
    
    # 支持新的事件类型
    if t in ("before_tool", "after_tool", "fallback_attempt", "fallback_result", "plan_selected", 
             "tool_attempt", "tool_result", "auto_retry", "user_decision_needed", 
             "user_approved_retry", "user_skipped", "execution_plan", "step_starting"):
        
        # 特殊格式化执行计划和步骤开始
        if t == "execution_plan":
            print(f"📋 {msg}")
            if event.get("steps"):
                for step_desc in event["steps"]:
                    print(f"   {step_desc}")
            if event.get("estimated_duration"):
                print(f"   ⏱️  {event['estimated_duration']}")
            return
        
        if t == "step_starting":
            print(f"🔧 {msg}")
            if event.get("details"):
                print(f"   💡 {event['details']}")
            return
        
        # 格式化输出
        if attempt:
            print(f"➡️  {t} | step={step or '-'} | attempt={attempt} | {msg}")
        else:
            print(f"➡️  {t} | step={step or '-'} | tool={tool or '-'} | {msg}")
        
        # 显示参数
        if t in ("before_tool", "tool_attempt") and event.get("params"):
            print(f"   📋 params: {json.dumps(event['params'], ensure_ascii=False)}")
        
        # 显示结果
        if t in ("after_tool", "fallback_result", "tool_result") and event.get("result"):
            result = event["result"]
            success = result.get("success", False)
            status = result.get("status", "unknown")
            error = result.get("error", "")
            print(f"   {'✅' if success else '❌'} result: success={success}, status={status}")
            if error and not success:
                print(f"   🚨 error: {error}")
        
        # Additional info for special events
        if t == "user_decision_needed":
            print(f"   ⚠️  User decision needed: tool has failed twice consecutively")
        elif t == "auto_retry":
            print(f"   🔄 Auto-retrying...")
        elif t == "user_skipped":
            print(f"   ⏭️  User chose to skip this step")

async def main():
    print("Starting Robot Operation Agent (Claude Sonnet 4)")
    
    # 1) Connect MCP tools
    tools = await load_tools_from_mcp()
    print(f"Loaded {len(tools)} MCP tools")

    # 2) Create Claude model
    try:
        llm = create_claude_model()
        print("✅ Claude Sonnet 4 model loaded")
    except Exception as e:
        print(f"❌ Model creation failed: {e}")
        return

    # 3) Create robot Agent
    agent = create_react_agent(llm, tools, prompt=SYSTEM_PROMPT)

    print("\n🤖 Robot Agent Ready!")
    print("Enter commands to start operations, for example:")
    print("- 'run main' or 'execute main flow [with object_id:object_2 / dst:object_3 / no pour]' ")
    print("- Other Q&A will use limited ReAct for explanation/parsing only, no direct actions")
    print("\nPress Ctrl+C to exit.\n")
    
    while True:
        try:
            user = input("You: ").strip()
            if not user:
                continue

            if user.lower().startswith("run main") or user.lower().startswith("execute main"):
                print("\n⚙️ Starting deterministic task graph main flow (pick -> move_to_pour -> place -> return)...")
                summary = await run_main_sequence_cli(tools, user, reporter=console_reporter)
                print("\n🤖 Agent:")
                print(json.dumps(summary, ensure_ascii=False, indent=2))
                continue

            # Limited ReAct: explanation/parsing only, no direct robot control
            result = await agent.ainvoke({"messages":[{"role":"user","content":user}]})
            final_msg = result["messages"][-1].content
            print(f"\n🤖 Agent: {final_msg}\n")
        except KeyboardInterrupt:
            print("\n👋 Goodbye!"); break

if __name__ == "__main__":
    asyncio.run(main())
