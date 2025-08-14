#!/usr/bin/env python3
import sys
import json
import time
import argparse
from typing import Optional, Dict, Any

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from mtc_interface.action import ExecutePour

DEFAULTS: Dict[str, Any] = dict(
    tilt_start_deg=45.0,
    tilt_end_deg=120.0,
    tilt_speed_deg_s=25.0,
    pour_hold_sec=2.0,
    lift_height=0.12,
    approach_min=0.05,
    approach_max=0.15,
    plan_only=False,
    target_id="",
)


def _validate(params: Dict[str, Any]) -> Dict[str, Any]:
    p = {**DEFAULTS, **params}
    if p["approach_min"] <= 0 or p["approach_max"] <= 0:
        raise ValueError("approach_min/max must be > 0")
    if p["approach_min"] >= p["approach_max"]:
        raise ValueError("approach_min must be < approach_max")
    if p["tilt_speed_deg_s"] <= 0:
        raise ValueError("tilt_speed_deg_s must be > 0")
    if p["pour_hold_sec"] < 0:
        raise ValueError("pour_hold_sec must be >= 0")
    for k in ("tilt_start_deg", "tilt_end_deg"):
        p[k] = max(min(float(p[k]), 180.0), -180.0)
    return p


class PourTool(Node):
    def __init__(self, action_name: str = 'execute_pour'):
        super().__init__('pour_tool')
        self._ac = ActionClient(self, ExecutePour, action_name)

    def call(self, params: Dict[str, Any], timeout_sec: float = 180.0,
             cancel_after: Optional[float] = None) -> Dict[str, Any]:
        p = _validate(params)
        goal = ExecutePour.Goal()
        goal.target_id = p.get('target_id', '')
        goal.tilt_start_deg = float(p['tilt_start_deg'])
        goal.tilt_end_deg = float(p['tilt_end_deg'])
        goal.tilt_speed_deg_s = float(p['tilt_speed_deg_s'])
        goal.pour_hold_sec = float(p['pour_hold_sec'])
        goal.lift_height = float(p['lift_height'])
        goal.approach_min = float(p['approach_min'])
        goal.approach_max = float(p['approach_max'])
        goal.plan_only = bool(p['plan_only'])

        self.get_logger().info(
            f"Send goal: tilt {goal.tilt_start_deg}->{goal.tilt_end_deg} deg @ {goal.tilt_speed_deg_s} deg/s, "
            f"hold {goal.pour_hold_sec}s, lift {goal.lift_height} m, "
            f"approach[{goal.approach_min}, {goal.approach_max}], plan_only={goal.plan_only}")

        if not self._ac.wait_for_server(timeout_sec=5.0):
            return {"ok": False, "status": "no_server", "msg": "Action server /execute_pour not available"}

        send_future = self._ac.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            return {"ok": False, "status": "rejected", "msg": "Goal rejected by server", "params": p}

        self.get_logger().info('Goal accepted')
        start = time.time()

        if cancel_after is not None and cancel_after > 0:
            import threading

            def _schedule_cancel():
                time.sleep(cancel_after)
                self.get_logger().warn(f"Cancel after {cancel_after}s…")
                gh.cancel_goal_async()

            threading.Thread(target=_schedule_cancel, daemon=True).start()

        res_future = gh.get_result_async()
        while not res_future.done():
            if timeout_sec and time.time() - start > timeout_sec:
                self.get_logger().error(f"Timeout {timeout_sec}s, cancel goal")
                gh.cancel_goal_async()
                break
            rclpy.spin_once(self, timeout_sec=0.2)

        if not res_future.done():
            return {"ok": False, "status": "timeout", "msg": f"Timeout {timeout_sec}s", "params": p}

        result_msg = res_future.result()
        res = result_msg.result
        status = result_msg.status  # 4 SUCCEEDED, 5 CANCELED, 6 ABORTED
        return {
            "ok": bool(res.success) and status == 4,
            "status": {4: "succeeded", 5: "canceled", 6: "aborted"}.get(status, "unknown"),
            "success": bool(res.success),
            "duration_sec": float(res.duration_sec),
            "error": res.error_msg,
            "params": p,
        }

    def _on_feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(f"[{fb.stage}] progress={fb.progress:.2f}, tilt={fb.current_tilt_deg:.1f}")


# 供 MCP 直接 import 使用的简单入口

def call_once(params: Dict[str, Any], action_name: str = 'execute_pour',
              timeout_sec: float = 180.0, cancel_after: Optional[float] = None) -> Dict[str, Any]:
    """确保 rclpy 生命周期安全地执行一次调用并返回 dict。"""
    created = False
    if not rclpy.ok():
        rclpy.init()
        created = True
    node = PourTool(action_name)
    try:
        return node.call(params, timeout_sec=timeout_sec, cancel_after=cancel_after)
    finally:
        node.destroy_node()
        if created:
            rclpy.shutdown()


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--action-name', default='execute_pour')
    ap.add_argument('--plan-only', action='store_true')
    ap.add_argument('--start', type=float, default=DEFAULTS['tilt_start_deg'])
    ap.add_argument('--end', type=float, default=DEFAULTS['tilt_end_deg'])
    ap.add_argument('--speed', type=float, default=DEFAULTS['tilt_speed_deg_s'])
    ap.add_argument('--hold', type=float, default=DEFAULTS['pour_hold_sec'])
    ap.add_argument('--lift', type=float, default=DEFAULTS['lift_height'])
    ap.add_argument('--amin', type=float, default=DEFAULTS['approach_min'])
    ap.add_argument('--amax', type=float, default=DEFAULTS['approach_max'])
    ap.add_argument('--target-id', type=str, default=DEFAULTS['target_id'])
    ap.add_argument('--timeout', type=float, default=180.0)
    ap.add_argument('--cancel-after', type=float, default=None)
    ap.add_argument('--json', action='store_true', help='print JSON result to stdout')
    args = ap.parse_args(argv)

    rclpy.init()
    node = PourTool(action_name=args.action_name)
    try:
        res = node.call(dict(
            tilt_start_deg=args.start, tilt_end_deg=args.end,
            tilt_speed_deg_s=args.speed, pour_hold_sec=args.hold,
            lift_height=args.lift, approach_min=args.amin, approach_max=args.amax,
            plan_only=args.plan_only, target_id=args.target_id,
        ), timeout_sec=args.timeout, cancel_after=args.cancel_after)
        if args.json:
            print(json.dumps(res, ensure_ascii=False))
        else:
            print("RESULT:", res)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:]) 