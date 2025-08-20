# Agent友好工具使用指南

## 📋 工具概览

### 新增的Agent辅助工具：

1. **`get_task_state()`** - 系统状态检查工具
2. **`abort_and_reset(reason)`** - 紧急中止和恢复工具

这些工具专为LLM Agent设计，提供丰富的状态信息和可靠的失败恢复能力。

## 🤖 Agent使用场景

### 场景1: 任务前的系统检查

```python
# Agent调用 get_task_state() 检查系统状态
state = await mcp_client.call_tool("get_task_state")

if state["system_ready"]:
    print("✅ 系统就绪，可以开始任务")
    if state["gripper_state"] == "open":
        print("夹爪已张开，准备抓取")
    elif state["gripper_state"] == "grasping":
        print("夹爪正在抓取物体")
else:
    print("❌ 系统未就绪")
    print(f"错误原因: {state['last_error']}")
    # Agent决定是否需要重启或修复
```

### 场景2: 任务执行中的监控

```python
# Agent在倾倒任务执行过程中定期检查状态
async def monitor_pour_task():
    while True:
        state = await mcp_client.call_tool("get_task_state")
        
        if state["stage"] == "executing":
            print(f"任务进行中，夹爪状态: {state['gripper_state']}")
            
        elif state["stage"] == "error":
            print(f"⚠️ 检测到错误: {state['last_error']}")
            # Agent决定是否需要中止任务
            await mcp_client.call_tool("abort_and_reset", {
                "reason": f"监控检测到错误: {state['last_error']}"
            })
            break
            
        elif state["stage"] == "completed":
            print("✅ 任务完成")
            break
            
        await asyncio.sleep(2)  # 每2秒检查一次
```

### 场景3: 异常处理和恢复

```python
# Agent在检测到异常时的处理流程
async def handle_emergency():
    try:
        # 尝试执行倾倒任务
        result = await mcp_client.call_tool("execute_pour", {
            "tilt_start_deg": 45,
            "tilt_end_deg": 120,
            "plan_only": False
        })
        
        if not result["success"]:
            print("倾倒任务失败，启动恢复程序...")
            
            # 中止并重置到安全状态
            abort_result = await mcp_client.call_tool("abort_and_reset", {
                "reason": f"倾倒失败: {result.get('error', 'Unknown error')}"
            })
            
            if abort_result["success"]:
                print("✅ 成功恢复到安全状态")
                # Agent可以决定是否重试或报告问题
            else:
                print("❌ 恢复失败，需要人工干预")
                print(f"警告: {abort_result['warnings']}")
                
    except Exception as e:
        print(f"严重错误: {e}")
        # 紧急中止
        await mcp_client.call_tool("abort_and_reset", {
            "reason": f"Python异常: {str(e)}"
        })
```

### 场景4: 智能任务规划

```python
# Agent根据系统状态智能规划任务
async def smart_pour_planning():
    # 1. 检查初始状态
    state = await mcp_client.call_tool("get_task_state")
    
    if not state["system_ready"]:
        return {"error": "系统未就绪", "details": state["last_error"]}
    
    # 2. 根据夹爪状态决定下一步动作
    if state["gripper_state"] == "closed":
        print("检测到夹爪闭合，可能已有物体，直接开始倾倒")
        pour_params = {"tilt_start_deg": 30}  # 较小的起始角度
        
    elif state["gripper_state"] == "open": 
        print("夹爪张开，需要先抓取物体")
        # 可以调用其他抓取工具...
        
    elif state["gripper_state"] == "grasping":
        print("检测到正在抓取，等待完成后开始倾倒")
        pour_params = {"tilt_start_deg": 45}  # 标准角度
        
    # 3. 执行倾倒
    result = await mcp_client.call_tool("execute_pour", pour_params)
    
    return {
        "success": result["success"],
        "initial_state": state,
        "action_taken": "pour" if result["success"] else "abort",
        "final_result": result
    }
```

## 📊 状态信息详解

### get_task_state() 返回的状态字段：

```json
{
  "stage": "idle|planning|executing|completed|error",
  "last_error": "错误描述或null",
  "robot_pose": {
    "position": {"x": 0.5, "y": -0.3, "z": 0.8},
    "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
  },
  "gripper_state": "open|closed|grasping",
  "gripper_position": 0.42,           // 原始位置值
  "gripper_position_normalized": 0.5, // 归一化到0-1
  "action_status": "available|unavailable|error",
  "system_ready": true,
  "timestamp": 1634567890.123
}
```

### abort_and_reset() 返回的结果字段：

```json
{
  "success": true,
  "reason": "用户请求的中止原因",
  "actions_taken": [
    "Abort requested: 原因",
    "Cancelled all pour actions",
    "Robot moved to safe pose via MoveGroup",
    "Gripper opened"
  ],
  "warnings": ["可选的警告信息"],
  "robot_safe": true,
  "timestamp": 1634567890.123
}
```

## 🎯 Agent决策逻辑建议

### 基于状态的决策树：

```
检查 system_ready
├─ True: 系统正常
│  ├─ stage == "idle" → 可以开始新任务
│  ├─ stage == "executing" → 监控进展
│  ├─ stage == "completed" → 任务成功
│  └─ stage == "error" → 错误处理
└─ False: 系统异常
   └─ 检查 last_error，决定修复策略
```

### 夹爪状态决策：

```
检查 gripper_state
├─ "open" → 准备抓取
├─ "closed" → 可能已抓取物体
├─ "grasping" → 正在抓取过程中
└─ "error" → 夹爪异常，需要重置
```

## 🔧 测试命令

### CLI测试：
```bash
# 测试状态检查
python3 mtc_mcp_tools.py get-state

# 测试中止功能
python3 mtc_mcp_tools.py abort --reason "测试中止功能"

# 运行完整测试
python3 test_agent_tools.py
```

### MCP客户端测试：
```bash
# 启动MCP服务器
python3 mtc_mcp_server.py

# 在另一个终端测试状态检查
echo '{"method": "tools/call", "params": {"name": "get_task_state"}}' | nc localhost 3000
```

## 💡 最佳实践

1. **定期状态检查**: 在执行长时间任务时，每1-2秒检查一次状态
2. **错误预处理**: 在执行危险操作前，先检查系统状态
3. **优雅降级**: 出现错误时，优先使用 abort_and_reset 而不是强制停止
4. **日志记录**: 使用有意义的 reason 参数，便于调试和审计
5. **并发安全**: 避免同时调用多个可能冲突的操作

这些工具让Agent能够更智能、更安全地执行复杂的机器人任务！ 