# LangGraph Agent for Robot Control

基于 LangGraph 和 Claude 的机器人操作 AI Agent。

## 快速开始

### 1. 安装依赖

```bash
cd Langraph_Agent
pip install -r simple_requirements.txt
```

### 2. 配置 API Key

**方式 1：使用环境变量（推荐）**
```bash
export ANTHROPIC_API_KEY="your-anthropic-api-key"
```

**方式 2：使用 .env 文件**
```bash
# 复制示例文件
cp .env.example .env

# 编辑 .env 文件，填入您的 API Key
nano .env
```

**重要提示：**
- ⚠️ **永远不要**将真实的 API Key 提交到 Git 仓库
- `.env` 文件已添加到 `.gitignore`
- 使用 `.env.example` 作为模板

### 3. 启动系统

```bash
# 使用启动脚本
bash start_simple.sh

# 或手动启动
python3 agent_app.py
```

## 系统架构

### 主要组件

1. **agent_app.py**: Agent 主程序
   - 集成 Claude Sonnet 4 模型
   - MCP 工具适配器
   - 任务执行控制器

2. **task_graph.py**: 任务图定义
   - 确定性任务流程
   - 状态管理
   - 错误处理

3. **simple_backend.py**: 后端服务
   - MCP 服务器管理
   - 工具调用接口
   - 结果反馈

### 工作流程

```
用户指令 → Agent 解析 → 任务图规划 → MCP 工具调用 → 机器人执行
```

## 安全最佳实践

### API Key 管理

1. **开发环境**
   ```bash
   # 添加到 ~/.bashrc 或 ~/.zshrc
   export ANTHROPIC_API_KEY="your-key"
   ```

2. **生产环境**
   - 使用密钥管理系统（如 AWS Secrets Manager）
   - 使用环境变量注入
   - 定期轮换 API Key

3. **检查泄露**
   ```bash
   # 检查是否有硬编码的 key
   grep -r "sk-ant-api" .
   
   # 清理 git 历史中的 secrets
   git filter-branch --force --index-filter \
     "git rm --cached --ignore-unmatch Langraph_Agent/agent_app.py" \
     --prune-empty --tag-name-filter cat -- --all
   ```

## 功能特性

- ✅ 基于 Task Graph 的确定性工作流
- ✅ MCP 协议标准化接口
- ✅ 实时状态反馈
- ✅ 错误恢复机制
- ✅ 任务确认门控

## 故障排查

### 问题：Agent 无法启动

**解决方案：**
1. 检查 API Key 是否设置
   ```bash
   echo $ANTHROPIC_API_KEY
   ```

2. 检查依赖是否安装
   ```bash
   pip list | grep -E "(langgraph|anthropic|langchain)"
   ```

### 问题：MCP 工具无法调用

**解决方案：**
1. 确保 MTC 服务器已启动
   ```bash
   ros2 launch mtc_tutorial modular_task_server.launch.py
   ```

2. 检查 MCP 配置
   ```bash
   cat mcp_server.json
   ```

## 开发指南

### 添加新任务

1. 在 `task_graph.py` 中定义任务节点
2. 在 `agent_app.py` 中注册任务处理器
3. 测试任务执行流程

### 调试模式

```bash
# 启用详细日志
export LANGCHAIN_TRACING_V2=true
export LANGCHAIN_ENDPOINT=https://api.smith.langchain.com
export LANGCHAIN_API_KEY="your-langsmith-key"

# 运行 Agent
python3 agent_app.py
```

## 相关文档

- [架构分析](../docs/architecture_analysis.md)
- [MTC 使用指南](../docs/mtc_usage_guide.md)
- [优化总结](OPTIMIZATION_SUMMARY.md)

## 许可证

TODO: 添加许可证信息
