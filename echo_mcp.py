# /home/wenhao/uf_custom_ws/echo_mcp.py
from mcp.server.fastmcp import FastMCP

server = FastMCP("echo")

@server.tool()
async def echo(text: str) -> str:
    return text

if __name__ == "__main__":
    server.run(transport='stdio')
