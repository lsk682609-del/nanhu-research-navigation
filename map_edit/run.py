#!/usr/bin/env python3
"""通过 SSH 控制 3588 上的定位和导航脚本。

用法：
    python3 run.py loc
    python3 run.py nav
    python3 run.py restart
    python3 run.py stop

所有连接信息和远程路径都从同目录的 remote_config.json 读取，不进行交互输入。
"""

from __future__ import annotations

import argparse
import json
import shlex
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

try:
    import paramiko
except ImportError:
    print(
        "缺少 Paramiko。请在笔记本上安装：python3 -m pip install paramiko",
        file=sys.stderr,
    )
    sys.exit(2)


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = SCRIPT_DIR / "remote_config.json"
DEFAULT_REMOTE_DIR = "/root/jky_car8"
DEFAULT_STATE_DIR = "/root/jky_car8/log"
DEFAULT_LOG_DIR = "/root/jky_car8/log"


@dataclass
class Config:
    host: str
    username: str
    port: int = 22
    remote_dir: str = DEFAULT_REMOTE_DIR
    loc_script: str = f"{DEFAULT_REMOTE_DIR}/run.sh"
    nav_script: str = f"{DEFAULT_REMOTE_DIR}/3588_launch.sh"
    state_dir: str = DEFAULT_STATE_DIR
    log_dir: str = DEFAULT_LOG_DIR
    connect_timeout: float = 10.0


SERVICES = {
    "loc": "loc_script",
    "nav": "nav_script",
}


def load_config(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise RuntimeError(f"配置文件不存在: {path}")
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"读取配置文件失败 {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise RuntimeError(f"配置文件必须是 JSON 对象：{path}")
    return data


def make_config(args: argparse.Namespace) -> Tuple[Config, Optional[str]]:
    values = load_config(Path(args.config).expanduser())

    def required(name: str) -> str:
        value = values.get(name)
        if value is None or str(value).strip() == "":
            raise RuntimeError(f"配置文件缺少必填字段: {name}")
        return str(value)

    def value(name: str, default: Any) -> Any:
        return values.get(name, default)

    config = Config(
        host=required("host"),
        username=required("username"),
        port=int(value("port", 22)),
        remote_dir=required("remote_dir").rstrip("/") or "/",
        loc_script=required("loc_script"),
        nav_script=required("nav_script"),
        state_dir=required("state_dir").rstrip("/") or "/",
        log_dir=required("log_dir").rstrip("/") or "/",
        connect_timeout=float(value("connect_timeout", 10.0)),
    )

    password = required("password")
    return config, password


def q(value: str) -> str:
    return shlex.quote(value)


def service_paths(config: Config, service: str) -> Tuple[str, str, str]:
    script = str(getattr(config, SERVICES[service]))
    pidfile = f"{config.state_dir}/{service}.pid" if config.state_dir != "/" else f"/{service}.pid"
    logfile = f"{config.log_dir}/{service}.log" if config.log_dir != "/" else f"/{service}.log"
    return script, pidfile, logfile


def start_command(config: Config, service: str) -> str:
    script, pidfile, logfile = service_paths(config, service)
    # setsid --wait 让远程脚本拥有独立会话；PID 文件保存的是等待该脚本的进程。
    return f"""set -u
state_dir={q(config.state_dir)}
log_dir={q(config.log_dir)}
script={q(script)}
pidfile={q(pidfile)}
logfile={q(logfile)}
mkdir -p "$state_dir" "$log_dir"
if [ ! -f "$script" ]; then
    echo "远程脚本不存在: $script" >&2
    exit 2
fi
if [ -s "$pidfile" ]; then
    old_pid=$(sed -n '1p' "$pidfile" || true)
    if [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null; then
        echo "{service} 已在运行，PID=$old_pid，日志=$logfile"
        exit 0
    fi
    rm -f "$pidfile"
fi
: > "$logfile"
printf '===== {service} start %s =====\\n' "$(date '+%F %T')" >> "$logfile"
cd {q(config.remote_dir)}
nohup setsid --wait bash "$script" >> "$logfile" 2>&1 < /dev/null &
pid=$!
printf '%s\\n' "$pid" > "$pidfile"
sleep 0.4
if ! kill -0 "$pid" 2>/dev/null; then
    echo "{service} 启动后立即退出，请查看日志: $logfile" >&2
    rm -f "$pidfile"
    exit 1
fi
sid=$(ps -o sid= -p "$pid" 2>/dev/null | tr -d ' ' || true)
printf '%s %s\\n' "$pid" "$sid" > "$pidfile"
echo "{service} 已启动，PID=$pid，日志=$logfile"
"""


def stop_command(config: Config, service: str) -> str:
    _, pidfile, logfile = service_paths(config, service)
    return f"""set -u
pidfile={q(pidfile)}
logfile={q(logfile)}
if [ ! -s "$pidfile" ]; then
    echo "{service} 未找到 PID 文件，跳过"
    exit 0
fi
pid_line=$(sed -n '1p' "$pidfile" || true)
pid=$(printf '%s' "$pid_line" | awk '{{print $1}}')
sid=$(printf '%s' "$pid_line" | awk '{{print $2}}')
if [ -z "$pid" ] || ! printf '%s' "$pid" | grep -Eq '^[0-9]+$'; then
    echo "{service} PID 文件无效，已清理: $pidfile" >&2
    rm -f "$pidfile"
    exit 0
fi

kill_tree() {{
    target="$1"
    for child in $(pgrep -P "$target" 2>/dev/null || true); do
        kill_tree "$child"
    done
    kill -TERM "$target" 2>/dev/null || true
}}

if [ -z "$sid" ] && kill -0 "$pid" 2>/dev/null; then
    sid=$(ps -o sid= -p "$pid" 2>/dev/null | tr -d ' ' || true)
fi

# ros2 launch 可能在停止父脚本后被重新挂到 PID 1，按父子关系无法覆盖。
# 只杀掉该服务自己的 Session，避免影响 SSH 停止命令所在的 Session。
own_sid=$(ps -o sid= -p "$$" 2>/dev/null | tr -d ' ' || true)
if printf '%s' "$sid" | grep -Eq '^[0-9]+$' && [ "$sid" != "1" ] && [ "$sid" != "$own_sid" ]; then
    pkill -TERM -s "$sid" 2>/dev/null || true
fi

if kill -0 "$pid" 2>/dev/null; then
    kill_tree "$pid"
    for _ in $(seq 1 20); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.25
    done
    if kill -0 "$pid" 2>/dev/null; then
        kill -KILL "$pid" 2>/dev/null || true
    fi
else
    echo "{service} 进程已不存在，清理 PID 文件"
fi

if printf '%s' "$sid" | grep -Eq '^[0-9]+$' && [ "$sid" != "1" ] && [ "$sid" != "$own_sid" ]; then
    pkill -KILL -s "$sid" 2>/dev/null || true
    echo "{service} 已停止（包含同一 Session 的脱离子进程），日志=$logfile"
else
    echo "{service} 已停止，日志=$logfile"
fi
rm -f "$pidfile"
"""


def run_remote(client: paramiko.SSHClient, command: str) -> str:
    stdin, stdout, stderr = client.exec_command(command, get_pty=False)
    # 读取 stdout/stderr，避免远程命令因管道缓冲区满而阻塞。
    out = stdout.read().decode("utf-8", errors="replace").strip()
    err = stderr.read().decode("utf-8", errors="replace").strip()
    status = stdout.channel.recv_exit_status()
    if status != 0:
        detail = err or out or f"远程命令退出码 {status}"
        raise RuntimeError(detail)
    if err:
        print(err, file=sys.stderr)
    return out


def connect(config: Config, password: str) -> paramiko.SSHClient:
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    # 用户只需提供 IP/用户名/密码；首次连接时自动接受主机指纹。
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(
            hostname=config.host,
            port=config.port,
            username=config.username,
            password=password,
            timeout=config.connect_timeout,
            banner_timeout=config.connect_timeout,
            auth_timeout=config.connect_timeout,
            allow_agent=False,
            look_for_keys=False,
        )
    except (paramiko.AuthenticationException, paramiko.BadHostKeyException) as exc:
        client.close()
        raise RuntimeError(f"SSH 认证或主机密钥失败: {exc}") from exc
    except (paramiko.SSHException, socket.error, OSError) as exc:
        client.close()
        raise RuntimeError(f"无法连接 3588 {config.host}:{config.port}: {exc}") from exc
    return client


def execute_action(client: paramiko.SSHClient, config: Config, action: str) -> None:
    if action in SERVICES:
        print(run_remote(client, start_command(config, action)))
        return

    if action == "stop":
        for service in ("loc", "nav"):
            try:
                print(run_remote(client, stop_command(config, service)))
            except RuntimeError as exc:
                print(f"停止 {service} 失败: {exc}", file=sys.stderr)
        return

    if action == "restart":
        for service in ("loc", "nav"):
            try:
                print(run_remote(client, stop_command(config, service)))
            except RuntimeError as exc:
                print(f"停止 {service} 失败: {exc}", file=sys.stderr)
        time.sleep(0.5)
        for service in ("loc", "nav"):
            print(run_remote(client, start_command(config, service)))
        return

    raise RuntimeError(f"不支持的操作: {action}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="通过 SSH 控制 3588 的定位/导航脚本")
    parser.add_argument("action", choices=("loc", "nav", "restart", "stop"))
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="JSON 配置文件路径")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    client = None
    try:
        config, password = make_config(args)
        client = connect(config, password or "")
        execute_action(client, config, args.action)
        return 0
    except (RuntimeError, KeyboardInterrupt) as exc:
        if isinstance(exc, KeyboardInterrupt):
            print("已取消", file=sys.stderr)
        else:
            print(f"错误: {exc}", file=sys.stderr)
        return 1
    finally:
        if client is not None:
            client.close()


if __name__ == "__main__":
    sys.exit(main())
