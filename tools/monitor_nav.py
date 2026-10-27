#!/usr/bin/env python3
"""
导航进程 CPU/内存监控（分模块统计）
用法: python3 tools/monitor_nav.py [namespace]
  默认 car1，Ctrl+C 停止，结果实时写入文件，最后打印各模块均值/峰值
"""

import subprocess
import time
import sys
import os

# ---- 模块定义: {显示名: (可执行名, 是否用 namespace 过滤)} ----
MODULES = [
    ("controller_server",       "controller_server",        True),
    ("net_planner",             "net_planner_action_server",True),
    ("net_navigator",           "net_navigator_node",       True),
    ("input_bridge",            "input_bridge_node",        True),
    ("log_server",              "log_server_node",          True),
    ("lifecycle_mgr",           "lifecycle_manager",        True),
]

# ---- 参数 ----
ns = sys.argv[1] if len(sys.argv) > 1 else "car1"
ns_filter = f"__ns:=/{ns}"
all_modules = list(MODULES)

# ---- 输出文件 ----
out_dir = os.path.dirname(os.path.abspath(__file__))
out_file = os.path.join(out_dir, f"nav_monitor_{ns}_{time.strftime('%Y%m%d_%H%M%S')}.log")

# ---- 数据收集: {模块名: {"cpu":[], "mem":[], "rss":[]}} ----
history = {name: {"cpu": [], "mem": [], "rss": []} for name, _, _ in all_modules}

def get_pid_stats(exec_name, use_ns):
    """根据可执行名 + 命名空间过滤获取该模块的所有 pid 的 cpu/mem/rss 之和"""
    cpu, mem, rss = 0.0, 0.0, 0
    try:
        if use_ns:
            pattern = f"{exec_name}.*{ns_filter}"
        else:
            pattern = exec_name
        out = subprocess.check_output(["pgrep", "-f", pattern], text=True).strip()
        if not out:
            return cpu, mem, rss
        for pid in out.split():
            try:
                stat = subprocess.check_output(
                    ["ps", "-p", pid, "-o", "%cpu,%mem,rss", "--no-headers"], text=True
                ).strip()
                parts = stat.split()
                cpu += float(parts[0])
                mem += float(parts[1])
                rss += int(parts[2])
            except subprocess.CalledProcessError:
                pass
    except subprocess.CalledProcessError:
        pass
    return cpu, mem, rss


# ---- 写文件 ----
def write_line(f, text):
    f.write(text + "\n")
    f.flush()


with open(out_file, "w") as f:
    write_line(f, f"监控目标: namespace=/{ns}")
    write_line(f, f"开始时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    write_line(f, f"输出文件: {out_file}\n")

    # 表头
    header = f"{'时间':<10}"
    for name, _, _ in all_modules:
        header += f" {name+'CPU':>14} {name+'MEM':>10} {name+'RSS':>11}"
    write_line(f, header)

    try:
        while True:
            row = f"{time.strftime('%H:%M:%S'):<10}"
            for name, exe, use_ns in all_modules:
                cpu, mem, rss = get_pid_stats(exe, use_ns)
                history[name]["cpu"].append(cpu)
                history[name]["mem"].append(mem)
                history[name]["rss"].append(rss)
                row += f" {cpu:>13.1f}% {mem:>9.1f}% {rss/1024:>10.1f}M"
            write_line(f, row)
            time.sleep(1)

    except KeyboardInterrupt:
        if not history[all_modules[0][0]]["cpu"]:
            print("\n无采样数据")
            sys.exit(1)

        n = len(history[all_modules[0][0]]["cpu"])

        write_line(f, f"\n{'='*80}")
        write_line(f, f"采样次数: {n}")
        write_line(f, f"{'='*80}\n")

        # 各模块统计
        total_cpu = [0.0] * n
        total_mem = [0.0] * n
        total_rss = [0.0] * n

        write_line(f, f"{'模块':<20} {'CPU均值':>8} {'CPU峰值':>8} {'MEM均值':>8} {'MEM峰值':>8} {'RSS均值':>10} {'RSS峰值':>10}")
        write_line(f, "-" * 80)

        for name, _, _ in all_modules:
            c = history[name]["cpu"]
            m = history[name]["mem"]
            r = history[name]["rss"]
            if any(v > 0 for v in c):
                write_line(f, f"{name:<20} {sum(c)/n:>7.1f}% {max(c):>7.1f}% "
                           f"{sum(m)/n:>7.1f}% {max(m):>7.1f}% "
                           f"{sum(r)/n/1024:>9.1f}M {max(r)/1024:>9.1f}M")
            else:
                write_line(f, f"{name:<20} {'(未启动)':>44}")

            for i in range(n):
                total_cpu[i] += c[i]
                total_mem[i] += m[i]
                total_rss[i] += r[i]

        # 总计
        write_line(f, "-" * 80)
        write_line(f, f"{'合计':<20} {sum(total_cpu)/n:>7.1f}% {max(total_cpu):>7.1f}% "
                   f"{sum(total_mem)/n:>7.1f}% {max(total_mem):>7.1f}% "
                   f"{sum(total_rss)/n/1024:>9.1f}M {max(total_rss)/1024:>9.1f}M")

        write_line(f, f"\n结束时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")

    # 终端也输出摘要
    print(f"\n{'='*80}")
    print(f"采样次数: {n}  输出文件: {out_file}")
    print(f"{'='*80}")
    print(f"{'模块':<20} {'CPU均值':>8} {'CPU峰值':>8} {'MEM均值':>8} {'MEM峰值':>8}")
    print("-" * 60)
    for name, _, _ in all_modules:
        c = history[name]["cpu"]
        m = history[name]["mem"]
        if any(v > 0 for v in c):
            print(f"{name:<20} {sum(c)/n:>7.1f}% {max(c):>7.1f}% {sum(m)/n:>7.1f}% {max(m):>7.1f}%")
        else:
            print(f"{name:<20} {'(未启动)'}")
    print("-" * 60)
    print(f"{'合计':<20} {sum(total_cpu)/n:>7.1f}% {max(total_cpu):>7.1f}% {sum(total_mem)/n:>7.1f}% {max(total_mem):>7.1f}%")
