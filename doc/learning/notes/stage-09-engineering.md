# 阶段 9 · 工程化与协作

- 日期：2026-11-06
- 对应计划：[阶段 9](../PLAN.md)
- 状态：进行中（实操已完成，待复核）

## 1. 目标

构建、脚本、日志、Git 四件事能自主完成，别人拿到仓库能复现。

## 2. 构建参数（[build.sh](file:///home/lxh/nanhu-research-navigation/scripts/build.sh)）

```bash
bash scripts/build.sh --packages-select nav2_net_system
```

实测输出：

```text
[build] distro=humble
Starting >>> nav2_net_system
Finished <<< nav2_net_system [0.10s]
Summary: 1 package finished [0.23s]
```

| 参数 | 作用 |
| --- | --- |
| `--symlink-install` | `install/` 里大量文件用符号链接指向 `build/`，减少重复拷贝与磁盘占用 |
| `-DBUILD_TESTING=OFF` | 不编译测试，缩短构建时间、减少测试依赖 |
| `-DDISTRO_ROS=humble` | livox 驱动靠它选分支，缺失会把头文件目录解析成 NOTFOUND |
| `--packages-select` | 只编译指定包，改一个包不必全量构建 |

**注意**：yaml/launch/world 经 `install(DIRECTORY)` 在构建时被**复制**到 `build/`，`install/` 只是软链到 `build/`。所以改源码里的 yaml，必须重建该包才会同步；阶段 8 里"新 yaml 直接传 src 绝对路径"是绕开这一步的调试手段，不是常规发布方式。

## 3. goal.sh（[scripts/goal.sh](file:///home/lxh/nanhu-research-navigation/scripts/goal.sh)）

职责：一条命令发目标，先做可用性检查，再带 `--feedback` 发送。

```bash
./scripts/goal.sh 12 8                 # 默认目标 (12,8)
NS=car1 TIMEOUT=60 ./scripts/goal.sh 9 8
```

四组实测：

| # | 用例 | 实际输出 | 退出码 |
| --- | --- | --- | --- |
| 1 | 未启动任何节点 | `[goal] 未找到 /car1/navigate_to_pose，请先启动导航栈` | **2** |
| 2 | 起栈后发 (12,8) | feedback 逐行输出，末尾 `Goal finished with status: SUCCEEDED`；末次 `distance_remaining=0.661`、`number_of_recoveries=0` | 0 |
| 3 | 发 (9,18)（超吸附 4.0 m） | `Goal finished with status: ABORTED` | **0** |
| 4 | 日志侧证 | `state_change PLANNING -> PLAN_FAILED` | — |

**关键坑 1**：`ros2 action send_goal` 对 **ABORTED 也返回退出码 0**。脚本里不能只看 `$?`，要抓输出里的 `Goal finished with status:` 才能区分成功与失败。

**关键坑 2**：脚本开头写 `set -euo pipefail` 会在 `source /opt/ros/humble/setup.bash` 时报 `AMENT_TRACE_SETUP_FILES：未绑定的变量`。正确顺序是：

```bash
set -eo pipefail          # 先不要 -u
source .../setup.bash
source .../install/setup.bash
set -u                    # source 完再打开未定义变量检查
```

## 4. Git 提交（原子提交）

本阶段历史（`git log --oneline --graph -8`）：

```text
* 9a0a3a7 merge: 解决冲突演练
|\
| * 9d0583f docs(stage9): 冲突演练分支A
* | 090679e docs(stage9): 冲突演练分支B
|/
* edb86b9 docs(stage9): 添加冲突演练文件
* e1d65a1 docs(learning): 阶段8验收并开启阶段9工程化
* 7f36866 feat(stage8): 增加 0.9 m 门与门位 13.5 m 场景副本
* 0cc8ec6 docs(learning): 阶段0 实操记录（自测+复现结果）
```

提交规范：

- 格式 `type(scope): 主题`，type 用 `feat / fix / docs / chore / refactor / test`；
- 一次提交只做一件事（场景副本、脚本、笔记分开）；
- 用 `git add <具体文件>`，不用 `git add -A`：本仓库里 `logs/`、`frames_*.gv`、`doc/learning/rviz/`、`notes/assets/` 都是需要逐个判断的产物；
- 合并提交用 `merge: ...`。

**push 卡点**：仓库没有配置 `credential.helper`，`git push` 会在凭据输入处挂起；`gh` 本身已登录（`gh api user` 正常），但 git 没接上。解决办法：

```bash
git -c credential.helper= -c credential.helper='!gh auth git-credential' push
```

成功输出 `30015d0..e1d65a1 main -> main`。长期方案是执行一次 `gh auth setup-git`（会写入全局 git 配置）。

## 5. 冲突演练（实测）

```bash
git switch -c conflict-demo
# 同一行改成「分支 A」并提交
git switch main
# 同一行改成「分支 B」并提交
git merge conflict-demo
```

冲突现场：

```text
<<<<<<< HEAD
演练标记：分支 B
=======
演练标记：分支 A
>>>>>>> conflict-demo
```

`<<<<<<< HEAD` = 当前分支（main）的版本；`=======` = 分隔线；`>>>>>>> conflict-demo` = 被合并分支的版本。

解决：整段替换为 `演练标记：已合并（A+B）` → `git add` → `git commit -m "merge: 解决冲突演练"` → `git branch -d conflict-demo`。放弃合并用 `git merge --abort`。

## 6. 日志与可观测性

实测：

```text
logs/navigation.log   210 KB   （JSON 行，含 [SNN] plan request/result、state_change）
logs/recorder.csv     715 KB
~/.ros/log/           python3_*.log、controller_server_*.log、net_planner_action_server_*.log
```

`logs/` 由 `nav2_log_record` 按 `./logs/navigation.log` 写入，路径相对**节点进程的工作目录**——从项目根目录起 launch 才会落到这里。`.gitignore` 已排除 `build/ install/ log/ logs/ snn_runs/`。

## 7. 关键结论

1. `--symlink-install` 不等于"改源码立即生效"；C++ 与安装类文件都要重建对应包。
2. 发目标的可靠性来自脚本的三件事：服务检查、超时、结果解析——而不是 CLI 退出码。
3. Git 一次提交一件事，配合明确的 `type(scope)`，历史才能读；冲突不可怕，先看清 HEAD/对方两侧再手工合并。
4. 复现别人的环境，先对齐三样：构建参数、日志落盘位置、凭据配置。

## 8. 卡点

| 现象 | 原因 | 解决 |
| --- | --- | --- |
| `git push` 无输出、长时间挂起 | 未配置 `credential.helper`，git 等凭据输入 | 用 `gh auth git-credential` 一次性推送，或 `gh auth setup-git` |
| goal.sh 一开始报 `AMENT_TRACE_SETUP_FILES 未绑定` | `set -u` 在 source ROS 环境前生效 | `set -eo pipefail` → source → `set -u` |
| 新增 yaml 在 `install/share` 里找不到 | `install(DIRECTORY)` 在构建时复制 | 重建对应包，或调试时传 src 绝对路径 |

## 9. 自测作答

1. **两个构建参数解决什么？** `--symlink-install` 减少拷贝、节省磁盘；`BUILD_TESTING=OFF` 跳过测试缩短构建。新增 yaml 仍需重建，因为它是在构建时被复制进 `build/` 的。
2. **为什么不用 `git add -A`？** 会把 `logs/`、`frames_*.gv`、`doc/learning/rviz/`、`notes/assets/` 等需要逐个判断的产物一起加进来，容易提交垃圾或敏感文件。
3. **三个冲突标记代表谁？** `<<<<<<< HEAD` 当前分支，`=======` 分隔，`>>>>>>> 分支名` 被合并分支。

## 10. 产出

- [x] [scripts/goal.sh](file:///home/lxh/nanhu-research-navigation/scripts/goal.sh)（含服务检查、超时、命名空间）
- [x] 4 组合规提交 + 1 次 merge 提交
- [x] 冲突演练记录（[conflict-demo.txt](file:///home/lxh/nanhu-research-navigation/doc/learning/notes/conflict-demo.txt)）
- [x] 日志路径与证据

## 11. 下次从哪继续

- 阶段 10（视觉选修）或直接 Capstone；阶段 5/6/7 的待补项仍需收尾。
