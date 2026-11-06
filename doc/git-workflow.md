# 团队 Git 协作规范

> 适用范围：`nanhu-research-navigation` 及其下属 ROS 2 工作空间。
> 目标：让多人在同一套导航代码上并行开发时，**主线始终可编译、可跑仿真**，改动可追溯、可回滚、可评审。
>
> 最后更新：2026-10-28

---

## 1. 基本约定

### 1.1 四个区域

```text
  工作区              暂存区            本地仓库                远程（GitHub）
 (改动的文件) ──add──► (待提交清单) ──commit──► (本地历史) ──push──► origin
      ▲                                                                    │
      └──────────────────────────── pull ───────────────────────────────────┘
```

- 只改文件不会进入版本库，必须 `git add` + `git commit`。
- 只有 `git push` 之后，同事才能看到你的改动。

### 1.2 远端与分支

| 名称 | 含义 |
| --- | --- |
| `origin` | 主仓库 `https://github.com/lsk682609-del/nanhu-research-navigation.git` |
| `main` | 稳定主线，**只接受 PR 合并**，禁止直接推送 |
| `feature/*` `fix/*` `docs/*` | 个人工作分支，从 `main` 切出，完成即删 |

### 1.3 必须遵守的三条底线

1. **不提交构建产物与运行产物**：`build/`、`install/`、`log/`、`logs/`、`snn_runs/`（已在 `.gitignore` 中）。
2. **不提交任何密钥**：GitHub token、密码、含账号信息的 `remote_config.json`。
3. **不改写已推送的历史**：禁止对 `main` 使用 `git push --force`；需要撤销时用 `git revert`。

---

## 2. 分支模型

### 2.1 分支命名

| 类型 | 格式 | 示例 |
| --- | --- | --- |
| 新功能 | `feature/<简短英文>` | `feature/door-servo-optimize` |
| 缺陷修复 | `fix/<简短英文>` | `fix/front-stop-creep` |
| 文档 | `docs/<简短英文>` | `docs/git-workflow` |
| 重构 | `refactor/<简短英文>` | `refactor/costmap-layers` |
| 实验性验证 | `exp/<简短英文>` | `exp/theta-star-weight` |

规则：全小写、用连字符、不超过 30 个字符、不要用中文和空格。

### 2.2 生命周期

```bash
git switch main
git pull
git switch -c feature/door-servo-optimize
# ... 开发、提交 ...
git push -u origin feature/door-servo-optimize
# 在 GitHub 发起 PR → 评审 → 合并 → 删除分支
```

合并完成后本地清理：

```bash
git switch main && git pull
git branch -d feature/door-servo-optimize
```

### 2.3 `main` 分支保护（仓库管理员设置一次）

GitHub → Settings → Branches → Add branch protection rule：

- Branch name pattern：`main`
- 勾选 `Require a pull request before merging`
- 勾选 `Require approvals`（至少 1 人）
- 勾选 `Require status checks to pass before merging`（后续接入 ROS 编译 CI 时）

---

## 3. 提交规范

### 3.1 格式

```text
<类型>(<范围>): <一句话说明>

<可选：为什么改、验证了什么>
```

- 标题不超过 72 个字符，不加句号。
- 类型与范围用小写英文，说明用中文。
- **一个提交只做一件事**：改代码和改文档分开，改控制器和改地图分开。

### 3.2 类型

| 类型 | 用途 |
| --- | --- |
| `feat` | 新增功能 |
| `fix` | 修复缺陷 |
| `docs` | 仅文档 |
| `refactor` | 重构，不改变外部行为 |
| `perf` | 性能优化 |
| `test` | 测试用例或测试脚本 |
| `chore` | 构建、依赖、配置（非业务代码） |
| `revert` | 回滚某次提交 |

### 3.3 范围（本仓库常用）

`controller`、`costmap`、`planner`、`navigator`、`bridge`、`sim`、`launch`、`config`、`build`

### 3.4 示例

```text
fix(controller): 后退自救改用里程计实测位移，避免受阻误判完成
feat(controller): 门区新增 CROSS 阶段航向锁存
docs: 新增团队 Git 协作规范
chore(build): 增量编译仅构建 nav2_a_star_rpp
test(sim): 补充封门场景回归用例
```

反例（请在评审中打回）：`update`、`修改一下`、`提交代码`、`fix bug`。

### 3.5 提交粒度

- 一个提交应当能独立编译通过（至少能通过 `colcon build --packages-select <改动的包>`）。
- 大改动拆成“可解释的步骤”：先加参数、再接逻辑、最后调默认值。
- 不要把格式化整个文件、重命名无关变量混进功能提交里，否则 diff 无法评审。

---

## 4. 日常开发流程

### 4.1 标准七步

```bash
# 1) 同步主线
git switch main && git pull

# 2) 开分支
git switch -c feature/xxx

# 3) 改代码

# 4) 自检
git status
git diff

# 5) 编译验证（只编译改动的包，节省时间）
colcon build --symlink-install --packages-select nav2_a_star_rpp --cmake-args -DBUILD_TESTING=OFF

# 6) 提交
git add <改动文件>
git commit -m "fix(controller): ..."

# 7) 推送并在 GitHub 发起 PR
git push -u origin feature/xxx
```

### 4.2 提交前检查清单

- [ ] `git status` 中没有 `build/`、`install/`、`log/`、`logs/`、大文件、密钥
- [ ] `git diff` 只包含本次任务相关的改动
- [ ] 编译通过（至少改动包）
- [ ] 涉及导航行为时，已在 Gazebo 中跑过对应场景
- [ ] 新增参数已在 `gazebo_net_nav.yaml` 与代码默认值中保持一致
- [ ] 提交信息符合第 3 节格式

### 4.3 导航代码的专项验证要求

改动以下模块时，PR 描述中必须写明验证场景与结果：

| 模块 | 至少验证的场景 |
| --- | --- |
| `nav2_a_star_rpp`（控制器/门区/自救） | 正常穿门 `(12,8)` + 走廊受阻 `(9,11)` + 封门场景 |
| `nav2_costmap_2d`（代价图层/膨胀） | 窄门可通过、坡道不误判致命 |
| `nav2_net_planner` / `net_navigator` | 目标可达、目标不可达时能明确失败 |
| `robot_launch` 配置 | 全套 launch 能正常拉起，RViz 有 TF |

记录格式建议：

```text
验证场景：test_world.world，起点 (9,8)，目标 (9,11)，前方 0.5 m 静态箱
结果：距障碍 0.50 m 停车 → BACKUP ×2 → 重试 2/2 → EXEC_FAILED
日志关键字：[安全]、[恢复]、重试 n/2
```

---

## 5. 同步与冲突

### 5.1 推送被拒（远端有新提交）

```bash
git pull --rebase
git push
```

> 团队统一使用 `git pull --rebase`，避免产生无意义的 merge 提交。

配置一次即可全局生效：

```bash
git config --global pull.rebase true
```

### 5.2 冲突处理

```bash
git status                 # 列出 both modified 的文件
# 编辑冲突文件，保留最终内容，删除 <<<<<<< ======= >>>>>>> 标记
git add <已解决的文件>
git rebase --continue
```

- 冲突涉及**导航参数、门区几何、路网数据**时，不要自行猜一个值，先找对应模块负责人确认。
- 处理不了就 `git rebase --abort` 回到干净状态，再求助。

### 5.3 本地改动未完成时切分支

```bash
git stash
git switch main && git pull
git switch feature/xxx
git stash pop
```

---

## 6. Pull Request 规范

### 6.1 标题

与提交信息同格式，例如：`fix(controller): 前向停车净空 0.45 m 并补充受阻判据`

### 6.2 描述模板

```markdown
## 背景
为什么要改（对应问题/现象/需求）。

## 改动
- 具体改了哪些文件、哪些逻辑

## 验证
- 编译：colcon build --packages-select ...
- 场景：test_world.world，起点/目标/障碍布置
- 结果：日志关键字与关键数值

## 风险
- 可能影响的模块（门区/自救/代价地图等）
- 回滚方式
```

### 6.3 评审要点

评审人重点看：

1. **行为是否可解释**：参数改了之后，逻辑闭环是否仍然成立（例如停车净空与降速带是否匹配）。
2. **默认值一致性**：`hpp` 默认值、`cpp` `declare_parameter`、`yaml` 三处是否一致。
3. **边界条件**：净空刚好等于阈值、门框擦边、车尾受阻等。
4. **日志可观测**：新增分支是否有 `RCLCPP_WARN/INFO` 便于现场排查。
5. **无副作用**：是否影响正常穿门场景。

### 6.4 合并方式

- 默认 **Squash and merge**：一个 PR 压成一个提交，保持 `main` 历史线性。
- 合并后删除源分支。
- 合并前必须：CI/编译通过 + 至少 1 人 approve + 冲突已解决。

---

## 7. 版本与发布

| 动作 | 命令 / 位置 |
| --- | --- |
| 打标签 | `git tag -a v1.2.0 -m "窄门穿越与两级恢复稳定版"`，然后 `git push origin v1.2.0` |
| 发布说明 | GitHub → Releases → Draft a new release，选标签，列出主要改动与验证场景 |
| 版本号 | `主版本.次版本.修订号`：导航行为变化升次版本，缺陷修复升修订号 |

---

## 8. 禁止事项

| 禁止 | 原因 | 正确做法 |
| --- | --- | --- |
| 直接向 `main` push | 破坏主线稳定性 | 走分支 + PR |
| `git push --force` 到共享分支 | 覆盖同事提交 | 用 `git revert` |
| 提交 `build/ install/ log/` | 体积巨大、机器相关、冲突频繁 | 保持 `.gitignore`，误加用 `git rm -r --cached <目录>` |
| 提交 token / 密码 | 泄露账号 | 用环境变量或本地凭据助手，泄露后立即撤销重建 |
| 提交 >100 MB 单文件 | GitHub 硬限制 | 用 Git LFS 或外部存储 |
| 在 PR 中夹带无关格式化 | 无法评审 | 单独提 `chore` 提交 |
| 用 `git reset --hard` 清理别人也在用的分支 | 丢工作 | 先沟通，必要时 `revert` |

---

## 9. 凭据与安全

### 9.1 推荐方式：SSH 密钥（一次配置，长期免密）

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
cat ~/.ssh/id_ed25519.pub        # 粘贴到 GitHub → Settings → SSH and GPG keys
git remote set-url origin git@github.com:lsk682609-del/nanhu-research-navigation.git
```

### 9.2 使用 HTTPS + Token 时

- 密码栏填 **token**，不是账号密码。
- Classic token 必须勾选 `repo`；Fine-grained token 必须给目标仓库 `Contents: Read and write`。
- 自检（不要把 token 发到聊天或工单里）：

```bash
curl -sI -H "Authorization: token <TOKEN>" https://api.github.com/user | grep -i x-oauth-scopes
```

- 缓存凭据（个人电脑可用，明文存放于 `~/.git-credentials`）：

```bash
git config --global credential.helper store
```

- token 一旦出现在聊天、日志、截图中，**立即撤销并重新生成**。

### 9.3 首次配置身份

```bash
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"
```

团队内提交作者必须可识别，便于追溯。

---

## 10. 常见问题

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `Permission ... denied` / 403 | token 无写权限 | 重新生成带 `repo` 或 `Contents: Read and write` 的 token |
| `Bad credentials` / 401 | token 无效、复制不全、已撤销 | 重新复制完整 token，注意无空格换行 |
| `non-fast-forward` / `rejected` | 远端有新提交 | `git pull --rebase` 后重新 push |
| `Your local changes would be overwritten` | 有未提交改动 | 先 `git stash` 或提交，再 pull |
| 推送后 GitHub 上看不到文件 | 未 `commit` 或推错分支 | `git log --oneline -3` 与 `git branch -vv` 核对 |
| 仓库体积异常增大 | 误提交构建产物/大文件 | `git rm -r --cached`，必要时清理历史 |
| 提交作者显示为 unknown | 未配置 `user.name/email` | 见 9.3，历史提交可 `git commit --amend --reset-author` |

---

## 11. 命令速查

```bash
git status                       # 当前状态
git diff                         # 未暂存的改动
git diff --staged                # 已暂存的改动
git switch -c feature/xxx        # 新建并切换分支
git add <文件>                    # 加入暂存区
git commit -m "类型(范围): 说明"   # 提交
git push -u origin <分支>         # 首次推送分支
git pull --rebase                # 同步并变基
git log --oneline -10            # 最近提交
git log --graph --oneline --all  # 分支图
git show <提交号>                 # 查看某次提交
git blame <文件>                  # 逐行追溯
git stash / git stash pop        # 暂存 / 恢复
git restore <文件>                # 丢弃未提交改动
git restore --staged <文件>       # 撤出暂存区
git revert <提交号>               # 安全回滚已推送的提交
git tag -a v1.2.0 -m "说明"       # 打标签
```

---

## 12. 新成员接入清单

- [ ] 安装 Git、配置 `user.name` / `user.email`
- [ ] 配置 SSH key 或 token，`git clone` 成功
- [ ] 编译通过（`src/gazebo_sim` → 主工作空间）
- [ ] 能独立启动 `simulation.launch.py` + `gazebo_net_nav.launch.py`，RViz 有 TF
- [ ] 能发 `/goal_pose` 并观察到车正常穿门
- [ ] 阅读 [README.md](../README.md) 与 [doc/PROJECT_GUIDE.md](PROJECT_GUIDE.md)
- [ ] 提一个 `docs/` 分支的小 PR 练手，走完整评审流程
