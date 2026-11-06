# 学习中心

> 这里记录「从零复刻南湖路网导航工作区」的全部学习过程。目标不是看懂代码，而是**能自己搭世界、建路网、配导航栈、调通并解释每一步**。
>
> 配套：[分阶段计划 PLAN.md](PLAN.md) ｜ [进度打卡 PROGRESS.md](PROGRESS.md) ｜ [笔记模板 TEMPLATE.md](TEMPLATE.md) ｜ [术语速查 GLOSSARY.md](GLOSSARY.md)

---

## 一、怎么用这套材料（每次学习照做）

```text
① 打开 PROGRESS.md，看「下一步」写的是什么
② 打开 PLAN.md 对应阶段，按「要学什么」逐条勾选
③ 跑「动手任务」，把命令和输出记下来
④ 复制 TEMPLATE.md 到 notes/stage-XX-主题.md，填完
⑤ 回 PROGRESS.md 更新状态、日期、一句话总结、卡点
```

规则很简单：**没有笔记 = 没学**。笔记不追求好看，只要求「三个月后的自己能照着复现」。

---

## 二、目录结构

| 路径 | 作用 |
| --- | --- |
| [PLAN.md](PLAN.md) | 阶段 0–10 + 毕业项目的完整计划（目标、任务、命令、验收、坑、产出） |
| [PROGRESS.md](PROGRESS.md) | 总进度看板、学习日志、卡点台账、复习记录 |
| [TEMPLATE.md](TEMPLATE.md) | 单次学习笔记模板（复制使用） |
| [GLOSSARY.md](GLOSSARY.md) | 术语速查表，按模块分组，每条含「一句话解释 + 对应文件」 |
| [notes/](notes/) | 每个阶段的正式笔记、截图（`notes/assets/`） |

---

## 三、总进度看板

> 状态：`未开始` / `进行中` / `已验收`。笔记链接在写完对应文件后补上。

| 阶段 | 主题 | 状态 | 笔记 | 验收通过 |
| --- | --- | --- | --- | --- |
| 0 | 环境与调试基本功 | 已验收 | [stage-00](notes/stage-00-environment.md) | ☑ |
| 1 | Gazebo：会造世界 | 已验收 | [stage-01](notes/stage-01-gazebo-world.md) | ☑ |
| 2 | RViz 可视化 | 已验收 | [stage-02](notes/stage-02-rviz.md) | ☑ |
| 3 | 定位与 TF | 已验收 | [stage-03](notes/stage-03-tf.md) | ☑ |
| 4 | 拓扑路网与全局规划 | 已验收 | [stage-04](notes/stage-04-topo-plan.md) | ☑ |
| 5 | 代价地图与可通行性 | 进行中 | [stage-05](notes/stage-05-costmap.md) | ◐ |
| 6 | 控制器（门区/自救/停车保护） | 进行中 | [stage-06](notes/stage-06-controller.md) | ◐ |
| 7 | 状态机与对外接口 | 进行中 | [stage-07](notes/stage-07-state-machine.md) | ◐ |
| 8 | 参数化与多场景 | 已验收 | [stage-08](notes/stage-08-params.md) | ☑ |
| 9 | 工程化与协作 | 进行中 | [stage-09](notes/stage-09-engineering.md) | ◐ |
| 10 | 视觉与相机（选修） | 未开始 | [stage-10](notes/stage-10-vision.md) | ☐ |
| C | Capstone 复刻新场景 | 未开始 | [capstone](notes/capstone-report.md) | ☐ |

---

## 四、隔一段时间怎么快速捡回来（3 个入口）

### 入口 1：5 分钟唤醒（每次学习前）

1. 打开 [PROGRESS.md](PROGRESS.md) 最后一行，看「下次从哪继续」；
2. 打开 [GLOSSARY.md](GLOSSARY.md)，只看当前阶段相关的词条；
3. 打开上次笔记，只看末尾「复习要点（3 条）」。

### 入口 2：20 分钟复盘（每 3 个阶段一次）

跑一遍该阶段 PLAN 里的「验收（自测题）」，**先不看答案自己回答**，再对照笔记。答不上来的条目回去重做对应「动手任务」。

### 入口 3：30 分钟整体回顾（准备汇报/面试）

只看 PROGRESS 的「一句话总结」列，按阶段顺序复述一遍，然后看 Capstone 报告的三段式：世界几何 → 路网 → 导航栈与调优。

---

## 五、留痕规则（保证以后能查）

1. **命名**：`notes/stage-XX-<英文短名>.md`，XX 两位数（`00`、`01`…），不要用中文文件名。
2. **截图**：统一放 `notes/assets/`，命名 `stage-XX-<说明>.png`。
3. **每条笔记必须包含**：跑过的命令、真实输出关键行、5 条关键结论、3 条复习要点。
4. **提交**：`docs(learning): 完成阶段X - <主题>`，一次阶段一个提交，方便以后按提交回溯。

---

## 六、毕业标准（同 [PLAN.md](PLAN.md) 第一节）

能独立完成：搭世界、生成路网、写导航栈配置、跑通三个验收场景（正常通过 / 受阻能重规划 / 不可达明确失败），并能写出一份别人可复现的报告。
