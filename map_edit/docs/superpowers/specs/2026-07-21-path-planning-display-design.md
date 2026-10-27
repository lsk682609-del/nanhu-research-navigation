# 路径规划显示功能 — 设计文档

**日期**: 2026-07-21
**状态**: 已批准

## 概述

在路网编辑器中添加路径规划功能：通过工具栏按钮激活，依次点击两个已有路网节点作为起点和终点，自动调用 A* 算法规划路径，并在画布上可视化结果。

## 交互流程

```
点击"路径规划"按钮 → 进入路径规划模式
  → 第1次点击节点 → 标记为起点（绿色圆点），操作日志提示"已选起点 node=XX"
  → 第2次点击节点 → 标记为终点（红色圆点），操作日志提示"已选终点 node=XX，规划中..."
      → 找到路径：高亮经过的边（粗彩色线段），操作日志输出路径节点序列
      → 没找到路径：操作日志提示"起点终点不连通"，清除终点标记，保留起点（用户可重新选终点）
  → 再次点击节点 → 清空旧路径和起终点，重新选起点，循环
```

- 切换到其他工具时，自动清除所有路径可视化
- 删除路径涉及到的节点/边时，自动清除路径可视化

## 涉及的改动

### 1. `include/topo_map.hpp`
- 已有 `astar(int startId, int goalId)` 方法，无需改动

### 2. `include/TopoEditorView.hpp`
- `enum class Tool` 新增枚举值 `PlanPath`
- 新增成员变量：
  - `int planStartId_ = -1` — 当前选中的起点 nodeId，-1 表示未选
  - `int planGoalId_ = -1` — 当前选中的终点 nodeId
  - `QGraphicsEllipseItem *planStartMarker_ = nullptr` — 起点绿色标记
  - `QGraphicsEllipseItem *planGoalMarker_ = nullptr` — 终点红色标记
  - `QVector<QGraphicsLineItem *> planPathLines_` — 路径高亮线段
- 新增方法：
  - `void clearPlanPathSelection()` — 清除路径标记和状态
  - `void runPlanPath()` — 执行 A* 并绘制结果

### 3. `src/TopoEditorView.cpp`
- `mousePressEvent` 中新增 `Tool::PlanPath` 分支：
  - 调用 `nodeItemAt(scenePos)` 判断是否命中节点
  - 未命中：忽略（不报错，方便用户调整视角）
  - 命中节点：
    - 若已有一对完整起终点：调用 `clearPlanPathSelection()` 清空，从新起点开始
    - 设置起点 → 绘制绿色标记
    - 设置终点 → 绘制红色标记，调用 `runPlanPath()`
- `runPlanPath()` 实现：
  - 调用 `topo_.astar(planStartId_, planGoalId_)`
  - 路径为空：`emit operationLogged("起点终点不连通")`，清除终点保留起点
  - 路径非空：遍历路径节点对，在每对相邻节点间创建 `QGraphicsLineItem`（橙色/金色，线宽 3px），存入 `planPathLines_`
  - 同时 `emit operationLogged` 输出路径 id 序列
- `clearPlanPathSelection()` 实现：
  - 删除所有 `planPathLines_` 中的 item 并清空
  - 删除起终点标记 item 并置空指针
  - 重置 `planStartId_ = planGoalId_ = -1`
- 在 `deleteSelection`、`rebuildScene` 中调用 `clearPlanPathSelection()`（撤销/重做后拓扑可能已变，直接清除更安全）
- 工具切换时（`setTool`）自动调用 `clearPlanPathSelection()`

### 4. `src/MainWindow.cpp`
- 在 `leftToolGrp_` 按钮组中新增 "路径规划" radio button
- 在 `updateLeftTool` 的 switch/id 映射中添加对应 case，调用 `view_->setTool(TopoEditorView::Tool::PlanPath)`

## 视觉效果规格

| 元素 | 样式 |
|---|---|
| 起点标记 | 绿色实心圆，半径 6px（场景坐标），叠加在起点节点中心 |
| 终点标记 | 红色实心圆，半径 6px（场景坐标），叠加在终点节点中心 |
| 路径线段 | 颜色 `#FF8C00`（暗橙色），线宽 3px，Z 值在节点/边之上 |
| 起点=终点 | 不画线，日志提示"起点终点相同" |

## 边界情况

- **起点和终点相同**：不运行 A*，日志提示，保留该节点作为起点
- **不连通**：日志提示，清除终点，保留起点，用户可以重新选终点
- **点击非节点区域**：忽略，保持当前选择状态
- **切换工具**：自动清除所有路径可视化
- **删除路径上的节点/边**：在 `deleteSelection`/删除操作后清除路径可视化
- **撤销/重做**：`rebuildScene` 中自动清除路径可视化，用户需重新规划
