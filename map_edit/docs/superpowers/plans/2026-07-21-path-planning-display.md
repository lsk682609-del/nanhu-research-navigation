# 路径规划显示功能 — 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在路网编辑器中添加工具栏按钮，依次点击两个节点作为起终点，调用 A* 规划路径并高亮显示。

**Architecture:** 在 TopoEditorView 中新增 PlanPath 工具模式和路径可视化状态，在 MainWindow 左侧工具栏添加按钮。利用已有的 `topo_.astar()` 和场景 item 体系完成路径绘制。

**Tech Stack:** C++17, Qt5/Qt6 Widgets, QGraphicsView/QGraphicsScene

## Global Constraints

- C++17，与项目现有 CMakeLists.txt 一致
- 遵循现有代码风格：中文注释、`tr()` 包裹用户可见字符串、驼峰命名
- 不引入新的第三方依赖

---

## 文件结构

| 文件 | 职责 | 操作 |
|---|---|---|
| `include/TopoEditorView.hpp` | 新增 Tool::PlanPath 枚举、路径状态变量、clearPlanPathSelection/runPlanPath 声明 | 修改 |
| `src/TopoEditorView.cpp` | 实现路径规划交互逻辑、A* 调用、场景绘制、清理 | 修改 |
| `src/MainWindow.cpp` | 新增工具栏按钮、updateLeftTool 映射、refreshToolLegend 说明 | 修改 |

---

### Task 1: 在 TopoEditorView.hpp 中新增 PlanPath 枚举与状态声明

**Files:**
- Modify: `include/TopoEditorView.hpp`

**Interfaces:**
- Produces: `TopoEditorView::Tool::PlanPath` 枚举值、`planStartId_`/`planGoalId_` 成员、`planStartMarker_`/`planGoalMarker_`/`planPathLines_` 场景 item 指针、`clearPlanPathSelection()`/`runPlanPath()` 方法声明

- [ ] **Step 1: 在 Tool 枚举中添加 PlanPath**

在 `enum class Tool` 末尾（`RasterMask` 之后）添加 `PlanPath`：

```cpp
    enum class Tool { Select, AddNode, AddEdge, KeepoutPoly, PassablePoly, Measure,
                      RasterPaint, RasterErase, RasterUnscan, RasterMask, PlanPath };
```

- [ ] **Step 2: 在 private 区域添加路径规划状态变量**

找到 `Tool tool_ = Tool::Select;` 所在行（约第 226 行），在其附近添加：

```cpp
    // ---- 路径规划 ----
    int planStartId_ = -1;
    int planGoalId_ = -1;
    QGraphicsEllipseItem *planStartMarker_ = nullptr;
    QGraphicsEllipseItem *planGoalMarker_ = nullptr;
    QVector<QGraphicsLineItem *> planPathLines_;
```

- [ ] **Step 3: 在 public 区域添加方法声明**

在 `void snapNodesToGrid(double step);` 声明附近添加：

```cpp
    void clearPlanPathSelection();
    void runPlanPath();
```

- [ ] **Step 4: 构建验证**

```bash
cd /home/nhy/ww/map_editor_snn/build && cmake --build . 2>&1
```

预期：编译通过（新增枚举值和未实现的声明不会导致链接错误，因为方法尚未被调用）。

---

### Task 2: 在 TopoEditorView.cpp 中实现路径规划逻辑

**Files:**
- Modify: `src/TopoEditorView.cpp`

**Interfaces:**
- Consumes: `Tool::PlanPath`、`planStartId_`、`planGoalId_`、`planStartMarker_`、`planGoalMarker_`、`planPathLines_`（Task 1 声明）
- Produces: `clearPlanPathSelection()`、`runPlanPath()` 实现，`mousePressEvent` PlanPath 分支，`setTool`/`rebuildScene` 中的清理调用

- [ ] **Step 1: 在 setTool 中添加路径规划清理**

在 `setTool()` 方法末尾（`updatePolyDraft();` 之前，约第 317 行）添加：

```cpp
    if (t != Tool::PlanPath)
        clearPlanPathSelection();
```

完整上下文（`setTool` 方法末尾）：

```cpp
    const bool isRaster = (t == Tool::RasterPaint || t == Tool::RasterErase ||
                           t == Tool::RasterUnscan || t == Tool::RasterMask);
    setDragMode((t == Tool::Select && !isRaster) ? QGraphicsView::RubberBandDrag : QGraphicsView::NoDrag);
    if (t != Tool::PlanPath)       // <-- 新增
        clearPlanPathSelection();  // <-- 新增
    updatePolyDraft();
```

- [ ] **Step 2: 在 rebuildScene 开头添加路径可视化指针置空**

在 `rebuildScene()` 开头，`scene()->clear();` 之后，其他指针置空行附近（约第 543-554 行），添加：

```cpp
    planStartMarker_ = nullptr;
    planGoalMarker_ = nullptr;
    planPathLines_.clear();
```

插入位置在 `rasterPreviewItem_ = nullptr;` 之后即可：

```cpp
    rasterPreviewItem_ = nullptr;
    planStartMarker_ = nullptr;      // <-- 新增
    planGoalMarker_ = nullptr;       // <-- 新增
    planPathLines_.clear();          // <-- 新增
    planStartId_ = -1;               // <-- 新增
    planGoalId_ = -1;                // <-- 新增
```

- [ ] **Step 3: 实现 clearPlanPathSelection()**

在文件末尾（`#include` 之后的任何合适位置，建议在 `snapNodesToGrid` 之后）添加：

```cpp
void TopoEditorView::clearPlanPathSelection()
{
    for (QGraphicsLineItem *ln : planPathLines_) {
        if (ln) {
            scene()->removeItem(ln);
            delete ln;
        }
    }
    planPathLines_.clear();

    if (planStartMarker_) {
        scene()->removeItem(planStartMarker_);
        delete planStartMarker_;
        planStartMarker_ = nullptr;
    }
    if (planGoalMarker_) {
        scene()->removeItem(planGoalMarker_);
        delete planGoalMarker_;
        planGoalMarker_ = nullptr;
    }

    planStartId_ = -1;
    planGoalId_ = -1;
}
```

- [ ] **Step 4: 实现 runPlanPath()**

在 `clearPlanPathSelection()` 之后添加：

```cpp
void TopoEditorView::runPlanPath()
{
    if (planStartId_ < 0 || planGoalId_ < 0)
        return;

    if (planStartId_ == planGoalId_) {
        emit operationLogged(tr("路径规划：起点终点相同"));
        return;
    }

    const std::vector<int> path = topo_.astar(planStartId_, planGoalId_);

    if (path.empty()) {
        emit operationLogged(tr("路径规划：起点 %1 与终点 %2 不连通").arg(planStartId_).arg(planGoalId_));
        // 清除终点，保留起点以便用户重新选终点
        if (planGoalMarker_) {
            scene()->removeItem(planGoalMarker_);
            delete planGoalMarker_;
            planGoalMarker_ = nullptr;
        }
        planGoalId_ = -1;
        return;
    }

    // 绘制路径边
    const QPen pathPen(QColor(0xFF, 0x8C, 0x00), 3);  // 暗橙色，线宽 3px
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const int id1 = path[i];
        const int id2 = path[i + 1];
        QPointF p1, p2;
        if (!tryNodeScenePos(id1, &p1) || !tryNodeScenePos(id2, &p2))
            continue;
        auto *ln = new QGraphicsLineItem(QLineF(p1, p2));
        ln->setPen(pathPen);
        ln->setZValue(20);  // 在节点之上
        scene()->addItem(ln);
        planPathLines_.append(ln);
    }

    // 输出路径序列到日志
    QStringList ids;
    for (int id : path)
        ids.append(QString::number(id));
    emit operationLogged(tr("路径规划完成：%1").arg(ids.join(QStringLiteral(" → "))));
}
```

- [ ] **Step 5: 在 mousePressEvent 中添加 PlanPath 分支**

在 `mousePressEvent` 中，**"添加边时右键取消选中第一个节点"** 分支（`if (tool_ == Tool::AddEdge && event->button() == Qt::RightButton ...`）之后、左键处理块（`if (event->button() == Qt::LeftButton) {`）之前插入。约在第 1191 行附近：

```cpp
    // ---- 路径规划 ----
    if (tool_ == Tool::PlanPath) {
        if (event->button() == Qt::LeftButton) {
            QGraphicsItem *hit = nodeItemAt(sp);  // sp 已在函数顶部定义
            const int nid = nodeIdFromItem(hit);
            if (nid < 0) {
                event->accept();
                return;  // 未命中节点，忽略
            }

            // 若已有完整路径，清空重来
            if (!planPathLines_.isEmpty()) {
                clearPlanPathSelection();
            }

            if (planStartId_ < 0) {
                // 设置起点
                planStartId_ = nid;
                QPointF c;
                if (tryNodeScenePos(nid, &c)) {
                    const double r = effectiveNodeRadiusScene() + 4.0;
                    planStartMarker_ = new QGraphicsEllipseItem(c.x() - r, c.y() - r, 2.0 * r, 2.0 * r);
                    planStartMarker_->setBrush(QBrush(QColor(0, 200, 0)));    // 绿色
                    planStartMarker_->setPen(QPen(Qt::black, 2));
                    planStartMarker_->setZValue(30);
                    scene()->addItem(planStartMarker_);
                }
                emit operationLogged(tr("路径规划：已选起点 node=%1").arg(nid));
            } else if (planGoalId_ < 0) {
                // 设置终点
                planGoalId_ = nid;
                QPointF c;
                if (tryNodeScenePos(nid, &c)) {
                    const double r = effectiveNodeRadiusScene() + 4.0;
                    planGoalMarker_ = new QGraphicsEllipseItem(c.x() - r, c.y() - r, 2.0 * r, 2.0 * r);
                    planGoalMarker_->setBrush(QBrush(QColor(220, 30, 30)));   // 红色
                    planGoalMarker_->setPen(QPen(Qt::black, 2));
                    planGoalMarker_->setZValue(30);
                    scene()->addItem(planGoalMarker_);
                }
                emit operationLogged(tr("路径规划：已选终点 node=%1，规划中…").arg(nid));
                runPlanPath();
            }
            event->accept();
            return;
        }
        // PlanPath 模式下右键取消选择
        if (event->button() == Qt::RightButton) {
            clearPlanPathSelection();
            event->accept();
            return;
        }
    }
```

**插入位置参考**：紧接在 `if (tool_ == Tool::AddEdge && event->button() == Qt::RightButton && edgePickA_ >= 0)` 块（约第 1186-1191 行）之后、`if (event->button() == Qt::LeftButton) {`（约第 1193 行）之前。

- [ ] **Step 6: 构建验证**

```bash
cd /home/nhy/ww/map_editor_snn/build && cmake --build . 2>&1
```

预期：编译通过。

---

### Task 3: 在 MainWindow.cpp 中添加工具栏按钮并连线

**Files:**
- Modify: `src/MainWindow.cpp`

**Interfaces:**
- Consumes: `Tool::PlanPath`（Task 1）、`clearPlanPathSelection()`/`runPlanPath()`（Task 2）
- Produces: 左侧工具栏"路径规划"按钮、`updateLeftTool` 映射、`refreshToolLegend` 帮助文本

- [ ] **Step 1: 添加工具 ID 常量**

在文件顶部的常量定义区域（约第 53-59 行），`kToolRasterMask` 之后添加：

```cpp
static constexpr int kToolPlanPath = 10;
```

- [ ] **Step 2: 在构造函数中添加工具栏按钮**

在 `addLeftToolBtn(tr("禁区"), kToolRasterMask);` 之后（约第 237 行），`leftToolLay->addStretch();` 之前，插入一个分隔线和新按钮：

```cpp
    // 分隔
    auto *sep2 = new QFrame;
    sep2->setFrameShape(QFrame::HLine);
    sep2->setStyleSheet(QStringLiteral("QFrame { color: #aaa; }"));
    leftToolLay->addWidget(sep2);
    addLeftToolBtn(tr("路径规划"), kToolPlanPath);
```

位置在 `addLeftToolBtn(tr("禁区"), kToolRasterMask);` 之后，`leftToolLay->addStretch();` 之前。

- [ ] **Step 3: 在 updateLeftTool 中添加 PlanPath 映射**

在 `updateLeftTool` 的 switch 语句中（约第 715-727 行），`case kToolRasterMask:` 之后、`default:` 之前添加：

```cpp
    case kToolPlanPath:  view_->setTool(TopoEditorView::Tool::PlanPath); break;
```

- [ ] **Step 4: 在 refreshToolLegend 中添加说明文本**

在 `refreshToolLegend` 的 switch 中（约第 785-813 行），在 `case kToolMeasure:` 块之后、`default:` 之前添加：

```cpp
    case kToolPlanPath:
        legendLabel_->setText(tr("【路径规划】依次左键点击两个节点：第一个为起点（绿色），第二个为终点（红色）。\n"
                                 "选中后自动运行 A* 算法，路径以橙色高亮显示。\n"
                                 "右键清除选择。\n"
                                 "%1").arg(common));
        return;
```

- [ ] **Step 5: 构建并运行验证**

```bash
cd /home/nhy/ww/map_editor_snn/build && cmake --build . 2>&1
```

预期：编译通过，0 错误。

- [ ] **Step 6: 功能验证**

手动测试以下场景：
1. 打开一个路网文件，点击"路径规划"按钮
2. 左键点击节点 A → 出现绿色起点标记，日志提示"已选起点"
3. 左键点击节点 B → 出现红色终点标记 + 橙色路径线，日志输出路径序列
4. 右键 → 清除所有标记和路径
5. 切换回"选择"工具 → 路径自动清除
6. 选两个不连通的节点 → 日志提示不连通，清除终点但保留起点
7. 选同一个节点两次 → 日志提示"起点终点相同"
