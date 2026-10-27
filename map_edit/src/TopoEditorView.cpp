/**
******************************************************************************
*
* @file       TopoEditorView.cpp
* @author     Wei Wang -> shaxikai@outlook.com
* @date       2026.4.10
* @version    V1.0.0
* @brief      路网拓扑画布：场景重建、工具交互与节点/边编辑对话框。
*
*****************************************************************************/

#include "TopoEditorView.hpp"

#include "region_io.hpp"
#include "snn_topo_planner.hpp"

#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFont>
#include <QFormLayout>
#include <QGraphicsEllipseItem>
#include <QGraphicsItem>
#include <QGraphicsTextItem>
#include <QGraphicsLineItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsPathItem>
#include <QFocusEvent>
#include <QKeyEvent>
#include <QQueue>
#include <QLabel>
#include <QMenu>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QFontMetrics>
#include <QPainterPath>
#include <QPen>
#include <QRectF>
#include <QRegularExpression>
#include <QRubberBand>
#include <QCursor>
#include <QTransform>
#include <QWheelEvent>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <QScrollBar>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStackedWidget>

#include <QHash>
#include <QtGlobal>

#include <algorithm>
#include <cmath>

namespace {

int topoTextAdvance(const QFontMetrics &fm, const QString &s)
{
#if QT_VERSION >= QT_VERSION_CHECK(5, 11, 0)
    return fm.horizontalAdvance(s);
#else
    return fm.width(s);
#endif
}

constexpr int kDataKind = 0; // "node" / "edge" / "region"
constexpr int kDataId = 1;

QString kindNode() { return QStringLiteral("node"); }
QString kindEdge() { return QStringLiteral("edge"); }
QString kindRegion() { return QStringLiteral("region"); }

/// 从图元关联的视图变换估计「场景长度 → 屏幕像素」的缩放（与 TopoEditorView::sceneToViewportScale 一致）
static double viewportScaleFromItemScene(const QGraphicsItem *item)
{
    if (!item || !item->scene())
        return 1.0;
    const QList<QGraphicsView *> vs = item->scene()->views();
    if (vs.isEmpty())
        return 1.0;
    const QTransform m = vs.first()->viewportTransform();
    const double ax = std::hypot(m.m11(), m.m12());
    const double ay = std::hypot(m.m21(), m.m22());
    return std::max(std::max(ax, ay), 1e-9);
}

/// 与 TopoEditorView::effectiveNodeRadiusScene 相同：目标屏上直径 kTopoNodeTargetDiamPx，半径夹紧在场景坐标
double kTopoNodeTargetDiamPx = 28.0;  // 运行时可调
constexpr double kTopoNodeRadiusSceneMin = 4.0;
constexpr double kTopoNodeRadiusSceneMax = 800.0;

static double sceneTopoNodeRadiusFromViewScale(double s)
{
    const double r = 0.5 * kTopoNodeTargetDiamPx / std::max(s, 1e-9);
    return std::clamp(r, kTopoNodeRadiusSceneMin, kTopoNodeRadiusSceneMax);
}

/// 标签像素高与节点直径（场景/物品坐标下 2*nr）成固定比，缩放与圆圈一致
constexpr double kTopoIdFontHeightOverNodeDiameter = 0.52;

static QFont topoIdFontForSceneNodeRadius(double radiusScene)
{
    const double diam = 2.0 * std::max(radiusScene, 1e-6);
    const double px = std::clamp(kTopoIdFontHeightOverNodeDiameter * diam, 3.0, 512.0);
    QFont f;
    f.setPixelSize(std::max(1, static_cast<int>(std::lround(px))));
    f.setBold(true);
    return f;
}

// 在同一 paint() 里先画圆/线再画字，避免子项 Text 被压在填充色下面；AlignCenter 对齐圆心与线段中点
class TopoNodeEllipseItem final : public QGraphicsEllipseItem {
public:
    TopoNodeEllipseItem(qreal x, qreal y, qreal w, qreal h, int id)
        : QGraphicsEllipseItem(x, y, w, h)
        , id_(id)
    {
    }

    void setSceneSizedRectAndPen(const QPointF &c, double nr, const QPen &pen)
    {
        prepareGeometryChange();
        setRect(c.x() - nr, c.y() - nr, 2 * nr, 2 * nr);
        setPen(pen);
    }

    QRectF boundingRect() const override
    {
        const QRectF er = QGraphicsEllipseItem::boundingRect();
        const double nr = er.width() * 0.5;
        const QFontMetrics fm(topoIdFontForSceneNodeRadius(nr));
        const QString s = QString::number(id_);
        QRectF tr(0, 0, topoTextAdvance(fm, s), fm.height());
        tr.moveCenter(er.center());
        return er.united(tr);
    }

    void paint(QPainter *p, const QStyleOptionGraphicsItem *opt, QWidget *w) override
    {
        QGraphicsEllipseItem::paint(p, opt, w);
        const QRectF r = rect();
        p->save();
        const QPointF c = r.center();
        p->translate(c);
        p->scale(1, -1);
        p->translate(-c);
        const QFont font = topoIdFontForSceneNodeRadius(r.width() * 0.5);
        p->setFont(font);
        p->setPen(QPen(QColor(28, 28, 36)));
        p->drawText(r, Qt::AlignCenter, QString::number(id_));
        p->restore();
    }

private:
    int id_;
};

class TopoEdgeLineItem final : public QGraphicsLineItem {
public:
    explicit TopoEdgeLineItem(const QLineF &line, int id)
        : QGraphicsLineItem(line)
        , id_(id)
    {
    }

    void notifyIdLabelScaleChanged()
    {
        prepareGeometryChange();
    }

    QRectF boundingRect() const override
    {
        QRectF br = QGraphicsLineItem::boundingRect();
        const double nr = sceneTopoNodeRadiusFromViewScale(viewportScaleFromItemScene(this));
        const QFontMetrics fm(topoIdFontForSceneNodeRadius(nr));
        const QString s = QString::number(id_);
        QRectF tr(0, 0, topoTextAdvance(fm, s), fm.height());
        const QLineF ln = line();
        tr.moveCenter((ln.p1() + ln.p2()) * 0.5);
        return br.united(tr);
    }

    void paint(QPainter *p, const QStyleOptionGraphicsItem *opt, QWidget *w) override
    {
        QGraphicsLineItem::paint(p, opt, w);
        const QLineF ln = line();
        const QPointF mid = (ln.p1() + ln.p2()) * 0.5;
        const QString s = QString::number(id_);
        const double nr = sceneTopoNodeRadiusFromViewScale(viewportScaleFromItemScene(this));
        const QFont font = topoIdFontForSceneNodeRadius(nr);
        const QFontMetrics fm(font);
        QRectF tr(0, 0, topoTextAdvance(fm, s), fm.height());
        tr.moveCenter(mid);
        p->save();
        p->translate(mid);
        p->scale(1, -1);
        p->translate(-mid);
        p->setFont(font);
        p->setPen(QPen(QColor(28, 28, 36)));
        p->drawText(tr, Qt::AlignCenter, s);
        p->restore();
    }

private:
    int id_;
};

} // namespace

TopoEditorView::TopoEditorView(QWidget *parent)
    : QGraphicsView(parent)
    , measureClearTimer_(this)
    , odomSpinTimer_(this)
{
    setScene(new QGraphicsScene(this));
    setRenderHint(QPainter::Antialiasing, true);
    // 选择工具的左/右键交互在 mouse*Event 中分别处理：左键平移，右键框选。
    setDragMode(QGraphicsView::NoDrag);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setMouseTracking(true); // 添加边时无按键移动也要更新虚线预览
    setFocusPolicy(Qt::StrongFocus);
    measureClearTimer_.setSingleShot(true);
    QObject::connect(&measureClearTimer_, &QTimer::timeout, this, [this] { clearMeasureOverlay(); });
    odomSpinTimer_.setInterval(30);
    QObject::connect(&odomSpinTimer_, &QTimer::timeout, this, [this] {
        if (goalNode_ && odomSubscription_ && rclcpp::ok())
            rclcpp::spin_some(goalNode_);
    });
    scale(1, -1); // 世界 Y 向上

    cursorHud_ = new QLabel(viewport());
    cursorHud_->setAttribute(Qt::WA_TransparentForMouseEvents);
    cursorHud_->setStyleSheet(
        QStringLiteral("QLabel { background-color: rgba(32,32,32,210); color: #f5f5f5; padding: 3px 8px; "
                       "border-radius: 4px; font-size: 11px; }"));
    cursorHud_->hide();

    newProject();
}

TopoEditorView::~TopoEditorView()
{
    resetGoalPublishers();
}

void TopoEditorView::setActiveMapId(int mapId)
{
    const bool changed = (mapId_ != mapId);
    mapId_ = mapId;
    rebuildScene();
    if (!changed)
        return;
    if (mapId < 0)
        emit operationLogged(tr("子图显示：全部"));
    else
        emit operationLogged(tr("子图显示：地图 ID %1").arg(mapId));
}

QVector<QPair<int, QString>> TopoEditorView::subMapOptions() const
{
    QVector<QPair<int, QString>> opts;
    opts.reserve(static_cast<int>(topo_.subMaps.size()));
    for (const auto &sm : topo_.subMaps)
        opts.append(qMakePair(sm.id, QString::fromStdString(sm.name)));
    return opts;
}

void TopoEditorView::zoomIn()
{
    constexpr double kFactor = 1.15;
    scale(kFactor, kFactor);
    updateTopoNodeVisualScale();
    updateCursorHud(viewport()->mapFromGlobal(QCursor::pos()));
}

void TopoEditorView::zoomOut()
{
    constexpr double kFactor = 1.15;
    scale(1.0 / kFactor, 1.0 / kFactor);
    updateTopoNodeVisualScale();
    updateCursorHud(viewport()->mapFromGlobal(QCursor::pos()));
}

void TopoEditorView::zoomReset()
{
    resetTransform();
    scale(1.0, -1.0);
    updateTopoNodeVisualScale();
    updateCursorHud(viewport()->mapFromGlobal(QCursor::pos()));
}

void TopoEditorView::setTool(Tool t)
{
    if (nodeDragEditMode_) {
        clearNodeDragEditState();
        rebuildScene();
    }
    clearMeasureOverlay();
    clearEdgePreview();
    // 清除栅格预览
    if (rasterPreviewItem_) {
        scene()->removeItem(rasterPreviewItem_);
        delete rasterPreviewItem_;
        rasterPreviewItem_ = nullptr;
    }
    clearRasterCursorCircle();
    if (rasterDraftItem_) {
        scene()->removeItem(rasterDraftItem_);
        delete rasterDraftItem_;
        rasterDraftItem_ = nullptr;
    }
    rasterPolyDraft_.clear();
    rasterDragging_ = false;
    tool_ = t;
    edgePickA_ = -1;
    polyDraft_.clear();
    if (draftItem_) {
        scene()->removeItem(draftItem_);
        delete draftItem_;
        draftItem_ = nullptr;
    }
    clearSelectMouseState();
    setDragMode(QGraphicsView::NoDrag);
    if (t != Tool::PlanPath)
        clearPlanPathSelection();
    updatePolyDraft();
}

QPointF TopoEditorView::sceneToWorld(const QPointF &scenePt) const
{
    return QPointF(scenePt.x() / kScale, scenePt.y() / kScale);
}

QPointF TopoEditorView::worldToScene(const QPointF &w) const
{
    return QPointF(w.x() * kScale, w.y() * kScale);
}

double TopoEditorView::sceneToViewportScale() const
{
    const QTransform m = viewportTransform();
    const double ax = std::hypot(m.m11(), m.m12());
    const double ay = std::hypot(m.m21(), m.m22());
    return std::max(std::max(ax, ay), 1e-9);
}

double TopoEditorView::effectiveNodeRadiusScene() const
{
    return sceneTopoNodeRadiusFromViewScale(sceneToViewportScale());
}

void TopoEditorView::updateTopoNodeVisualScale()
{
    if (!scene())
        return;
    const double nr = effectiveNodeRadiusScene();
    const double pw = std::clamp(2.2 / sceneToViewportScale(), 0.6, 8.0);
    const QList<QGraphicsItem *> lst = scene()->items();
    for (QGraphicsItem *it : lst) {
        if (it->data(kDataKind).toString() != kindNode())
            continue;
        auto *el = qgraphicsitem_cast<TopoNodeEllipseItem *>(it);
        if (!el)
            continue;
        const QPointF c = el->rect().center();
        el->setSceneSizedRectAndPen(c, nr, QPen(Qt::black, pw));
    }
    for (QGraphicsEllipseItem *ghost : nodeDragEditGhostItems_) {
        if (!ghost)
            continue;
        const QPointF c = ghost->rect().center();
        ghost->setRect(c.x() - nr, c.y() - nr, 2.0 * nr, 2.0 * nr);
    }
    if (odomVehicleItem_) {
        const QPointF c = odomVehicleItem_->rect().center();
        const double vehicleRadius = std::max(nr * 0.65, 5.0);
        odomVehicleItem_->setRect(c.x() - vehicleRadius, c.y() - vehicleRadius,
                                  2.0 * vehicleRadius, 2.0 * vehicleRadius);
    }
    for (QGraphicsItem *it : lst) {
        if (it->data(kDataKind).toString() != kindEdge())
            continue;
        auto *ln = qgraphicsitem_cast<TopoEdgeLineItem *>(it);
        if (!ln)
            continue;
        ln->notifyIdLabelScaleChanged();
        ln->update();
    }
}

int TopoEditorView::nextNodeId() const
{
    int m = -1;
    for (const auto &n : topo_.nodes)
        m = std::max(m, n.id);
    return m + 1;
}

int TopoEditorView::nextEdgeId() const
{
    int m = -1;
    for (const auto &e : topo_.edges)
        m = std::max(m, e.id);
    return m + 1;
}

int TopoEditorView::nextRegionId() const
{
    int m = -1;
    for (const RegionPolygon &r : regions_)
        m = std::max(m, r.id);
    return m + 1;
}

QGraphicsItem *TopoEditorView::nodeItemAt(const QPointF &scenePos) const
{
    const QList<QGraphicsItem *> items = scene()->items(scenePos);
    for (QGraphicsItem *it : items) {
        for (QGraphicsItem *cur = it; cur; cur = cur->parentItem()) {
            if (cur->data(kDataKind).toString() == kindNode())
                return cur;
        }
    }
    return nullptr;
}

QGraphicsItem *TopoEditorView::nodeItemById(int nodeId) const
{
    if (!scene())
        return nullptr;
    const QList<QGraphicsItem *> items = scene()->items();
    for (QGraphicsItem *it : items) {
        if (it->data(kDataKind).toString() == kindNode() && it->data(kDataId).toInt() == nodeId)
            return it;
    }
    return nullptr;
}

int TopoEditorView::nodeIdFromItem(QGraphicsItem *it) const
{
    if (!it || it->data(kDataKind).toString() != kindNode())
        return -1;
    return it->data(kDataId).toInt();
}

QGraphicsItem *TopoEditorView::topoHitItemAt(const QPointF &scenePos) const
{
    const QList<QGraphicsItem *> lst = scene()->items(scenePos);
    for (QGraphicsItem *it : lst) {
        for (QGraphicsItem *cur = it; cur; cur = cur->parentItem()) {
            const QString k = cur->data(kDataKind).toString();
            if (k == kindNode() || k == kindEdge())
                return cur;
        }
    }
    return nullptr;
}

bool TopoEditorView::canUndo() const
{
    return !undoStack_.isEmpty();
}

bool TopoEditorView::canRedo() const
{
    return !redoStack_.isEmpty();
}

void TopoEditorView::recordUndoCheckpoint()
{
    MapSnapshot s;
    s.topo = topo_;
    s.regions = regions_;
    undoStack_.push_back(std::move(s));
    if (undoStack_.size() > kMaxUndo)
        undoStack_.removeFirst();
    redoStack_.clear();
    emit undoRedoAvailabilityChanged();
}

void TopoEditorView::clearUndoHistory()
{
    undoStack_.clear();
    redoStack_.clear();
    emit undoRedoAvailabilityChanged();
}

void TopoEditorView::undo()
{
    if (undoStack_.isEmpty())
        return;
    MapSnapshot cur;
    cur.topo = topo_;
    cur.regions = regions_;
    redoStack_.push_back(std::move(cur));

    MapSnapshot prev = std::move(undoStack_.last());
    undoStack_.pop_back();
    topo_ = std::move(prev.topo);
    regions_ = std::move(prev.regions);

    edgePickA_ = -1;
    polyDraft_.clear();
    if (draftItem_) {
        scene()->removeItem(draftItem_);
        delete draftItem_;
        draftItem_ = nullptr;
    }
    rebuildScene();
    emit undoRedoAvailabilityChanged();
    emit useLlaCoordinatesChanged();
    emit operationLogged(tr("撤销"));
}

void TopoEditorView::redo()
{
    if (redoStack_.isEmpty())
        return;
    MapSnapshot cur;
    cur.topo = topo_;
    cur.regions = regions_;
    undoStack_.push_back(std::move(cur));

    MapSnapshot next = std::move(redoStack_.last());
    redoStack_.pop_back();
    topo_ = std::move(next.topo);
    regions_ = std::move(next.regions);

    edgePickA_ = -1;
    polyDraft_.clear();
    if (draftItem_) {
        scene()->removeItem(draftItem_);
        delete draftItem_;
        draftItem_ = nullptr;
    }
    rebuildScene();
    emit undoRedoAvailabilityChanged();
    emit useLlaCoordinatesChanged();
    emit operationLogged(tr("重做"));
}

void TopoEditorView::newProject()
{
    resetGoalPublishers();
    clearUndoHistory();
    topo_.subMaps.clear();
    topo_.subMaps.push_back({0, std::string("default")});
    topo_.nodes.clear();
    topo_.edges.clear();
    topo_.useLlaCoordinates = false;
    topo_.refLlaValid = false;
    topo_.refLatDeg = topo_.refLonDeg = topo_.refAltM = 0.0;
    regions_.clear();
    path_.clear();
    pgm_.reset();
    yaml_ = RosMapYaml{};
    mask_.clear();
    undoStackRaster_.clear();
    redoStackRaster_.clear();
    activeStroke_.clear();
    touchedPgm_.clear();
    touchedMask_.clear();
    edgePickA_ = -1;
    polyDraft_.clear();
    if (draftItem_) {
        scene()->removeItem(draftItem_);
        delete draftItem_;
        draftItem_ = nullptr;
    }
    mapId_ = -1;
    rebuildScene();
    ensureNavigationInterfaces();
    ensureOdomSubscription();
    if (topo_.useLlaCoordinates)
        ensureGpGoalPublisher();
    emit useLlaCoordinatesChanged();
}

void TopoEditorView::rebuildScene()
{
    // 拖动编辑预览不直接写入 topo_；重建场景时清理预览即可恢复原位置。
    clearNodeDragEditState();
    scene()->clear();
    draftItem_ = nullptr;
    rasterDraftItem_ = nullptr;
    rasterCursorCircle_ = nullptr;
    rasterPixmapItem_ = nullptr;
    maskPixmapItem_ = nullptr;
    edgePreviewLine_ = nullptr;
    rasterPreviewItem_ = nullptr;
    planStartMarker_ = nullptr;
    planGoalMarker_ = nullptr;
    planPathLines_.clear();
    navigationPathLines_.clear();
    odomVehicleItem_ = nullptr;
    planStartId_ = -1;
    planGoalId_ = -1;
    measureClearTimer_.stop();
    measureDragging_ = false;
    measureLine_ = nullptr;
    measureText_ = nullptr;

    // 固定场景范围，避免 clear() 后瞬间「无图元」导致 sceneRect/滚动条重算，视口抖一下
    const double halfScene = 2500.0 * kScale; // ±2500 m（场景坐标）
    scene()->setSceneRect(-halfScene, -halfScene, 2.0 * halfScene, 2.0 * halfScene);

    rebuildRasterItem();

    int ridx = 0;
    for (const RegionPolygon &r : regions_) {
        if (r.points.size() < 3)
            continue;
        if (mapId_ >= 0 && r.mapId != mapId_)
            continue;
        QPolygonF sc;
        for (const QPointF &p : r.points)
            sc.append(worldToScene(p));
        auto *poly = scene()->addPolygon(sc, QPen(r.keepout ? QColor(200, 60, 60) : QColor(60, 180, 80), 2),
                                         QBrush(r.keepout ? QColor(255, 0, 0, 60) : QColor(0, 255, 100, 50)));
        poly->setData(kDataKind, kindRegion());
        poly->setData(kDataId, ridx);
        poly->setZValue(-5);
        ++ridx;
    }

    QHash<int, QPointF> nodeScenePos;
    for (const auto &n : topo_.nodes) {
        if (mapId_ >= 0 && n.mapId != mapId_)
            continue;
        const QPointF c = worldToScene(QPointF(n.x, n.y));
        nodeScenePos.insert(n.id, c);
    }

    for (const auto &e : topo_.edges) {
        if (mapId_ >= 0 && e.mapId != mapId_)
            continue;
        if (!nodeScenePos.contains(e.nodeId1) || !nodeScenePos.contains(e.nodeId2))
            continue;
        const QPointF a = nodeScenePos.value(e.nodeId1);
        const QPointF b = nodeScenePos.value(e.nodeId2);
        auto *ln = new TopoEdgeLineItem(QLineF(a, b), e.id);
        QPen pe(QColor(255, 220, 80), edgeWidthPx_);
        pe.setCosmetic(true);
        ln->setPen(pe);
        ln->setData(kDataKind, kindEdge());
        ln->setData(kDataId, e.id);
        ln->setZValue(0);
        scene()->addItem(ln);
    }

    const double nr = effectiveNodeRadiusScene();
    const double nodePenW = std::clamp(2.2 / sceneToViewportScale(), 0.6, 8.0);
    for (const auto &n : topo_.nodes) {
        if (mapId_ >= 0 && n.mapId != mapId_)
            continue;
        const QPointF c = worldToScene(QPointF(n.x, n.y));
        auto *el = new TopoNodeEllipseItem(c.x() - nr, c.y() - nr, 2 * nr, 2 * nr, n.id);
        el->setPen(QPen(Qt::black, nodePenW));
        el->setBrush(QBrush(QColor(0, 220, 255)));
        el->setFlag(QGraphicsItem::ItemIsSelectable, true);
        el->setData(kDataKind, kindNode());
        el->setData(kDataId, n.id);
        el->setZValue(10);
        scene()->addItem(el);
    }
    if (!navigationPath_.poses.empty())
        updateNavigationPath(navigationPath_);
    if (odomVehicleValid_)
        updateOdomVehicleMarker(odomVehicleX_, odomVehicleY_);
}

void TopoEditorView::updatePolyDraft()
{
    if (tool_ != Tool::KeepoutPoly && tool_ != Tool::PassablePoly) {
        if (draftItem_) {
            scene()->removeItem(draftItem_);
            delete draftItem_;
            draftItem_ = nullptr;
        }
        return;
    }
    QPainterPath path;
    if (polyDraft_.isEmpty()) {
        if (draftItem_) {
            scene()->removeItem(draftItem_);
            delete draftItem_;
            draftItem_ = nullptr;
        }
        return;
    }
    const QPointF first = worldToScene(polyDraft_.first());
    path.moveTo(first);
    for (int i = 1; i < polyDraft_.size(); ++i)
        path.lineTo(worldToScene(polyDraft_[i]));

    if (!draftItem_) {
        draftItem_ = new QGraphicsPathItem;
        draftItem_->setZValue(20);
        scene()->addItem(draftItem_);
    }
    draftItem_->setPath(path);
    const QColor col = polyIsKeepout_ ? QColor(255, 100, 100) : QColor(100, 255, 150);
    draftItem_->setPen(QPen(col, 2, Qt::DashLine));
}

bool TopoEditorView::tryNodeScenePos(int nodeId, QPointF *outScene) const
{
    if (!outScene)
        return false;
    for (const auto &n : topo_.nodes) {
        if (n.id != nodeId)
            continue;
        if (mapId_ >= 0 && n.mapId != mapId_)
            return false;
        *outScene = worldToScene(QPointF(n.x, n.y));
        return true;
    }
    return false;
}

bool TopoEditorView::tryEdgeSceneRect(int edgeId, QRectF *outRect) const
{
    if (!outRect)
        return false;
    const TopoMap::Edge *ep = nullptr;
    for (const auto &e : topo_.edges) {
        if (e.id == edgeId) {
            ep = &e;
            break;
        }
    }
    if (!ep)
        return false;
    if (mapId_ >= 0 && ep->mapId != mapId_)
        return false;
    QPointF a, b;
    if (!tryNodeScenePos(ep->nodeId1, &a) || !tryNodeScenePos(ep->nodeId2, &b))
        return false;
    QRectF r(a, b);
    r = r.normalized();
    constexpr double pad = 100.0;
    r.adjust(-pad, -pad, pad, pad);
    constexpr double minSide = 180.0;
    if (r.width() < minSide) {
        const double cx = r.center().x();
        r.setLeft(cx - minSide * 0.5);
        r.setRight(cx + minSide * 0.5);
    }
    if (r.height() < minSide) {
        const double cy = r.center().y();
        r.setTop(cy - minSide * 0.5);
        r.setBottom(cy + minSide * 0.5);
    }
    *outRect = r;
    return true;
}

bool TopoEditorView::focusViewOnNodeId(int nodeId)
{
    QPointF c;
    if (!tryNodeScenePos(nodeId, &c))
        return false;
    constexpr double half = 140.0;
    fitInView(QRectF(c.x() - half, c.y() - half, 2 * half, 2 * half), Qt::KeepAspectRatio);
    return true;
}

bool TopoEditorView::focusViewOnEdgeId(int edgeId)
{
    QRectF r;
    if (!tryEdgeSceneRect(edgeId, &r))
        return false;
    fitInView(r, Qt::KeepAspectRatio);
    return true;
}

bool TopoEditorView::focusViewOnNodeAndEdge(int nodeId, int edgeId)
{
    QRectF united;
    bool any = false;
    QPointF nc;
    if (tryNodeScenePos(nodeId, &nc)) {
        constexpr double half = 140.0;
        united |= QRectF(nc.x() - half, nc.y() - half, 2 * half, 2 * half);
        any = true;
    }
    QRectF er;
    if (tryEdgeSceneRect(edgeId, &er)) {
        united |= er;
        any = true;
    }
    if (!any)
        return false;
    fitInView(united, Qt::KeepAspectRatio);
    return true;
}

bool TopoEditorView::stepVisibleNodeId(int fromId, int delta, int *outId) const
{
    if (!outId || delta == 0)
        return false;
    QVector<int> ids;
    ids.reserve(topo_.nodes.size());
    for (const auto &n : topo_.nodes) {
        if (mapId_ >= 0 && n.mapId != mapId_)
            continue;
        ids.append(n.id);
    }
    if (ids.isEmpty())
        return false;
    std::sort(ids.begin(), ids.end());
    int idx = -1;
    for (int i = 0; i < ids.size(); ++i) {
        if (ids[i] == fromId) {
            idx = i;
            break;
        }
    }
    if (idx < 0)
        return false;
    const int n = ids.size();
    int nidx = idx + delta;
    nidx %= n;
    if (nidx < 0)
        nidx += n;
    *outId = ids[nidx];
    return true;
}

bool TopoEditorView::stepVisibleEdgeId(int fromId, int delta, int *outId) const
{
    if (!outId || delta == 0)
        return false;
    QVector<int> ids;
    ids.reserve(topo_.edges.size());
    for (const auto &e : topo_.edges) {
        if (mapId_ >= 0 && e.mapId != mapId_)
            continue;
        QRectF unused;
        if (!tryEdgeSceneRect(e.id, &unused))
            continue;
        ids.append(e.id);
    }
    if (ids.isEmpty())
        return false;
    std::sort(ids.begin(), ids.end());
    int idx = -1;
    for (int i = 0; i < ids.size(); ++i) {
        if (ids[i] == fromId) {
            idx = i;
            break;
        }
    }
    if (idx < 0)
        return false;
    const int n = ids.size();
    int nidx = idx + delta;
    nidx %= n;
    if (nidx < 0)
        nidx += n;
    *outId = ids[nidx];
    return true;
}

void TopoEditorView::clearEdgePreview()
{
    if (edgePreviewLine_) {
        scene()->removeItem(edgePreviewLine_);
        delete edgePreviewLine_;
        edgePreviewLine_ = nullptr;
    }
}

void TopoEditorView::updateCursorHud(const QPoint &viewportPos)
{
    if (!cursorHud_ || !viewport()->rect().contains(viewportPos)) {
        if (cursorHud_)
            cursorHud_->hide();
        return;
    }
    const QPointF w = sceneToWorld(mapToScene(viewportPos));
    cursorHud_->setText(tr("%1, %2 m")
                            .arg(w.x(), 0, 'f', 2)
                            .arg(w.y(), 0, 'f', 2));
    cursorHud_->adjustSize();
    const QSize sz = cursorHud_->size();
    const QRect vr = viewport()->rect();
    QPoint p = viewportPos + QPoint(10, 10);
    if (p.x() + sz.width() > vr.right())
        p.setX(viewportPos.x() - sz.width() - 8);
    if (p.y() + sz.height() > vr.bottom())
        p.setY(viewportPos.y() - sz.height() - 8);
    const int maxX = std::max(vr.left(), vr.right() - sz.width());
    const int maxY = std::max(vr.top(), vr.bottom() - sz.height());
    p.setX(std::clamp(p.x(), vr.left(), maxX));
    p.setY(std::clamp(p.y(), vr.top(), maxY));
    cursorHud_->move(p);
    cursorHud_->show();
    cursorHud_->raise();
}

void TopoEditorView::clearSpacePanState()
{
    if (viewport()->mouseGrabber() == viewport())
        viewport()->releaseMouse();
    spaceDown_ = false;
    spacePanDragging_ = false;
    viewport()->unsetCursor();
}

void TopoEditorView::clearSelectMouseState()
{
    if (viewport()->mouseGrabber() == viewport())
        viewport()->releaseMouse();
    selectMousePressed_ = false;
    selectPanDragging_ = false;
    selectRubberBandDragging_ = false;
    selectContextHit_ = nullptr;
    if (selectRubberBand_) {
        selectRubberBand_->hide();
        delete selectRubberBand_;
        selectRubberBand_ = nullptr;
    }
    viewport()->unsetCursor();
}

void TopoEditorView::clearNodeDragEditState()
{
    const bool active = nodeDragEditMode_ || nodeDragEditMousePressed_;
    if (active && viewport()->mouseGrabber() == viewport())
        viewport()->releaseMouse();
    nodeDragEditMode_ = false;
    nodeDragEditMousePressed_ = false;
    nodeDragEditMoved_ = false;
    nodeDragEditId_ = -1;
    nodeDragEditItem_ = nullptr;
    nodeDragEditIds_.clear();
    nodeDragEditOriginalScenePositions_.clear();
    nodeDragEditPreviewScenePositions_.clear();
    for (QGraphicsEllipseItem *ghost : nodeDragEditGhostItems_) {
        if (!ghost)
            continue;
        if (ghost->scene())
            scene()->removeItem(ghost);
        delete ghost;
    }
    nodeDragEditGhostItems_.clear();
    if (nodeDragEditDeltaLabel_) {
        nodeDragEditDeltaLabel_->hide();
        delete nodeDragEditDeltaLabel_;
        nodeDragEditDeltaLabel_ = nullptr;
    }
    if (nodeDragEditCancelButton_) {
        nodeDragEditCancelButton_->hide();
        delete nodeDragEditCancelButton_;
        nodeDragEditCancelButton_ = nullptr;
    }
    if (nodeDragEditConfirmButton_) {
        nodeDragEditConfirmButton_->hide();
        delete nodeDragEditConfirmButton_;
        nodeDragEditConfirmButton_ = nullptr;
    }
    if (active)
        viewport()->unsetCursor();
}

void TopoEditorView::beginNodeDragEdit(int nodeId)
{
    if (nodeDragEditMode_) {
        clearNodeDragEditState();
        rebuildScene();
    }

    QGraphicsItem *anchorItem = nodeItemById(nodeId);
    if (!anchorItem)
        return;

    QVector<int> ids;
    QSet<int> seen;
    if (anchorItem->isSelected()) {
        for (QGraphicsItem *selected : scene()->selectedItems()) {
            if (selected->data(kDataKind).toString() != kindNode())
                continue;
            const int selectedId = selected->data(kDataId).toInt();
            if (seen.contains(selectedId) || !nodeItemById(selectedId))
                continue;
            QPointF selectedPos;
            if (!tryNodeScenePos(selectedId, &selectedPos))
                continue;
            seen.insert(selectedId);
            ids.append(selectedId);
        }
    }
    if (ids.isEmpty())
        ids.append(nodeId);

    nodeDragEditIds_ = ids;
    nodeDragEditMode_ = true;
    nodeDragEditId_ = nodeId;
    nodeDragEditItem_ = anchorItem;
    for (const int id : nodeDragEditIds_) {
        QPointF pos;
        if (!tryNodeScenePos(id, &pos))
            continue;
        nodeDragEditOriginalScenePositions_.insert(id, pos);
        nodeDragEditPreviewScenePositions_.insert(id, pos);
    }
    if (!nodeDragEditOriginalScenePositions_.contains(nodeId)) {
        clearNodeDragEditState();
        return;
    }
    const double ghostRadius = effectiveNodeRadiusScene();
    QPen ghostPen(QColor(35, 105, 235, 210), 2.0, Qt::DashLine);
    ghostPen.setCosmetic(true);
    const QBrush ghostBrush(QColor(45, 125, 245, 45));
    for (const int id : nodeDragEditIds_) {
        if (!nodeDragEditOriginalScenePositions_.contains(id))
            continue;
        const QPointF pos = nodeDragEditOriginalScenePositions_.value(id);
        auto *ghost = new QGraphicsEllipseItem(pos.x() - ghostRadius, pos.y() - ghostRadius,
                                               2.0 * ghostRadius, 2.0 * ghostRadius);
        ghost->setPen(ghostPen);
        ghost->setBrush(ghostBrush);
        ghost->setZValue(8);
        scene()->addItem(ghost);
        nodeDragEditGhostItems_.append(ghost);
    }
    nodeDragEditOriginalScenePos_ = nodeDragEditOriginalScenePositions_.value(nodeId);
    nodeDragEditPreviewScenePos_ = nodeDragEditOriginalScenePos_;
    nodeDragEditMoved_ = false;
    anchorItem->setSelected(true);
    setFocus(Qt::OtherFocusReason);
    emit operationLogged(tr("%1 个节点进入拖动编辑，拖动后点击 ✓ 确认或 × 取消")
                              .arg(nodeDragEditIds_.size()));
}

void TopoEditorView::updateNodeDragVisual(const QPointF &centerScene)
{
    if (!nodeDragEditMode_ || !nodeDragEditItem_ || !nodeDragEditItem_->scene() ||
        !nodeDragEditPreviewScenePositions_.contains(nodeDragEditId_))
        return;

    const QPointF currentAnchor = nodeDragEditPreviewScenePositions_.value(nodeDragEditId_);
    const QPointF delta = centerScene - currentAnchor;
    for (const int id : nodeDragEditIds_) {
        const QPointF next = nodeDragEditPreviewScenePositions_.value(id) + delta;
        nodeDragEditPreviewScenePositions_[id] = next;
        if (QGraphicsItem *item = nodeItemById(id)) {
            if (auto *nodeItem = qgraphicsitem_cast<TopoNodeEllipseItem *>(item))
                nodeItem->setSceneSizedRectAndPen(next, effectiveNodeRadiusScene(), nodeItem->pen());
        }
    }
    nodeDragEditPreviewScenePos_ = nodeDragEditPreviewScenePositions_.value(nodeDragEditId_);

    for (QGraphicsItem *it : scene()->items()) {
        if (it->data(kDataKind).toString() != kindEdge())
            continue;
        auto *edgeItem = qgraphicsitem_cast<TopoEdgeLineItem *>(it);
        if (!edgeItem)
            continue;

        const int edgeId = it->data(kDataId).toInt();
        for (const auto &edge : topo_.edges) {
            if (edge.id != edgeId)
                continue;
            QPointF a;
            QPointF b;
            if (!tryNodeScenePos(edge.nodeId1, &a) || !tryNodeScenePos(edge.nodeId2, &b))
                break;
            if (nodeDragEditPreviewScenePositions_.contains(edge.nodeId1))
                a = nodeDragEditPreviewScenePositions_.value(edge.nodeId1);
            if (nodeDragEditPreviewScenePositions_.contains(edge.nodeId2))
                b = nodeDragEditPreviewScenePositions_.value(edge.nodeId2);
            edgeItem->setLine(QLineF(a, b));
            break;
        }
    }
    updateNodeDragEditOverlay();
}

void TopoEditorView::updateNodeDragEditOverlay()
{
    if (!nodeDragEditMode_ || !nodeDragEditMoved_ || !nodeDragEditItem_ || !nodeDragEditItem_->scene())
        return;

    if (!nodeDragEditDeltaLabel_) {
        nodeDragEditDeltaLabel_ = new QLabel(viewport());
        nodeDragEditDeltaLabel_->setAttribute(Qt::WA_TransparentForMouseEvents);
        nodeDragEditDeltaLabel_->setAlignment(Qt::AlignCenter);
        nodeDragEditDeltaLabel_->setStyleSheet(
            QStringLiteral("QLabel { background: rgba(32, 32, 32, 220); color: #ffffff; padding: 3px 7px; "
                           "border-radius: 4px; font-size: 11px; }"));
    }
    const QPointF deltaWorld = sceneToWorld(nodeDragEditPreviewScenePos_ - nodeDragEditOriginalScenePos_);
    nodeDragEditDeltaLabel_->setText(
        tr("ΔX: %1 m    ΔY: %2 m").arg(deltaWorld.x(), 0, 'f', 2).arg(deltaWorld.y(), 0, 'f', 2));
    nodeDragEditDeltaLabel_->adjustSize();

    if (!nodeDragEditCancelButton_) {
        nodeDragEditCancelButton_ = new QToolButton(viewport());
        nodeDragEditCancelButton_->setText(QStringLiteral("×"));
        nodeDragEditCancelButton_->setFocusPolicy(Qt::NoFocus);
        nodeDragEditCancelButton_->setToolTip(tr("取消拖动编辑"));
        nodeDragEditCancelButton_->setFixedSize(30, 30);
        nodeDragEditCancelButton_->setStyleSheet(
            QStringLiteral("QToolButton { background: #f3f3f3; color: #c62828; border: 1px solid #999; "
                           "border-radius: 4px; font-size: 20px; font-weight: bold; }"
                           "QToolButton:hover { background: #ffe0e0; }"));
        QObject::connect(nodeDragEditCancelButton_, &QToolButton::clicked, this,
                         [this] { finishNodeDragEdit(false); });
    }
    if (!nodeDragEditConfirmButton_) {
        nodeDragEditConfirmButton_ = new QToolButton(viewport());
        nodeDragEditConfirmButton_->setText(QStringLiteral("✓"));
        nodeDragEditConfirmButton_->setFocusPolicy(Qt::NoFocus);
        nodeDragEditConfirmButton_->setToolTip(tr("确认拖动编辑"));
        nodeDragEditConfirmButton_->setFixedSize(30, 30);
        nodeDragEditConfirmButton_->setStyleSheet(
            QStringLiteral("QToolButton { background: #f3f3f3; color: #2e7d32; border: 1px solid #999; "
                           "border-radius: 4px; font-size: 18px; font-weight: bold; }"
                           "QToolButton:hover { background: #e0f2e0; }"));
        QObject::connect(nodeDragEditConfirmButton_, &QToolButton::clicked, this,
                         [this] { finishNodeDragEdit(true); });
    }

    const QPoint center = mapFromScene(nodeDragEditPreviewScenePos_);
    const int gap = 4;
    const int width = nodeDragEditCancelButton_->width();
    const int height = nodeDragEditCancelButton_->height();
    const int totalWidth = 2 * width + gap;
    const QRect viewRect = viewport()->rect();
    int x = center.x() + 12;
    int y = center.y() - height - 12;
    if (x + totalWidth > viewRect.right())
        x = center.x() - totalWidth - 12;
    if (y < viewRect.top())
        y = center.y() + 12;
    x = std::clamp(x, viewRect.left(), std::max(viewRect.left(), viewRect.right() - totalWidth + 1));
    y = std::clamp(y, viewRect.top(), std::max(viewRect.top(), viewRect.bottom() - height + 1));

    nodeDragEditCancelButton_->move(x, y);
    nodeDragEditConfirmButton_->move(x + width + gap, y);
    const int labelWidth = nodeDragEditDeltaLabel_->width();
    const int labelHeight = nodeDragEditDeltaLabel_->height();
    int labelX = center.x() - labelWidth / 2;
    int labelY = y - labelHeight - 4;
    if (labelY < viewRect.top())
        labelY = y + height + 4;
    labelX = std::clamp(labelX, viewRect.left(),
                        std::max(viewRect.left(), viewRect.right() - labelWidth + 1));
    labelY = std::clamp(labelY, viewRect.top(),
                        std::max(viewRect.top(), viewRect.bottom() - labelHeight + 1));
    nodeDragEditDeltaLabel_->move(labelX, labelY);
    nodeDragEditDeltaLabel_->show();
    nodeDragEditCancelButton_->show();
    nodeDragEditConfirmButton_->show();
    nodeDragEditDeltaLabel_->raise();
    nodeDragEditCancelButton_->raise();
    nodeDragEditConfirmButton_->raise();
}

void TopoEditorView::finishNodeDragEdit(bool accept)
{
    if (!nodeDragEditMode_)
        return;

    const int nodeCount = nodeDragEditIds_.size();
    const bool changed = nodeDragEditMoved_;

    if (accept && changed) {
        bool anyChanged = false;
        for (const int id : nodeDragEditIds_) {
            if (!nodeDragEditOriginalScenePositions_.contains(id) ||
                !nodeDragEditPreviewScenePositions_.contains(id))
                continue;
            if (QLineF(nodeDragEditOriginalScenePositions_.value(id),
                       nodeDragEditPreviewScenePositions_.value(id))
                    .length() > 1e-9) {
                anyChanged = true;
                break;
            }
        }
        if (anyChanged) {
            recordUndoCheckpoint();
            for (auto &node : topo_.nodes) {
                if (!nodeDragEditPreviewScenePositions_.contains(node.id))
                    continue;
                const QPointF world = sceneToWorld(nodeDragEditPreviewScenePositions_.value(node.id));
                node.x = static_cast<float>(world.x());
                node.y = static_cast<float>(world.y());
                if (topo_.useLlaCoordinates && topo_.refLlaValid)
                    topo_.applyLlaFromEnu(node);
            }
            for (auto &edge : topo_.edges) {
                if (nodeDragEditPreviewScenePositions_.contains(edge.nodeId1) ||
                    nodeDragEditPreviewScenePositions_.contains(edge.nodeId2))
                    topo_.edgeCost(edge);
            }
            clearNodeDragEditState();
            rebuildScene();
            emit operationLogged(tr("拖动编辑 %1 个节点：坐标已应用到当前工程；请使用“保存路网”写入文件")
                                     .arg(nodeCount));
            return;
        }
    }

    const bool hadPreview = nodeDragEditMoved_;
    clearNodeDragEditState();
    if (hadPreview)
        rebuildScene();
}

void TopoEditorView::updateSpacePanCursor()
{
    if (!spaceDown_) {
        viewport()->unsetCursor();
        return;
    }
    viewport()->setCursor(spacePanDragging_ ? Qt::ClosedHandCursor : Qt::OpenHandCursor);
}

void TopoEditorView::keyPressEvent(QKeyEvent *event)
{
    if (nodeDragEditMode_ && event->key() == Qt::Key_Escape) {
        finishNodeDragEdit(false);
        event->accept();
        return;
    }
    if (nodeDragEditMode_ && !event->isAutoRepeat()) {
        QPointF deltaWorld;
        switch (event->key()) {
        case Qt::Key_Left:  deltaWorld.setX(-0.1); break;
        case Qt::Key_Right: deltaWorld.setX(0.1); break;
        case Qt::Key_Up:    deltaWorld.setY(0.1); break;
        case Qt::Key_Down:  deltaWorld.setY(-0.1); break;
        default: break;
        }
        if (!qFuzzyIsNull(deltaWorld.x()) || !qFuzzyIsNull(deltaWorld.y())) {
            nodeDragEditMoved_ = true;
            updateNodeDragVisual(nodeDragEditPreviewScenePos_ + worldToScene(deltaWorld));
            event->accept();
            return;
        }
    }
    if (event->key() == Qt::Key_Space && !event->isAutoRepeat()) {
        spaceDown_ = true;
        updateSpacePanCursor();
        event->accept();
        return;
    }
    QGraphicsView::keyPressEvent(event);
}

void TopoEditorView::keyReleaseEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Space && !event->isAutoRepeat()) {
        spaceDown_ = false;
        spacePanDragging_ = false;
        if (viewport()->mouseGrabber() == viewport())
            viewport()->releaseMouse();
        viewport()->unsetCursor();
        event->accept();
        return;
    }
    QGraphicsView::keyReleaseEvent(event);
}

void TopoEditorView::focusOutEvent(QFocusEvent *event)
{
    clearSpacePanState();
    clearSelectMouseState();
    if (nodeDragEditMode_) {
        clearNodeDragEditState();
        rebuildScene();
    }
    QGraphicsView::focusOutEvent(event);
}

void TopoEditorView::enterEvent(QEvent *event)
{
    QGraphicsView::enterEvent(event);
    updateCursorHud(viewport()->mapFromGlobal(QCursor::pos()));
    if (spaceDown_)
        updateSpacePanCursor();
}

void TopoEditorView::leaveEvent(QEvent *event)
{
    if (cursorHud_)
        cursorHud_->hide();
    clearRasterCursorCircle();
    QGraphicsView::leaveEvent(event);
}

void TopoEditorView::clearMeasureOverlay()
{
    measureClearTimer_.stop();
    measureDragging_ = false;
    if (measureLine_) {
        if (measureLine_->scene())
            scene()->removeItem(measureLine_);
        delete measureLine_;
        measureLine_ = nullptr;
    }
    if (measureText_) {
        if (measureText_->scene())
            scene()->removeItem(measureText_);
        delete measureText_;
        measureText_ = nullptr;
    }
}

void TopoEditorView::updateMeasureVisual(const QPointF &sceneA, const QPointF &sceneB)
{
    if (!measureLine_) {
        measureLine_ = new QGraphicsLineItem;
        QPen p(QColor(0, 55, 130), 3);
        p.setCosmetic(true);
        measureLine_->setPen(p);
        measureLine_->setZValue(30);
        scene()->addItem(measureLine_);
    }
    measureLine_->setLine(QLineF(sceneA, sceneB));

    const QPointF w0 = sceneToWorld(sceneA);
    const QPointF w1 = sceneToWorld(sceneB);
    const double m = std::hypot(w1.x() - w0.x(), w1.y() - w0.y());

    if (!measureText_) {
        measureText_ = new QGraphicsTextItem;
        measureText_->setDefaultTextColor(QColor(0, 35, 85));
        measureText_->setFlag(QGraphicsItem::ItemIgnoresTransformations);
        QFont f = measureText_->font();
        f.setPointSize(11);
        f.setBold(true);
        measureText_->setFont(f);
        measureText_->setZValue(31);
        scene()->addItem(measureText_);
    }
    measureText_->setPlainText(tr("%1 m").arg(m, 0, 'f', 3));
    measureText_->setPos(sceneB + QPointF(10, 10));
}

void TopoEditorView::updateEdgePreview(const QPointF &cursorScene)
{
    if (edgePickA_ < 0)
        return;
    QPointF origin;
    if (!tryNodeScenePos(edgePickA_, &origin)) {
        clearEdgePreview();
        edgePickA_ = -1;
        return;
    }
    if (!edgePreviewLine_) {
        edgePreviewLine_ = new QGraphicsLineItem;
        QPen p(QColor(255, 220, 80, 220));
        p.setStyle(Qt::DashLine);
        p.setWidthF(4);
        p.setCosmetic(true);
        edgePreviewLine_->setPen(p);
        edgePreviewLine_->setZValue(15);
        scene()->addItem(edgePreviewLine_);
    }
    edgePreviewLine_->setLine(QLineF(origin, cursorScene));
}

void TopoEditorView::finishPolygon()
{
    if ((tool_ != Tool::KeepoutPoly && tool_ != Tool::PassablePoly) || polyDraft_.size() < 3)
        return;
    recordUndoCheckpoint();
    RegionPolygon r;
    r.id = nextRegionId();
    r.mapId = mapId_ >= 0 ? mapId_ : 0;
    r.keepout = polyIsKeepout_;
    r.points = polyDraft_;
    regions_.append(r);
    polyDraft_.clear();
    if (draftItem_) {
        scene()->removeItem(draftItem_);
        delete draftItem_;
        draftItem_ = nullptr;
    }
    rebuildScene();
    emit operationLogged(tr("添加%1，ID %2（%3 个顶点）")
                             .arg(r.keepout ? tr("禁区") : tr("可行区"))
                             .arg(r.id)
                             .arg(r.points.size()));
}

void TopoEditorView::deleteSelection()
{
    if (tool_ != Tool::Select)
        return;
    const QList<QGraphicsItem *> sel = scene()->selectedItems();
    if (sel.isEmpty())
        return;

    recordUndoCheckpoint();

    QSet<int> delNodes;
    QSet<int> delEdges;
    QSet<int> delRegIdx;

    for (QGraphicsItem *it : sel) {
        const QString k = it->data(kDataKind).toString();
        if (k == kindNode())
            delNodes.insert(it->data(kDataId).toInt());
        else if (k == kindEdge())
            delEdges.insert(it->data(kDataId).toInt());
        else if (k == kindRegion())
            delRegIdx.insert(it->data(kDataId).toInt());
    }

    if (!delNodes.isEmpty()) {
        topo_.nodes.erase(std::remove_if(topo_.nodes.begin(), topo_.nodes.end(),
                                         [&](const TopoMap::Node &n) { return delNodes.contains(n.id); }),
                          topo_.nodes.end());
        topo_.edges.erase(std::remove_if(topo_.edges.begin(), topo_.edges.end(),
                                         [&](const TopoMap::Edge &e) {
                                             return delNodes.contains(e.nodeId1) || delNodes.contains(e.nodeId2);
                                         }),
                          topo_.edges.end());
    }
    if (!delEdges.isEmpty()) {
        topo_.edges.erase(std::remove_if(topo_.edges.begin(), topo_.edges.end(),
                                         [&](const TopoMap::Edge &e) { return delEdges.contains(e.id); }),
                          topo_.edges.end());
    }
    if (!delRegIdx.isEmpty()) {
        QVector<RegionPolygon> kept;
        for (int i = 0; i < regions_.size(); ++i) {
            if (!delRegIdx.contains(i))
                kept.append(regions_[i]);
        }
        regions_ = kept;
    }

    rebuildScene();
    emit operationLogged(tr("删除选中：节点 %1 个，边 %2 个，区域 %3 个")
                             .arg(delNodes.size())
                             .arg(delEdges.size())
                             .arg(delRegIdx.size()));
}

void TopoEditorView::mousePressEvent(QMouseEvent *event)
{
    updateCursorHud(event->pos());

    if (spaceDown_ && event->button() == Qt::LeftButton) {
        spacePanDragging_ = true;
        spacePanLastPos_ = event->pos();
        viewport()->grabMouse();
        updateSpacePanCursor();
        event->accept();
        return;
    }

    const QPointF sp = mapToScene(event->pos());

    if (tool_ == Tool::Select && nodeDragEditMode_ && event->button() == Qt::LeftButton) {
        QGraphicsItem *hit = nodeItemAt(sp);
        if (nodeIdFromItem(hit) == nodeDragEditId_) {
            nodeDragEditMousePressed_ = true;
            nodeDragEditPressPos_ = event->pos();
            nodeDragEditPressScenePos_ = sp;
            nodeDragEditPressOffsetScene_ = nodeDragEditPreviewScenePos_ - sp;
            viewport()->grabMouse();
            viewport()->setCursor(Qt::OpenHandCursor);
            event->accept();
            return;
        }
    }

    if (tool_ == Tool::Select &&
        (event->button() == Qt::LeftButton || event->button() == Qt::RightButton)) {
        selectMousePressed_ = true;
        selectPanDragging_ = false;
        selectRubberBandDragging_ = false;
        selectPressPos_ = event->pos();
        selectPanLastPos_ = event->pos();
        selectPressScenePos_ = sp;
        selectContextHit_ = event->button() == Qt::RightButton ? topoHitItemAt(sp) : nullptr;
        viewport()->grabMouse();

        if (event->button() == Qt::LeftButton)
            viewport()->setCursor(Qt::OpenHandCursor);
        event->accept();
        return;
    }

    if (tool_ == Tool::Measure) {
        if (event->button() == Qt::RightButton) {
            clearMeasureOverlay();
            event->accept();
            return;
        }
        if (event->button() == Qt::LeftButton) {
            clearMeasureOverlay();
            measureDragging_ = true;
            measureAnchorScene_ = sp;
            updateMeasureVisual(sp, sp);
            event->accept();
            return;
        }
    }

    if (tool_ == Tool::KeepoutPoly || tool_ == Tool::PassablePoly) {
        if (event->button() == Qt::RightButton) {
            if (!polyDraft_.isEmpty()) {
                polyDraft_.removeLast();
                updatePolyDraft();
            }
            event->accept();
            return;
        }
        if (event->button() == Qt::MiddleButton) {
            finishPolygon();
            event->accept();
            return;
        }
    }

    // ---- 栅格编辑工具 ----
    if (tool_ == Tool::RasterPaint || tool_ == Tool::RasterErase ||
        tool_ == Tool::RasterUnscan || tool_ == Tool::RasterMask) {
        if (!pgm_) {
            event->accept();
            return;
        }
        const QPointF w = sceneToWorld(sp);

        if (event->button() == Qt::LeftButton) {
            switch (rasterShape_) {
            case RasterShape::Point:
                beginRasterStroke();
                applyRasterBrushAtCanvas(sp);
                finishRasterStroke();
                break;
            case RasterShape::Line:
                rasterDragging_ = true;
                rasterLineStart_ = w;
                rasterPreviewA_ = w;
                rasterPreviewB_ = w;
                break;
            case RasterShape::Rect:
                rasterDragging_ = true;
                rasterLineStart_ = w;
                rasterPreviewA_ = w;
                rasterPreviewB_ = w;
                break;
            case RasterShape::Polygon:
                rasterPolyDraft_.append(w);
                break;
            }
            event->accept();
            return;
        }

        if (event->button() == Qt::RightButton) {
            if (rasterShape_ == RasterShape::Polygon) {
                if (!rasterPolyDraft_.isEmpty()) {
                    rasterPolyDraft_.removeLast();
                    rebuildRasterCaches();
                    rebuildRasterItem();
                }
            } else {
                if (rasterPreviewItem_) {
                    scene()->removeItem(rasterPreviewItem_);
                    delete rasterPreviewItem_;
                    rasterPreviewItem_ = nullptr;
                }
            }
            event->accept();
            return;
        }

        if (event->button() == Qt::MiddleButton) {
            if (rasterShape_ == RasterShape::Polygon && rasterPolyDraft_.size() >= 3) {
                commitRasterPolygon();
            }
            event->accept();
            return;
        }
    }

    // 添加边时右键取消选中的第一个节点
    if (tool_ == Tool::AddEdge && event->button() == Qt::RightButton && edgePickA_ >= 0) {
        edgePickA_ = -1;
        clearEdgePreview();
        event->accept();
        return;
    }

    // ---- 路径规划 ----
    if (tool_ == Tool::PlanPath) {
        if (event->button() == Qt::LeftButton) {
            QGraphicsItem *hit = nodeItemAt(sp);
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

    if (event->button() == Qt::LeftButton) {
        if (tool_ == Tool::AddNode) {
            if (topo_.useLlaCoordinates && !topo_.refLlaValid) {
                QMessageBox::information(this, tr("添加节点"),
                                         tr("请先在右侧「地图配置」中选择「经纬度坐标系」并填写原点。"));
                event->accept();
                return;
            }
            const QPointF w = sceneToWorld(sp);
            recordUndoCheckpoint();
            TopoMap::Node n;
            n.id = nextNodeId();
            n.type = TopoMap::NodeType::Waypoint;
            n.mapId = mapId_ >= 0 ? mapId_ : 0;
            n.x = static_cast<float>(w.x());
            n.y = static_cast<float>(w.y());
            n.z = 0.f;
            topo_.nodes.push_back(n);
            rebuildScene();
            emit operationLogged(tr("添加节点，ID %1").arg(n.id));
            event->accept();
            return;
        }
        if (tool_ == Tool::AddEdge) {
            QGraphicsItem *hit = nodeItemAt(sp);
            const int nid = nodeIdFromItem(hit);
            if (nid < 0) {
                event->accept();
                return;
            }
            if (edgePickA_ < 0) {
                edgePickA_ = nid;
                updateEdgePreview(sp);
            } else if (edgePickA_ != nid) {
                recordUndoCheckpoint();
                int emap = mapId_ >= 0 ? mapId_ : 0;
                for (const auto &nn : topo_.nodes) {
                    if (nn.id == edgePickA_) {
                        emap = nn.mapId;
                        break;
                    }
                }
                TopoMap::Edge e;
                e.id = nextEdgeId();
                e.type = TopoMap::EdgeType::Bidirectional;
                e.mapId = emap;
                e.nodeId1 = edgePickA_;
                e.nodeId2 = nid;
                e.obt = 0;
                topo_.edgeCost(e);
                topo_.edges.push_back(e);
                edgePickA_ = -1;
                rebuildScene();
                emit operationLogged(tr("添加边，ID %1（端点 %2 — %3）").arg(e.id).arg(e.nodeId1).arg(e.nodeId2));
            }
            event->accept();
            return;
        }
        if (tool_ == Tool::KeepoutPoly || tool_ == Tool::PassablePoly) {
            polyIsKeepout_ = (tool_ == Tool::KeepoutPoly);
            polyDraft_.append(sceneToWorld(sp));
            updatePolyDraft();
            event->accept();
            return;
        }
    }

    QGraphicsView::mousePressEvent(event);
}

void TopoEditorView::mouseMoveEvent(QMouseEvent *event)
{
    updateCursorHud(event->pos());

    if (spacePanDragging_) {
        const QPoint d = event->pos() - spacePanLastPos_;
        spacePanLastPos_ = event->pos();
        if (QScrollBar *h = horizontalScrollBar())
            h->setValue(h->value() - d.x());
        if (QScrollBar *v = verticalScrollBar())
            v->setValue(v->value() - d.y());
        event->accept();
        return;
    }

    if (tool_ == Tool::Select && nodeDragEditMode_ && nodeDragEditMousePressed_ &&
        (event->buttons() & Qt::LeftButton)) {
        const int dragDistance = (event->pos() - nodeDragEditPressPos_).manhattanLength();
        if (dragDistance >= QApplication::startDragDistance()) {
            nodeDragEditMoved_ = true;
            viewport()->setCursor(Qt::ClosedHandCursor);
            updateNodeDragVisual(mapToScene(event->pos()) + nodeDragEditPressOffsetScene_);
        }
        event->accept();
        return;
    }

    if (tool_ == Tool::Select && selectMousePressed_) {
        const int dragDistance = (event->pos() - selectPressPos_).manhattanLength();
        if (event->buttons() & Qt::LeftButton) {
            if (!selectPanDragging_ && dragDistance >= QApplication::startDragDistance()) {
                selectPanDragging_ = true;
                selectPanLastPos_ = event->pos();
                viewport()->setCursor(Qt::ClosedHandCursor);
            }
            if (selectPanDragging_) {
                const QPoint d = event->pos() - selectPanLastPos_;
                selectPanLastPos_ = event->pos();
                if (QScrollBar *h = horizontalScrollBar())
                    h->setValue(h->value() - d.x());
                if (QScrollBar *v = verticalScrollBar())
                    v->setValue(v->value() - d.y());
            }
            event->accept();
            return;
        }

        if (event->buttons() & Qt::RightButton) {
            if (!selectRubberBandDragging_ && dragDistance >= QApplication::startDragDistance()) {
                selectRubberBandDragging_ = true;
                if (!selectRubberBand_)
                    selectRubberBand_ = new QRubberBand(QRubberBand::Rectangle, viewport());
                selectRubberBand_->setGeometry(QRect(selectPressPos_, event->pos()).normalized());
                selectRubberBand_->show();
            } else if (selectRubberBandDragging_) {
                selectRubberBand_->setGeometry(QRect(selectPressPos_, event->pos()).normalized());
            }
            event->accept();
            return;
        }
    }

    if (tool_ == Tool::Measure && measureDragging_) {
        updateMeasureVisual(measureAnchorScene_, mapToScene(event->pos()));
        event->accept();
        return;
    }

    if (tool_ == Tool::AddEdge && edgePickA_ >= 0)
        updateEdgePreview(mapToScene(event->pos()));

    // ---- 栅格编辑鼠标移动 ----
    if (tool_ == Tool::RasterPaint || tool_ == Tool::RasterErase ||
        tool_ == Tool::RasterUnscan || tool_ == Tool::RasterMask) {
        const QPointF sp = mapToScene(event->pos());
        const QPointF w = sceneToWorld(sp);
        updateRasterCursorCircle(w);

        if (rasterDragging_ && rasterShape_ == RasterShape::Line) {
            rasterPreviewB_ = w;
            const QPointF sa = worldToScene(rasterPreviewA_);
            const QPointF sb = worldToScene(rasterPreviewB_);
            QPainterPath pp;
            pp.moveTo(sa);
            pp.lineTo(sb);
            if (!rasterPreviewItem_) {
                rasterPreviewItem_ = new QGraphicsPathItem;
                QPen p(QColor(255, 180, 0), 2);
                p.setCosmetic(true);
                p.setStyle(Qt::DashLine);
                rasterPreviewItem_->setPen(p);
                rasterPreviewItem_->setZValue(35);
                scene()->addItem(rasterPreviewItem_);
            }
            rasterPreviewItem_->setPath(pp);
        }

        if (rasterDragging_ && rasterShape_ == RasterShape::Rect) {
            rasterPreviewB_ = w;
            const QPointF a = worldToScene(rasterPreviewA_);
            const QPointF b = worldToScene(w);
            const QRectF r(a, b);
            QPainterPath pp;
            pp.addRect(r.normalized());
            if (!rasterPreviewItem_) {
                rasterPreviewItem_ = new QGraphicsPathItem;
                QPen p(QColor(255, 180, 0), 2);
                p.setCosmetic(true);
                p.setStyle(Qt::DashLine);
                rasterPreviewItem_->setPen(p);
                rasterPreviewItem_->setZValue(35);
                scene()->addItem(rasterPreviewItem_);
            }
            rasterPreviewItem_->setPath(pp);
        }
    }

    // 非栅格工具时清除圆圈
    if (!(tool_ == Tool::RasterPaint || tool_ == Tool::RasterErase ||
          tool_ == Tool::RasterUnscan || tool_ == Tool::RasterMask)) {
        clearRasterCursorCircle();
    }

    // 多边形草稿预览（已有顶点 + 橡皮筋到光标 + 闭合边预览）
    if ((tool_ == Tool::RasterPaint || tool_ == Tool::RasterErase ||
         tool_ == Tool::RasterUnscan || tool_ == Tool::RasterMask) &&
        rasterShape_ == RasterShape::Polygon &&
        !rasterPolyDraft_.isEmpty()) {
        const QPointF sp = mapToScene(event->pos());
        QPainterPath pp;
        const QPointF firstSc = worldToScene(rasterPolyDraft_.first());
        pp.moveTo(firstSc);
        for (int i = 1; i < rasterPolyDraft_.size(); ++i)
            pp.lineTo(worldToScene(rasterPolyDraft_[i]));
        // 橡皮筋：最后一个顶点 → 当前鼠标位置
        pp.lineTo(sp);
        // 闭合边预览（≥2个顶点时显示）
        if (rasterPolyDraft_.size() >= 2)
            pp.lineTo(firstSc);
        if (!rasterDraftItem_) {
            rasterDraftItem_ = new QGraphicsPathItem;
            rasterDraftItem_->setPen(QPen(QColor(255, 180, 0), 2, Qt::DashLine));
            rasterDraftItem_->setZValue(36);
            scene()->addItem(rasterDraftItem_);
        }
        rasterDraftItem_->setPath(pp);
    } else if (rasterDraftItem_) {
        scene()->removeItem(rasterDraftItem_);
        delete rasterDraftItem_;
        rasterDraftItem_ = nullptr;
    }

    if ((tool_ == Tool::KeepoutPoly || tool_ == Tool::PassablePoly) && !polyDraft_.isEmpty() && draftItem_) {
        QPainterPath path;
        const QPointF first = worldToScene(polyDraft_.first());
        path.moveTo(first);
        for (int i = 1; i < polyDraft_.size(); ++i)
            path.lineTo(worldToScene(polyDraft_[i]));
        path.lineTo(mapToScene(event->pos()));
        draftItem_->setPath(path);
    }
    QGraphicsView::mouseMoveEvent(event);
}

void TopoEditorView::mouseReleaseEvent(QMouseEvent *event)
{
    updateCursorHud(event->pos());
    if (spacePanDragging_ && event->button() == Qt::LeftButton) {
        spacePanDragging_ = false;
        if (viewport()->mouseGrabber() == viewport())
            viewport()->releaseMouse();
        updateSpacePanCursor();
        event->accept();
        return;
    }

    if (tool_ == Tool::Select && nodeDragEditMode_ && nodeDragEditMousePressed_ &&
        event->button() == Qt::LeftButton) {
        nodeDragEditMousePressed_ = false;
        if (viewport()->mouseGrabber() == viewport())
            viewport()->releaseMouse();
        if (nodeDragEditMoved_) {
            updateNodeDragEditOverlay();
            viewport()->unsetCursor();
        } else {
            viewport()->unsetCursor();
        }
        event->accept();
        return;
    }

    if (tool_ == Tool::Select && selectMousePressed_ &&
        (event->button() == Qt::LeftButton || event->button() == Qt::RightButton)) {
        const bool wasPanDragging = selectPanDragging_;
        const bool wasRubberBandDragging = selectRubberBandDragging_;
        const QPoint pressPos = selectPressPos_;
        const QPointF pressScenePos = selectPressScenePos_;
        QGraphicsItem *contextHit = selectContextHit_;
        const Qt::KeyboardModifiers modifiers = event->modifiers();

        clearSelectMouseState();

        if (event->button() == Qt::LeftButton) {
            if (!wasPanDragging) {
                QGraphicsItem *hit = topoHitItemAt(pressScenePos);
                const bool toggle = modifiers.testFlag(Qt::ControlModifier);
                if (!toggle)
                    scene()->clearSelection();
                if (hit && (hit->flags() & QGraphicsItem::ItemIsSelectable))
                    hit->setSelected(toggle ? !hit->isSelected() : true);
            }
            event->accept();
            return;
        }

        if (wasRubberBandDragging) {
            const QRect bandRect(pressPos, event->pos());
            const QPointF sceneA = mapToScene(bandRect.topLeft());
            const QPointF sceneB = mapToScene(bandRect.bottomRight());
            QPainterPath selectionPath;
            selectionPath.addRect(QRectF(sceneA, sceneB).normalized());

            const bool toggle = modifiers.testFlag(Qt::ControlModifier);
            if (!toggle)
                scene()->clearSelection();
            const QList<QGraphicsItem *> hits =
                scene()->items(selectionPath, Qt::IntersectsItemShape, Qt::DescendingOrder, viewportTransform());
            for (QGraphicsItem *it : hits) {
                if (!it || !(it->flags() & QGraphicsItem::ItemIsSelectable))
                    continue;
                const QString kind = it->data(kDataKind).toString();
                if (kind != kindNode() && kind != kindEdge())
                    continue;
                it->setSelected(toggle ? !it->isSelected() : true);
            }
        } else if (contextHit) {
            showSelectContextMenu(pressPos, contextHit);
        }
        event->accept();
        return;
    }

    if (tool_ == Tool::Measure && event->button() == Qt::LeftButton && measureDragging_) {
        measureDragging_ = false;
        updateMeasureVisual(measureAnchorScene_, mapToScene(event->pos()));
        measureClearTimer_.start(5000);
        event->accept();
        return;
    }

    // ---- 栅格编辑释放 ----
    if (tool_ == Tool::RasterPaint || tool_ == Tool::RasterErase ||
        tool_ == Tool::RasterUnscan || tool_ == Tool::RasterMask) {
        if (event->button() == Qt::LeftButton) {
            if (rasterShape_ == RasterShape::Line || rasterShape_ == RasterShape::Rect) {
                rasterDragging_ = false;
                const QPointF w = sceneToWorld(mapToScene(event->pos()));
                if (rasterPreviewItem_) {
                    scene()->removeItem(rasterPreviewItem_);
                    delete rasterPreviewItem_;
                    rasterPreviewItem_ = nullptr;
                }
                beginRasterStroke();
                if (rasterShape_ == RasterShape::Line)
                    commitRasterLine(rasterPreviewA_, w);
                else
                    commitRasterRect(rasterPreviewA_, w);
                finishRasterStroke();
            }
            event->accept();
            return;
        }
    }

    QGraphicsView::mouseReleaseEvent(event);
}

void TopoEditorView::wheelEvent(QWheelEvent *event)
{
    constexpr double kFactor = 1.15;
    if (event->angleDelta().y() > 0)
        scale(kFactor, kFactor);
    else
        scale(1.0 / kFactor, 1.0 / kFactor);
    updateTopoNodeVisualScale();
    event->accept();
    updateCursorHud(viewport()->mapFromGlobal(QCursor::pos()));
}

bool TopoEditorView::loadProject(const QString &path, QString *err)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        if (err)
            *err = tr("无法打开文件");
        return false;
    }
    const QString full = QString::fromUtf8(f.readAll());
    QString topoBody;
    QVector<RegionPolygon> regs;
    QString e2;
    if (!splitTopoProjectText(full, &topoBody, &regs, &e2)) {
        if (err)
            *err = e2;
        return false;
    }
    if (!topo_.loadTopoMapFromString(topoBody.toStdString())) {
        if (err)
            *err = tr("路网拓扑解析失败");
        return false;
    }
    regions_ = regs;
    path_ = path;
    edgePickA_ = -1;
    polyDraft_.clear();
    clearUndoHistory();
    rebuildScene();
    resetGoalPublishers();
    ensureNavigationInterfaces();
    ensureOdomSubscription();
    if (topo_.useLlaCoordinates)
        ensureGpGoalPublisher();
    emit useLlaCoordinatesChanged();
    emit operationLogged(tr("打开工程：%1").arg(path));
    return true;
}

void TopoEditorView::setUseLlaCoordinates(bool on)
{
    if (topo_.useLlaCoordinates == on)
        return;
    resetGoalPublishers();
    topo_.useLlaCoordinates = on;
    ensureNavigationInterfaces();
    ensureOdomSubscription();
    if (on)
        ensureGpGoalPublisher();
    emit useLlaCoordinatesChanged();
}

bool TopoEditorView::saveProject(const QString &path, QString *err)
{
    const std::string topoTxt = topo_.serializeTopoText();
    QString body = QString::fromStdString(topoTxt);
    body += serializeRegionSections(regions_);
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        if (err)
            *err = tr("无法写入文件");
        return false;
    }
    f.write(body.toUtf8());
    path_ = path;
    emit operationLogged(tr("保存工程：%1").arg(path));
    return true;
}

bool TopoEditorView::setGoalTopicNamespace(const QString &namespaceText, QString *error)
{
    QString normalized = namespaceText.trimmed();
    while (normalized.startsWith(QLatin1Char('/')))
        normalized.remove(0, 1);
    while (normalized.endsWith(QLatin1Char('/')))
        normalized.chop(1);

    if (!normalized.isEmpty()) {
        static const QRegularExpression tokenRe(QStringLiteral("^[A-Za-z_][A-Za-z0-9_]*$"));
        const QStringList tokens = normalized.split(QLatin1Char('/'), Qt::KeepEmptyParts);
        for (const QString &token : tokens) {
            if (!tokenRe.match(token).hasMatch()) {
                if (error) {
                    *error = tr("命名空间格式无效。请使用字母、数字和下划线，并以字母或下划线开头；\n"
                                "可以使用多级命名空间，例如 car1 或 car1/navigation。\n"
                                "当前输入：%1").arg(namespaceText);
                }
                return false;
            }
        }
    }

    const bool changed = (goalTopicNamespace_ != normalized);
    if (changed) {
        navigationRequestClient_.reset();
        navigationStatusSubscription_.reset();
        navigationPathSubscription_.reset();
        odomSubscription_.reset();
    }
    goalTopicNamespace_ = normalized;

    ensureNavigationInterfaces();
    ensureOdomSubscription();
    return true;
}

QString TopoEditorView::goalPoseTopic() const
{
    if (goalTopicNamespace_.isEmpty())
        return QStringLiteral("/goal_pose");
    return QStringLiteral("/") + goalTopicNamespace_ + QStringLiteral("/goal_pose");
}

QString TopoEditorView::odomTopic() const
{
    if (goalTopicNamespace_.isEmpty())
        return QStringLiteral("/odom_local");
    return QStringLiteral("/") + goalTopicNamespace_ + QStringLiteral("/odom_local");
}

QString TopoEditorView::navigationRequestService() const
{
    return goalTopicNamespace_.isEmpty() ? QStringLiteral("/navigation_request")
                                         : QStringLiteral("/") + goalTopicNamespace_ + QStringLiteral("/navigation_request");
}

QString TopoEditorView::navigationStatusTopic() const
{
    return goalTopicNamespace_.isEmpty() ? QStringLiteral("/navigation_status")
                                         : QStringLiteral("/") + goalTopicNamespace_ + QStringLiteral("/navigation_status");
}

QString TopoEditorView::navigationGlobalPathTopic() const
{
    return goalTopicNamespace_.isEmpty() ? QStringLiteral("/navigation_global_path")
                                         : QStringLiteral("/") + goalTopicNamespace_ + QStringLiteral("/navigation_global_path");
}

void TopoEditorView::ensureGoalNode()
{
    if (goalNode_ || !rclcpp::ok())
        return;
    goalNode_ = std::make_shared<rclcpp::Node>("map_edit_goal_publisher");
}

void TopoEditorView::ensureNavigationInterfaces()
{
    ensureGoalNode();
    if (!goalNode_)
        return;
    if (!navigationRequestClient_) {
        navigationRequestClient_ = goalNode_->create_client<nav2_msgs::srv::NavigationRequest>(
            navigationRequestService().toStdString());
    }
    if (!navigationStatusSubscription_) {
        navigationStatusSubscription_ = goalNode_->create_subscription<nav2_msgs::msg::NavigationStatus>(
            navigationStatusTopic().toStdString(), rclcpp::QoS(1).reliable().transient_local(),
            [this](nav2_msgs::msg::NavigationStatus::SharedPtr msg) {
                if (!msg)
                    return;
                emit navigationStatusChanged(msg->state, msg->distance_remaining,
                                             QString::fromStdString(msg->message));
                if (msg->state == nav2_msgs::msg::NavigationStatus::PLANNING ||
                    msg->state == nav2_msgs::msg::NavigationStatus::CANCELED ||
                    msg->state == nav2_msgs::msg::NavigationStatus::FAILED)
                    clearNavigationPath();
            });
    }
    if (!navigationPathSubscription_) {
        navigationPathSubscription_ = goalNode_->create_subscription<nav_msgs::msg::Path>(
            navigationGlobalPathTopic().toStdString(), rclcpp::QoS(1).reliable().transient_local(),
            [this](nav_msgs::msg::Path::SharedPtr msg) {
                if (msg)
                    updateNavigationPath(*msg);
            });
    }
    odomSpinTimer_.start();
}

void TopoEditorView::ensureGpGoalPublisher()
{
    if (gpGoalPublisher_)
        return;
    ensureGoalNode();
    if (!goalNode_)
        return;
    gpGoalPublisher_ = goalNode_->create_publisher<sensor_msgs::msg::NavSatFix>(
        "/gp_goal", rclcpp::QoS(10));
}

void TopoEditorView::ensureOdomSubscription()
{
    if (odomSubscription_)
        return;
    ensureGoalNode();
    if (!goalNode_)
        return;

    odomSubscription_ = goalNode_->create_subscription<nav_msgs::msg::Odometry>(
        odomTopic().toStdString(), rclcpp::QoS(10),
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
            if (!msg)
                return;
            const auto &p = msg->pose.pose.position;
            if (std::isfinite(p.x) && std::isfinite(p.y))
                updateOdomVehicleMarker(p.x, p.y);
        });
    odomSpinTimer_.start();
}

void TopoEditorView::resetGoalPublishers()
{
    navigationRequestClient_.reset();
    navigationStatusSubscription_.reset();
    navigationPathSubscription_.reset();
    odomSubscription_.reset();
    gpGoalPublisher_.reset();
    goalNode_.reset();
    odomSpinTimer_.stop();
}

void TopoEditorView::updateOdomVehicleMarker(double x, double y)
{
    odomVehicleValid_ = true;
    odomVehicleX_ = x;
    odomVehicleY_ = y;

    if (!scene())
        return;
    const QPointF center = worldToScene(QPointF(x, y));
    if (!odomVehicleItem_) {
        odomVehicleItem_ = new QGraphicsEllipseItem;
        QPen pen(QColor(0, 95, 35), 2.0);
        pen.setCosmetic(true);
        odomVehicleItem_->setPen(pen);
        odomVehicleItem_->setBrush(QBrush(QColor(30, 210, 80, 230)));
        odomVehicleItem_->setZValue(25);
        scene()->addItem(odomVehicleItem_);
    }
    const double radius = std::max(effectiveNodeRadiusScene() * 0.65, 5.0);
    odomVehicleItem_->setRect(center.x() - radius, center.y() - radius, 2.0 * radius, 2.0 * radius);
    odomVehicleItem_->update();
}

void TopoEditorView::clearNavigationPath()
{
    for (QGraphicsLineItem *line : navigationPathLines_) {
        if (line) {
            scene()->removeItem(line);
            delete line;
        }
    }
    navigationPathLines_.clear();
    navigationPath_.poses.clear();
}

void TopoEditorView::updateNavigationPath(const nav_msgs::msg::Path &path)
{
    for (QGraphicsLineItem *line : navigationPathLines_) {
        if (line) {
            scene()->removeItem(line);
            delete line;
        }
    }
    navigationPathLines_.clear();
    navigationPath_ = path;
    if (!scene() || path.poses.size() < 2)
        return;

    QPen pen(QColor(20, 120, 255), 4.0);
    pen.setCosmetic(true);
    for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
        const auto &a = path.poses[i].pose.position;
        const auto &b = path.poses[i + 1].pose.position;
        auto *line = new QGraphicsLineItem(
            QLineF(worldToScene(QPointF(a.x, a.y)), worldToScene(QPointF(b.x, b.y))));
        line->setPen(pen);
        line->setZValue(19);
        scene()->addItem(line);
        navigationPathLines_.append(line);
    }
    emit operationLogged(tr("收到导航全局路径：%1 个最简路径点").arg(path.poses.size()));
}

void TopoEditorView::publishGoalPoseMapFrame(double eastM, double northM, double upM)
{
    ensureNavigationInterfaces();
    if (!navigationRequestClient_ || !goalNode_) {
        QMessageBox::warning(this, tr("导航到点"),
                             tr("无法创建 ROS2 服务客户端，请确认 ROS2 已初始化。"));
        return;
    }
    if (!navigationRequestClient_->service_is_ready()) {
        QMessageBox::warning(this, tr("导航到点"),
                             tr("导航服务 %1 不可用。").arg(navigationRequestService()));
        return;
    }

    auto request = std::make_shared<nav2_msgs::srv::NavigationRequest::Request>();
    request->command_id = QStringLiteral("map_edit_%1")
                              .arg(QDateTime::currentMSecsSinceEpoch()).toStdString();
    request->cmd = "New";
    request->target_pose.header.stamp = goalNode_->get_clock()->now();
    request->target_pose.header.frame_id = "map";
    request->target_pose.pose.position.x = eastM;
    request->target_pose.pose.position.y = northM;
    request->target_pose.pose.position.z = upM;
    request->target_pose.pose.orientation.w = 1.0;
    const QString commandId = QString::fromStdString(request->command_id);
    emit operationLogged(tr("正在调用 %1，下发目标 x=%2 y=%3 z=%4，指令 %5")
                             .arg(navigationRequestService())
                             .arg(QString::number(eastM, 'g', 16))
                             .arg(QString::number(northM, 'g', 16))
                             .arg(QString::number(upM, 'g', 16))
                             .arg(commandId));
    navigationRequestClient_->async_send_request(
        request, [this, commandId](rclcpp::Client<nav2_msgs::srv::NavigationRequest>::SharedFuture future) {
            try {
                const auto response = future.get();
                const QString message = QString::fromStdString(response->message);
                if (response->success) {
                    emit operationLogged(tr("导航指令 %1 下发成功：%2").arg(commandId, message));
                } else {
                    emit operationLogged(tr("导航指令 %1 下发失败：%2").arg(commandId, message));
                    QMessageBox::warning(this, tr("导航下发失败"), message);
                }
            } catch (const std::exception &e) {
                const QString message = QString::fromUtf8(e.what());
                emit operationLogged(tr("导航服务调用异常：%1").arg(message));
                QMessageBox::warning(this, tr("导航下发失败"), message);
            }
        });
}

bool TopoEditorView::tryResolveNodeWgs84Lla(int nodeId, double *latDeg, double *lonDeg, double *altM) const
{
    if (!latDeg || !lonDeg || !altM)
        return false;
    const TopoMap::Node *np = nullptr;
    for (const auto &n : topo_.nodes) {
        if (n.id == nodeId) {
            np = &n;
            break;
        }
    }
    if (!np)
        return false;
    if (std::isfinite(np->latDeg) && std::isfinite(np->lonDeg) && std::isfinite(np->altM)) {
        *latDeg = np->latDeg;
        *lonDeg = np->lonDeg;
        *altM = np->altM;
        return true;
    }
    if (!topo_.refLlaValid)
        return false;
    TopoMap::enuToLla(static_cast<double>(np->x), static_cast<double>(np->y), static_cast<double>(np->z),
                      topo_.refLatDeg, topo_.refLonDeg, topo_.refAltM, latDeg, lonDeg, altM);
    return std::isfinite(*latDeg) && std::isfinite(*lonDeg) && std::isfinite(*altM);
}

void TopoEditorView::publishGpGoalNavSatFix(double latDeg, double lonDeg, double altM)
{
    ensureGpGoalPublisher();
    if (!gpGoalPublisher_ || !goalNode_) {
        QMessageBox::warning(this, tr("导航到点"),
                             tr("无法创建 ROS2 /gp_goal publisher，请确认 ROS2 已初始化。"));
        return;
    }

    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = goalNode_->get_clock()->now();
    msg.header.frame_id = "gps";
    msg.latitude = latDeg;
    msg.longitude = lonDeg;
    msg.altitude = altM;
    gpGoalPublisher_->publish(msg);

    emit operationLogged(tr("已发布 /gp_goal NavSatFix（纬度 %1，经度 %2，高度 %3）")
                             .arg(QString::number(latDeg, 'f', 12))
                             .arg(QString::number(lonDeg, 'f', 12))
                             .arg(QString::number(altM, 'f', 12)));
}

void TopoEditorView::showSelectContextMenu(const QPoint &viewportPos, QGraphicsItem *hit)
{
    if (!hit)
        return;
    const QString k = hit->data(kDataKind).toString();

    QMenu menu(this);
    QAction *actEdit = menu.addAction(tr("编辑…"));
    QAction *actDragEdit = nullptr;
    if (k == kindNode()) {
        int selectedNodeCount = 0;
        if (hit->isSelected()) {
            for (QGraphicsItem *selected : scene()->selectedItems()) {
                if (selected->data(kDataKind).toString() == kindNode())
                    ++selectedNodeCount;
            }
        }
        actDragEdit = menu.addAction(selectedNodeCount > 1
                                         ? tr("拖动编辑（%1 个节点）").arg(selectedNodeCount)
                                         : tr("拖动编辑"));
    }
    QAction *actNav = nullptr;
    if (k == kindNode()) {
        const int nid = hit->data(kDataId).toInt();
        for (const auto &n : topo_.nodes) {
            if (n.id == nid) {
                actNav = menu.addAction(tr("导航到点（ROS2）"));
                break;
            }
        }
    }
    QAction *actDel = menu.addAction(tr("删除"));
    QAction *chosen = menu.exec(viewport()->mapToGlobal(viewportPos));
    if (!chosen)
        return;
    if (chosen == actDel) {
        if (k == kindNode())
            deleteTopoNodeId(hit->data(kDataId).toInt());
        else if (k == kindEdge())
            deleteTopoEdgeId(hit->data(kDataId).toInt());
        return;
    }
    if (actNav && chosen == actNav) {
        const int nid = hit->data(kDataId).toInt();
        for (const auto &n : topo_.nodes) {
            if (n.id != nid)
                continue;

            publishGoalPoseMapFrame(static_cast<double>(n.x), static_cast<double>(n.y),
                                    static_cast<double>(n.z));
            break;
        }
        return;
    }
    if (actDragEdit && chosen == actDragEdit) {
        beginNodeDragEdit(hit->data(kDataId).toInt());
        return;
    }
    if (chosen == actEdit) {
        if (k == kindNode())
            openEditNodeDialog(hit->data(kDataId).toInt());
        else if (k == kindEdge())
            openEditEdgeDialog(hit->data(kDataId).toInt());
    }
}

void TopoEditorView::deleteTopoNodeId(int id)
{
    recordUndoCheckpoint();
    topo_.nodes.erase(std::remove_if(topo_.nodes.begin(), topo_.nodes.end(),
                                     [id](const TopoMap::Node &n) { return n.id == id; }),
                      topo_.nodes.end());
    topo_.edges.erase(std::remove_if(topo_.edges.begin(), topo_.edges.end(),
                                     [id](const TopoMap::Edge &e) { return e.nodeId1 == id || e.nodeId2 == id; }),
                          topo_.edges.end());
    rebuildScene();
    emit operationLogged(tr("删除节点，ID %1").arg(id));
}

void TopoEditorView::deleteTopoEdgeId(int edgeId)
{
    recordUndoCheckpoint();
    topo_.edges.erase(std::remove_if(topo_.edges.begin(), topo_.edges.end(),
                                     [edgeId](const TopoMap::Edge &e) { return e.id == edgeId; }),
                          topo_.edges.end());
    rebuildScene();
    emit operationLogged(tr("删除边，ID %1").arg(edgeId));
}

void TopoEditorView::activateLlaModeWithOrigin(double latDeg, double lonDeg, double altM)
{
    recordUndoCheckpoint();
    resetGoalPublishers();
    topo_.useLlaCoordinates = true;
    topo_.refLlaValid = true;
    topo_.refLatDeg = latDeg;
    topo_.refLonDeg = lonDeg;
    topo_.refAltM = altM;
    for (auto &n : topo_.nodes) {
        if (std::isfinite(n.latDeg) && std::isfinite(n.lonDeg) && std::isfinite(n.altM))
            topo_.applyEnuFromLla(n);
    }
    for (auto &e : topo_.edges)
        topo_.edgeCost(e);
    rebuildScene();
    ensureNavigationInterfaces();
    ensureOdomSubscription();
    ensureGpGoalPublisher();
    emit useLlaCoordinatesChanged();
    emit operationLogged(tr("启用经纬度模式并设置 WGS84 原点"));
}

void TopoEditorView::resumeLlaModeWithStoredRef()
{
    if (!topo_.refLlaValid)
        return;
    recordUndoCheckpoint();
    resetGoalPublishers();
    topo_.useLlaCoordinates = true;
    for (auto &n : topo_.nodes) {
        if (std::isfinite(n.latDeg) && std::isfinite(n.lonDeg) && std::isfinite(n.altM))
            topo_.applyEnuFromLla(n);
    }
    for (auto &e : topo_.edges)
        topo_.edgeCost(e);
    rebuildScene();
    ensureNavigationInterfaces();
    ensureOdomSubscription();
    ensureGpGoalPublisher();
    emit useLlaCoordinatesChanged();
    emit operationLogged(tr("恢复经纬度模式"));
}

void TopoEditorView::commitReferenceAndRecomputeNodes(double latDeg, double lonDeg, double altM)
{
    if (!topo_.useLlaCoordinates)
        return;
    if (topo_.refLlaValid && std::abs(topo_.refLatDeg - latDeg) < 1e-14 && std::abs(topo_.refLonDeg - lonDeg) < 1e-14
        && std::abs(topo_.refAltM - altM) < 1e-9)
        return;
    recordUndoCheckpoint();
    topo_.refLlaValid = true;
    topo_.refLatDeg = latDeg;
    topo_.refLonDeg = lonDeg;
    topo_.refAltM = altM;
    for (auto &n : topo_.nodes) {
        if (std::isfinite(n.latDeg) && std::isfinite(n.lonDeg) && std::isfinite(n.altM))
            topo_.applyEnuFromLla(n);
    }
    for (auto &e : topo_.edges)
        topo_.edgeCost(e);
    rebuildScene();
    emit operationLogged(tr("更新原点并重算节点坐标"));
}

void TopoEditorView::openEditNodeDialog(int nodeId)
{
    TopoMap::Node *np = nullptr;
    for (auto &n : topo_.nodes) {
        if (n.id == nodeId) {
            np = &n;
            break;
        }
    }
    if (!np)
        return;

    QDialog dlg(this);
    dlg.setWindowTitle(tr("编辑节点"));

    auto *idSpin = new QSpinBox(&dlg);
    idSpin->setRange(-999999, 999999);
    idSpin->setValue(np->id);

    auto *typeCombo = new QComboBox(&dlg);
    typeCombo->addItem(tr("路点 (Waypoint)"), static_cast<int>(TopoMap::NodeType::Waypoint));
    typeCombo->addItem(tr("电梯 (Elevator)"), static_cast<int>(TopoMap::NodeType::Elevator));
    typeCombo->addItem(tr("楼梯 (Stair)"), static_cast<int>(TopoMap::NodeType::Stair));
    typeCombo->addItem(tr("入口 (Entrance)"), static_cast<int>(TopoMap::NodeType::Entrance));
    const int tval = static_cast<int>(np->type);
    for (int i = 0; i < typeCombo->count(); ++i) {
        if (typeCombo->itemData(i).toInt() == tval) {
            typeCombo->setCurrentIndex(i);
            break;
        }
    }

    auto *mapSpin = new QSpinBox(&dlg);
    mapSpin->setRange(-9999, 9999);
    mapSpin->setValue(np->mapId);

    QDoubleSpinBox *sx = nullptr;
    QDoubleSpinBox *sy = nullptr;
    QDoubleSpinBox *sz = nullptr;
    QDoubleSpinBox *sLon = nullptr;
    QDoubleSpinBox *sLat = nullptr;
    QDoubleSpinBox *sAlt = nullptr;
    QComboBox *coordModeCombo = nullptr;

    auto *form = new QFormLayout;
    form->addRow(tr("ID:"), idSpin);
    form->addRow(tr("类型:"), typeCombo);
    form->addRow(tr("地图:"), mapSpin);

    if (topo_.useLlaCoordinates) {
        coordModeCombo = new QComboBox(&dlg);
        coordModeCombo->addItem(tr("局部坐标系"));
        coordModeCombo->addItem(tr("经纬度坐标系"));
        coordModeCombo->setCurrentIndex(0);
        form->addRow(tr("坐标系:"), coordModeCombo);

        sx = new QDoubleSpinBox(&dlg);
        sx->setDecimals(6);
        sx->setRange(-1e9, 1e9);
        sx->setValue(np->x);
        sy = new QDoubleSpinBox(&dlg);
        sy->setDecimals(6);
        sy->setRange(-1e9, 1e9);
        sy->setValue(np->y);
        sz = new QDoubleSpinBox(&dlg);
        sz->setDecimals(6);
        sz->setRange(-1e9, 1e9);
        sz->setValue(np->z);

        sLon = new QDoubleSpinBox(&dlg);
        sLon->setRange(-180.0, 180.0);
        sLon->setDecimals(8);
        sLat = new QDoubleSpinBox(&dlg);
        sLat->setRange(-90.0, 90.0);
        sLat->setDecimals(8);
        sAlt = new QDoubleSpinBox(&dlg);
        sAlt->setRange(-1000.0, 10000.0);
        sAlt->setDecimals(3);
        sLon->setValue(std::isfinite(np->lonDeg) ? np->lonDeg : 0.0);
        sLat->setValue(std::isfinite(np->latDeg) ? np->latDeg : 0.0);
        sAlt->setValue(std::isfinite(np->altM) ? np->altM : 0.0);

        auto *pageLocal = new QWidget(&dlg);
        auto *layLocal = new QFormLayout(pageLocal);
        layLocal->setContentsMargins(0, 0, 0, 0);
        layLocal->addRow(tr("X (m)"), sx);
        layLocal->addRow(tr("Y (m)"), sy);
        layLocal->addRow(tr("Z (m)"), sz);

        auto *pageGeo = new QWidget(&dlg);
        auto *layGeo = new QFormLayout(pageGeo);
        layGeo->setContentsMargins(0, 0, 0, 0);
        layGeo->addRow(tr("经度"), sLon);
        layGeo->addRow(tr("纬度"), sLat);
        layGeo->addRow(tr("高度"), sAlt);

        auto *stack = new QStackedWidget(&dlg);
        stack->addWidget(pageLocal);
        stack->addWidget(pageGeo);

        auto syncLocalEnuToLlaSpins = [this, sx, sy, sz, sLon, sLat, sAlt]() {
            if (!topo_.refLlaValid)
                return;
            double la, lo, al;
            TopoMap::enuToLla(sx->value(), sy->value(), sz->value(), topo_.refLatDeg, topo_.refLonDeg, topo_.refAltM,
                             &la, &lo, &al);
            QSignalBlocker bl(*sLon);
            QSignalBlocker bm(*sLat);
            QSignalBlocker bn(*sAlt);
            sLat->setValue(la);
            sLon->setValue(lo);
            sAlt->setValue(al);
        };

        QObject::connect(coordModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), &dlg,
                         [stack, syncLocalEnuToLlaSpins](int idx) {
                             stack->setCurrentIndex(idx);
                             if (idx == 0)
                                 syncLocalEnuToLlaSpins();
                         });

        auto onLocalXyzChanged = [coordModeCombo, syncLocalEnuToLlaSpins]() {
            if (coordModeCombo->currentIndex() == 0)
                syncLocalEnuToLlaSpins();
        };
        QObject::connect(sx, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg, onLocalXyzChanged);
        QObject::connect(sy, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg, onLocalXyzChanged);
        QObject::connect(sz, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg, onLocalXyzChanged);

        if (topo_.refLlaValid)
            syncLocalEnuToLlaSpins();

        form->addRow(stack);
    } else {
        sx = new QDoubleSpinBox(&dlg);
        sx->setDecimals(6);
        sx->setRange(-1e9, 1e9);
        sx->setValue(np->x);
        sy = new QDoubleSpinBox(&dlg);
        sy->setDecimals(6);
        sy->setRange(-1e9, 1e9);
        sy->setValue(np->y);
        sz = new QDoubleSpinBox(&dlg);
        sz->setDecimals(6);
        sz->setRange(-1e9, 1e9);
        sz->setValue(np->z);
        form->addRow(tr("X (m)"), sx);
        form->addRow(tr("Y (m)"), sy);
        form->addRow(tr("Z (m)"), sz);
    }

    auto *bb = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QObject::connect(bb, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    QObject::connect(bb, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto *vl = new QVBoxLayout(&dlg);
    vl->addLayout(form);
    vl->addWidget(bb);

    if (dlg.exec() != QDialog::Accepted)
        return;

    const int newId = idSpin->value();
    const int oldId = np->id;
    if (newId != oldId) {
        for (const auto &o : topo_.nodes) {
            if (o.id == newId && &o != np) {
                QMessageBox::warning(this, tr("编辑节点"), tr("节点 ID %1 已存在。").arg(newId));
                return;
            }
        }
    }

    const int editNodeCoordMode = (topo_.useLlaCoordinates && coordModeCombo) ? coordModeCombo->currentIndex() : 0;
    if (topo_.useLlaCoordinates && editNodeCoordMode == 1 && !topo_.refLlaValid) {
        QMessageBox::warning(this, tr("编辑节点"), tr("请先在右侧「地图配置」中设置 WGS84 原点。"));
        return;
    }

    recordUndoCheckpoint();
    if (newId != oldId) {
        for (auto &e : topo_.edges) {
            if (e.nodeId1 == oldId)
                e.nodeId1 = newId;
            if (e.nodeId2 == oldId)
                e.nodeId2 = newId;
        }
    }
    np->id = newId;
    np->type = static_cast<TopoMap::NodeType>(typeCombo->currentData().toInt());
    np->mapId = mapSpin->value();

    if (topo_.useLlaCoordinates && editNodeCoordMode == 1) {
        np->lonDeg = sLon->value();
        np->latDeg = sLat->value();
        np->altM = sAlt->value();
        topo_.applyEnuFromLla(*np);
    } else {
        np->x = static_cast<float>(sx->value());
        np->y = static_cast<float>(sy->value());
        np->z = static_cast<float>(sz->value());
        if (topo_.useLlaCoordinates && topo_.refLlaValid)
            topo_.applyLlaFromEnu(*np);
    }

    for (auto &e : topo_.edges) {
        if (e.nodeId1 == np->id || e.nodeId2 == np->id)
            topo_.edgeCost(e);
    }
    rebuildScene();
    emit operationLogged(tr("编辑节点，ID %1").arg(np->id));
}

void TopoEditorView::openEditEdgeDialog(int edgeId)
{
    TopoMap::Edge *ep = nullptr;
    for (auto &e : topo_.edges) {
        if (e.id == edgeId) {
            ep = &e;
            break;
        }
    }
    if (!ep)
        return;

    QDialog dlg(this);
    dlg.setWindowTitle(tr("编辑边"));

    auto *idSpin = new QSpinBox(&dlg);
    idSpin->setRange(-999999, 999999);
    idSpin->setValue(ep->id);

    auto *typeCombo = new QComboBox(&dlg);
    typeCombo->addItem(tr("双向"), static_cast<int>(TopoMap::EdgeType::Bidirectional));
    typeCombo->addItem(tr("单向"), static_cast<int>(TopoMap::EdgeType::Unidirectional));
    const int et = static_cast<int>(ep->type);
    for (int i = 0; i < typeCombo->count(); ++i) {
        if (typeCombo->itemData(i).toInt() == et) {
            typeCombo->setCurrentIndex(i);
            break;
        }
    }

    auto *mapSpin = new QSpinBox(&dlg);
    mapSpin->setRange(-9999, 9999);
    mapSpin->setValue(ep->mapId);

    auto *n1 = new QSpinBox(&dlg);
    n1->setRange(-999999, 999999);
    n1->setValue(ep->nodeId1);
    auto *n2 = new QSpinBox(&dlg);
    n2->setRange(-999999, 999999);
    n2->setValue(ep->nodeId2);

    auto *obtSpin = new QSpinBox(&dlg);
    obtSpin->setRange(-999999, 999999);
    obtSpin->setValue(ep->obt);

    auto *form = new QFormLayout;
    form->addRow(tr("ID:"), idSpin);
    form->addRow(tr("类型:"), typeCombo);
    form->addRow(tr("地图:"), mapSpin);
    form->addRow(tr("起点节点 ID:"), n1);
    form->addRow(tr("终点节点 ID:"), n2);
    form->addRow(tr("obt:"), obtSpin);

    auto *bb = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QObject::connect(bb, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    QObject::connect(bb, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto *vl = new QVBoxLayout(&dlg);
    vl->addLayout(form);
    vl->addWidget(bb);

    if (dlg.exec() != QDialog::Accepted)
        return;

    const int newEid = idSpin->value();
    const int oldEid = ep->id;
    if (newEid != oldEid) {
        for (const auto &o : topo_.edges) {
            if (o.id == newEid && &o != ep) {
                QMessageBox::warning(this, tr("编辑边"), tr("边 ID %1 已存在。").arg(newEid));
                return;
            }
        }
    }

    auto nodeExists = [this](int nid) {
        for (const auto &n : topo_.nodes) {
            if (n.id == nid)
                return true;
        }
        return false;
    };
    const int nid1 = n1->value();
    const int nid2 = n2->value();
    if (!nodeExists(nid1) || !nodeExists(nid2)) {
        QMessageBox::warning(this, tr("编辑边"), tr("起点或终点节点 ID 在路网中不存在。"));
        return;
    }
    if (nid1 == nid2) {
        QMessageBox::warning(this, tr("编辑边"), tr("起点与终点不能为同一节点。"));
        return;
    }

    recordUndoCheckpoint();
    ep->id = newEid;
    ep->type = static_cast<TopoMap::EdgeType>(typeCombo->currentData().toInt());
    ep->mapId = mapSpin->value();
    ep->nodeId1 = nid1;
    ep->nodeId2 = nid2;
    ep->obt = obtSpin->value();
    topo_.edgeCost(*ep);
    rebuildScene();
    emit operationLogged(tr("编辑边，ID %1").arg(ep->id));
}

// ========================================================================
//  栅格地图实现
// ========================================================================

bool TopoEditorView::hasRasterMap() const
{
    return pgm_ && pgm_->width > 0 && pgm_->height > 0;
}

bool TopoEditorView::loadRasterMap(const QString &yamlPath, QString *err)
{
    QFile fy(yamlPath);
    if (!fy.open(QIODevice::ReadOnly)) {
        if (err) *err = tr("无法打开栅格 YAML 文件");
        return false;
    }
    const QString yamlText = QString::fromUtf8(fy.readAll());
    fy.close();

    RosMapYaml parsedYaml;
    if (!parseRosMapYaml(yamlText, parsedYaml)) {
        if (err) *err = tr("栅格 YAML 解析失败");
        return false;
    }
    if (parsedYaml.image.isEmpty()) {
        if (err) *err = tr("YAML 中缺少 image 字段");
        return false;
    }

    const QDir yamlDir = QFileInfo(yamlPath).absoluteDir();
    const QString pgmPath = yamlDir.filePath(parsedYaml.image);
    QFile fp(pgmPath);
    if (!fp.open(QIODevice::ReadOnly)) {
        if (err) *err = tr("无法打开 PGM 文件: %1").arg(pgmPath);
        return false;
    }
    const QByteArray raw = fp.readAll();
    fp.close();

    PgmImage parsedPgm;
    QString perr;
    if (!parsePgm(raw, parsedPgm, &perr)) {
        if (err) *err = perr;
        return false;
    }

    yaml_ = parsedYaml;
    pgm_ = std::make_unique<PgmImage>(std::move(parsedPgm));
    mask_ = QByteArray(pgm_->pixelCount(), static_cast<char>(255));
    undoStackRaster_.clear();
    redoStackRaster_.clear();
    activeStroke_.clear();

    rebuildRasterCaches();
    rebuildScene();
    emit rasterStateChanged();
    emit operationLogged(tr("已加载栅格地图: %1 (%2x%3)")
                             .arg(yamlPath)
                             .arg(pgm_->width)
                             .arg(pgm_->height));
    return true;
}

bool TopoEditorView::saveRasterMap(const QString &yamlAbsDir, const QString &pgmFilename, QString *err)
{
    if (!pgm_)
        return false;
    const QByteArray pgmData = encodePgmP5(*pgm_);
    const QString pgmAbs = QDir(yamlAbsDir).filePath(pgmFilename);
    QFile fp(pgmAbs);
    if (!fp.open(QIODevice::WriteOnly)) {
        if (err) *err = tr("无法写入 PGM: %1").arg(pgmAbs);
        return false;
    }
    fp.write(pgmData);
    fp.close();

    const QString yamlText = serializeRosMapYaml(yaml_, pgmFilename);
    const QString yamlAbs = QDir(yamlAbsDir).filePath(QStringLiteral("map.yaml"));
    QFile fy(yamlAbs);
    if (!fy.open(QIODevice::WriteOnly)) {
        if (err) *err = tr("无法写入 YAML: %1").arg(yamlAbs);
        return false;
    }
    fy.write(yamlText.toUtf8());
    fy.close();

    emit operationLogged(tr("已保存栅格地图到: %1").arg(yamlAbsDir));
    return true;
}

bool TopoEditorView::saveMaskMap(const QString &yamlAbsDir, const QString &pgmFilename, QString *err)
{
    if (!pgm_ || mask_.size() != pgm_->pixelCount())
        return false;
    PgmImage maskPgm;
    maskPgm.width = pgm_->width;
    maskPgm.height = pgm_->height;
    maskPgm.maxval = 255;
    maskPgm.pixels.resize(static_cast<size_t>(maskPgm.pixelCount()));
    for (int i = 0; i < maskPgm.pixelCount(); ++i)
        maskPgm.pixels[static_cast<size_t>(i)] = static_cast<quint8>(mask_[i]);

    const QByteArray pgmData = encodePgmP5(maskPgm);
    const QString pgmAbs = QDir(yamlAbsDir).filePath(pgmFilename);
    QFile fp(pgmAbs);
    if (!fp.open(QIODevice::WriteOnly)) {
        if (err) *err = tr("无法写入禁区 PGM: %1").arg(pgmAbs);
        return false;
    }
    fp.write(pgmData);
    fp.close();

    RosMapYaml my = yaml_;
    my.hasNegate = false;
    const QString yt = serializeRosMapYaml(my, pgmFilename);
    const QString yamlAbs = QDir(yamlAbsDir).filePath(QStringLiteral("keepout.yaml"));
    QFile fy(yamlAbs);
    if (!fy.open(QIODevice::WriteOnly)) {
        if (err) *err = tr("无法写入禁区 YAML: %1").arg(yamlAbs);
        return false;
    }
    fy.write(yt.toUtf8());
    fy.close();

    emit operationLogged(tr("已保存禁区地图到: %1").arg(yamlAbsDir));
    return true;
}

void TopoEditorView::setShowRaster(bool on)
{
    showRaster_ = on;
    rebuildRasterItem();
}

void TopoEditorView::setShowMask(bool on)
{
    showMask_ = on;
    rebuildRasterItem();
}

void TopoEditorView::setBrushRadius(int r)
{
    brushRadius_ = std::max(1, std::min(r, 60));
}

void TopoEditorView::setNodeDiameterPx(double d)
{
    nodeDiameterPx_ = std::clamp(d, 8.0, 500.0);
    kTopoNodeTargetDiamPx = nodeDiameterPx_;
    updateTopoNodeVisualScale();
    viewport()->update();
}

double TopoEditorView::nodeDiameterPx() const
{
    return nodeDiameterPx_;
}

void TopoEditorView::setEdgeWidthPx(double w)
{
    edgeWidthPx_ = std::clamp(w, 1.0, 20.0);
    rebuildScene();
}

double TopoEditorView::edgeWidthPx() const
{
    return edgeWidthPx_;
}

bool TopoEditorView::canUndoRaster() const
{
    return !undoStackRaster_.isEmpty();
}

bool TopoEditorView::canRedoRaster() const
{
    return !redoStackRaster_.isEmpty();
}

int TopoEditorView::worldToPixelIdx(const QPointF &worldPt) const
{
    if (!pgm_)
        return -1;
    const double res = yaml_.resolution > 0.0 ? yaml_.resolution : 0.05;
    const double ox = yaml_.origin.size() >= 1 ? yaml_.origin[0] : 0.0;
    const double oy = yaml_.origin.size() >= 2 ? yaml_.origin[1] : 0.0;
    const int w = pgm_->width;
    const int h = pgm_->height;
    // yaw=0 简化
    const double u = worldPt.x() - ox;
    const double v = worldPt.y() - oy;
    const int col = static_cast<int>(std::floor(u / res));
    const int row = h - 1 - static_cast<int>(std::floor(v / res));
    if (col < 0 || col >= w || row < 0 || row >= h)
        return -1;
    return row * w + col;
}

QPointF TopoEditorView::pixelIdxToWorld(int idx) const
{
    if (!pgm_ || idx < 0 || idx >= pgm_->pixelCount())
        return {};
    const int w = pgm_->width;
    const int h = pgm_->height;
    const double res = yaml_.resolution > 0.0 ? yaml_.resolution : 0.05;
    const double ox = yaml_.origin.size() >= 1 ? yaml_.origin[0] : 0.0;
    const double oy = yaml_.origin.size() >= 2 ? yaml_.origin[1] : 0.0;
    const int row = idx / w;
    const int col = idx % w;
    const double u = (col + 0.5) * res;
    const double v = (h - 1 - row + 0.5) * res;
    return QPointF(ox + u, oy + v);
}

void TopoEditorView::snapNodesToGrid(double step)
{
    if (step <= 0.0 || topo_.nodes.empty())
        return;
    recordUndoCheckpoint();
    const double inv = 1.0 / step;
    for (auto &n : topo_.nodes) {
        n.x = std::round(static_cast<double>(n.x) * inv) * step;
        n.y = std::round(static_cast<double>(n.y) * inv) * step;
        if (topo_.useLlaCoordinates && topo_.refLlaValid)
            topo_.applyLlaFromEnu(n);
    }
    for (auto &e : topo_.edges)
        topo_.edgeCost(e);
    rebuildScene();
    emit operationLogged(tr("坐标规整：最小单位 %1 m，已更新 %2 个节点")
                             .arg(step, 0, 'f', 2)
                             .arg(topo_.nodes.size()));
}

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

void TopoEditorView::runPlanPath()
{
    if (planStartId_ < 0 || planGoalId_ < 0)
        return;

    if (planStartId_ == planGoalId_) {
        emit operationLogged(tr("路径规划：起点终点相同"));
        if (planGoalMarker_) {
            scene()->removeItem(planGoalMarker_);
            delete planGoalMarker_;
            planGoalMarker_ = nullptr;
        }
        planGoalId_ = -1;
        return;
    }

    const SnnPlanResult planResult = SnnTopoPlanner{}.plan(topo_, planStartId_, planGoalId_);
    const std::vector<int> &path = planResult.nodeIds;

    if (path.empty()) {
        QString reason;
        switch (planResult.status) {
        case SnnPlanResult::Status::InvalidStart:
            reason = tr("起点无效");
            break;
        case SnnPlanResult::Status::InvalidGoal:
            reason = tr("终点无效");
            break;
        case SnnPlanResult::Status::InvalidDynamics:
            reason = tr("神经元动力学参数不满足首次脉冲规划条件");
            break;
        case SnnPlanResult::Status::Unreachable:
        default:
            reason = tr("起点与终点不连通");
            break;
        }
        emit operationLogged(tr("SNN 显式动力学路径规划失败：%1").arg(reason));
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
    QPen pathPen(QColor(0xFF, 0x00, 0xFF), 3);  // 品红，线宽 3px
    pathPen.setCosmetic(true);
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
    emit operationLogged(tr("SNN 显式动力学规划完成：总代价 %1，起点放电 t=%2，激活节点 %3，突触事件 %4，神经元更新 %5，路径 %6")
                             .arg(planResult.totalDelay, 0, 'f', 3)
                             .arg(planResult.startSpikeTime, 0, 'f', 3)
                             .arg(planResult.firedNeuronCount)
                             .arg(planResult.processedSynapticEventCount)
                             .arg(planResult.dynamicsTrace.size())
                             .arg(ids.join(QStringLiteral(" → "))));
}

void TopoEditorView::translateMapData(double dx, double dy)
{
    recordUndoCheckpoint();

    // 节点平移
    for (auto &n : topo_.nodes) {
        n.x += static_cast<float>(dx);
        n.y += static_cast<float>(dy);
        if (topo_.useLlaCoordinates && topo_.refLlaValid)
            topo_.applyLlaFromEnu(n);
    }

    // 区域多边形平移
    for (auto &r : regions_) {
        for (auto &p : r.points)
            p += QPointF(dx, dy);
    }

    // 栅格地图：平移 YAML origin
    if (pgm_) {
        if (yaml_.origin.size() >= 2) {
            yaml_.origin[0] += dx;
            yaml_.origin[1] += dy;
        }
        // YAML rawText 粗略更新
        yaml_.rawText = serializeRosMapYaml(yaml_, yaml_.image);
    }

    for (auto &e : topo_.edges)
        topo_.edgeCost(e);

    rebuildScene();
    emit operationLogged(tr("平移地图：Δx=%1 m, Δy=%2 m").arg(dx, 0, 'f', 3).arg(dy, 0, 'f', 3));
}

void TopoEditorView::rotateMapData(double cx, double cy, double angleDeg)
{
    recordUndoCheckpoint();

    const double kPi = 3.14159265358979323846;
    const double rad = angleDeg * kPi / 180.0;
    const double c = std::cos(rad);
    const double s = std::sin(rad);

    auto rotatePt = [&](QPointF p) -> QPointF {
        const double dx = p.x() - cx;
        const double dy = p.y() - cy;
        return QPointF(cx + dx * c - dy * s, cy + dx * s + dy * c);
    };

    // 节点旋转
    for (auto &n : topo_.nodes) {
        QPointF np = rotatePt(QPointF(n.x, n.y));
        n.x = static_cast<float>(np.x());
        n.y = static_cast<float>(np.y());
        if (topo_.useLlaCoordinates && topo_.refLlaValid)
            topo_.applyLlaFromEnu(n);
    }

    // 区域多边形旋转
    for (auto &r : regions_) {
        for (auto &p : r.points)
            p = rotatePt(p);
    }

    // 栅格地图旋转
    if (pgm_ && yaml_.resolution > 0.0) {
        rotatePgmAndMask(cx, cy, c, s);
    }

    for (auto &e : topo_.edges)
        topo_.edgeCost(e);

    rebuildScene();
    emit operationLogged(tr("旋转地图：角度=%1°，中心 (%2, %3)").arg(angleDeg, 0, 'f', 2).arg(cx, 0, 'f', 2).arg(cy, 0, 'f', 2));
}

void TopoEditorView::rotatePgmAndMask(double cx, double cy, double cosA, double sinA)
{
    const int oldW = pgm_->width;
    const int oldH = pgm_->height;
    const double res = yaml_.resolution;
    const double ox = yaml_.origin.size() >= 1 ? yaml_.origin[0] : 0.0;
    const double oy = yaml_.origin.size() >= 2 ? yaml_.origin[1] : 0.0;

    // 旧图四角的世界坐标
    auto worldToRotated = [&](double wx, double wy) {
        const double dx = wx - cx;
        const double dy = wy - cy;
        return QPointF(cx + dx * cosA - dy * sinA, cy + dx * sinA + dy * cosA);
    };

    QPointF corners[4] = {
        worldToRotated(ox, oy),                          // 左下
        worldToRotated(ox + oldW * res, oy),             // 右下
        worldToRotated(ox + oldW * res, oy + oldH * res), // 右上
        worldToRotated(ox, oy + oldH * res),              // 左上
    };

    double newLeft = corners[0].x(), newRight = corners[0].x();
    double newBot  = corners[0].y(), newTop  = corners[0].y();
    for (int i = 1; i < 4; ++i) {
        if (corners[i].x() < newLeft) newLeft = corners[i].x();
        if (corners[i].x() > newRight) newRight = corners[i].x();
        if (corners[i].y() < newBot)  newBot  = corners[i].y();
        if (corners[i].y() > newTop)  newTop  = corners[i].y();
    }

    const int newW = std::min(16384, static_cast<int>(std::ceil((newRight - newLeft) / res)));
    const int newH = std::min(16384, static_cast<int>(std::ceil((newTop - newBot) / res)));
    if (newW < 1 || newH < 1)
        return;

    const double newOx = newLeft;
    const double newOy = newBot;

    // 逆变换函数：新图世界坐标 → 旧图世界坐标
    const double invCos = cosA;   // cos(-a) = cos(a)
    const double invSin = -sinA;  // sin(-a) = -sin(a)

    const quint16 fillVal = static_cast<quint16>(pgm_->maxval);
    std::vector<quint16> newPixels(static_cast<size_t>(newW * newH));
    QByteArray newMask(static_cast<int>(newW) * newH, static_cast<char>(255));

    for (int nr = 0; nr < newH; ++nr) {
        for (int nc = 0; nc < newW; ++nc) {
            const double wx = newOx + (nc + 0.5) * res;
            const double wy = newOy + (nr + 0.5) * res;
            // 逆旋转到旧图世界坐标
            const double dx = wx - cx;
            const double dy = wy - cy;
            const double oldWx = cx + dx * invCos - dy * invSin;
            const double oldWy = cy + dx * invSin + dy * invCos;
            // 旧图像素索引
            const int oc = static_cast<int>((oldWx - ox) / res);
            const int or_ = static_cast<int>((oldWy - oy) / res);
            const size_t newIdx = static_cast<size_t>(nr * newW + nc);
            if (oc >= 0 && oc < oldW && or_ >= 0 && or_ < oldH) {
                const size_t oldIdx = static_cast<size_t>(or_ * oldW + oc);
                newPixels[newIdx] = pgm_->pixels[oldIdx];
                if (mask_.size() == pgm_->pixelCount())
                    newMask[static_cast<int>(newIdx)] = mask_[static_cast<int>(oldIdx)];
            } else {
                newPixels[newIdx] = fillVal;
            }
        }
    }

    pgm_->width = newW;
    pgm_->height = newH;
    pgm_->pixels = std::move(newPixels);
    mask_ = std::move(newMask);

    yaml_.origin = {newOx, newOy, yaml_.origin.size() >= 3 ? yaml_.origin[2] : 0.0};
    yaml_.rawText = serializeRosMapYaml(yaml_, yaml_.image);

    rebuildRasterCaches();
}

void TopoEditorView::undoRaster()
{
    if (undoStackRaster_.isEmpty())
        return;
    QVector<PixelChange> stroke = std::move(undoStackRaster_.last());
    undoStackRaster_.pop_back();

    QVector<PixelChange> redoStroke;
    redoStroke.reserve(stroke.size());
    for (auto &pc : stroke) {
        PixelChange rpc;
        rpc.isMask = pc.isMask;
        rpc.idx = pc.idx;
        rpc.prev = pc.next;
        rpc.next = pc.prev;
        if (pc.isMask) {
            mask_[pc.idx] = static_cast<char>(pc.prev);
        } else {
            pgm_->pixels[static_cast<size_t>(pc.idx)] = pc.prev;
        }
        redoStroke.append(rpc);
    }
    redoStackRaster_.append(redoStroke);

    rebuildRasterCaches();
    rebuildRasterItem();
    emit undoRedoAvailabilityChanged();
    emit operationLogged(tr("撤销栅格编辑"));
}

void TopoEditorView::redoRaster()
{
    if (redoStackRaster_.isEmpty())
        return;
    QVector<PixelChange> stroke = std::move(redoStackRaster_.last());
    redoStackRaster_.pop_back();

    QVector<PixelChange> undoStroke;
    undoStroke.reserve(stroke.size());
    for (auto &pc : stroke) {
        PixelChange upc;
        upc.isMask = pc.isMask;
        upc.idx = pc.idx;
        upc.prev = pc.next;
        upc.next = pc.prev;
        if (pc.isMask) {
            mask_[pc.idx] = static_cast<char>(pc.next);
        } else {
            pgm_->pixels[static_cast<size_t>(pc.idx)] = pc.next;
        }
        undoStroke.append(upc);
    }
    undoStackRaster_.append(undoStroke);

    rebuildRasterCaches();
    rebuildRasterItem();
    emit undoRedoAvailabilityChanged();
    emit operationLogged(tr("重做栅格编辑"));
}

void TopoEditorView::rebuildRasterCaches()
{
    if (!pgm_)
        return;
    mapCache_ = buildMapPixmap();
    maskCache_ = buildMaskPixmap();
}

void TopoEditorView::rebuildRasterItem()
{
    if (rasterPixmapItem_) {
        scene()->removeItem(rasterPixmapItem_);
        delete rasterPixmapItem_;
        rasterPixmapItem_ = nullptr;
    }
    if (maskPixmapItem_) {
        scene()->removeItem(maskPixmapItem_);
        delete maskPixmapItem_;
        maskPixmapItem_ = nullptr;
    }

    if (!pgm_)
        return;

    const double res = yaml_.resolution > 0.0 ? yaml_.resolution : 0.05;
    const double ox = yaml_.origin.size() >= 1 ? yaml_.origin[0] : 0.0;
    const double oy = yaml_.origin.size() >= 2 ? yaml_.origin[1] : 0.0;

    // 地图世界范围：左下 (ox, oy)，右上 (ox + w*res, oy + h*res)
    const double worldW = pgm_->width * res;
    const double worldH = pgm_->height * res;

    if (showRaster_ && !mapCache_.isNull()) {
        rasterPixmapItem_ = new QGraphicsPixmapItem(mapCache_);
        // QPixmap row 0 = PGM row 0 = 地图顶部
        // 地图顶部世界 y = oy + worldH
        const QPointF topLeftScene = worldToScene(QPointF(ox, oy + worldH));
        // 缩放：像素 → 世界米 → 场景
        const double sx = worldW * kScale / mapCache_.width();
        const double sy = worldH * kScale / mapCache_.height();
        rasterPixmapItem_->setPos(topLeftScene);
        rasterPixmapItem_->setTransform(QTransform::fromScale(sx, -sy));
        rasterPixmapItem_->setZValue(-10);
        scene()->addItem(rasterPixmapItem_);
    }

    if (showMask_ && !maskCache_.isNull()) {
        maskPixmapItem_ = new QGraphicsPixmapItem(maskCache_);
        const QPointF topLeftScene = worldToScene(QPointF(ox, oy + worldH));
        const double sx = worldW * kScale / maskCache_.width();
        const double sy = worldH * kScale / maskCache_.height();
        maskPixmapItem_->setPos(topLeftScene);
        maskPixmapItem_->setTransform(QTransform::fromScale(sx, -sy));
        maskPixmapItem_->setZValue(-5);
        scene()->addItem(maskPixmapItem_);
    }
}

QPixmap TopoEditorView::buildMapPixmap() const
{
    if (!pgm_)
        return {};
    const int w = pgm_->width;
    const int h = pgm_->height;

    // 大图采样
    constexpr int kMaxCachePixels = 16 * 1024 * 1024;
    int step = 1;
    while (static_cast<qint64>(w / step) * (h / step) > kMaxCachePixels)
        step <<= 1;

    const int cw = w / step;
    const int ch = h / step;
    const int maxv = pgm_->maxval;

    QImage img(cw, ch, QImage::Format_Grayscale8);
    for (int r = 0; r < ch; ++r) {
        uchar *line = img.scanLine(r);
        for (int c = 0; c < cw; ++c) {
            quint16 val = pgm_->pixels[static_cast<size_t>(r * step * w + c * step)];
            if (maxv > 255)
                val = static_cast<quint16>((static_cast<quint32>(val) * 255) / maxv);
            line[c] = static_cast<uchar>(qMin(255, static_cast<int>(val)));
        }
    }
    return QPixmap::fromImage(img);
}

QPixmap TopoEditorView::buildMaskPixmap() const
{
    if (!pgm_ || mask_.size() != pgm_->pixelCount())
        return {};
    const int w = pgm_->width;
    const int h = pgm_->height;

    int step = 1;
    constexpr int kMaxCachePixels = 16 * 1024 * 1024;
    while (static_cast<qint64>(w / step) * (h / step) > kMaxCachePixels)
        step <<= 1;

    const int cw = w / step;
    const int ch = h / step;
    QImage img(cw, ch, QImage::Format_ARGB32);
    img.fill(Qt::transparent);
    for (int r = 0; r < ch; ++r) {
        for (int c = 0; c < cw; ++c) {
            const int srcIdx = r * step * w + c * step;
            if (static_cast<quint8>(mask_[srcIdx]) <= 20)
                img.setPixelColor(c, r, QColor(255, 0, 0, 120));
        }
    }
    return QPixmap::fromImage(img);
}

void TopoEditorView::beginRasterStroke()
{
    activeStroke_.clear();
    touchedPgm_.clear();
    touchedMask_.clear();
}

void TopoEditorView::finishRasterStroke()
{
    if (activeStroke_.isEmpty())
        return;
    undoStackRaster_.append(std::move(activeStroke_));
    if (undoStackRaster_.size() > kMaxRasterUndo)
        undoStackRaster_.removeFirst();
    redoStackRaster_.clear();
    activeStroke_.clear();
    rebuildRasterCaches();
    rebuildRasterItem();
    emit undoRedoAvailabilityChanged();
    emit operationLogged(tr("栅格编辑已应用"));
}

void TopoEditorView::setPixelWithUndo(int idx, quint16 val, bool isMask)
{
    if (idx < 0)
        return;
    if (isMask) {
        if (idx >= mask_.size())
            return;
        const quint16 prev = static_cast<quint8>(mask_[idx]);
        if (prev == val)
            return;
        if (touchedMask_.contains(idx))
            return;
        touchedMask_.insert(idx);
        mask_[idx] = static_cast<char>(val);
        activeStroke_.append({true, idx, prev, val});
    } else {
        if (!pgm_ || idx >= pgm_->pixelCount())
            return;
        const quint16 prev = pgm_->pixels[static_cast<size_t>(idx)];
        if (prev == val)
            return;
        if (touchedPgm_.contains(idx))
            return;
        touchedPgm_.insert(idx);
        pgm_->pixels[static_cast<size_t>(idx)] = val;
        activeStroke_.append({false, idx, prev, val});
    }
}

void TopoEditorView::applyRasterBrushAtCanvas(const QPointF &canvasPt)
{
    if (!pgm_)
        return;
    const QPointF w = sceneToWorld(canvasPt);
    const int centerIdx = worldToPixelIdx(w);
    if (centerIdx < 0)
        return;
    const QVector<int> idxs = pixelsInRadius(centerIdx);
    const bool isMask = (tool_ == Tool::RasterMask);
    rasterFillPixels(idxs, isMask);
}

void TopoEditorView::commitRasterLine(const QPointF &a, const QPointF &b)
{
    const int idxA = worldToPixelIdx(a);
    const int idxB = worldToPixelIdx(b);
    if (idxA < 0 || idxB < 0)
        return;
    QVector<int> lineIdxs;
    thickLinePixelIdxs(idxA, idxB, lineIdxs);
    const bool isMask = (tool_ == Tool::RasterMask);
    rasterFillPixels(lineIdxs, isMask);
}

void TopoEditorView::commitRasterRect(const QPointF &a, const QPointF &b)
{
    const int idxA = worldToPixelIdx(a);
    const int idxB = worldToPixelIdx(b);
    if (idxA < 0 || idxB < 0 || !pgm_)
        return;
    const int w = pgm_->width;
    const int h = pgm_->height;
    const int r1 = idxA / w;
    const int c1 = idxA % w;
    const int r2 = idxB / w;
    const int c2 = idxB % w;
    const int rmin = std::max(0, std::min(r1, r2));
    const int rmax = std::min(h - 1, std::max(r1, r2));
    const int cmin = std::max(0, std::min(c1, c2));
    const int cmax = std::min(w - 1, std::max(c1, c2));
    const bool isMask = (tool_ == Tool::RasterMask);

    // 矩形轮廓
    QVector<int> outline;
    for (int c = cmin; c <= cmax; ++c) {
        outline.append(rmin * w + c);
        if (rmax != rmin) outline.append(rmax * w + c);
    }
    for (int r = rmin + 1; r < rmax; ++r) {
        outline.append(r * w + cmin);
        if (cmax != cmin) outline.append(r * w + cmax);
    }
    QVector<int> thickOutline;
    for (int idx : outline) {
        const QVector<int> ring = pixelsInRadius(idx);
        for (int p : ring) {
            if (!thickOutline.contains(p))
                thickOutline.append(p);
        }
    }
    rasterFillPixels(thickOutline, isMask);

    // 填充矩形内部（橡皮时强制填充，其他工具看复选框）
    const bool shouldFill = (tool_ == Tool::RasterErase) || (rasterFill_ && !isMask);
    if (shouldFill) {
        quint16 fillVal;
        if (tool_ == Tool::RasterErase)
            fillVal = static_cast<quint16>(pgm_->maxval);
        else
            fillVal = 0;
        for (int r = rmin + 1; r < rmax; ++r) {
            for (int c = cmin + 1; c < cmax; ++c) {
                setPixelWithUndo(r * w + c, fillVal, false);
            }
        }
    }
}

void TopoEditorView::rasterFillPixels(const QVector<int> &idxs, bool isMask)
{
    if (!pgm_)
        return;
    quint16 pixelVal;
    if (isMask) {
        pixelVal = 0;
    } else if (tool_ == Tool::RasterPaint) {
        pixelVal = 0;
    } else if (tool_ == Tool::RasterErase) {
        pixelVal = static_cast<quint16>(pgm_->maxval);
    } else { // Unscan
        pixelVal = static_cast<quint16>(qRound(pgm_->maxval * 0.8039));
    }
    for (int idx : idxs)
        setPixelWithUndo(idx, pixelVal, isMask);
}

QVector<int> TopoEditorView::pixelsInRadius(int centerIdx) const
{
    QVector<int> out;
    if (!pgm_ || centerIdx < 0)
        return out;
    const int w = pgm_->width;
    const int h = pgm_->height;
    const int cr = centerIdx / w;
    const int cc = centerIdx % w;
    const int rad = brushRadius_;
    for (int dr = -rad; dr <= rad; ++dr) {
        const int r = cr + dr;
        if (r < 0 || r >= h) continue;
        for (int dc = -rad; dc <= rad; ++dc) {
            const int c = cc + dc;
            if (c < 0 || c >= w) continue;
            if (dr * dr + dc * dc <= rad * rad)
                out.append(r * w + c);
        }
    }
    return out;
}

void TopoEditorView::thickLinePixelIdxs(int idxA, int idxB, QVector<int> &out) const
{
    if (!pgm_ || idxA < 0 || idxB < 0)
        return;
    // Bresenham 直线连接，每步加笔刷圆
    const int w = pgm_->width;
    const int h = pgm_->height;
    const int x0 = idxA % w;
    const int y0 = idxA / w;
    const int x1 = idxB % w;
    const int y1 = idxB / w;
    const int dx = std::abs(x1 - x0);
    const int dy = std::abs(y1 - y0);
    const int sx = x0 < x1 ? 1 : -1;
    const int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    int x = x0, y = y0;
    QSet<int> seen;
    while (true) {
        if (x >= 0 && x < w && y >= 0 && y < h) {
            const int baseIdx = y * w + x;
            if (!seen.contains(baseIdx)) {
                seen.insert(baseIdx);
                const QVector<int> ring = pixelsInRadius(baseIdx);
                for (int p : ring) {
                    if (!seen.contains(p)) {
                        seen.insert(p);
                        out.append(p);
                    }
                }
            }
        }
        if (x == x1 && y == y1) break;
        const int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 < dx)  { err += dx; y += sy; }
    }
}

void TopoEditorView::updateRasterCursorCircle(const QPointF &worldPos)
{
    if (!pgm_)
        return;
    const QPointF sc = worldToScene(worldPos);
    const double nr = brushRadius_ * (yaml_.resolution > 0.0 ? yaml_.resolution : 0.05) * kScale;
    if (!rasterCursorCircle_) {
        rasterCursorCircle_ = new QGraphicsEllipseItem;
        rasterCursorCircle_->setPen(QPen(QColor(255, 120, 0, 200), 1.5));
        rasterCursorCircle_->setBrush(QColor(255, 120, 0, 40));
        rasterCursorCircle_->setZValue(40);
        scene()->addItem(rasterCursorCircle_);
    }
    rasterCursorCircle_->setRect(sc.x() - nr, sc.y() - nr, 2.0 * nr, 2.0 * nr);
    rasterCursorCircle_->show();
}

void TopoEditorView::clearRasterCursorCircle()
{
    if (rasterCursorCircle_) {
        if (rasterCursorCircle_->scene())
            scene()->removeItem(rasterCursorCircle_);
        delete rasterCursorCircle_;
        rasterCursorCircle_ = nullptr;
    }
}

void TopoEditorView::commitRasterPolyline()
{
    if (rasterPolyDraft_.size() < 2)
        return;
    const bool isMask = (tool_ == Tool::RasterMask);
    beginRasterStroke();
    QVector<int> allIdxs;
    for (int i = 1; i < rasterPolyDraft_.size(); ++i) {
        const int ia = worldToPixelIdx(rasterPolyDraft_[i - 1]);
        const int ib = worldToPixelIdx(rasterPolyDraft_[i]);
        if (ia >= 0 && ib >= 0)
            thickLinePixelIdxs(ia, ib, allIdxs);
    }
    rasterFillPixels(allIdxs, isMask);
    finishRasterStroke();
    rasterPolyDraft_.clear();
    rebuildRasterCaches();
    rebuildRasterItem();
    emit operationLogged(tr("栅格折线已提交"));
}

void TopoEditorView::commitRasterPolygon()
{
    if (rasterPolyDraft_.size() < 3)
        return;
    const bool isMask = (tool_ == Tool::RasterMask);
    beginRasterStroke();

    // 绘制所有边
    QVector<int> outlineIdxs;
    for (int i = 1; i < rasterPolyDraft_.size(); ++i) {
        const int ia = worldToPixelIdx(rasterPolyDraft_[i - 1]);
        const int ib = worldToPixelIdx(rasterPolyDraft_[i]);
        if (ia >= 0 && ib >= 0)
            thickLinePixelIdxs(ia, ib, outlineIdxs);
    }
    // 闭合边
    {
        const int ia = worldToPixelIdx(rasterPolyDraft_.last());
        const int ib = worldToPixelIdx(rasterPolyDraft_.first());
        if (ia >= 0 && ib >= 0)
            thickLinePixelIdxs(ia, ib, outlineIdxs);
    }
    rasterFillPixels(outlineIdxs, isMask);

    // 填充内部（橡皮时强制填充；障碍/未知/禁区看复选框）
    const bool shouldFill = (tool_ == Tool::RasterErase) || rasterFill_;
    if (shouldFill) {
        rasterFillPolygonInterior(outlineIdxs);
    }

    finishRasterStroke();
    rasterPolyDraft_.clear();
    rebuildRasterCaches();
    rebuildRasterItem();
    emit operationLogged(tr("栅格多边形已提交"));
}

void TopoEditorView::rasterFillPolygonInterior(const QVector<int> &outlineIdxs)
{
    if (!pgm_ || outlineIdxs.isEmpty())
        return;

    const int w = pgm_->width;
    const int h = pgm_->height;
    const bool isMask = (tool_ == Tool::RasterMask);

    // 取多边形顶点的像素索引，计算内部一个种子点（顶点平均位置）
    QVector<int> vertexIdxs;
    for (const QPointF &vp : rasterPolyDraft_) {
        const int vi = worldToPixelIdx(vp);
        if (vi >= 0)
            vertexIdxs.append(vi);
    }
    if (vertexIdxs.isEmpty())
        return;

    // 多边形几何中心作为种子候选
    double sumR = 0, sumC = 0;
    for (int vi : vertexIdxs) {
        sumR += vi / w;
        sumC += vi % w;
    }
    int seedR = static_cast<int>(sumR / vertexIdxs.size());
    int seedC = static_cast<int>(sumC / vertexIdxs.size());
    int seedIdx = seedR * w + seedC;

    // 轮廓集合
    QSet<int> outlineSet;
    outlineSet.reserve(outlineIdxs.size());
    for (int idx : outlineIdxs)
        outlineSet.insert(idx);

    // 如果种子落在轮廓上，沿行方向找最近的内部点
    if (outlineSet.contains(seedIdx)) {
        bool found = false;
        for (int spread = 1; spread < std::max(w, h); ++spread) {
            for (int dr = -spread; dr <= spread && !found; ++dr) {
                for (int dc = -spread; dc <= spread && !found; ++dc) {
                    if (std::abs(dr) != spread && std::abs(dc) != spread)
                        continue;
                    const int nr = seedR + dr;
                    const int nc = seedC + dc;
                    if (nr < 0 || nr >= h || nc < 0 || nc >= w)
                        continue;
                    const int nidx = nr * w + nc;
                    if (!outlineSet.contains(nidx)) {
                        seedIdx = nidx;
                        found = true;
                    }
                }
            }
        }
        if (!found)
            return; // 无法找到内部种子点
        if (outlineSet.contains(seedIdx))
            return;
    }

    // BFS 洪水填充
    quint16 pixelVal;
    if (isMask) {
        pixelVal = 0;
    } else if (tool_ == Tool::RasterErase) {
        pixelVal = static_cast<quint16>(pgm_->maxval);
    } else if (tool_ == Tool::RasterUnscan) {
        pixelVal = static_cast<quint16>(qRound(pgm_->maxval * 0.8039));
    } else { // Paint
        pixelVal = 0;
    }

    QSet<int> visited = outlineSet; // 轮廓不可穿越
    QQueue<int> q;
    q.enqueue(seedIdx);
    visited.insert(seedIdx);

    while (!q.empty()) {
        const int cur = q.dequeue();
        setPixelWithUndo(cur, pixelVal, isMask);

        const int r = cur / w;
        const int c = cur % w;
        const int neighbors[] = {
            (r > 0)     ? cur - w : -1,
            (r < h - 1) ? cur + w : -1,
            (c > 0)     ? cur - 1 : -1,
            (c < w - 1) ? cur + 1 : -1,
        };
        for (int nb : neighbors) {
            if (nb >= 0 && !visited.contains(nb)) {
                visited.insert(nb);
                q.enqueue(nb);
            }
        }
    }
}
