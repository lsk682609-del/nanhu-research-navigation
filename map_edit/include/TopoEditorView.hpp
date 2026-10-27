/**
******************************************************************************
*
* @file       TopoEditorView.hpp
* @author     Wei Wang -> shaxikai@outlook.com
* @date       2026.4.10
* @version    V1.0.0
* @brief      路网拓扑 QGraphicsView 编辑画布：节点/边/区域绘制与交互，栅格底图显示与编辑。
*
*****************************************************************************/

#pragma once

#include "region_io.hpp"
#include "pgm_io.hpp"
#include "map_yaml.hpp"

#include <QEvent>
#include <QPoint>
#include <QGraphicsView>
#include <QPair>
#include <QPointF>
#include <QPixmap>
#include <QRectF>
#include <QString>
#include <QTimer>
#include <QVector>
#include <QSet>
#include <QHash>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/msg/navigation_status.hpp>
#include <nav2_msgs/srv/navigation_request.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "topo_map.hpp"

class QFocusEvent;
class QGraphicsItem;
class QGraphicsEllipseItem;
class QGraphicsLineItem;
class QGraphicsTextItem;
class QKeyEvent;
class QLabel;
class QRubberBand;
class QToolButton;

class TopoEditorView : public QGraphicsView
{
    Q_OBJECT
public:
    enum class Tool { Select, AddNode, AddEdge, KeepoutPoly, PassablePoly, Measure,
                      RasterPaint, RasterErase, RasterUnscan, RasterMask, PlanPath };
    enum class RasterShape { Point, Line, Rect, Polygon };

    explicit TopoEditorView(QWidget *parent = nullptr);
    ~TopoEditorView() override;

    void setActiveMapId(int mapId);
    int activeMapId() const { return mapId_; }
    QVector<QPair<int, QString>> subMapOptions() const;

    void setTool(Tool t);
    Tool tool() const { return tool_; }

    void newProject();
    bool loadProject(const QString &path, QString *err = nullptr);
    bool saveProject(const QString &path, QString *err = nullptr);
    QString currentPath() const { return path_; }
    void setCurrentPath(const QString &p) { path_ = p; }

    bool canUndo() const;
    bool canRedo() const;

    /// 无向图意义下路网是否全连通
    bool isUndirectedTopoConnected() const { return topo_.isUndirectedTopoConnected(); }
    int undirectedTopoComponentCount() const { return topo_.countUndirectedTopoComponents(); }

    bool useLlaCoordinates() const { return topo_.useLlaCoordinates; }
    bool refLlaValid() const { return topo_.refLlaValid; }
    double refLatDeg() const { return topo_.refLatDeg; }
    double refLonDeg() const { return topo_.refLonDeg; }
    double refAltM() const { return topo_.refAltM; }

    void activateLlaModeWithOrigin(double latDeg, double lonDeg, double altM);
    void resumeLlaModeWithStoredRef();
    void commitReferenceAndRecomputeNodes(double latDeg, double lonDeg, double altM);

    bool focusViewOnNodeId(int nodeId);
    bool focusViewOnEdgeId(int edgeId);
    bool focusViewOnNodeAndEdge(int nodeId, int edgeId);

    bool stepVisibleNodeId(int fromId, int delta, int *outId) const;
    bool stepVisibleEdgeId(int fromId, int delta, int *outId) const;

    // ---- 栅格地图 ----

    bool hasRasterMap() const;
    bool loadRasterMap(const QString &yamlPath, QString *err = nullptr);
    bool saveRasterMap(const QString &yamlAbsDir, const QString &pgmFilename, QString *err = nullptr);
    bool saveMaskMap(const QString &yamlAbsDir, const QString &pgmFilename, QString *err = nullptr);

    void setShowRaster(bool on);
    bool showRaster() const { return showRaster_; }
    void setShowMask(bool on);
    bool showMask() const { return showMask_; }

    void setRasterShape(RasterShape s) { rasterShape_ = s; }
    RasterShape rasterShape() const { return rasterShape_; }
    void setRasterFill(bool on) { rasterFill_ = on; }
    bool rasterFill() const { return rasterFill_; }
    void setBrushRadius(int r);
    int brushRadius() const { return brushRadius_; }

    void setNodeDiameterPx(double d);
    double nodeDiameterPx() const;
    void setEdgeWidthPx(double w);
    double edgeWidthPx() const;

    /// 设置导航 ROS 接口使用的命名空间。空字符串表示根命名空间。
    bool setGoalTopicNamespace(const QString &namespaceText, QString *error = nullptr);
    QString goalTopicNamespace() const { return goalTopicNamespace_; }
    QString goalPoseTopic() const;
    QString odomTopic() const;
    QString navigationRequestService() const;
    QString navigationStatusTopic() const;
    QString navigationGlobalPathTopic() const;

    bool canUndoRaster() const;
    bool canRedoRaster() const;

    void snapNodesToGrid(double step);

    void clearPlanPathSelection();
    void runPlanPath();

    void translateMapData(double dx, double dy);
    void rotateMapData(double cx, double cy, double angleDeg);

    RosMapYaml rasterYaml() const { return yaml_; }

    /// 将世界坐标 (米) 转换为 PGM 像素索引，失败返回 -1；yaw=0 简化
    int worldToPixelIdx(const QPointF &worldPt) const;
    /// 由像素索引返回世界坐标（像素中心，米）
    QPointF pixelIdxToWorld(int idx) const;

public slots:
    void undo();
    void redo();
    void finishPolygon();
    void deleteSelection();
    void zoomIn();
    void zoomOut();
    void zoomReset();
    void setUseLlaCoordinates(bool on);

    // 栅格 undo/redo
    void undoRaster();
    void redoRaster();

signals:
    void undoRedoAvailabilityChanged();
    void useLlaCoordinatesChanged();
    void operationLogged(const QString &message);
    void rasterStateChanged();
    void navigationStatusChanged(int state, float distanceRemaining, const QString &message);

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void enterEvent(QEvent *event) override;
    void leaveEvent(QEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    void focusOutEvent(QFocusEvent *event) override;

private:
    struct MapSnapshot {
        TopoMap topo;
        QVector<RegionPolygon> regions;
    };

    struct PixelChange {
        bool isMask = false;
        int idx = 0;
        quint16 prev = 0;
        quint16 next = 0;
    };

    void recordUndoCheckpoint();
    void clearUndoHistory();
    void rebuildScene();
    void updatePolyDraft();
    bool tryNodeScenePos(int nodeId, QPointF *outScene) const;
    bool tryEdgeSceneRect(int edgeId, QRectF *outRect) const;
    void updateEdgePreview(const QPointF &cursorScene);
    void clearEdgePreview();
    void clearMeasureOverlay();
    void updateMeasureVisual(const QPointF &sceneA, const QPointF &sceneB);
    void updateCursorHud(const QPoint &viewportPos);
    QPointF sceneToWorld(const QPointF &scenePt) const;
    QPointF worldToScene(const QPointF &w) const;
    int nextNodeId() const;
    int nextEdgeId() const;
    int nextRegionId() const;
    QGraphicsItem *nodeItemAt(const QPointF &scenePos) const;
    QGraphicsItem *nodeItemById(int nodeId) const;
    int nodeIdFromItem(QGraphicsItem *it) const;
    QGraphicsItem *topoHitItemAt(const QPointF &scenePos) const;
    void showSelectContextMenu(const QPoint &viewportPos, QGraphicsItem *hit);
    void publishGoalPoseMapFrame(double eastM, double northM, double upM);
    bool tryResolveNodeWgs84Lla(int nodeId, double *latDeg, double *lonDeg, double *altM) const;
    void publishGpGoalNavSatFix(double latDeg, double lonDeg, double altM);
    void ensureGoalNode();
    void ensureNavigationInterfaces();
    void ensureGpGoalPublisher();
    void ensureOdomSubscription();
    void resetGoalPublishers();
    void deleteTopoNodeId(int id);
    void deleteTopoEdgeId(int edgeId);
    void openEditNodeDialog(int nodeId);
    void openEditEdgeDialog(int edgeId);
    void clearSpacePanState();
    void updateSpacePanCursor();
    void clearSelectMouseState();
    void beginNodeDragEdit(int nodeId);
    void finishNodeDragEdit(bool accept);
    void clearNodeDragEditState();
    void updateNodeDragEditOverlay();
    void updateNodeDragVisual(const QPointF &centerScene);
    void updateOdomVehicleMarker(double x, double y);
    void updateNavigationPath(const nav_msgs::msg::Path &path);
    void clearNavigationPath();

    double sceneToViewportScale() const;
    double effectiveNodeRadiusScene() const;
    void updateTopoNodeVisualScale();

    // ---- 栅格渲染 ----
    void rebuildRasterCaches();
    void rebuildRasterItem();
    QPixmap buildMapPixmap() const;
    QPixmap buildMaskPixmap() const;

    // ---- 栅格编辑 ----
    void beginRasterStroke();
    void finishRasterStroke();
    void setPixelWithUndo(int idx, quint16 val, bool isMask);
    void applyRasterBrushAtCanvas(const QPointF &canvasPt);
    void commitRasterLine(const QPointF &a, const QPointF &b);
    void commitRasterRect(const QPointF &a, const QPointF &b);
    void commitRasterPolyline();
    void commitRasterPolygon();
    void rasterFillPixels(const QVector<int> &idxs, bool isMask);
    void rasterFillPolygonInterior(const QVector<int> &outlineIdxs);
    QVector<int> pixelsInRadius(int centerIdx) const;
    void thickLinePixelIdxs(int idxA, int idxB, QVector<int> &out) const;
    void updateRasterCursorCircle(const QPointF &worldPos);
    void clearRasterCursorCircle();
    void rotatePgmAndMask(double cx, double cy, double cosA, double sinA);

    TopoMap topo_;
    QVector<RegionPolygon> regions_;
    QString path_;
    QString goalTopicNamespace_;
    rclcpp::Node::SharedPtr goalNode_;
    rclcpp::Client<nav2_msgs::srv::NavigationRequest>::SharedPtr navigationRequestClient_;
    rclcpp::Subscription<nav2_msgs::msg::NavigationStatus>::SharedPtr navigationStatusSubscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr navigationPathSubscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gpGoalPublisher_;
    int mapId_ = -1;
    Tool tool_ = Tool::Select;

    // ---- 路径规划 ----
    int planStartId_ = -1;
    int planGoalId_ = -1;
    QGraphicsEllipseItem *planStartMarker_ = nullptr;
    QGraphicsEllipseItem *planGoalMarker_ = nullptr;
    QVector<QGraphicsLineItem *> planPathLines_;
    QVector<QGraphicsLineItem *> navigationPathLines_;
    nav_msgs::msg::Path navigationPath_;

    int edgePickA_ = -1;
    QVector<QPointF> polyDraft_;
    bool polyIsKeepout_ = true;
    QGraphicsPathItem *draftItem_ = nullptr;
    QGraphicsLineItem *edgePreviewLine_ = nullptr;

    bool measureDragging_ = false;
    QPointF measureAnchorScene_;
    QGraphicsLineItem *measureLine_ = nullptr;
    QGraphicsTextItem *measureText_ = nullptr;
    QTimer measureClearTimer_;
    QTimer odomSpinTimer_;
    QLabel *cursorHud_ = nullptr;

    bool odomVehicleValid_ = false;
    double odomVehicleX_ = 0.0;
    double odomVehicleY_ = 0.0;
    QGraphicsEllipseItem *odomVehicleItem_ = nullptr;

    QVector<MapSnapshot> undoStack_;
    QVector<MapSnapshot> redoStack_;

    bool spaceDown_ = false;
    bool spacePanDragging_ = false;
    QPoint spacePanLastPos_;

    // ---- 选择工具鼠标交互 ----
    bool selectMousePressed_ = false;
    bool selectPanDragging_ = false;
    bool selectRubberBandDragging_ = false;
    QPoint selectPressPos_;
    QPoint selectPanLastPos_;
    QPointF selectPressScenePos_;
    QGraphicsItem *selectContextHit_ = nullptr;
    QRubberBand *selectRubberBand_ = nullptr;

    // ---- 节点拖动编辑 ----
    bool nodeDragEditMode_ = false;
    bool nodeDragEditMousePressed_ = false;
    bool nodeDragEditMoved_ = false;
    int nodeDragEditId_ = -1;
    QPoint nodeDragEditPressPos_;
    QPointF nodeDragEditPressScenePos_;
    QPointF nodeDragEditPressOffsetScene_;
    QPointF nodeDragEditOriginalScenePos_;
    QPointF nodeDragEditPreviewScenePos_;
    QVector<int> nodeDragEditIds_;
    QHash<int, QPointF> nodeDragEditOriginalScenePositions_;
    QHash<int, QPointF> nodeDragEditPreviewScenePositions_;
    QVector<QGraphicsEllipseItem *> nodeDragEditGhostItems_;
    QGraphicsItem *nodeDragEditItem_ = nullptr;
    QLabel *nodeDragEditDeltaLabel_ = nullptr;
    QToolButton *nodeDragEditCancelButton_ = nullptr;
    QToolButton *nodeDragEditConfirmButton_ = nullptr;

    // ---- 栅格数据 ----
    std::unique_ptr<PgmImage> pgm_;
    RosMapYaml yaml_;
    QByteArray mask_;
    bool showRaster_ = true;
    bool showMask_ = true;
    QPixmap mapCache_;
    QPixmap maskCache_;

    // 场景中的栅格/遮罩图元（rebuildRasterItem 维护）
    QGraphicsPixmapItem *rasterPixmapItem_ = nullptr;
    QGraphicsPixmapItem *maskPixmapItem_ = nullptr;

    // 栅格编辑状态
    RasterShape rasterShape_ = RasterShape::Point;
    int brushRadius_ = 2;
    double nodeDiameterPx_ = 28.0;
    double edgeWidthPx_ = 4.0;
    bool rasterFill_ = false;
    bool rasterDragging_ = false;
    QPointF rasterLineStart_;       // 世界坐标
    QPointF rasterLastWorld_;       // 世界坐标，自由绘制用
    QPointF rasterPreviewA_;        // line/rect 预览起点，世界坐标
    QPointF rasterPreviewB_;        // line/rect 预览终点，世界坐标
    QVector<QPointF> rasterPolyDraft_;  // polyline/polygon 顶点，世界坐标
    QGraphicsPathItem *rasterPreviewItem_ = nullptr;
    QGraphicsEllipseItem *rasterCursorCircle_ = nullptr;
    QGraphicsPathItem *rasterDraftItem_ = nullptr;

    // 栅格 undo
    QVector<PixelChange> activeStroke_;
    QVector<QVector<PixelChange>> undoStackRaster_;
    QVector<QVector<PixelChange>> redoStackRaster_;
    QSet<int> touchedPgm_;
    QSet<int> touchedMask_;
    static constexpr int kMaxRasterUndo = 100;

    static constexpr double kScale = 40.0;
    static constexpr int kMaxUndo = 100;
};
