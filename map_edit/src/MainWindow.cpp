/**
******************************************************************************
*
* @file       MainWindow.cpp
* @author     Wei Wang -> shaxikai@outlook.com
* @date       2026.4.10
* @version    V1.0.0
* @brief      主窗口实现：布局、地图配置、视图定位、栅格文件操作与操作日志。
*
*****************************************************************************/

#include "MainWindow.hpp"
#include "TopoEditorView.hpp"

#include <QAbstractButton>
#include <QAction>
#include <QButtonGroup>
#include <QCheckBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QKeySequence>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QScrollArea>
#include <QShortcut>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSettings>
#include <QSlider>
#include <QSplitter>
#include <QStackedWidget>
#include <QTextCursor>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWidget>

// ---- 常量 ----------------------------------------------------------------
static constexpr int kToolSelect   = 0;
static constexpr int kToolAddNode  = 1;
static constexpr int kToolAddEdge  = 2;
static constexpr int kToolKeepout  = 3;
static constexpr int kToolPassable = 4;
static constexpr int kToolMeasure  = 5;
static constexpr int kToolRasterPaint   = 6;
static constexpr int kToolRasterErase   = 7;
static constexpr int kToolRasterUnscan  = 8;
static constexpr int kToolRasterMask    = 9;
static constexpr int kToolPlanPath      = 10;

static bool isRasterToolId(int id)
{
    return id >= kToolRasterPaint && id <= kToolRasterMask;
}

static QString navigationStateText(int state)
{
    switch (state) {
    case 0: return QObject::tr("空闲");
    case 1: return QObject::tr("规划中");
    case 2: return QObject::tr("导航中");
    case 3: return QObject::tr("成功");
    case 4: return QObject::tr("已取消");
    case 5: return QObject::tr("失败");
    default: return QObject::tr("未知");
    }
}

// ========================================================================

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle(tr("地图编辑器"));
    resize(1280, 800);

    view_ = new TopoEditorView;
    // 导航接口命名空间是 GUI 设置，不写入地图工程文件；沿用原设置键以兼容旧配置。
    view_->setGoalTopicNamespace(QSettings().value(QStringLiteral("goal_pose_namespace")).toString());

    // ---- 地图配置 ----
    mapConfigGroup_ = new QGroupBox(tr("地图配置"));
    auto *cfgLay = new QVBoxLayout(mapConfigGroup_);
    coordButtonGroup_ = new QButtonGroup(this);
    radioCoordLocal_ = new QRadioButton(tr("局部坐标系"));
    radioCoordGeo_ = new QRadioButton(tr("经纬度坐标系"));
    coordButtonGroup_->addButton(radioCoordLocal_, 0);
    coordButtonGroup_->addButton(radioCoordGeo_, 1);
    radioCoordLocal_->setChecked(true);
    cfgLay->addWidget(radioCoordLocal_);
    cfgLay->addWidget(radioCoordGeo_);

    originPanel_ = new QWidget;
    auto *originForm = new QFormLayout(originPanel_);
    originForm->setContentsMargins(0, 0, 0, 0);
    originLon_ = new QDoubleSpinBox;
    originLon_->setRange(-180.0, 180.0);
    originLon_->setDecimals(8);
    originLat_ = new QDoubleSpinBox;
    originLat_->setRange(-90.0, 90.0);
    originLat_->setDecimals(8);
    originAlt_ = new QDoubleSpinBox;
    originAlt_->setRange(-1000.0, 10000.0);
    originAlt_->setDecimals(3);
    originForm->addRow(tr("原点经度"), originLon_);
    originForm->addRow(tr("原点纬度"), originLat_);
    originForm->addRow(tr("原点高度"), originAlt_);
    originPanel_->setVisible(false);
    cfgLay->addWidget(originPanel_);

    QObject::connect(originLon_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onOriginEdited);
    QObject::connect(originLat_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onOriginEdited);
    QObject::connect(originAlt_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onOriginEdited);

    QObject::connect(coordButtonGroup_, QOverload<QAbstractButton *>::of(&QButtonGroup::buttonClicked), this,
                     &MainWindow::onCoordButtonClicked);
    QObject::connect(view_, &TopoEditorView::useLlaCoordinatesChanged, this, &MainWindow::syncMapConfigFromView);

    // ---- 视图定位 ----
    viewLocateGroup_ = new QGroupBox(tr("视图定位"));
    auto *locOuter = new QVBoxLayout(viewLocateGroup_);
    auto *locForm = new QFormLayout;
    locForm->setContentsMargins(0, 0, 0, 0);
    findNodeIdEdit_ = new QLineEdit;
    findEdgeIdEdit_ = new QLineEdit;
    locForm->addRow(tr("节点 ID"), findNodeIdEdit_);
    locForm->addRow(tr("边 ID"), findEdgeIdEdit_);
    locOuter->addLayout(locForm);
    auto *locBtnRow = new QHBoxLayout;
    auto *btnLocatePrev = new QPushButton(tr("上一个"));
    auto *btnLocateNext = new QPushButton(tr("下一个"));
    locBtnRow->addWidget(btnLocatePrev);
    locBtnRow->addWidget(btnLocateNext);
    locBtnRow->addStretch();
    auto *btnFindJump = new QPushButton(tr("跳转"));
    locBtnRow->addWidget(btnFindJump);
    locOuter->addLayout(locBtnRow);
    QObject::connect(btnLocatePrev, &QPushButton::clicked, this, &MainWindow::onLocatePrev);
    QObject::connect(btnLocateNext, &QPushButton::clicked, this, &MainWindow::onLocateNext);
    QObject::connect(btnFindJump, &QPushButton::clicked, this, &MainWindow::onFindJump);
    QObject::connect(findNodeIdEdit_, &QLineEdit::returnPressed, this, &MainWindow::onFindJump);
    QObject::connect(findEdgeIdEdit_, &QLineEdit::returnPressed, this, &MainWindow::onFindJump);

    // ---- 导航状态 ----
    auto *navigationGroup = new QGroupBox(tr("导航状态"));
    auto *navigationForm = new QFormLayout(navigationGroup);
    navigationStateLabel_ = new QLabel(tr("等待状态"));
    navigationDistanceLabel_ = new QLabel(QStringLiteral("-- m"));
    navigationMessageLabel_ = new QLabel(tr("尚未收到导航状态"));
    navigationMessageLabel_->setWordWrap(true);
    navigationForm->addRow(tr("状态"), navigationStateLabel_);
    navigationForm->addRow(tr("剩余长度"), navigationDistanceLabel_);
    navigationForm->addRow(tr("信息"), navigationMessageLabel_);

    // ---- YAML 预览 ----
    auto *yamlGroup = new QGroupBox(tr("栅格 YAML"));
    auto *yamlLay = new QVBoxLayout(yamlGroup);
    yamlPreview_ = new QPlainTextEdit;
    yamlPreview_->setReadOnly(true);
    yamlPreview_->setMaximumBlockCount(100);
    yamlPreview_->setMinimumHeight(100);
    yamlPreview_->setPlaceholderText(tr("加载栅格地图后显示 YAML 内容"));
    yamlPreview_->setStyleSheet(
        QStringLiteral("QPlainTextEdit { background-color: #ffffff; border: 1px solid #c8c8c8; border-radius: 3px; font-size: 11px; }"));
    yamlLay->addWidget(yamlPreview_);

    // ---- 说明 ----
    helpGroup_ = new QGroupBox(tr("说明"));
    auto *helpLay = new QVBoxLayout(helpGroup_);
    helpLay->setContentsMargins(8, 8, 8, 8);
    legendLabel_ = new QLabel;
    legendLabel_->setWordWrap(true);
    legendLabel_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    auto *helpScroll = new QScrollArea;
    helpScroll->setWidgetResizable(true);
    helpScroll->setFrameShape(QFrame::NoFrame);
    helpScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    helpScroll->setMinimumHeight(100);
    helpScroll->setWidget(legendLabel_);
    helpLay->addWidget(helpScroll);

    // ---- 操作日志 ----
    auto *logSection = new QWidget;
    auto *logSectionLay = new QVBoxLayout(logSection);
    logSectionLay->setContentsMargins(0, 0, 0, 0);
    logSectionLay->setSpacing(4);
    auto *logTitle = new QLabel(tr("操作日志"));
    logTitle->setStyleSheet(QStringLiteral("font-weight: 600;"));
    opLogEdit_ = new QPlainTextEdit;
    opLogEdit_->setReadOnly(true);
    opLogEdit_->setMaximumBlockCount(800);
    opLogEdit_->setMinimumHeight(100);
    opLogEdit_->setPlaceholderText(tr("编辑路网时的操作将记录在此处"));
    opLogEdit_->setStyleSheet(
        QStringLiteral("QPlainTextEdit { background-color: #ffffff; border: 1px solid #c8c8c8; border-radius: 3px; }"));
    logSectionLay->addWidget(logTitle);
    logSectionLay->addWidget(opLogEdit_, 1);

    // ---- 右侧面板 ----
    auto *right = new QWidget;
    right->setMinimumWidth(260);
    auto *rightLay = new QVBoxLayout(right);
    rightLay->addWidget(mapConfigGroup_);
    rightLay->addWidget(viewLocateGroup_);
    rightLay->addWidget(navigationGroup);
    rightLay->addWidget(yamlGroup);
    rightLay->addWidget(helpGroup_, 1);
    rightLay->addWidget(logSection, 1);

    QObject::connect(view_, &TopoEditorView::operationLogged, this, &MainWindow::appendOperationLog);
    QObject::connect(view_, &TopoEditorView::rasterStateChanged, this, &MainWindow::onRasterStateChanged);
    QObject::connect(view_, &TopoEditorView::navigationStatusChanged,
                     this, &MainWindow::updateNavigationStatus);

    // ---- 中心分割器 ----
    auto *centerSplit = new QSplitter(Qt::Horizontal, this);
    centerSplit->addWidget(view_);
    centerSplit->addWidget(right);
    centerSplit->setStretchFactor(0, 1);
    centerSplit->setStretchFactor(1, 0);
    centerSplit->setSizes({1000, 280});

    // ---- 左工具栏 ----
    auto *leftToolPanel = new QWidget;
    leftToolPanel->setMinimumWidth(80);
    auto *leftToolLay = new QVBoxLayout(leftToolPanel);
    leftToolLay->setContentsMargins(6, 8, 6, 8);
    leftToolLay->setSpacing(6);
    leftToolGrp_ = new QButtonGroup(this);
    leftToolGrp_->setExclusive(true);
    auto addLeftToolBtn = [leftToolLay, this](const QString &text, int id) -> QPushButton * {
        auto *b = new QPushButton(text);
        b->setCheckable(true);
        b->setMinimumHeight(40);
        leftToolGrp_->addButton(b, id);
        leftToolLay->addWidget(b);
        return b;
    };
    addLeftToolBtn(tr("选择"), kToolSelect)->setChecked(true);
    addLeftToolBtn(tr("添加节点"), kToolAddNode);
    addLeftToolBtn(tr("添加边"), kToolAddEdge);
    addLeftToolBtn(tr("禁区"), kToolKeepout);
    addLeftToolBtn(tr("可行区"), kToolPassable);
    addLeftToolBtn(tr("测距"), kToolMeasure);
    // 分隔
    auto *sep = new QFrame;
    sep->setFrameShape(QFrame::HLine);
    sep->setStyleSheet(QStringLiteral("QFrame { color: #aaa; }"));
    leftToolLay->addWidget(sep);
    addLeftToolBtn(tr("障碍区"), kToolRasterPaint);
    addLeftToolBtn(tr("橡皮"), kToolRasterErase);
    addLeftToolBtn(tr("未知区"), kToolRasterUnscan);
    addLeftToolBtn(tr("禁区"), kToolRasterMask);
    // 分隔
    auto *sep2 = new QFrame;
    sep2->setFrameShape(QFrame::HLine);
    sep2->setStyleSheet(QStringLiteral("QFrame { color: #aaa; }"));
    leftToolLay->addWidget(sep2);
    addLeftToolBtn(tr("路径规划"), kToolPlanPath);
    leftToolLay->addStretch();

    // ---- 主分割器 ----
    auto *mainSplit = new QSplitter(Qt::Horizontal, this);
    mainSplit->addWidget(leftToolPanel);
    mainSplit->addWidget(centerSplit);
    mainSplit->setStretchFactor(0, 0);
    mainSplit->setStretchFactor(1, 1);
    mainSplit->setSizes({100, 1200});

    setCentralWidget(mainSplit);

    // ---- 菜单栏 ----
    auto *mb = menuBar();

    QMenu *fileMenu = mb->addMenu(tr("文件"));
    fileMenu->addAction(tr("打开文件夹…"), this, &MainWindow::onOpenFolder);
    fileMenu->addSeparator();
    fileMenu->addAction(tr("打开路网"), this, &MainWindow::onOpen);
    fileMenu->addAction(tr("保存路网"), this, &MainWindow::onSave);
    fileMenu->addAction(tr("路网另存为…"), this, &MainWindow::onSaveAs);
    fileMenu->addSeparator();
    fileMenu->addAction(tr("打开栅格地图…"), this, &MainWindow::onOpenRasterMap);
    fileMenu->addAction(tr("保存栅格地图…"), this, &MainWindow::onSaveRasterMap);
    fileMenu->addAction(tr("保存禁区地图…"), this, &MainWindow::onSaveMask);

    QMenu *editMenu = mb->addMenu(tr("编辑"));
    actUndo_ = editMenu->addAction(tr("撤销"));
    actUndo_->setShortcut(QKeySequence::Undo);
    actRedo_ = editMenu->addAction(tr("重做"));
    actRedo_->setShortcut(QKeySequence::Redo);
    addAction(actUndo_);
    addAction(actRedo_);
    QObject::connect(actUndo_, &QAction::triggered, view_, [this]() {
        if (isRasterToolId(leftToolGrp_->checkedId()))
            view_->undoRaster();
        else
            view_->undo();
    });
    QObject::connect(actRedo_, &QAction::triggered, view_, [this]() {
        if (isRasterToolId(leftToolGrp_->checkedId()))
            view_->redoRaster();
        else
            view_->redo();
    });

    editMenu->addSeparator();
    QAction *actSnap = editMenu->addAction(tr("坐标规整…"));
    QObject::connect(actSnap, &QAction::triggered, this, [this]() {
        if (!view_)
            return;
        bool ok = false;
        const double step = QInputDialog::getDouble(this, tr("坐标规整"),
                                                     tr("最小单位（米）:"), 0.1, 0.001, 100.0, 3, &ok);
        if (ok && step > 0.0) {
            view_->snapNodesToGrid(step);
        }
    });

    editMenu->addSeparator();
    actTranslate_ = editMenu->addAction(tr("平移…"));
    QObject::connect(actTranslate_, &QAction::triggered, this, [this]() {
        if (!view_ || view_->useLlaCoordinates())
            return;
        QDialog dlg(this);
        dlg.setWindowTitle(tr("平移地图"));
        auto *dxSpin = new QDoubleSpinBox(&dlg);
        dxSpin->setRange(-10000, 10000);
        dxSpin->setDecimals(3);
        dxSpin->setValue(0);
        auto *dySpin = new QDoubleSpinBox(&dlg);
        dySpin->setRange(-10000, 10000);
        dySpin->setDecimals(3);
        dySpin->setValue(0);
        auto *form = new QFormLayout;
        form->addRow(tr("Δx (米)"), dxSpin);
        form->addRow(tr("Δy (米)"), dySpin);
        auto *bb = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        QObject::connect(bb, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        QObject::connect(bb, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        auto *vl = new QVBoxLayout(&dlg);
        vl->addLayout(form);
        vl->addWidget(bb);
        if (dlg.exec() == QDialog::Accepted)
            view_->translateMapData(dxSpin->value(), dySpin->value());
    });

    actRotate_ = editMenu->addAction(tr("旋转…"));
    QObject::connect(actRotate_, &QAction::triggered, this, [this]() {
        if (!view_ || view_->useLlaCoordinates())
            return;
        // 默认旋转中心：有栅格时用地图中心，否则原点
        double defCx = 0, defCy = 0;
        if (view_->hasRasterMap()) {
            const RosMapYaml y = view_->rasterYaml();
            const double ox = y.origin.size() >= 1 ? y.origin[0] : 0;
            const double oy = y.origin.size() >= 2 ? y.origin[1] : 0;
            // 从 pgm 宽高推算... 但 view 没有直接暴露 pgm 尺寸
            // 用 scene bounds 反推大概范围
            defCx = ox;
            defCy = oy;
        }
        QDialog dlg(this);
        dlg.setWindowTitle(tr("旋转地图"));
        auto *cxSpin = new QDoubleSpinBox(&dlg);
        cxSpin->setRange(-1e6, 1e6);
        cxSpin->setDecimals(3);
        cxSpin->setValue(defCx);
        auto *cySpin = new QDoubleSpinBox(&dlg);
        cySpin->setRange(-1e6, 1e6);
        cySpin->setDecimals(3);
        cySpin->setValue(defCy);
        auto *angleSpin = new QDoubleSpinBox(&dlg);
        angleSpin->setRange(-180, 180);
        angleSpin->setDecimals(2);
        angleSpin->setSuffix(tr("°"));
        angleSpin->setValue(0);
        auto *form = new QFormLayout;
        form->addRow(tr("旋转中心 X (米)"), cxSpin);
        form->addRow(tr("旋转中心 Y (米)"), cySpin);
        form->addRow(tr("旋转角度"), angleSpin);
        auto *bb = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        QObject::connect(bb, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        QObject::connect(bb, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        auto *vl = new QVBoxLayout(&dlg);
        vl->addLayout(form);
        vl->addWidget(bb);
        if (dlg.exec() == QDialog::Accepted && std::abs(angleSpin->value()) > 0.001)
            view_->rotateMapData(cxSpin->value(), cySpin->value(), angleSpin->value());
    });

    // 图层菜单
    QMenu *layerMenu = mb->addMenu(tr("图层"));
    actShowRaster_ = layerMenu->addAction(tr("显示栅格地图"));
    actShowRaster_->setCheckable(true);
    actShowRaster_->setChecked(true);
    QObject::connect(actShowRaster_, &QAction::toggled, view_, &TopoEditorView::setShowRaster);
    actShowMask_ = layerMenu->addAction(tr("显示禁区"));
    actShowMask_->setCheckable(true);
    actShowMask_->setChecked(true);
    QObject::connect(actShowMask_, &QAction::toggled, view_, &TopoEditorView::setShowMask);

    // 占位菜单
    QMenu *winMenu = mb->addMenu(tr("窗口"));
    QAction *ph = winMenu->addAction(tr("（功能待添加）"));
    ph->setEnabled(false);

    QMenu *selMenu = mb->addMenu(tr("选择"));
    QAction *ph2 = selMenu->addAction(tr("（功能待添加）"));
    ph2->setEnabled(false);

    QMenu *settingsMenu = mb->addMenu(tr("设置"));
    QAction *actGoalTopicNamespace = settingsMenu->addAction(tr("导航接口命名空间…"));
    QObject::connect(actGoalTopicNamespace, &QAction::triggered, this, &MainWindow::onGoalTopicNamespace);
    settingsMenu->addSeparator();
    QAction *actNodeSize = settingsMenu->addAction(tr("节点大小…"));
    QObject::connect(actNodeSize, &QAction::triggered, this, [this]() {
        if (!view_) return;
        bool ok = false;
        const double v = QInputDialog::getDouble(this, tr("节点大小"),
                                                   tr("节点直径（像素，8-500）:"),
                                                   view_->nodeDiameterPx(), 8.0, 500.0, 0, &ok);
        if (ok)
            view_->setNodeDiameterPx(v);
    });
    QAction *actEdgeWidth = settingsMenu->addAction(tr("边宽度…"));
    QObject::connect(actEdgeWidth, &QAction::triggered, this, [this]() {
        if (!view_) return;
        bool ok = false;
        const double v = QInputDialog::getDouble(this, tr("边宽度"),
                                                   tr("边线宽（像素，1-20）:"),
                                                   view_->edgeWidthPx(), 1.0, 20.0, 0, &ok);
        if (ok)
            view_->setEdgeWidthPx(v);
    });

    QMenu *moreMenu = mb->addMenu(tr("更多"));
    QAction *ph3 = moreMenu->addAction(tr("（功能待添加）"));
    ph3->setEnabled(false);

    // ---- 主工具栏 ----
    auto *tbMain = addToolBar(tr("主工具栏"));
    tbMain->setMovable(false);
    tbMain->addAction(actUndo_);
    tbMain->addAction(actRedo_);
    tbMain->addSeparator();

    // 工具栏堆栈：页面 0 = 缩放，页面 1 = 栅格形状+笔刷
    toolbarStack_ = new QStackedWidget;

    auto *zoomPage = new QWidget;
    auto *zoomLay = new QHBoxLayout(zoomPage);
    zoomLay->setContentsMargins(4, 2, 4, 2);
    zoomLay->setSpacing(6);
    auto *btnZoomOut = new QPushButton(tr("缩小"));
    auto *btnZoomIn = new QPushButton(tr("放大"));
    auto *btnZoomReset = new QPushButton(tr("重置缩放"));
    QObject::connect(btnZoomOut, &QPushButton::clicked, view_, &TopoEditorView::zoomOut);
    QObject::connect(btnZoomIn, &QPushButton::clicked, view_, &TopoEditorView::zoomIn);
    QObject::connect(btnZoomReset, &QPushButton::clicked, view_, &TopoEditorView::zoomReset);
    zoomLay->addWidget(btnZoomOut);
    zoomLay->addWidget(btnZoomIn);
    zoomLay->addWidget(btnZoomReset);
    zoomLay->addStretch();

    auto *rasterPage = new QWidget;
    auto *rasterLay = new QHBoxLayout(rasterPage);
    rasterLay->setContentsMargins(4, 2, 4, 2);
    rasterLay->setSpacing(6);

    rasterShapeGrp_ = new QButtonGroup(this);
    rasterShapeGrp_->setExclusive(true);
    auto addShapeBtn = [rasterLay, this](const QString &text, int id) -> QPushButton* {
        auto *b = new QPushButton(text);
        b->setCheckable(true);
        b->setMinimumWidth(28);
        b->setStyleSheet(QStringLiteral("QPushButton { padding: 2px 4px; }"));
        rasterShapeGrp_->addButton(b, id);
        rasterLay->addWidget(b);
        return b;
    };
    addShapeBtn(tr("单点"), 0)->setChecked(true);
    addShapeBtn(tr("直线"), 1);
    addShapeBtn(tr("矩形"), 2);
    addShapeBtn(tr("多边形"), 3);

    QObject::connect(rasterShapeGrp_, static_cast<void (QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked),
                     this, &MainWindow::updateRasterShape);

    // 填充复选框（橡皮时不显示，单点/直线时不显示）
    rasterFillCheck_ = new QCheckBox(tr("填充"));
    QObject::connect(rasterFillCheck_, &QCheckBox::toggled, view_, &TopoEditorView::setRasterFill);
    rasterLay->addWidget(rasterFillCheck_);
    rasterFillCheck_->hide();

    brushLabel_ = new QLabel(tr("笔刷:2"));
    rasterLay->addWidget(brushLabel_);
    brushSlider_ = new QSlider(Qt::Horizontal);
    brushSlider_->setRange(1, 60);
    brushSlider_->setValue(2);
    brushSlider_->setFixedWidth(100);
    QObject::connect(brushSlider_, &QSlider::valueChanged, view_, [this](int v) {
        view_->setBrushRadius(v);
        brushLabel_->setText(tr("笔刷:%1").arg(v));
    });
    rasterLay->addWidget(brushSlider_);
    rasterLay->addStretch();

    toolbarStack_->addWidget(zoomPage);
    toolbarStack_->addWidget(rasterPage);
    toolbarStack_->setCurrentIndex(0);
    tbMain->addWidget(toolbarStack_);

    zoomPage->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    // ---- 连接 ----
    QObject::connect(leftToolGrp_, static_cast<void (QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked), this,
                     &MainWindow::updateLeftTool);
    QObject::connect(view_, &TopoEditorView::undoRedoAvailabilityChanged, this, &MainWindow::syncUndoRedoActions);

    auto *delSc = new QShortcut(QKeySequence::Delete, this);
    QObject::connect(delSc, &QShortcut::activated, view_, &TopoEditorView::deleteSelection);

    updateLeftTool(leftToolGrp_->checkedId());
    syncUndoRedoActions();
    syncMapConfigFromView();
    appendOperationLog(tr("就绪。可在下方查看操作记录。"));
}

void MainWindow::updateNavigationStatus(
    int state, float distanceRemaining, const QString &message)
{
    navigationStateLabel_->setText(navigationStateText(state));
    const QString color = state == 3 ? QStringLiteral("#16833b") :
                          state == 5 ? QStringLiteral("#c62828") :
                          state == 4 ? QStringLiteral("#d06a00") :
                          state == 1 ? QStringLiteral("#9a7200") :
                          state == 2 ? QStringLiteral("#1565c0") : QStringLiteral("#555555");
    navigationStateLabel_->setStyleSheet(
        QStringLiteral("font-weight: 700; color: %1;").arg(color));
    navigationDistanceLabel_->setText(
        distanceRemaining >= 0.0F ? tr("%1 m").arg(distanceRemaining, 0, 'f', 2)
                                  : QStringLiteral("-- m"));
    navigationMessageLabel_->setText(message.isEmpty() ? QStringLiteral("--") : message);
}

void MainWindow::onGoalTopicNamespace()
{
    if (!view_)
        return;

    bool ok = false;
    const QString namespaceText = QInputDialog::getText(
        this, tr("导航接口命名空间"),
        tr("请输入命名空间（例如 car1 或 /car1；留空使用根命名空间）："),
        QLineEdit::Normal, view_->goalTopicNamespace(), &ok);
    if (!ok)
        return;

    QString error;
    if (!view_->setGoalTopicNamespace(namespaceText, &error)) {
        QMessageBox::warning(this, tr("导航接口命名空间"), error);
        return;
    }

    QSettings settings;
    settings.setValue(QStringLiteral("goal_pose_namespace"), view_->goalTopicNamespace());
    appendOperationLog(tr("导航服务已设置为 %1；车辆位置订阅 %2")
                           .arg(view_->navigationRequestService(), view_->odomTopic()));
}

void MainWindow::syncUndoRedoActions()
{
    if (!view_) return;
    const bool isRaster = isRasterToolId(leftToolGrp_ ? leftToolGrp_->checkedId() : -1);
    if (actUndo_)
        actUndo_->setEnabled(isRaster ? view_->canUndoRaster() : view_->canUndo());
    if (actRedo_)
        actRedo_->setEnabled(isRaster ? view_->canRedoRaster() : view_->canRedo());
}

void MainWindow::syncMapConfigFromView()
{
    if (!view_ || !radioCoordLocal_ || !originLon_)
        return;
    suppressOriginSync_ = true;
    {
        QSignalBlocker b0(radioCoordLocal_);
        QSignalBlocker b1(radioCoordGeo_);
        if (view_->useLlaCoordinates())
            radioCoordGeo_->setChecked(true);
        else
            radioCoordLocal_->setChecked(true);
    }
    originPanel_->setVisible(view_->useLlaCoordinates());
    if (view_->useLlaCoordinates()) {
        QSignalBlocker bl(originLon_);
        QSignalBlocker bl2(originLat_);
        QSignalBlocker bl3(originAlt_);
        originLon_->setValue(view_->refLonDeg());
        originLat_->setValue(view_->refLatDeg());
        originAlt_->setValue(view_->refAltM());
    }
    suppressOriginSync_ = false;
    if (actTranslate_)
        actTranslate_->setEnabled(false);
    if (actRotate_)
        actRotate_->setEnabled(false);
    refreshToolLegend();
}

bool MainWindow::runOriginSetupDialog(double *outLatDeg, double *outLonDeg, double *outAltM)
{
    QDialog dlg(this);
    dlg.setWindowTitle(tr("请填写原点WGS84坐标"));
    auto *lonSpin = new QDoubleSpinBox(&dlg);
    lonSpin->setRange(-180.0, 180.0);
    lonSpin->setDecimals(8);
    auto *latSpin = new QDoubleSpinBox(&dlg);
    latSpin->setRange(-90.0, 90.0);
    latSpin->setDecimals(8);
    auto *altSpin = new QDoubleSpinBox(&dlg);
    altSpin->setRange(-1000.0, 10000.0);
    altSpin->setDecimals(3);
    lonSpin->setValue(view_->refLlaValid() ? view_->refLonDeg() : 0.0);
    latSpin->setValue(view_->refLlaValid() ? view_->refLatDeg() : 0.0);
    altSpin->setValue(view_->refLlaValid() ? view_->refAltM() : 0.0);
    auto *form = new QFormLayout;
    form->addRow(tr("经度"), lonSpin);
    form->addRow(tr("纬度"), latSpin);
    form->addRow(tr("高度"), altSpin);
    auto *bb = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QObject::connect(bb, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    QObject::connect(bb, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto *vl = new QVBoxLayout(&dlg);
    vl->addLayout(form);
    vl->addWidget(bb);
    if (dlg.exec() != QDialog::Accepted)
        return false;
    *outLatDeg = latSpin->value();
    *outLonDeg = lonSpin->value();
    *outAltM = altSpin->value();
    return true;
}

void MainWindow::onCoordButtonClicked(QAbstractButton *btn)
{
    if (!view_)
        return;
    if (btn == radioCoordLocal_) {
        const bool wasGeo = view_->useLlaCoordinates();
        view_->setUseLlaCoordinates(false);
        originPanel_->setVisible(false);
        refreshToolLegend();
        if (wasGeo)
            appendOperationLog(tr("地图配置：切换为局部坐标系"));
        return;
    }
    if (btn != radioCoordGeo_)
        return;

    if (view_->useLlaCoordinates()) {
        originPanel_->setVisible(true);
        refreshToolLegend();
        return;
    }

    if (!view_->refLlaValid()) {
        double la, lo, al;
        if (!runOriginSetupDialog(&la, &lo, &al)) {
            QSignalBlocker b0(radioCoordGeo_);
            QSignalBlocker b1(radioCoordLocal_);
            radioCoordLocal_->setChecked(true);
            radioCoordGeo_->setChecked(false);
            return;
        }
        view_->activateLlaModeWithOrigin(la, lo, al);
    } else {
        view_->resumeLlaModeWithStoredRef();
    }
    originPanel_->setVisible(true);
    syncMapConfigFromView();
}

void MainWindow::onOriginEdited()
{
    if (suppressOriginSync_ || !view_ || !view_->useLlaCoordinates())
        return;
    view_->commitReferenceAndRecomputeNodes(originLat_->value(), originLon_->value(), originAlt_->value());
}

void MainWindow::onFindJump()
{
    if (!view_ || !findNodeIdEdit_ || !findEdgeIdEdit_)
        return;
    const QString ns = findNodeIdEdit_->text().trimmed();
    const QString es = findEdgeIdEdit_->text().trimmed();
    if (ns.isEmpty() && es.isEmpty()) {
        resetLocateNavigation();
        QMessageBox::information(this, tr("视图定位"), tr("请至少填写节点 ID 或边 ID。"));
        return;
    }
    bool nodeOk = false;
    bool edgeOk = false;
    int nodeId = 0;
    int edgeId = 0;
    if (!ns.isEmpty()) {
        nodeOk = true;
        nodeId = ns.toInt(&nodeOk);
        if (!nodeOk) {
            QMessageBox::warning(this, tr("视图定位"), tr("节点 ID 不是有效整数。"));
            return;
        }
    }
    if (!es.isEmpty()) {
        edgeOk = true;
        edgeId = es.toInt(&edgeOk);
        if (!edgeOk) {
            QMessageBox::warning(this, tr("视图定位"), tr("边 ID 不是有效整数。"));
            return;
        }
    }
    bool ok = false;
    if (nodeOk && edgeOk)
        ok = view_->focusViewOnNodeAndEdge(nodeId, edgeId);
    else if (nodeOk)
        ok = view_->focusViewOnNodeId(nodeId);
    else
        ok = view_->focusViewOnEdgeId(edgeId);
    if (!ok) {
        QMessageBox::information(this, tr("视图定位"), tr("未找到该 ID（可能不在当前子图或不存在）。"));
        resetLocateNavigation();
        return;
    }
    if (nodeOk && edgeOk)
        appendOperationLog(tr("视图定位：节点 %1，边 %2").arg(nodeId).arg(edgeId));
    else if (nodeOk)
        appendOperationLog(tr("视图定位：节点 %1").arg(nodeId));
    else
        appendOperationLog(tr("视图定位：边 %1").arg(edgeId));

    if (nodeOk && edgeOk) {
        locateNavKind_ = LocateNavKind::Node;
        locateAnchorId_ = nodeId;
    } else if (nodeOk) {
        locateNavKind_ = LocateNavKind::Node;
        locateAnchorId_ = nodeId;
    } else {
        locateNavKind_ = LocateNavKind::Edge;
        locateAnchorId_ = edgeId;
    }
}

void MainWindow::onLocatePrev() { locateStep(-1); }
void MainWindow::onLocateNext() { locateStep(1); }

void MainWindow::locateStep(int delta)
{
    if (!view_ || !findNodeIdEdit_ || !findEdgeIdEdit_)
        return;
    if (locateNavKind_ == LocateNavKind::None) {
        QMessageBox::information(this, tr("视图定位"), tr("请先点击「跳转」定位到某一节点或边，再使用上一个/下一个。"));
        return;
    }
    int nextId = 0;
    if (locateNavKind_ == LocateNavKind::Node) {
        if (!view_->stepVisibleNodeId(locateAnchorId_, delta, &nextId)) {
            QMessageBox::information(this, tr("视图定位"), tr("无法切换节点（当前子图中没有可用节点，或锚点已失效）。"));
            resetLocateNavigation();
            return;
        }
        locateAnchorId_ = nextId;
        findNodeIdEdit_->setText(QString::number(nextId));
        if (!view_->focusViewOnNodeId(nextId)) {
            QMessageBox::information(this, tr("视图定位"), tr("无法对准该节点。"));
            resetLocateNavigation();
        } else {
            appendOperationLog(tr("视图定位：切换到节点 %1").arg(nextId));
        }
        return;
    }
    if (!view_->stepVisibleEdgeId(locateAnchorId_, delta, &nextId)) {
        QMessageBox::information(this, tr("视图定位"), tr("无法切换边（当前子图中没有可用边，或锚点已失效）。"));
        resetLocateNavigation();
        return;
    }
    locateAnchorId_ = nextId;
    findEdgeIdEdit_->setText(QString::number(nextId));
    if (!view_->focusViewOnEdgeId(nextId)) {
        QMessageBox::information(this, tr("视图定位"), tr("无法对准该边。"));
        resetLocateNavigation();
    } else {
        appendOperationLog(tr("视图定位：切换到边 %1").arg(nextId));
    }
}

void MainWindow::resetLocateNavigation()
{
    locateNavKind_ = LocateNavKind::None;
    locateAnchorId_ = 0;
}

void MainWindow::updateLeftTool(int id)
{
    switch (id) {
    case kToolSelect:   view_->setTool(TopoEditorView::Tool::Select); break;
    case kToolAddNode:  view_->setTool(TopoEditorView::Tool::AddNode); break;
    case kToolAddEdge:  view_->setTool(TopoEditorView::Tool::AddEdge); break;
    case kToolKeepout:  view_->setTool(TopoEditorView::Tool::KeepoutPoly); break;
    case kToolPassable: view_->setTool(TopoEditorView::Tool::PassablePoly); break;
    case kToolMeasure:  view_->setTool(TopoEditorView::Tool::Measure); break;
    case kToolRasterPaint:  view_->setTool(TopoEditorView::Tool::RasterPaint); break;
    case kToolRasterErase:  view_->setTool(TopoEditorView::Tool::RasterErase); break;
    case kToolRasterUnscan: view_->setTool(TopoEditorView::Tool::RasterUnscan); break;
    case kToolRasterMask:   view_->setTool(TopoEditorView::Tool::RasterMask); break;
    case kToolPlanPath:  view_->setTool(TopoEditorView::Tool::PlanPath); break;
    default: break;
    }

    // 切换工具栏页面
    if (toolbarStack_) {
        toolbarStack_->setCurrentIndex(isRasterToolId(id) ? 1 : 0);
    }

    syncRasterFillVisibility();
    refreshToolLegend();
    syncUndoRedoActions();
}

void MainWindow::updateRasterShape(int id)
{
    switch (id) {
    case 0: view_->setRasterShape(TopoEditorView::RasterShape::Point); break;
    case 1: view_->setRasterShape(TopoEditorView::RasterShape::Line); break;
    case 2: view_->setRasterShape(TopoEditorView::RasterShape::Rect); break;
    case 3: view_->setRasterShape(TopoEditorView::RasterShape::Polygon); break;
    }
    syncRasterFillVisibility();
    refreshToolLegend();
}

void MainWindow::refreshToolLegend()
{
    if (!legendLabel_ || !leftToolGrp_)
        return;

    const int lid = leftToolGrp_->checkedId();
    const QString common =
        tr("\n绿色点表示 %1 的最新车辆位置。\n%2 / %3 撤销/重做；Shift+滚轮缩放视图；Delete 删除选中节点或边。")
            .arg(view_->odomTopic(),
                 QKeySequence(QKeySequence::Undo).toString(QKeySequence::NativeText),
                 QKeySequence(QKeySequence::Redo).toString(QKeySequence::NativeText));

    if (isRasterToolId(lid)) {
        QString shapeName;
        switch (view_->rasterShape()) {
        case TopoEditorView::RasterShape::Point:    shapeName = tr("单点（左键点击即绘）"); break;
        case TopoEditorView::RasterShape::Line:     shapeName = tr("直线（左键拖拽）"); break;
        case TopoEditorView::RasterShape::Rect:     shapeName = tr("矩形（左键拖拽，%1）").arg(view_->rasterFill() ? tr("填充") : tr("轮廓")); break;
        case TopoEditorView::RasterShape::Polygon:  shapeName = tr("多边形（左键加点，右键撤销，中键提交，%1）").arg(view_->rasterFill() ? tr("填充") : tr("轮廓")); break;
        }
        QString toolName;
        switch (lid) {
        case kToolRasterPaint:  toolName = tr("障碍区"); break;
        case kToolRasterErase:  toolName = tr("橡皮"); break;
        case kToolRasterUnscan: toolName = tr("未知区"); break;
        case kToolRasterMask:   toolName = tr("禁区"); break;
        }
        legendLabel_->setText(
            tr("【%1】%2\n"
               "圆形光标代表笔刷大小。\n"
               "%3")
                .arg(toolName, shapeName, common));
        return;
    }

    switch (lid) {
    case kToolSelect:
        legendLabel_->setText(tr("【选择】左键短点击点选，左键拖动画布平移；右键拖动框选。\n"
                                 "在节点或边上右键短点击可打开菜单（编辑属性、删除）。\n"
                                 "节点可在右键菜单进入拖动编辑，框选多个节点后可整体拖动；方向键每次移动 0.1 米。\n"
                                 "拖动后点击 ✓ 确认或 × 取消，确认后仍需保存路网。\n"
                                 "右侧「视图定位」可按 ID 跳转。\n"
                                 "%1").arg(common));
        return;
    case kToolAddNode:
        legendLabel_->setText(
            view_->useLlaCoordinates()
                ? tr("【添加节点】左键放置节点。经纬度模式下可在节点编辑里填经纬高。\n%1").arg(common)
                : tr("【添加节点】左键在空白处放置节点（世界坐标，米）。\n%1").arg(common));
        return;
    case kToolAddEdge:
        legendLabel_->setText(tr("【添加边】左键点第一个节点后虚线跟随光标；右键取消重选；再左键点另一节点完成连接。\n%1").arg(common));
        return;
    case kToolKeepout:
        legendLabel_->setText(tr("【禁区】左键添加顶点；右键撤销上一个点；中键闭合（至少三点）。\n%1").arg(common));
        return;
    case kToolPassable:
        legendLabel_->setText(tr("【可行区】操作同禁区，用于标出可通行区域。\n%1").arg(common));
        return;
    case kToolMeasure:
        legendLabel_->setText(tr("【测距】左键按下并拖曳至另一点后松开，显示两点间直线距离（米）；约 5 秒后清除。\n%1").arg(common));
        return;
    case kToolPlanPath:
        legendLabel_->setText(tr("【路径规划】依次左键点击两个节点：第一个为起点（绿色），第二个为终点（红色）。\n"
                                 "选中后自动运行显式计算输入电流、膜电位与阈值放电的 SNN 波前传播，路径以橙色高亮显示。\n"
                                 "右键清除选择。\n"
                                 "%1").arg(common));
        return;
    default:
        legendLabel_->setText(tr("请选择左侧工具。"));
        return;
    }
}

// ---- 文件操作 ----

void MainWindow::onOpen()
{
    const QString path = QFileDialog::getOpenFileName(this, tr("打开路网"), {},
                                                      tr("工程 (*.topo *.txt *.yaml *.yml);;All (*)"));
    if (path.isEmpty())
        return;
    QString err;
    if (!view_->loadProject(path, &err)) {
        QMessageBox::warning(this, tr("打开失败"), err.isEmpty() ? tr("未知错误") : err);
        appendOperationLog(tr("打开失败：%1").arg(err.isEmpty() ? tr("未知错误") : err));
    } else {
        view_->setActiveMapId(-1);
        resetLocateNavigation();
    }
}

bool MainWindow::confirmSaveDespiteDisconnectedTopo()
{
    if (!view_ || view_->isUndirectedTopoConnected())
        return true;
    const int k = view_->undirectedTopoComponentCount();
    const auto r = QMessageBox::question(
        this, tr("路网未全连通"),
        tr("当前路网把边视为无向边后，共有 %1 个互不相通的区块（连通分量）。\n\n是否仍要保存？").arg(k),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    return r == QMessageBox::Yes;
}

void MainWindow::onSave()
{
    if (view_->currentPath().isEmpty()) {
        onSaveAs();
        return;
    }
    if (!confirmSaveDespiteDisconnectedTopo())
        return;
    QString err;
    if (!view_->saveProject(view_->currentPath(), &err)) {
        QMessageBox::warning(this, tr("保存失败"), err);
        appendOperationLog(tr("保存失败：%1").arg(err));
    }
}

void MainWindow::onSaveAs()
{
    QString path = QFileDialog::getSaveFileName(this, tr("路网另存为"), view_->currentPath(),
                                                tr("YAML 工程 (*.yaml *.yml);;Topo 文本 (*.topo);;文本 (*.txt);;All (*)"));
    if (path.isEmpty())
        return;
    const QString pl = path.toLower();
    const bool knownExt = pl.endsWith(QLatin1String(".yaml")) || pl.endsWith(QLatin1String(".yml"))
                          || pl.endsWith(QLatin1String(".topo")) || pl.endsWith(QLatin1String(".txt"));
    if (!knownExt)
        path += QLatin1String(".yaml");
    if (!confirmSaveDespiteDisconnectedTopo())
        return;
    QString err;
    if (!view_->saveProject(path, &err)) {
        QMessageBox::warning(this, tr("保存失败"), err);
        appendOperationLog(tr("保存失败：%1").arg(err));
    }
}

void MainWindow::onOpenFolder()
{
    const QString dir = QFileDialog::getExistingDirectory(this, tr("打开文件夹"));
    if (dir.isEmpty())
        return;

    const QDir d(dir);
    int loaded = 0;

    // 栅格地图: map.yaml
    const QString rasterYaml = d.filePath(QStringLiteral("map.yaml"));
    if (QFile::exists(rasterYaml)) {
        QString err;
        if (view_->loadRasterMap(rasterYaml, &err)) {
            loaded++;
        } else {
            appendOperationLog(tr("栅格地图加载失败: %1").arg(err));
        }
    }

    // 路网地图: net.yaml
    const QString netYaml = d.filePath(QStringLiteral("net.yaml"));
    if (QFile::exists(netYaml)) {
        QString err;
        if (view_->loadProject(netYaml, &err)) {
            view_->setActiveMapId(-1);
            loaded++;
        } else {
            appendOperationLog(tr("路网加载失败: %1").arg(err));
        }
    }

    resetLocateNavigation();

    if (loaded == 0)
        appendOperationLog(tr("文件夹中未找到 map.yaml 或 net.yaml: %1").arg(dir));
    else
        appendOperationLog(tr("从文件夹加载完成: %1（%2 项）").arg(dir).arg(loaded));
}

void MainWindow::onOpenRasterMap()
{
    const QString path = QFileDialog::getOpenFileName(this, tr("打开栅格地图 YAML"), {},
                                                      tr("YAML (*.yaml *.yml);;All (*)"));
    if (path.isEmpty())
        return;
    QString err;
    if (!view_->loadRasterMap(path, &err)) {
        QMessageBox::warning(this, tr("打开栅格地图失败"), err.isEmpty() ? tr("未知错误") : err);
        appendOperationLog(tr("打开栅格地图失败：%1").arg(err.isEmpty() ? tr("未知错误") : err));
    }
}

void MainWindow::onSaveRasterMap()
{
    if (!view_->hasRasterMap()) {
        QMessageBox::information(this, tr("保存栅格地图"), tr("没有已加载的栅格地图。"));
        return;
    }
    const QString dir = QFileDialog::getExistingDirectory(this, tr("选择保存目录"));
    if (dir.isEmpty())
        return;
    bool ok = false;
    const QString pgmName = QInputDialog::getText(this, tr("PGM 文件名"), tr("请输入 PGM 文件名:"),
                                                   QLineEdit::Normal, QStringLiteral("map.pgm"), &ok);
    if (!ok || pgmName.isEmpty())
        return;
    QString err;
    if (!view_->saveRasterMap(dir, pgmName, &err)) {
        QMessageBox::warning(this, tr("保存失败"), err);
        appendOperationLog(tr("保存栅格地图失败：%1").arg(err));
    }
}

void MainWindow::onSaveMask()
{
    if (!view_->hasRasterMap()) {
        QMessageBox::information(this, tr("保存禁区"), tr("没有已加载的栅格地图。"));
        return;
    }
    const QString dir = QFileDialog::getExistingDirectory(this, tr("选择保存目录"));
    if (dir.isEmpty())
        return;
    bool ok = false;
    const QString pgmName = QInputDialog::getText(this, tr("PGM 文件名"), tr("请输入禁区 PGM 文件名:"),
                                                   QLineEdit::Normal, QStringLiteral("keepout.pgm"), &ok);
    if (!ok || pgmName.isEmpty())
        return;
    QString err;
    if (!view_->saveMaskMap(dir, pgmName, &err)) {
        QMessageBox::warning(this, tr("保存失败"), err);
        appendOperationLog(tr("保存禁区失败：%1").arg(err));
    }
}

void MainWindow::onRasterStateChanged()
{
    if (yamlPreview_ && view_) {
        const RosMapYaml y = view_->rasterYaml();
        yamlPreview_->setPlainText(y.rawText.isEmpty()
                                       ? tr("（无栅格数据）")
                                       : y.rawText);
    }
    syncMapConfigFromView();
}

void MainWindow::syncRasterFillVisibility()
{
    if (!rasterFillCheck_ || !view_)
        return;
    const int lid = leftToolGrp_ ? leftToolGrp_->checkedId() : -1;
    const bool isEraser = (lid == kToolRasterErase);
    const auto shape = view_->rasterShape();
    const bool isAreaShape = (shape == TopoEditorView::RasterShape::Rect ||
                               shape == TopoEditorView::RasterShape::Polygon);
    // 橡皮时隐藏（自动填充），单点/直线时隐藏，其他工具+矩形/多边形时显示
    const bool show = !isEraser && isAreaShape;
    rasterFillCheck_->setVisible(show);
    if (!show)
        view_->setRasterFill(false);
}

void MainWindow::appendOperationLog(const QString &message)
{
    if (!opLogEdit_)
        return;
    const QString line =
        QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss ")) + message;
    opLogEdit_->appendPlainText(line);
    QTextCursor c = opLogEdit_->textCursor();
    c.movePosition(QTextCursor::End);
    opLogEdit_->setTextCursor(c);
}
