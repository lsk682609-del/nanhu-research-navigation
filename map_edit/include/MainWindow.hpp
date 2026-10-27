/**
******************************************************************************
*
* @file       MainWindow.hpp
* @author     Wei Wang -> shaxikai@outlook.com
* @date       2026.4.10
* @version    V1.0.0
* @brief      地图编辑器主窗口：菜单、侧栏工具、地图配置、栅格文件操作与操作日志。
*
*****************************************************************************/

#pragma once

#include <QMainWindow>

class TopoEditorView;
class QLabel;
class QButtonGroup;
class QAction;
class QCheckBox;
class QGroupBox;
class QRadioButton;
class QDoubleSpinBox;
class QLineEdit;
class QPlainTextEdit;
class QSlider;
class QWidget;
class QAbstractButton;
class QStackedWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);

private slots:
    void onOpen();
    void onSave();
    void onSaveAs();
    void onOpenFolder();
    void onOpenRasterMap();
    void onSaveRasterMap();
    void onSaveMask();
    void onGoalTopicNamespace();
    void updateLeftTool(int id);
    void updateRasterShape(int id);
    void refreshToolLegend();
    void syncUndoRedoActions();
    void syncMapConfigFromView();
    void onCoordButtonClicked(QAbstractButton *btn);
    void onOriginEdited();
    void onFindJump();
    void onLocatePrev();
    void onLocateNext();
    void appendOperationLog(const QString &message);
    void onRasterStateChanged();
    void syncRasterFillVisibility();
    void updateNavigationStatus(int state, float distanceRemaining, const QString &message);

private:
    enum class LocateNavKind { None, Node, Edge };

    bool runOriginSetupDialog(double *outLatDeg, double *outLonDeg, double *outAltM);
    void locateStep(int delta);
    void resetLocateNavigation();
    bool confirmSaveDespiteDisconnectedTopo();

    TopoEditorView *view_ = nullptr;
    QLabel *legendLabel_ = nullptr;
    QButtonGroup *leftToolGrp_ = nullptr;
    QButtonGroup *rasterShapeGrp_ = nullptr;
    QSlider *brushSlider_ = nullptr;
    QLabel *brushLabel_ = nullptr;
    QCheckBox *rasterFillCheck_ = nullptr;
    QAction *actUndo_ = nullptr;
    QAction *actRedo_ = nullptr;
    QStackedWidget *toolbarStack_ = nullptr;

    QGroupBox *mapConfigGroup_ = nullptr;
    QGroupBox *viewLocateGroup_ = nullptr;
    QGroupBox *helpGroup_ = nullptr;
    QButtonGroup *coordButtonGroup_ = nullptr;
    QRadioButton *radioCoordLocal_ = nullptr;
    QRadioButton *radioCoordGeo_ = nullptr;
    QWidget *originPanel_ = nullptr;
    QDoubleSpinBox *originLon_ = nullptr;
    QDoubleSpinBox *originLat_ = nullptr;
    QDoubleSpinBox *originAlt_ = nullptr;
    bool suppressOriginSync_ = false;

    QLineEdit *findNodeIdEdit_ = nullptr;
    QLineEdit *findEdgeIdEdit_ = nullptr;
    QPlainTextEdit *opLogEdit_ = nullptr;
    QPlainTextEdit *yamlPreview_ = nullptr;
    QLabel *navigationStateLabel_ = nullptr;
    QLabel *navigationDistanceLabel_ = nullptr;
    QLabel *navigationMessageLabel_ = nullptr;

    QAction *actShowRaster_ = nullptr;
    QAction *actShowMask_ = nullptr;
    QAction *actTranslate_ = nullptr;
    QAction *actRotate_ = nullptr;
    QMenu *subMapMenu_ = nullptr;

    LocateNavKind locateNavKind_ = LocateNavKind::None;
    int locateAnchorId_ = 0;
};
