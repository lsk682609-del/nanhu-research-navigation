/**
******************************************************************************
*
* @file       region_io.hpp
* @author     Wei Wang -> shaxikai@outlook.com
* @date       2026.4.10
* @version    V1.0.0
* @brief      工程文本中 keepout/passable 区域段解析与序列化。
*
*****************************************************************************/

#pragma once

#include <QPolygonF>
#include <QString>
#include <QVector>

struct RegionPolygon {
    int id = 0;
    int mapId = 0;
    bool keepout = true; // false = 可行区域
    QPolygonF points;    // 世界坐标 (米)，与路网节点坐标系一致
};

/// 从完整工程文本中拆出标准 topo 段与区域列表（keepout:/passable: 段）
bool splitTopoProjectText(const QString &fullText, QString *outTopoBody, QVector<RegionPolygon> *outRegions,
                          QString *errorMsg = nullptr);

/// 将区域序列化为追加文本（含 keepout:/passable: 段）
QString serializeRegionSections(const QVector<RegionPolygon> &regions);
