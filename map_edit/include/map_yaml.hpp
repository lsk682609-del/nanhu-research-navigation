/**
******************************************************************************
*
* @file       map_yaml.hpp
* @author     Wei Wang -> shaxikai@outlook.com
* @date       2026.4.10
* @version    V1.0.0
* @brief      ROS 地图 YAML（resolution/origin/image 等）数据结构声明。
*
*****************************************************************************/

#pragma once

#include <QString>
#include <QVector>

struct RosMapYaml {
    double resolution = 0.0;
    QVector<double> origin; // [x,y,yaw]
    int negate = 0;
    bool hasNegate = false;
    QString image;
    QString rawText;

    bool hasResolution() const { return resolution > 0.0; }
};

bool parseRosMapYaml(const QString &text, RosMapYaml &out, QString *errorMsg = nullptr);
QString serializeRosMapYaml(const RosMapYaml &base, const QString &imageFilename);

bool looksLikeTopoYamlText(const QString &text);
