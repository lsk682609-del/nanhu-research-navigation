/**
******************************************************************************
*
* @file       pgm_io.hpp
* @author     Wei Wang -> shaxikai@outlook.com
* @date       2026.4.10
* @version    V1.0.0
* @brief      PGM 图像数据结构及解析/编码接口声明。
*
*****************************************************************************/

#pragma once

#include <QByteArray>
#include <QString>
#include <vector>

struct PgmImage {
    char magic[3] = {'P', '5', '\0'};
    int width = 0;
    int height = 0;
    int maxval = 255;
    std::vector<quint16> pixels;

    int pixelCount() const { return width * height; }
};

bool parsePgm(const QByteArray &raw, PgmImage &out, QString *errorMsg = nullptr);
QByteArray encodePgmP5(const PgmImage &pgm);
