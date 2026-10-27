/**
******************************************************************************
*
* @file       region_io.cpp
* @author     Wei Wang -> shaxikai@outlook.com
* @date       2026.4.10
* @version    V1.0.0
* @brief      区域多边形文本解析与序列化实现。
*
*****************************************************************************/

#include "region_io.hpp"

#include <QStringList>

#include <sstream>
#include <string>

namespace {

QString trimLine(const QString &s)
{
    return s.trimmed();
}

bool parseRegionLine(const QString &line, bool keepout, RegionPolygon *out, QString *err)
{
    *out = RegionPolygon{};
    const QString t = trimLine(line);
    if (t.isEmpty() || t.startsWith(QLatin1Char('#')))
        return true;
    std::istringstream iss(t.toStdString());
    int id = 0;
    int mapId = 0;
    int n = 0;
    iss >> id >> mapId >> n;
    if (!iss || n < 3) {
        if (err)
            *err = QStringLiteral("无效的区域行: %1").arg(line);
        return false;
    }
    QPolygonF poly;
    poly.reserve(n);
    for (int i = 0; i < n; ++i) {
        double x = 0, y = 0;
        iss >> x >> y;
        if (!iss) {
            if (err)
                *err = QStringLiteral("区域顶点数量不足: %1").arg(line);
            return false;
        }
        poly.append(QPointF(x, y));
    }
    out->id = id;
    out->mapId = mapId;
    out->keepout = keepout;
    out->points = poly;
    return true;
}

} // namespace

bool splitTopoProjectText(const QString &fullText, QString *outTopoBody, QVector<RegionPolygon> *outRegions,
                          QString *errorMsg)
{
    if (!outTopoBody || !outRegions)
        return false;
    outRegions->clear();

    const QStringList lines = fullText.split(QLatin1Char('\n'));
    QString topoAccum;
    enum Section { Topo, Keepout, Passable };
    Section sec = Topo;

    for (const QString &raw : lines) {
        const QString t = trimLine(raw);
        if (t == QLatin1String("keepout:")) {
            sec = Keepout;
            continue;
        }
        if (t == QLatin1String("passable:")) {
            sec = Passable;
            continue;
        }
        if (sec == Topo) {
            topoAccum.append(raw);
            topoAccum.append(QLatin1Char('\n'));
        } else if (sec == Keepout) {
            RegionPolygon r;
            if (!parseRegionLine(raw, true, &r, errorMsg))
                return false;
            if (!r.points.isEmpty())
                outRegions->append(r);
        } else {
            RegionPolygon r;
            if (!parseRegionLine(raw, false, &r, errorMsg))
                return false;
            if (!r.points.isEmpty())
                outRegions->append(r);
        }
    }

    *outTopoBody = topoAccum;
    return true;
}

QString serializeRegionSections(const QVector<RegionPolygon> &regions)
{
    QString s;
    QVector<const RegionPolygon *> ko, pa;
    for (const RegionPolygon &r : regions) {
        if (r.keepout)
            ko.append(&r);
        else
            pa.append(&r);
    }
    if (!ko.isEmpty()) {
        s += QLatin1String("keepout:\n");
        for (const RegionPolygon *r : ko) {
            s += QString::number(r->id);
            s += QLatin1Char(' ');
            s += QString::number(r->mapId);
            s += QLatin1Char(' ');
            s += QString::number(r->points.size());
            for (const QPointF &p : r->points) {
                s += QLatin1Char(' ');
                s += QString::number(p.x(), 'g', 9);
                s += QLatin1Char(' ');
                s += QString::number(p.y(), 'g', 9);
            }
            s += QLatin1Char('\n');
        }
    }
    if (!pa.isEmpty()) {
        s += QLatin1String("passable:\n");
        for (const RegionPolygon *r : pa) {
            s += QString::number(r->id);
            s += QLatin1Char(' ');
            s += QString::number(r->mapId);
            s += QLatin1Char(' ');
            s += QString::number(r->points.size());
            for (const QPointF &p : r->points) {
                s += QLatin1Char(' ');
                s += QString::number(p.x(), 'g', 9);
                s += QLatin1Char(' ');
                s += QString::number(p.y(), 'g', 9);
            }
            s += QLatin1Char('\n');
        }
    }
    return s;
}
