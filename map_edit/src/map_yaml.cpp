/**
******************************************************************************
*
* @file       map_yaml.cpp
* @author     Wei Wang -> shaxikai@outlook.com
* @date       2026.4.10
* @version    V1.0.0
* @brief      ROS 地图 YAML 解析与序列化及路网 YAML 文本判别辅助。
*
*****************************************************************************/

#include "map_yaml.hpp"

#include <QRegularExpression>

bool looksLikeTopoYamlText(const QString &text)
{
    static const QRegularExpression nodeSec(QStringLiteral(R"((^|\r?\n)\s*node:\s*(\r?\n|$))"),
                                            QRegularExpression::MultilineOption);
    if (!text.contains(nodeSec))
        return false;
    const QString head = text.left(4000);
    static const QRegularExpression imageLine(QStringLiteral(R"(^\s*image\s*:\s*\S+)"),
                                                QRegularExpression::MultilineOption);
    if (imageLine.match(head).hasMatch())
        return false;
    return true;
}

static bool parseOriginList(const QString &inside, QVector<double> &out)
{
    out.clear();
    static const QRegularExpression num(QStringLiteral(R"([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)"));
    int pos = 0;
    while (true) {
        const QRegularExpressionMatch m = num.match(inside, pos);
        if (!m.hasMatch())
            break;
        out.append(m.captured(0).toDouble());
        pos = m.capturedEnd(0);
    }
    return out.size() >= 2;
}

bool parseRosMapYaml(const QString &text, RosMapYaml &out, QString *errorMsg)
{
    Q_UNUSED(errorMsg);
    out = RosMapYaml{};
    out.rawText = text;

    static const QRegularExpression resRe(
        QStringLiteral(R"(^\s*resolution\s*:\s*([0-9.eE+-]+)\s*$)"),
        QRegularExpression::MultilineOption);
    {
        const auto m = resRe.match(text);
        if (m.hasMatch())
            out.resolution = m.captured(1).toDouble();
    }

    static const QRegularExpression originRe(
        QStringLiteral(R"(^\s*origin\s*:\s*\[([^\]]*)\]\s*$)"),
        QRegularExpression::MultilineOption);
    {
        const auto m = originRe.match(text);
        if (m.hasMatch())
            parseOriginList(m.captured(1), out.origin);
    }

    static const QRegularExpression negRe(
        QStringLiteral(R"(^\s*negate\s*:\s*([01])\s*$)"),
        QRegularExpression::MultilineOption);
    {
        const auto m = negRe.match(text);
        if (m.hasMatch()) {
            out.negate = m.captured(1).toInt();
            out.hasNegate = true;
        }
    }

    static const QRegularExpression imgRe(
        QStringLiteral(R"(^\s*image\s*:\s*(\S+)\s*$)"),
        QRegularExpression::MultilineOption);
    {
        const auto m = imgRe.match(text);
        if (m.hasMatch())
            out.image = m.captured(1).trimmed();
    }

    return true;
}

QString serializeRosMapYaml(const RosMapYaml &base, const QString &imageFilename)
{
    const double res = base.hasResolution() ? base.resolution : 0.05;
    QString s;
    s += QStringLiteral("image: %1\n").arg(imageFilename);
    s += QStringLiteral("resolution: %1\n").arg(res, 0, 'g', 17);
    s += QLatin1String("origin: [");
    for (int i = 0; i < base.origin.size(); ++i) {
        if (i)
            s += QLatin1String(", ");
        s += QString::number(base.origin[i], 'g', 17);
    }
    if (base.origin.size() < 3) {
        while (base.origin.size() < 3) {
            if (!s.endsWith(QLatin1Char('[')))
                s += QLatin1String(", ");
            s += QLatin1Char('0');
        }
    }
    s += QLatin1String("]\n");
    if (base.hasNegate)
        s += QStringLiteral("negate: %1\n").arg(base.negate);
    return s;
}
