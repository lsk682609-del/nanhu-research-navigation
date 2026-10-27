/**
******************************************************************************
*
* @file       pgm_io.cpp
* @author     Wei Wang -> shaxikai@outlook.com
* @date       2026.4.10
* @version    V1.0.0
* @brief      PGM 灰度图解析与二进制 P5 编码实现。
*
*****************************************************************************/

#include "pgm_io.hpp"

#include <QRegularExpression>

#include <cctype>
#include <cstring>

namespace {

QString nextToken(const QByteArray &bytes, int &i)
{
    while (i < bytes.size() && std::isspace(static_cast<unsigned char>(bytes[i])))
        ++i;
    if (i >= bytes.size())
        return {};
    if (bytes[i] == '#') {
        while (i < bytes.size() && bytes[i] != '\n')
            ++i;
        return nextToken(bytes, i);
    }
    const int start = i;
    while (i < bytes.size()) {
        const unsigned char c = static_cast<unsigned char>(bytes[i]);
        if (std::isspace(c) || c == '#')
            break;
        ++i;
        if (i - start > 256) {
            i = start;
            return {};
        }
    }
    return QString::fromLatin1(bytes.constData() + start, i - start);
}

bool parseHeaderTokens(const QByteArray &uint8, QString &magic, int &w, int &h, int &maxv, int &dataStart,
                       QString *errorMsg)
{
    int i = 0;
    QString t = nextToken(uint8, i);
    if (t != QLatin1String("P5") && t != QLatin1String("P2")) {
        if (errorMsg)
            *errorMsg = QObject::tr("Unsupported PGM magic (need P5 or P2)");
        return false;
    }
    magic = t;
    t = nextToken(uint8, i);
    w = t.toInt();
    t = nextToken(uint8, i);
    h = t.toInt();
    t = nextToken(uint8, i);
    maxv = t.toInt();
    if (!(w > 0 && h > 0 && maxv > 0)) {
        if (errorMsg)
            *errorMsg = QObject::tr("Invalid PGM dimensions or maxval");
        return false;
    }
    while (i < uint8.size() && std::isspace(static_cast<unsigned char>(uint8[i])))
        ++i;
    dataStart = i;
    return true;
}

} // namespace

bool parsePgm(const QByteArray &raw, PgmImage &out, QString *errorMsg)
{
    QString magic;
    int w = 0, h = 0, maxv = 0, dataStart = 0;
    if (!parseHeaderTokens(raw, magic, w, h, maxv, dataStart, errorMsg))
        return false;

    out.magic[0] = magic.size() > 0 ? magic[0].toLatin1() : 'P';
    out.magic[1] = magic.size() > 1 ? magic[1].toLatin1() : '5';
    out.magic[2] = '\0';
    out.width = w;
    out.height = h;
    out.maxval = maxv;
    out.pixels.resize(static_cast<size_t>(w * h));

    if (magic == QLatin1String("P2")) {
        const QString rest = QString::fromUtf8(raw);
        static const QRegularExpression headerRe(
            QStringLiteral(R"(^\s*P2[\s\S]*?\b%1\b)").arg(maxv));
        const QRegularExpressionMatch m = headerRe.match(rest);
        const int start = m.hasMatch() ? m.captured(0).length() : 0;
        static const QRegularExpression numRe(QStringLiteral(R"(\d+)"));
        int pos = start;
        int idx = 0;
        while (idx < w * h) {
            const QRegularExpressionMatch nm = numRe.match(rest, pos);
            if (!nm.hasMatch()) {
                if (errorMsg)
                    *errorMsg = QObject::tr("P2 data too short");
                return false;
            }
            out.pixels[static_cast<size_t>(idx)] =
                static_cast<quint16>(qBound(0, nm.captured(0).toInt(), maxv));
            ++idx;
            pos = nm.capturedEnd(0);
        }
        return true;
    }

    const int bytesPer = maxv > 255 ? 2 : 1;
    const qint64 needed = static_cast<qint64>(w) * h * bytesPer;
    if (dataStart + needed > raw.size()) {
        if (errorMsg)
            *errorMsg = QObject::tr("P5 raster data too short");
        return false;
    }

    if (bytesPer == 1) {
        for (int p = 0; p < w * h; ++p)
            out.pixels[static_cast<size_t>(p)] =
                static_cast<quint8>(raw[dataStart + p]);
    } else {
        int di = dataStart;
        for (int p = 0; p < w * h; ++p) {
            const quint16 v =
                (static_cast<quint16>(static_cast<quint8>(raw[di])) << 8)
                | static_cast<quint8>(raw[di + 1]);
            out.pixels[static_cast<size_t>(p)] = v;
            di += 2;
        }
    }
    return true;
}

QByteArray encodePgmP5(const PgmImage &pgm)
{
    const int w = pgm.width;
    const int h = pgm.height;
    const int maxv = pgm.maxval;
    const QByteArray header = QByteArray("P5\n")
        + QByteArray::number(w) + ' ' + QByteArray::number(h) + '\n'
        + QByteArray::number(maxv) + '\n';

    const int bytesPer = maxv > 255 ? 2 : 1;
    QByteArray body(int(static_cast<qint64>(w) * h * bytesPer), Qt::Uninitialized);
    if (bytesPer == 1) {
        for (int i = 0; i < w * h; ++i)
            body[i] = static_cast<char>(qMin(255, static_cast<int>(pgm.pixels[static_cast<size_t>(i)])));
    } else {
        int j = 0;
        for (int i = 0; i < w * h; ++i) {
            const quint16 v = pgm.pixels[static_cast<size_t>(i)];
            body[j++] = static_cast<char>((v >> 8) & 0xFF);
            body[j++] = static_cast<char>(v & 0xFF);
        }
    }
    return header + body;
}
