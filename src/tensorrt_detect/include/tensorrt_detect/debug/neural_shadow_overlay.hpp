#pragma once
// Qt-only display adapter. Never linked into MapAnalyzer/Tracker or a ROS component.
#include <QJsonDocument>
#include <QJsonObject>
#include <QPainter>
#include <QPixmap>
#include <QString>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

namespace neural_shadow {
using Clock = std::chrono::steady_clock;
struct Suggestion {
    bool enabled = false, valid = false, flip_team = false;
    double world_x = 0, world_z = 0, ego_x = 0, ego_z = 0;
    double field_x = 0, field_y = 0, move_probability = 0, source_stamp = 0, ttl = 1.5;
    int robot_id = 0;
    QString action, candidate, reason = "Waiting for shadow node";
    Clock::time_point received = Clock::now();
};

inline Suggestion parse(const std::string& text) noexcept {
    Suggestion s; s.enabled = true; s.reason = "Invalid shadow message";
    try {
        if (text.size() > 4096) return s;
        QJsonParseError error;
        const auto document = QJsonDocument::fromJson(
            QByteArray(text.data(), static_cast<int>(text.size())), &error);
        if (error.error != QJsonParseError::NoError || !document.isObject()) return s;
        const auto o = document.object();
        if (o.value("version").toInt() != 1 || !o.value("shadow_only").toBool()) return s;
        if (!o.value("valid").toBool()) {
            s.reason = o.value("reason").toString("Unavailable").left(100);
            return s;
        }
        for (const auto key : {"world_x","world_z","ego_world_x","ego_world_z","field_x",
                               "field_y","source_stamp","ttl_s","move_probability"}) {
            if (!o.value(key).isDouble() || !std::isfinite(o.value(key).toDouble())) return s;
        }
        if (!o.value("flip_team").isBool()) return s;
        s.flip_team = o.value("flip_team").toBool();
        s.robot_id = o.value("robot_id").toInt();
        if (s.robot_id != (s.flip_team ? 7 : 107) ||
            o.value("team_id").toInt() != (s.flip_team ? 1 : 2)) return s;
        s.world_x = o.value("world_x").toDouble(); s.world_z = o.value("world_z").toDouble();
        s.ego_x = o.value("ego_world_x").toDouble(); s.ego_z = o.value("ego_world_z").toDouble();
        s.field_x = o.value("field_x").toDouble(); s.field_y = o.value("field_y").toDouble();
        s.source_stamp = o.value("source_stamp").toDouble(); s.ttl = o.value("ttl_s").toDouble();
        s.move_probability = o.value("move_probability").toDouble();
        s.action = o.value("action").toString(); s.candidate = o.value("candidate_id").toString();
        const int candidate_index = o.value("candidate_index").toInt(-1);
        if (s.action != "MOVE" && s.action != "HOLD") return s;
        if ((s.action == "HOLD" && (s.candidate != "HOLD" || candidate_index != 64)) ||
            (s.action == "MOVE" && (candidate_index < 0 || candidate_index >= 64 ||
             s.candidate != QString("candidate_%1").arg(candidate_index)))) return s;
        if (std::abs(s.world_x) > 7.5 || std::abs(s.world_z) > 14 ||
            std::abs(s.ego_x) > 7.5 || std::abs(s.ego_z) > 14 ||
            s.ttl <= 0 || s.ttl > 2 || s.source_stamp <= 0 ||
            s.move_probability < 0 || s.move_probability > 1 ||
            std::abs(s.field_x-(s.world_z+14)) > 0.01 ||
            std::abs(s.field_y-(s.world_x+7.5)) > 0.01) return s;
        s.valid = true; s.reason.clear();
    } catch (...) { s.valid = false; }
    return s;
}

inline Suggestion current(Suggestion s, bool flip_team, double map_stamp) {
    if (s.valid && (s.flip_team != flip_team ||
        std::chrono::duration<double>(Clock::now()-s.received).count() > s.ttl ||
        map_stamp <= 0 || std::abs(map_stamp-s.source_stamp) > s.ttl)) {
        s.valid = false; s.reason = "Expired / team changed";
    }
    return s;
}

inline void draw(QPixmap& pixmap, const Suggestion& s) noexcept {
    if (!s.enabled || pixmap.isNull()) return;
    try {
        QPainter p(&pixmap);
        p.setRenderHint(QPainter::Antialiasing);
        const QColor color(0,255,204);
        QFont font = p.font(); font.setPixelSize(13); p.setFont(font);
        if (s.valid) {
            // Exactly RadarMap::calibrate2 + worldtomapDisplay, on the final map image.
            auto point = [&](double wx, double wz) {
                double x = (wx/15.0+0.5)*pixmap.width();
                double y = (wz/28.0+0.5)*pixmap.height();
                if (s.flip_team) { x = pixmap.width()-1-x; y = pixmap.height()-1-y; }
                return QPointF(x,y);
            };
            const auto target = point(s.world_x,s.world_z), ego = point(s.ego_x,s.ego_z);
            p.setPen(QPen(Qt::black,5)); p.drawLine(ego,target);
            p.setPen(QPen(color,2,Qt::DashLine)); p.drawLine(ego,target);
            p.setPen(QPen(Qt::black,5)); p.drawEllipse(target,11,11);
            p.setPen(QPen(color,2)); p.drawEllipse(target,11,11);
            p.drawLine(target+QPointF(-15,0),target+QPointF(15,0));
            p.drawLine(target+QPointF(0,-15),target+QPointF(0,15));
        }
        const int height = s.valid ? 70 : 42;
        p.fillRect(0,0,pixmap.width(),height,QColor(0,0,0,210));
        p.setPen(s.valid ? color : QColor(190,190,190));
        if (s.valid) {
            p.drawText(8,18,QString("SHADOW S%1 %2 / %3").arg(s.robot_id).arg(s.action,s.candidate));
            p.drawText(8,38,QString("建议场地坐标 (%1, %2) m").arg(s.field_x,0,'f',2).arg(s.field_y,0,'f',2));
            p.drawText(8,58,QString("P(MOVE)=%1 | BC 建议，仅显示").arg(s.move_probability,0,'f',2));
        } else {
            p.drawText(8,17,"SHADOW unavailable | 原决策继续运行");
            p.drawText(8,35,s.reason.left(48));
        }
    } catch (...) { /* Display failures have no path back to radar state. */ }
}
} // namespace neural_shadow
