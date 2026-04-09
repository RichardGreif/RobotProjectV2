#include "MapWidget.h"

#include <QPaintEvent>
#include <QPainter>
#include <QPen>
#include <QPointF>

namespace SLAM
{
    MapWidget::MapWidget(QWidget* parent)
        : QWidget(parent)
    {
        setMinimumSize(600, 600);
    }

    void MapWidget::SetSnapshot(const MapSnapshot& snapshot)
    {
        snapshot_ = snapshot;
        update();
    }

    float MapWidget::GetPixelsPerMeter() const
    {
        return pixelsPerMeter_;
    }

    void MapWidget::SetPixelsPerMeter(float value)
    {
        if (value <= 0.0f)
        {
            return;
        }

        pixelsPerMeter_ = value;
        update();
    }

    float MapWidget::GetGridSpacingMeters() const
    {
        return gridSpacingMeters_;
    }

    void MapWidget::SetGridSpacingMeters(float value)
    {
        if (value <= 0.0f)
        {
            return;
        }

        gridSpacingMeters_ = value;
        update();
    }

    void MapWidget::paintEvent(QPaintEvent* event)
    {
        Q_UNUSED(event);

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);

        DrawBackground(painter);
        DrawGrid(painter);
        DrawAxes(painter);
        DrawWalls(painter);
    }

    QPointF MapWidget::WorldToScreen(const Vec2& point) const
    {
        const float centerX = width() * 0.5f;
        const float centerY = height() * 0.5f;

        const float screenX = centerX + point.x * pixelsPerMeter_;
        const float screenY = centerY - point.y * pixelsPerMeter_;

        return QPointF(screenX, screenY);
    }

    void MapWidget::DrawBackground(QPainter& painter)
    {
        painter.fillRect(rect(), QColor(250, 250, 250));
    }

    void MapWidget::DrawGrid(QPainter& painter)
    {
        const float spacingPixels = gridSpacingMeters_ * pixelsPerMeter_;
        if (spacingPixels <= 0.0f)
        {
            return;
        }

        QPen pen(QColor(225, 225, 225));
        pen.setWidth(1);
        painter.setPen(pen);

        const float centerX = width() * 0.5f;
        const float centerY = height() * 0.5f;

        for (float x = centerX; x < width(); x += spacingPixels)
        {
            painter.drawLine(QPointF(x, 0.0f), QPointF(x, height()));
        }

        for (float x = centerX - spacingPixels; x >= 0.0f; x -= spacingPixels)
        {
            painter.drawLine(QPointF(x, 0.0f), QPointF(x, height()));
        }

        for (float y = centerY; y < height(); y += spacingPixels)
        {
            painter.drawLine(QPointF(0.0f, y), QPointF(width(), y));
        }

        for (float y = centerY - spacingPixels; y >= 0.0f; y -= spacingPixels)
        {
            painter.drawLine(QPointF(0.0f, y), QPointF(width(), y));
        }
    }

    void MapWidget::DrawAxes(QPainter& painter)
    {
        QPen pen(QColor(120, 120, 120));
        pen.setWidth(2);
        painter.setPen(pen);

        const float centerX = width() * 0.5f;
        const float centerY = height() * 0.5f;

        painter.drawLine(QPointF(0.0f, centerY), QPointF(width(), centerY));
        painter.drawLine(QPointF(centerX, 0.0f), QPointF(centerX, height()));
    }

    void MapWidget::DrawWalls(QPainter& painter)
    {
        for (int i = 0; i < snapshot_.wallCount; ++i)
        {
            const WallSegment& wall = snapshot_.walls[i];

            QPointF start = WorldToScreen(wall.start);
            QPointF end = WorldToScreen(wall.end);

            int lineWidth = 2;
            if (wall.confidence > 2.0f)
            {
                lineWidth = 3;
            }
            if (wall.confidence > 5.0f)
            {
                lineWidth = 4;
            }

            QPen pen(QColor(30, 70, 160));
            pen.setWidth(lineWidth);
            painter.setPen(pen);

            painter.drawLine(start, end);

            painter.setBrush(QColor(180, 40, 40));
            painter.drawEllipse(start, 3.0, 3.0);
            painter.drawEllipse(end, 3.0, 3.0);
        }
    }
}