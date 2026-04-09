#pragma once

#include <QWidget>

#include "MapSnapshot.h"
#include "Vec2.h"

namespace SLAM
{
    class MapWidget : public QWidget
    {
    public:
        explicit MapWidget(QWidget* parent = nullptr);

        void SetSnapshot(const MapSnapshot& snapshot);

        float GetPixelsPerMeter() const;
        void SetPixelsPerMeter(float value);

        float GetGridSpacingMeters() const;
        void SetGridSpacingMeters(float value);

    protected:
        void paintEvent(QPaintEvent* event) override;

    private:
        QPointF WorldToScreen(const Vec2& point) const;

        void DrawBackground(QPainter& painter);
        void DrawGrid(QPainter& painter);
        void DrawAxes(QPainter& painter);
        void DrawWalls(QPainter& painter);

        MapSnapshot snapshot_{};
        float pixelsPerMeter_ = 100.0f;
        float gridSpacingMeters_ = 0.5f;
    };
}
