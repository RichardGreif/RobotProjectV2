#pragma once

#include <array>

#include "WallMap.h"
#include "WallMap.h"

namespace SLAM
{
    struct MapSnapshot
    {
        static constexpr int MaxWalls = WallMap::MaxSegments;

        std::array<WallSegment, MaxWalls> walls{};
        int wallCount = 0;

        void Clear();
    };

    MapSnapshot CreateMapSnapshot(const WallMap& wallMap);
}