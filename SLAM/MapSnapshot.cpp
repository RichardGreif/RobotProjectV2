#include "MapSnapshot.h"

namespace SLAM
{
    void MapSnapshot::Clear()
    {
        wallCount = 0;
    }

    MapSnapshot CreateMapSnapshot(const WallMap& wallMap)
    {
        MapSnapshot snapshot;
        snapshot.wallCount = wallMap.GetCount();

        for (int i = 0; i < snapshot.wallCount; ++i)
        {
            snapshot.walls[i] = wallMap.GetSegment(i);
        }

        return snapshot;
    }
}