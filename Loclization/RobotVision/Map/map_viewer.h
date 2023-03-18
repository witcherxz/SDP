#include "map.h"
#if !defined(SDP_ROBOTVISION_MAP_VIEWER_H)
#define SDP_ROBOTVISION_MAP_VIEWER_H

class MapViewer {
    private:
        GridMap map;
        const int scaler = 100;
    public:
        MapViewer(GridMap map);
        void drawMap();
};

#endif // SDP_ROBOTVISION_MAP_VIEWER_H