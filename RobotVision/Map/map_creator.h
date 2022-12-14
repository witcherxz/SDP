#include "map.h"

#ifndef SDP_ROBOTVISION_MAPCREATOR_H
#define SDP_ROBOTVISION_MAPCREATOR_H
class MapCreator {
    private:
        GridMap map;
    public:
        MapCreator();
        void addLine(Line line);
};
#endif // SDP_ROBOTVISION_MAPCREATOR_H