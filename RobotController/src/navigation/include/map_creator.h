#include "map.h"

#ifndef SDP_ROBOTVISION_MAPCREATOR_H
#define SDP_ROBOTVISION_MAPCREATOR_H

class MapCreator {
private:
    GridMap map;

public:
    MapCreator(GridMap map);

    void addLine(Line line, int margin);
};

#endif // SDP_ROBOTVISION_MAPCREATOR_H