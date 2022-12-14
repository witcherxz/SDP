#include "map_creator.h"
#include <opencv2/core/types.hpp>

MapCreator::MapCreator(GridMap map): map(map) {}

void MapCreator::addLine(Line line) {
    double cellSize = map.getCellSize();
    Point sp, ep;
    std::tie(sp, ep) = line.getPoints();
    int sx, sy;
    int ex, ey;
    std::tie(sx, sy) = sp.getCoordinate();
    std::tie(ex, ey) = ep.getCoordinate();
    if (sx == ex) {
        for(int i = sy; i <= ey; i++){
            map.occupyCell(sx, i);
        }
    }
    if (sy == ey) {
        for(int i = sx; i <= ex; i++){
            map.occupyCell(i, sy);
        }
    }
}