#include "../include/map_creator.h"
#include <opencv2/core/types.hpp>
#include "iostream"

MapCreator::MapCreator(GridMap map) : map(map) {}

void MapCreator::addLine(Line line, int margin)
{
    double cellSize = map.getCellSize();
    Point sp, ep;
    std::tie(sp, ep) = line.getPoints();
    double sx, sy;
    double ex, ey;
    std::tie(sx, sy) = sp.getCoordinate();
    std::tie(ex, ey) = ep.getCoordinate();
    if (sx == ex)
    {
        for (int i = sy/cellSize; i <= (ey/cellSize) + margin; i++)
        {
            for(int j = sx/cellSize-margin; j <= sx/cellSize+margin; j++){
                map.occupyCell(i, j);
            }
        }
    }
    if (sy == ey)
    {
        for (int i = sx/cellSize; i <= (ex/cellSize) + margin; i++)
        {
            for(int j = sy/cellSize-margin; j <= sy/cellSize+margin; j++){
                map.occupyCell(j, i);
            }
        }
    }
}