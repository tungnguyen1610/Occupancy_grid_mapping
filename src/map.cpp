#include "../include/map.h"
Map::Map(double mapWidth,double mapHeight,double gridWidth,double gridHeight,double l0,double locc,double lfree)
{
    this->mapWidth=mapWidth;
    this->mapHeight=mapHeight;
    this->gridWidth=gridWidth;
    this->gridHeight=gridHeight;
    this->l0=l0;
    this->lfree=lfree;
    this->locc=locc;
    logOddsValues.resize(mapWidth / gridWidth, vector<double>(mapHeight / gridHeight, 0));
}
double Map::get_occupancy(unsigned int i, unsigned int j){return logOddsValues[i][j];}
void Map::updateMap(Robot &robot, Sensor &sensor, double sensorData[])
{
    for (int i = 0; i < mapWidth / gridWidth; i++) {
        for (int j = 0; j < mapHeight / gridHeight; j++) {
            double xi = i * gridWidth + gridWidth / 2 - robot.getWidth();
            double yi = -(j * gridHeight + gridHeight / 2) + robot.getHeight();

            if (sqrt(pow(xi - robot.getPosx(), 2) + pow(yi - robot.getPosy(), 2)) <= sensor.getMaxRange()) {
                logOddsValues[i][j] += sensor.inverse_model(robot.getPosx(), robot.getPosy(), robot.getOrientation(), xi, yi, l0,locc,lfree,sensorData);
            }
        }
    }
}
double Map::get_map_width()
{
    return mapWidth;
}

double Map::get_map_height()
{
    return mapHeight;
}

double Map::get_grid_width()
{
    return gridWidth;
}

double Map::get_grid_height()
{
    return gridHeight;
}