#ifndef MAP_H
#define MAP_H
#include <vector>
#include "robot.h"
#include "sensor.h"
using namespace std;
class Map{
    private:
    // property //
    double mapWidth,mapHeight;
    double gridWidth,gridHeight;
    vector <vector<double>> logOddsValues;
    // log odds: unknown, occupied and free states
    double l0,locc,lfree;
    public:
    // Method //
    Map(double mapWidth,double mapHeight,double gridWidth,double gridHeight,double l0, double locc, double lfree);
    void updateMap(Robot &robot, Sensor &sensor,double sensorData[]);
    double get_occupancy(unsigned int i,unsigned int j);
    double get_map_width();
    double get_map_height();
    double get_grid_width();
    double get_grid_height();
};
#endif