#ifndef SENSOR_H
#define SENSOR_H
#include <iostream>
#include <math.h>
class Sensor{
    private:
    double min_range;
    double max_range;
    public:
    //Constructor
    Sensor (double min_range,double max_range):min_range(min_range),max_range(max_range){};
    //Inverse sensor model
    double getMaxRange(); 
    double inverse_model(double robot_x,double robot_y,double robot_theta, double grid_x,double grid_y, double l0, double locc, double lfree, double sensorData[]);
}; 
#endif