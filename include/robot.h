#ifndef ROBOT_H
#define ROBOT_H
#include <iostream>
#include <math.h>
struct Property{
    double timestamp;
    double x;
    double y;
    double orientation;
}; 
class Robot {
    private:
    // robot size
    double robot_width;
    double robot_height;
    // Property of robot
    Property properties; 
    public:
    // Constructor 
    Robot(double robot_width = 0.2, double robot_height = 0.1);
    // Method to update robot's position
    double getWidth();
    double getHeight();
    double getPosx();
    double getPosy();
    double getOrientation();
    void updatePos(double new_x,double new_y,double new_orientation);
};

#endif
