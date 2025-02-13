#include "../include/robot.h"
using namespace std;
Robot::Robot(double robot_width , double robot_height) : robot_width(robot_width), robot_height(robot_height)
{
    properties.timestamp=0;
    properties.x=0;
    properties.y=0;
    properties.orientation=0;
}
double Robot::getWidth(){return robot_width;}
double Robot::getHeight(){return robot_height;}
double Robot::getPosx(){return properties.x;}
double Robot::getPosy(){return properties.y;}
double Robot::getOrientation(){return properties.orientation;}
// Given that (0.25,3.742) is initial coordinate of robot.
void Robot::updatePos(double new_x,double new_y,double new_orientation)
{
    properties.x=new_x;
    properties.y=new_y;
    properties.orientation=new_orientation;
}
