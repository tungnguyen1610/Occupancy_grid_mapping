#include "../include/sensor.h"
using namespace std;
double Sensor::getMaxRange(){return max_range;}
double Sensor::inverse_model(double robot_x,double robot_y,double robot_theta, double grid_x,double grid_y, double l0, double locc, double lfree, double sensorData[])
{
    // Defining Sensor Characteristics
    double Zk, thetaK, sensorTheta;
    //minimum differences between angle 
    double minDelta = -1;
    // alpha: thickness obstacle (cell width) , beta: detection angle (cone)
    double alpha = 0.05, beta = 30;

    // Compute r and phi
    double r = sqrt(pow(grid_x - robot_x, 2) + pow(grid_y - robot_y, 2));
    double phi = atan2(grid_y - robot_y, grid_x - robot_x) - robot_theta;
    //cout << "R and phi:"<<r<<","<<phi<<endl;

    for (int i = 0; i < 4; i++) {
    if (i == 0)
        sensorTheta = -45 * (M_PI / 180);
    else if (i == 1)
        sensorTheta = 45 * (M_PI / 180);
    else if (i == 2)
        sensorTheta = -135 * (M_PI / 180);
    else if (i == 3)
        sensorTheta = 135 * (M_PI / 180);

    if (fabs(phi - sensorTheta) < minDelta || minDelta == -1) {
        Zk = sensorData[i];
        thetaK = sensorTheta;
        minDelta = fabs(phi - sensorTheta);
        }
    }

    // Evaluate the three occupancy cases
    if (r > min((double)max_range, Zk + alpha/2) || fabs(phi - thetaK) > beta / 2 || Zk > max_range || Zk < min_range)
        return l0;
    else if (Zk < max_range && fabs(r - Zk) < alpha / 2)
        return locc;
    else if (r <= Zk)
        return lfree;
    else
        return l0;
}