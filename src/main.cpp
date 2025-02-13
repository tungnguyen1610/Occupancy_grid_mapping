#include <fstream>
#include <sstream>
#include "../include/robot.h"
#include "../include/sensor.h"
#include "../include/map.h"
#include "../include/visualization.h"

using namespace std;

int main() {
    double timeStamp;
    double measurementData[4];
    double robotX, robotY, robotTheta;

    ifstream dataFile("../Data/robot.csv");
    if (!dataFile.is_open()) {
        cerr << "Error opening the file!" << endl;
        return 1;
    }

    // Create Robot (size)
    Robot robot(0.2,0.1);
    // Sensor characteristic (minimum, maximum effective ranges of sensor)
    Sensor sensor(0.05,4);
    // Map characteristic (Width,Height, Log odds threshold)
    Map map(10,5,0.05,0.05,0,0.85,-0.7);

    // Read each line of the file
    string line;
    while (getline(dataFile, line)) {
        stringstream ss(line);
        char comma;

        // Read timestamp, robot pose (X, Y, Theta)
        if (ss >> timeStamp >> comma >> robotX >> comma >> robotY >> comma >> robotTheta >> comma) {
            // Read measurement data values
            robot.updatePos(robotX,robotY,robotTheta);
            for (int i = 0; i < 4; i++) {
                if (ss >> measurementData[i]) {
                    // Convert time to distance (ultrasonic sensor)
                    measurementData[i] = (measurementData[i] * 343) / 2;
                }
                // Skip the commas between measurement data values
                if (i < 3) {
                    ss >> comma;  // Skip the comma after each measurement value
                }
            }

            // Update the map with the new sensor data
            map.updateMap(robot,sensor,measurementData);
        }
    }
    // Visualize the map at the final step
    cout << "Wait for the image to generate" << endl;
    Visualization::mapShow(&map);
    cout << "Done!" << endl;

    return 0;
}
