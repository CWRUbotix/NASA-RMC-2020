#ifndef TRAPEZOIDAL_PROFILE_H_
#define TRAPEZOIDAL_PROFILE_H_

#include <geometry_msgs/Point.h>

const double DT = 0.05;
const double MAX_VEL = 0.5;
const double MAX_ACCEL = 1;

// Calculate the trapezoidal or triangular motion profile for a distance
// The vector of geometry_msgs::Point will store position, velocity, and accel
// For each time step
void trapezoidal_profile(double distance, std::vector<geometry_msgs::Point> &profile);

#endif  // TRAPEZOIDAL_PROFILE_H_