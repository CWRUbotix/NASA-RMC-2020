#ifndef TRAPEZOIDAL_PROFILE_H_
#define TRAPEZOIDAL_PROFILE_H_

#include <geometry_msgs/Point.h>

// Calculate the trapezoidal or triangular motion profile for a distance
// The vector of geometry_msgs::Point will store position, velocity, and accel
// For each time step
void trapezoidal_profile(double distance, std::vector<geometry_msgs::Point> &profile, double max_vel, double max_accel, double dt);

#endif  // TRAPEZOIDAL_PROFILE_H_