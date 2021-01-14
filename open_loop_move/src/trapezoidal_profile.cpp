#include <open_loop_move/trapezoidal_profile.h>

void trapezoidal_profile(double distance, std::vector<geometry_msgs::Point> &profile)
{
    double pos = 0.0;
    double vel = 0.0;
    double accel = 0.0;

    // Reverse profile if going backwards
    int multiplier = (distance > 0) ? 1 : -1;
    distance = abs(distance);

    // Calculate rise, coast, and fall times
    double accel_time = MAX_VEL / MAX_ACCEL;
    double accel_distance = 0.5 * MAX_ACCEL * accel_time * accel_time;

    // Same as accel, would be different if end vel != 0
    double deaccel_time = MAX_VEL / MAX_ACCEL;
    double deaccel_distance = 0.5 * MAX_ACCEL * deaccel_time * deaccel_time;

    double coast_distance = distance - accel_distance - deaccel_distance;
    double coast_time = coast_distance / MAX_VEL;

    // Check if triangular profile
    if (coast_distance < 0)
    {
        coast_distance = 0;
        coast_time = 0;
        accel_distance = 0.5 * distance;
        deaccel_distance = 0.5 * distance;
        accel_time = sqrt(2 * accel_distance / MAX_ACCEL);
        deaccel_time = sqrt(2 * deaccel_distance / MAX_ACCEL);
    }

    // Convert to absolute time from durations
    coast_time = accel_time + coast_time;
    deaccel_time = coast_time + deaccel_time;

    // Calculate trajectory
    double t = 0;
    accel = MAX_ACCEL;
    while (t < accel_time)
    {
        geometry_msgs::Point point;
        point.x = pos * multiplier;
        point.y = vel * multiplier;
        point.z = accel * multiplier;
        profile.push_back(point);

        vel = t * accel;
        pos = 0.5 * accel * t * t;
        t += DT;
    }

    // Coast phase
    // May not exist if triangular
    accel = 0.0;
    vel = accel_time * MAX_ACCEL;
    while(t < coast_time)
    {
        geometry_msgs::Point point;
        point.x = pos * multiplier;
        point.y = vel * multiplier;
        point.z = 0;
        profile.push_back(point);

        pos = 0.5 * MAX_ACCEL * accel_time * accel_time + vel * (t - accel_time);
        t += DT;
    }

    // Deaccel phase
    // Stop when reached distance
    accel = -MAX_ACCEL;
    while (t < deaccel_time)
    {
        geometry_msgs::Point point;
        point.x = pos * multiplier;
        point.y = vel * multiplier;
        point.z = accel * multiplier;
        profile.push_back(point);

        vel = MAX_ACCEL * (deaccel_time - t);
        pos = distance - 0.5 * MAX_ACCEL * (deaccel_time - t) * (deaccel_time - t);
        t += DT;
    }

    // Add final point
    geometry_msgs::Point point;
    point.x = distance * multiplier;
    point.y = 0;
    point.z = 0;
    profile.push_back(point);
}
