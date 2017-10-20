// File used to keep all global constants

namespace constants
{
    const double TARGET_SPEED = 50. * 0.99;// max speed in mph with a safety margin
    const double TARGET_ACC = 10. * 0.5;   // max acceleration in m/s2
    const double TARGET_JERK = 10. * 0.5;  // max jerk in m/s2
    const double REFRESH_TIME = 0.02;      // refresh time in s between each car position
    const double LANE_WIDTH = 4.;          // Width of each lane in meters
}