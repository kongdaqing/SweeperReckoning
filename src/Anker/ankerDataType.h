#ifndef ANKERDATATYPE_H
#define ANKERDATATYPE_H

#define ODO_Resolution 0.001f
#define ACC_Resolution 0.000897825622f
#define GYRO_Resolution 0.0076294f
#define GYRO_RadResolution 0.000133158f
#define TIME_Resolution 1e-09f
#define OPT_Resolution 8.66306e-05f

struct AnkerData
{
    double time;
    unsigned long int time_ns;
    double gx;
    double gy;
    double gz;
    double ax;
    double ay;
    double az;
    double odo_right_pos;
    double odo_left_pos;
    double odo_right_vel;
    double odo_left_vel;
    double opt_pos_x;
    double opt_pos_y;
    double opt_quality;
    double wall_distance_right;
    double wall_distance_left;
};


#endif // ANKERDATATYPE_H
