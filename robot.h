#include <DynamixelWorkbench.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include <IMU.h>
#include <sensor_msgs/Imu.h>
void init();
void move_os(float vX, float vY, float wZ);
void read_joint_state(float* current_velocity, float* current_effort);
sensor_msgs::Imu read_IMU();
void gyro_calib();
float check_voltage();
void check_motors();
void scan1(uint16_t* Frontarray, uint16_t* Leftarray, uint16_t* Rightarray, uint16_t* Backarray);
