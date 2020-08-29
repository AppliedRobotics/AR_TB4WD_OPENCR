#include <DynamixelWorkbench.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include <IMU.h>
#include <sensor_msgs/Imu.h>
void init();
void move_os(float vX, float vY, float wZ);
void read_joint_state(float* current_velocity, float* current_effort);
float* read_IMU();
void gyro_calib();
float check_voltage();
void check_motors();
uint32_t* scan_front();
uint32_t* scan_left();
uint32_t* scan_right();
uint32_t* scan_back();
