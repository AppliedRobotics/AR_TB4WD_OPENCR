
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include  "robot.h" // подключаем файл с нужными функциями
#define BAUDRATE 1000000
#define WHEEL_NUM  4


sensor_msgs::JointState joint_state;
sensor_msgs::Imu imu_msg;
std_msgs::Float64 voltage;
std_msgs::UInt16MultiArray FrontLight;
std_msgs::UInt16MultiArray LeftLight;
std_msgs::UInt16MultiArray RightLight;
std_msgs::UInt16MultiArray BackLight;

ros::NodeHandle nh;//объект ноудхэндл (по сути создаем ноду)

float target_velocity[4] = {0.0, 0.0, 0.0, 0.0};
float current_velocity[4] =  {0.0, 0.0, 0.0, 0.0};
float current_effort[4] = {0.0, 0.0, 0.0, 0.0};
float current_position[4] = {0.0, 0.0, 0.0, 0.0};
uint16_t Frontarray[7] = {0,0,0,0,0,0,0};
uint16_t Leftarray[7]= {0,0,0,0,0,0,0};
uint16_t Rightarray[7]= {0,0,0,0,0,0,0};
uint16_t Backarray[7]= {0,0,0,0,0,0,0};

void cmd_cb(const geometry_msgs::Twist& vel){//колбэк топика текущего маркера
  target_velocity[0]= vel.linear.x;//переписываем данные маркера в глобальную переменную
  target_velocity[1]= vel.linear.y;
  target_velocity[2] = vel.angular.z;
}
int time_millis = 0;
ros::Publisher joint_state_pub("joint_state", &joint_state);//создаем объект паблишер, чтобы публиковать информацию о своем состоянии (свободен/занят)
ros::Publisher imu_pub("imu", &imu_msg);
ros::Publisher voltage_pub("battery_voltage", &voltage);
ros::Publisher front_light_pub("front_light_array", &FrontLight);
ros::Publisher left_light_pub("left_light_array", &LeftLight);
ros::Publisher right_light_pub("right_light_array", &RightLight);
ros::Publisher back_light_pub("back_light_array", &BackLight);
//объект подписчик<тип сообщение> имя_объекта("топик на который подписываемся", колбэк функция)
ros::Subscriber<geometry_msgs::Twist> sub_cmd("cmd_vel", cmd_cb);

HardwareTimer Timer(TIMER_CH4);//таймер для обновления информации приходящей из ROS-а
void setup() {
  init();//функция инициализирования Dxl устройств
  nh.getHardware()->setBaud(BAUDRATE);//задаем частоту общения с росом 
  nh.initNode(); //инициализируем ноду
  nh.advertise(joint_state_pub);//инициализируем паблишеры и субскрайберы
  nh.advertise(imu_pub);
  nh.advertise(voltage_pub);
  nh.advertise(front_light_pub);
  nh.advertise(left_light_pub);
  nh.advertise(right_light_pub);
  nh.advertise(back_light_pub);
  nh.subscribe(sub_cmd);
  
  Timer.stop();//инициализируем таймеры         
  Timer.setPeriod(30000);// in microseconds
  Timer.attachInterrupt(handler_nh);//при прерывании вызывается функция handler_nh
  Timer.start();
  define_joint_state();
}

void loop(){
  if(millis() - time_millis > 15)
  {
    int dt = millis() - time_millis;
    time_millis = millis();
    move_os(target_velocity[0], -target_velocity[1], -target_velocity[2]);
    read_joint_state(current_velocity, current_effort);
    for(int i = 0; i<WHEEL_NUM; i++){
      current_position[i] +=current_velocity[i]*dt/1000;  
    }
    joint_state.header.stamp = nh.now();
    joint_state.velocity = current_velocity;
    joint_state.effort = current_effort;
    joint_state.position = current_position;
    joint_state_pub.publish(&joint_state); 
    
    voltage.data = check_voltage();
    voltage_pub.publish(&voltage);

    
    scan1(Frontarray, Leftarray, Rightarray, Backarray);
    
    FrontLight.data = Frontarray;
    FrontLight.data_length = 7;
    LeftLight.data = Leftarray;
    LeftLight.data_length = 7;
    RightLight.data = Rightarray;
    RightLight.data_length = 7;
    BackLight.data = Backarray;
    BackLight.data_length = 7;
    
    front_light_pub.publish(&FrontLight);
    left_light_pub.publish(&LeftLight);
    right_light_pub.publish(&RightLight);
    back_light_pub.publish(&BackLight);
    imu_msg = read_IMU();
    imu_pub.publish(&imu_msg);
  }
}

void define_joint_state(){
  static char *joint_state_name[] = {(char*)"wheel_front_left_joint", (char*)"wheel_back_right_joint", (char*)"wheel_front_right_joint",(char*)"wheel_back_left_joint"};
  joint_state.header.frame_id = "base_link";
  joint_state.name            = joint_state_name;
  joint_state.name_length     = WHEEL_NUM;
  joint_state.position_length = WHEEL_NUM;
  joint_state.velocity_length = WHEEL_NUM;
  joint_state.effort_length   = WHEEL_NUM;
}
void handler_nh(void){//связываемся с ROS-ом по прерыванию
  nh.spinOnce();// метод spinOnce осуществляет один цикл общения с ROS-ом, если его не вызывать с заданной частотой, то устройства перестанут быть связаны
}
