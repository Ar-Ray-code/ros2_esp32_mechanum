#include <ros2arduino.h>
#include <user_config.h>

#include <WiFi.h>
#include <WiFiUdp.h>

#define MOTOR1_A  A14
#define MOTOR1_B  A15
#define MOTOR2_A  A16
#define MOTOR2_B  A17
#define MOTOR3_A  A18
#define MOTOR3_B  A19
#define MOTOR4_A  A4
#define MOTOR4_B  A5

#define SSID      "ssid"
#define PASSWORD  "pass"
#define SERVER    "agent_ip"
#define agent     2018

// Param ===============================
const float pi=3.14159;
float bias = 1.5;

int linear_x = 0;//speed x [m]
int linear_y = 0;//speed y [m]
int linear_z = 0;//speed y [m]
int angle_z  = 0;//speed z [rad/s]
int spd1, spd2, spd3, spd4 = 0;
WiFiUDP udp;

// ROS Subscribe function (twist_dataの受信)=========================
void messageCb(const geometry_msgs::Twist *twist, void* arg) {
  (void)(arg);
  
  linear_x  = twist->linear.x;
  linear_y  = twist->linear.y;
  linear_z  = twist->linear.z;
  angle_z   = twist->angular.z;
  calc_mecanum(linear_x, linear_y, angle_z);
  linear_x = 0; linear_y = 0; angle_z = 0;
}

// Setup function =====================================================
void motor_setup()
{
  ledcSetup(0,490,8);           ledcSetup(1,490,8);
  ledcSetup(2,490,8);           ledcSetup(3,490,8);
  ledcSetup(4,490,8);           ledcSetup(5,490,8);
  ledcSetup(6,490,8);           ledcSetup(7,490,8);
  
  ledcAttachPin(MOTOR1_A,0);    ledcAttachPin(MOTOR1_B,1);
  ledcAttachPin(MOTOR2_A,2);    ledcAttachPin(MOTOR2_B,3);
  ledcAttachPin(MOTOR3_A,4);    ledcAttachPin(MOTOR3_B,5);
  ledcAttachPin(MOTOR4_A,6);    ledcAttachPin(MOTOR4_B,7);
}

// Motor move function=========================================================
void send_data(int spd1, int spd2, int spd3, int spd4)
{
  if(spd1 > 0){       ledcWrite(0,spd1);  ledcWrite(1,0);         }
  else if(spd1 < 0){  ledcWrite(0,0);     ledcWrite(1,-1*spd1);   }
  else {              ledcWrite(0,0);     ledcWrite(1,0);         }
  //spd2
  if(spd2 > 0) {      ledcWrite(2,spd2);  ledcWrite(3,0);         }
  else if(spd2 < 0){  ledcWrite(2,0);     ledcWrite(3,-1*spd2);   }
  else {              ledcWrite(2,0);     ledcWrite(3,0);         }
  //spd3
  if(spd3 > 0){       ledcWrite(4,spd3);  ledcWrite(5,0);         }
  else if(spd3 < 0){  ledcWrite(4,0);     ledcWrite(5,-1*spd3);   }
  else{               ledcWrite(4,0);     ledcWrite(5,0);         }
  //spd4
  if(spd4 > 0){       ledcWrite(6,spd4);  ledcWrite(7,0);         }
  else if(spd4 < 0){  ledcWrite(6,0);     ledcWrite(7,-1*spd4);   }
  else{               ledcWrite(6,0);     ledcWrite(7,0);         }
}
void calc_mecanum(int Vx, int Vy, int wl)
{
  spd1 = (int)((1)*(Vx + Vy)/(sqrt(2.0f))* bias - wl);
  if(spd1 >= 125) spd1 = 125;
  else if(spd1 <= -125) spd1 = -125;
  
  spd2 = (int)((1)*(Vx - Vy)/(sqrt(2.0f))* bias + wl);
  if(spd2 >= 125) spd2 = 125;
  else if(spd2 <= -125) spd2 = -125;
  
  spd3 = (int)((-1)*(Vx + Vy)/(sqrt(2.0f))* bias - wl);
  if(spd3 >= 125) spd3 = 125;
  else if(spd3 <= -125) spd3 = -125;
  
  spd4 = (int)((-1)*(Vx - Vy)/(sqrt(2.0f))* bias + wl);
  if(spd4 >= 125) spd4 = 125;
  else if(spd4 <= -125) spd4 = -125;

  if(abs(spd1) <15)  spd1 = 0;  if(abs(spd2) <15)  spd2 = 0;
  if(abs(spd3) <15)  spd3 = 0;  if(abs(spd4) <15)  spd4 = 0;
  Serial.print("x:");
  Serial.print(Vx);
  Serial.print("  y:");
  Serial.print(Vy);
  Serial.print("  wl");
  Serial.println(wl);
  send_data(spd1,spd2,spd3,spd4);
}

class sub_twist : public ros2::Node
{
  public:
    sub_twist(): Node("esp32_mechanum")
    {
      this->createSubscriber<geometry_msgs::Twist>("cmd_vel", (ros2::CallbackFunc)messageCb, nullptr);
    }
};

void setup()
{ 
  //WiFi Configure======================
  Serial.begin(115200);
  WiFi.begin(SSID,PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("...");
  }
  ros2::init(&udp, SERVER, agent);
  
  motor_setup();
}
void loop()
{
  static sub_twist sub;
  Serial.println(WiFi.status());
  //calc_mecanum(linear_x, linear_y, angle_z);
  ros2::spin(&sub);
}
