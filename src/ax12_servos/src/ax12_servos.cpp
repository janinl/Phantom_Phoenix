#include "ax12Serial.cpp"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "../../Phantom_Phoenix/mytypes.h"
#include "../../Phantom_Phoenix/Hex_Cfg.h"


void callback(const std_msgs::Float64::ConstPtr& msg, int servoId, bool isReverse)
{
  ROS_INFO("I heard: servoId=%d [%f]", servoId, msg->data);

   // Convert pos from gazebo units (radians) to ax12 units (0-1023 for -150deg to +150deg)
   double posRad = msg->data;
   const double PI = 3.14159265359;
   double posDeg = std::fmod(posRad/PI*180.0,180);
   if (posDeg < -150 || posDeg > 150) {
     cout << "ERROR: servo position out of range" << endl;
     posDeg /= 0;
   }
   if (isReverse) posDeg = -posDeg;

   double pos = posDeg/150 + 1;
   int posInt = std::nearbyint(pos * 512);
   if (posInt < 0) posInt=0;
   if (posInt > 1023) posInt=1023;
  ROS_INFO(" => %d", posInt);

  ax12SetRegister(servoId, AX_GOAL_POSITION_L, posInt, 2);
}

int main(int argc, char **argv)
{
  ax12Init(1000000);

  ros::init(argc, argv, "ax12_servos");
  ros::NodeHandle n;

vector<string> servoId2jointName;
    servoId2jointName.resize(18+1);
    servoId2jointName[cRRCoxaPin] = "c1_rr";
    servoId2jointName[cRRFemurPin] = "thigh_rr";
    servoId2jointName[cRRTibiaPin] = "tibia_rr";
    servoId2jointName[cRMCoxaPin] = "c1_rm";
    servoId2jointName[cRMFemurPin] = "thigh_rm";
    servoId2jointName[cRMTibiaPin] = "tibia_rm";
    servoId2jointName[cRFCoxaPin] = "c1_rf";
    servoId2jointName[cRFFemurPin] = "thigh_rf";
    servoId2jointName[cRFTibiaPin] = "tibia_rf";
    servoId2jointName[cLRCoxaPin] = "c1_lr";
    servoId2jointName[cLRFemurPin] = "thigh_lr";
    servoId2jointName[cLRTibiaPin] = "tibia_lr";
    servoId2jointName[cLMCoxaPin] = "c1_lm";
    servoId2jointName[cLMFemurPin] = "thigh_lm";
    servoId2jointName[cLMTibiaPin] = "tibia_lm";
    servoId2jointName[cLFCoxaPin] = "c1_lf";
    servoId2jointName[cLFFemurPin] = "thigh_lf";
    servoId2jointName[cLFTibiaPin] = "tibia_lf";

    vector<ros::Subscriber> joint_channels;// = n.subscribe("/hexapd/hj_.../", 10, callback);
    joint_channels.resize(1); // adding empty space for unused servo 0
    for (int servoId=1; servoId<=18; ++servoId) {
      string jointName = "/phantomx/j_" + servoId2jointName[servoId] + "_position_controller/command";
      bool isReverse = false; //servoId2jointName[servoId].substr(0,5) == "tibia" || servoId2jointName[servoId].substr(0,2) == "c1";
      joint_channels.push_back( n.subscribe<std_msgs::Float64>(jointName, 10, boost::bind(&callback, _1, servoId, isReverse)) );
    }


  ros::spin();

  ax12Finish();
  return 0;
}



