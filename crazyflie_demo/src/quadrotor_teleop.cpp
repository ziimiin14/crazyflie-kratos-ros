//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
//               2018, Wolfgang Hoenig, USC
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


#include <ros/ros.h>
#include<math.h>  
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Pose.h>
#include "crazyflie_driver/Position.h"
#include "crazyflie_driver/DataExt.h"
#include "crazyflie_driver/DataExtMod.h"
#include "crazyflie_driver/GenericLogData.h"



class Teleop
{
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber joy_subscriber_, joy_pos_subscriber_, opti_pose_subscriber_, zranger_subscriber_;

  ros::Publisher velocity_publisher_;
  ros::Publisher position_publisher_;
  ros::Publisher data_ext_mod_publisher_;
  // ros::Publisher data_ext_publisher_;
  // geometry_msgs::Twist velocity_;
  // geometry_msgs::Pose pose_;

  crazyflie_driver::Position position_;
  crazyflie_driver::DataExtMod data_ext_mod_;
  // crazyflie_driver::DataExt data_ext_;

  

  struct Axis
  {
    int axis;
    double max;
    double min;
  };

  struct Button
  {
    int button;
  };

  struct Quat
  {
    double x;
    double y;
    double z;
    double w;
  };

  struct Pos
  {
    double x;
    double y;
    double z;
  };
  
  struct Controller
  {
    double thrust_cmd;
    double roll_cmd;
    double pitch_cmd;
    double mode_cmd;
  };

  struct
  {
    Axis thurst_axs;
    Axis roll_axs;
    Axis pitch_axs;
    Axis mode_axs;
  } axes1_;


  double frequency_;
  double lowest_, highest_, range_js_, rate_, heading_, tempx_, tempy_, pad_x_, pad_y_, pad_dir_, offset_; 
  double m1_offset_, m2_offset_;
  int conPad_, control_gain_, kp_;
  int range_motor_;
  double Pgain, Igain, Dgain, Pterm, Iterm, Dterm;
  double target_, dt, ratio, Istate, last, error, motor_hover, P, upper_ratio, lower_ratio, motor_upper_limit, motor_lower_limit;
  double throttle_percentage,altitude_range,altitude_offset;
  double flap_init, flap_move_p;
  double x_tar, y_tar, z_tar, x_cur, y_cur, z_cur;
  double traverse_range, max_range;
  double pad_gain_;
  double prev_x, prev_y;
  // int motor_hover, motor_upper_limit, motor_lower_limit;
  // double real_ratio, motor_interval, ratio_interval;

  Quat q_,q_temp_;
  Pos p_;
  Controller c_;

public:
  Teleop()
  {
    ros::NodeHandle params("~");

    params.param<int>("thrust_axis", axes1_.thurst_axs.axis, -3);
    params.param<int>("roll_axis", axes1_.roll_axs.axis, -1);
    params.param<int>("pitch_axis", axes1_.pitch_axs.axis, -2);
    params.param<int>("mode_axis", axes1_.mode_axs.axis, -5);

    params.param<double>("thrust_max", axes1_.thurst_axs.max, range_motor_-10);
    params.param<double>("thrust_min", axes1_.thurst_axs.min, 0+10);
    params.param<double>("roll_max", axes1_.roll_axs.max, 0*M_PI/180.0);
    params.param<double>("roll_min", axes1_.roll_axs.min, 180*M_PI/180.0);
    params.param<double>("pitch_max", axes1_.pitch_axs.max, 90*M_PI/180.0);
    params.param<double>("pitch_min", axes1_.pitch_axs.min, -90*M_PI/180.0);

    params.param<int>("kp",kp_,1500);
    params.param<double>("Pgain",Pgain,70);
    params.param<double>("Igain",Igain,40);
    params.param<double>("Dgain",Dgain,50);
    params.param<double>("offset",offset_,M_PI/4);

    params.param<double>("frequency", frequency_, 100);
    params.param<double>("traverse_range",traverse_range,30);

    // General parameters initialization
    lowest_ = 0.93494737;
    highest_ = -0.92405713;
    range_motor_ = 65535;
    range_js_ =  -(highest_-lowest_);
    rate_ = range_motor_/range_js_;
    m2_offset_ = 1*M_PI - offset_ - M_PI/2;
    m1_offset_ = 2*M_PI - offset_ - M_PI/2;
    c_.pitch_cmd, c_.roll_cmd, c_.mode_cmd = 0;
    c_.thrust_cmd = lowest_;

    // Position msg initialization
    position_.x,position_.y,position_.z,position_.yaw = 0;

    // DataExtMod msg initialization
    data_ext_mod_.padDir, data_ext_mod_.padGain, data_ext_mod_.conPadM3, data_ext_mod_.conPadM4 = 0;
    data_ext_mod_.heading, data_ext_mod_.Pterm, data_ext_mod_.Iterm, data_ext_mod_.Dterm = 0;
    data_ext_mod_.flapM1, data_ext_mod_.flapM2 = 0;
      
    // I state initailzation
    Istate = 0;
    last, dt, error = 0;

    // Ratio for motor
    ratio = 0.37;
    upper_ratio = 0.52;
    lower_ratio = 0.22;

    // Motor limit
    motor_upper_limit = upper_ratio*(range_motor_-35);
    motor_lower_limit = lower_ratio*(range_motor_-35);
    motor_hover = ratio*(range_motor_-35);

    target_ = 0;
    throttle_percentage = 0;
    altitude_range = 90;
    altitude_offset = 10;

    // Flap parameter
    flap_init = 0.5*(range_motor_-35) - 5240;
    data_ext_mod_.flapM1, data_ext_mod_.flapM2 = flap_init/65500;

    // Initialize x,y,z current and target, max_range
    x_tar, y_tar, z_tar = 0;
    x_cur, y_cur, z_cur = 0;
    max_range = sqrt(pow(2*traverse_range,2)+pow(2*traverse_range,2));

    

    // Subscriber and Publisher
    opti_pose_subscriber_ = node_handle_.subscribe<geometry_msgs::PoseStamped>("kratos/pose",10,boost::bind(&Teleop::quatCallback, this, _1));
    joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 10, boost::bind(&Teleop::joyCallback, this, _1));
    position_publisher_ = node_handle_.advertise<crazyflie_driver::Position>("cmd_position", 2);
    data_ext_mod_publisher_ = node_handle_.advertise<crazyflie_driver::DataExtMod>("data_ext_mod",2);
    // zranger_subscriber_ = node_handle_.subscribe<crazyflie_driver::GenericLogData>("log1", 10, boost::bind(&Teleop::zrangeCallback, this, _1));
    // data_ext_publisher_ = node_handle_.advertise<crazyflie_driver::DataExt>("data_ext",10);

  }

  ~Teleop()
  {
    stop();
  }

  void execute()
  {
    ros::Rate loop_rate(frequency_);
    double last_time = ros::Time::now().toSec();
    prev_x = p_.x*100;
    prev_y = p_.y*100;
    

    while (ros::ok()) {

      double cur_time = ros::Time::now().toSec();
      dt = cur_time-last_time;

      getHeading();

      if (c_.mode_cmd < -0.5){
        PID_alt_update(dt);
        P_position_control_update();
        // P_manual_control_update();
        updateMsg();
      }
      else{
        updatePos();
      }
      
      last_time = cur_time;


      position_publisher_.publish(position_);
      data_ext_mod_publisher_.publish(data_ext_mod_);

      ros::spinOnce();
      loop_rate.sleep();
    }

  }

  void joyCallback(const sensor_msgs::JoyConstPtr &joy)
  {
    c_.thrust_cmd = getAxis_Mod(joy, axes1_.thurst_axs);
    c_.roll_cmd = getAxis_Mod(joy, axes1_.roll_axs);
    c_.pitch_cmd = getAxis_Mod(joy, axes1_.pitch_axs);
    c_.mode_cmd = getAxis_Mod(joy, axes1_.mode_axs);
    // std::cout << c_.thrust_cmd << "  " << c_.roll_cmd << "  "<< c_.pitch_cmd << "  "<< c_.thrust_cmd << "  "
  }

  void quatCallback(const geometry_msgs::PoseStamped::ConstPtr &opti_pose)
  {
    q_.x = opti_pose->pose.orientation.x;
    q_.y = opti_pose->pose.orientation.y;
    q_.z = opti_pose->pose.orientation.z;
    q_.w = opti_pose->pose.orientation.w;
    p_.x = opti_pose->pose.position.x;
    p_.y = opti_pose->pose.position.y;
    p_.z = opti_pose->pose.position.z;    
  }

  // void zrangeCallback(const crazyflie_driver::GenericLogData::ConstPtr &zrange){
  //   p_.x = 0;
  //   p_.y = 0;
  //   p_.z = zrange->values[0];

  // }

  void getHeading()
  {
    q_temp_.x = q_.x;
    q_temp_.y = q_.y;
    q_temp_.z = q_.z;
    q_temp_.w = q_.w;
    
    tempy_ = 2*((q_temp_.x*q_temp_.y)+(q_temp_.w*q_temp_.z));
    tempx_ = 1- 2*((q_temp_.y*q_temp_.y)+(q_temp_.z*q_temp_.z));    
    heading_ =  atan2(tempy_,tempx_);
  }

  void PID_alt_update(double deltaT)
  {
    throttle_percentage = -(c_.thrust_cmd-lowest_)/range_js_;

    if (throttle_percentage < 0.2){
      z_tar = 0;
    }
    else{

      if (throttle_percentage < 0.6){
        z_tar =  ((throttle_percentage-0.2)*altitude_range/0.4)+altitude_offset;
      }

      else{
        z_tar = altitude_range + altitude_offset;
      }

    }

    x_cur = p_.x*100;  // whenever using optitrack, convert m to cm 
    y_cur = p_.y*100;  // whenever using optitrack, convert m to cm 
    z_cur = p_.z*100; // whenever using optitrack, convert m to cm 
    // z_cur = p_.z/10; // whenever using zranger, convert mm to cm 

    if (z_tar == 0){
      conPad_ = (throttle_percentage*15000/0.2);
    }

    else{

      error = z_tar - z_cur;
    
      Pterm = Pgain*error;
      
      Istate += error * deltaT;

      Iterm = Igain* Istate;

      Dterm = (Dgain * (z_cur-last))/deltaT;

      last = z_cur;

      conPad_ = motor_hover + (Pterm + Iterm - Dterm);
      
      if (conPad_ > motor_upper_limit){
        conPad_ = motor_upper_limit;
      }
      if (conPad_ < motor_lower_limit){
        conPad_ = motor_lower_limit;
      }
    }

  }

  void P_position_control_update()
  {

    if (c_.roll_cmd >=0){
      x_tar = ((lowest_-c_.roll_cmd)*traverse_range/lowest_)-traverse_range;
    }
    else{
      x_tar = ((-highest_-c_.roll_cmd)*traverse_range/-highest_)-traverse_range;
    }

    if (c_.pitch_cmd >=0){
      y_tar = ((lowest_-c_.pitch_cmd)*traverse_range/lowest_)-traverse_range;
    }
    else{
      y_tar = ((-highest_-c_.pitch_cmd)*traverse_range/-highest_)-traverse_range;
    }

    pad_x_ = x_tar - x_cur;
    pad_y_ = y_tar - y_cur;

    pad_dir_ = atan2(pad_y_,pad_x_);

    pad_gain_ = sqrt(pow(pad_x_,2) + pow(pad_y_,2))/max_range;
    // pad_gain_ = 1;

    // std::cout << "pad_gain:" << pad_gain_ << std::endl;

    if (pad_gain_>1){
      pad_gain_ = 1;
    }


    control_gain_ = kp_ * pad_gain_;

  }

  void P_manual_control_update()
  {
    pad_x_ = -c_.roll_cmd/0.94;
    pad_y_ = -c_.pitch_cmd/0.94;
    pad_dir_ = atan2(pad_y_,pad_x_);
    if (sqrt(pow(pad_x_,2) + pow(pad_y_,2)) < 0.5){
      pad_gain_ = 0;
      pad_dir_ = 0;
    }
    else{
      pad_gain_ = 1;

    }

    control_gain_ = kp_ * pad_gain_;
    x_tar =0;
    y_tar = 0;
    x_cur = p_.x*100;
    y_cur = p_.y*100;

    // std::cout << "ANGLE:" << atan2(y_cur-prev_y,x_cur-prev_x)*180/M_PI << std::endl;

    prev_y = y_cur;
    prev_x = x_cur;
  }

  void updateMsg()
  {
    position_.header.stamp = ros::Time::now();
    position_.x = conPad_; //m3
    position_.y = conPad_ ; // m4
    position_.z = flap_init + control_gain_*sin(heading_+m1_offset_-pad_dir_); //m1
    position_.yaw = flap_init + control_gain_*sin(heading_+m2_offset_-pad_dir_); //m2

    data_ext_mod_.header.stamp = position_.header.stamp;
    data_ext_mod_.conPadM3 = position_.x; // m3
    data_ext_mod_.conPadM4 = position_.y; // m4
    data_ext_mod_.Pterm = Pterm;
    data_ext_mod_.Iterm = Iterm;
    data_ext_mod_.Dterm = Dterm;
    data_ext_mod_.heading = heading_;
    if(pad_dir_ < 0){
      data_ext_mod_.padDir  = pad_dir_ + (2*M_PI);
    }
    else{
      data_ext_mod_.padDir = pad_dir_;
    }
    data_ext_mod_.padGain = pad_gain_;
    data_ext_mod_.flapM1 = position_.z/65500;
    data_ext_mod_.flapM2 = position_.yaw/65500;
    data_ext_mod_.poseCurrent.position.x = x_cur;
    data_ext_mod_.poseCurrent.position.y = y_cur;
    data_ext_mod_.poseCurrent.position.z = z_cur;
    data_ext_mod_.poseCurrent.orientation.x = q_temp_.x;
    data_ext_mod_.poseCurrent.orientation.y = q_temp_.y;
    data_ext_mod_.poseCurrent.orientation.z = q_temp_.z;
    data_ext_mod_.poseCurrent.orientation.w = q_temp_.w;
    data_ext_mod_.posTarget.x = x_tar;
    data_ext_mod_.posTarget.y = y_tar;
    data_ext_mod_.posTarget.z = z_tar;
  }

  // void updateAltitudeTarget()
  // {
  //   throttle_percentage = -(c_.thrust_cmd-lowest_)/range_js_;

  //   if (throttle_percentage < 0.2){
  //     z_tar = 0;
  //   }
  //   else{

  //     if (throttle_percentage < 0.6){
  //       z_tar =  ((throttle_percentage-0.2)*altitude_range/0.4)+altitude_offset;
  //     }

  //     else{
  //       z_tar = altitude_range + altitude_offset;
  //     }

  //   }
  // }

  void updatePos()
  {
    conPad_ = int(-1 *(c_.thrust_cmd-lowest_)*rate_);

    pad_x_ = -c_.roll_cmd/0.94;
    pad_y_ = -c_.pitch_cmd/0.94;
    pad_dir_ = atan2(pad_y_,pad_x_);

    if (sqrt(pow(pad_x_,2) + pow(pad_y_,2)) < 0.5){
      pad_gain_ = 0;
      pad_dir_ = 0;
    }
    else{
      pad_gain_ = 1;
    }

    control_gain_ = kp_ * pad_gain_;
    position_.header.stamp = ros::Time::now();
    position_.x = conPad_ + control_gain_*sin(heading_+m1_offset_-pad_dir_); // m3
    position_.y = conPad_ + control_gain_*sin(heading_+m2_offset_-pad_dir_); // m4
    
    if (position_.x < 10){
      position_.x = 10;
    }

    if (position_.y < 10){
      position_.y = 10;
    }

    if (position_.x > 65500){
      position_.x = 65500;
    }

    if (position_.y > 65500){
      position_.y = 65500;
    }

    position_.z = flap_init;
    position_.yaw = flap_init; 

  }


  sensor_msgs::Joy::_axes_type::value_type getAxis_Mod(const sensor_msgs::JoyConstPtr &joy, Axis axis)
  {
    if (axis.axis == 0) {
      return 0;
    }
    if (axis.axis < 0) {
      axis.axis = -axis.axis;
    }
    if ((size_t) axis.axis > joy->axes.size()) {
      return 0;
    }
    return joy->axes[axis.axis - 1];
  }


  sensor_msgs::Joy::_axes_type::value_type getAxis(const sensor_msgs::JoyConstPtr &joy, Axis axis)
  {
    if (axis.axis == 0) {
      return 0;
    }
    sensor_msgs::Joy::_axes_type::value_type sign = 1.0;
    if (axis.axis < 0) {
      sign = -1.0;
      axis.axis = -axis.axis;
    }
    if ((size_t) axis.axis > joy->axes.size()) {
      return 0;
    }
    return int(sign * (joy->axes[axis.axis - 1]- lowest_)* axis.max);
  }

  sensor_msgs::Joy::_buttons_type::value_type getButton(const sensor_msgs::JoyConstPtr &joy, int button)
  {
    if (button <= 0) {
      return 0;
    }
    if ((size_t) button > joy->buttons.size()) {
      return 0;
    }
    return joy->buttons[button - 1];
  }

  void stop()
  {
    if(position_publisher_.getNumSubscribers() > 0) {
      position_ = crazyflie_driver::Position();
      position_publisher_.publish(position_);  
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "quadrotor_teleop");

  Teleop teleop;
  teleop.execute();


  return 0;
}
