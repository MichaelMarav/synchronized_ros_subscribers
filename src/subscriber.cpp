#include <iostream>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h" // msg for Acceleration and angular velocity
#include "geometry_msgs/WrenchStamped.h" // msg for force
#include "nav_msgs/Odometry.h"
#include <fstream>


using namespace message_filters;

// Define ros messages for the subscribed topics
geometry_msgs::WrenchStamped force_msg;
sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom_msg;
bool data_flag{false};


typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::WrenchStamped,sensor_msgs::Imu, nav_msgs::Odometry> MySyncPolicy;

void callbackf( const geometry_msgs::WrenchStampedConstPtr& msg1,const sensor_msgs::ImuConstPtr & msg2, const nav_msgs::OdometryPtr &msg3)
{
  force_msg = *msg1;
  imu_msg  = *msg2;
  odom_msg = *msg3;

  data_flag = true;
}



int main(int argc, char **argv)
{


  ros::init(argc, argv, "subscriber");

  ros::NodeHandle nh;

  std::string topic1,topic2,topic3,path_to_write;
  int rate;
  bool WriteData;

  nh.getParam("/subscriber/topic_1",topic1);
  nh.getParam("/subscriber/topic_2",topic2);
  nh.getParam("/subscriber/topic_3",topic3);
  nh.getParam("/subscriber/subscribe_rate_hz",rate);
  nh.getParam("/subscriber/write_data",WriteData);
  nh.getParam("/subscriber/path_to_write",path_to_write);


  ros::Rate r(rate);
  std::ofstream data;
  data.open(path_to_write, std::ios::out | std::ios_base::app);

  // Define subscribers
  message_filters::Subscriber<geometry_msgs::WrenchStamped> LLegForceSub;

  message_filters::Subscriber<sensor_msgs::Imu> imu;

  message_filters::Subscriber<nav_msgs::Odometry> odom;

  message_filters::Synchronizer<MySyncPolicy> *ts_sync;

  // Subscribe to topics
  LLegForceSub.subscribe(nh,topic1, 1);
  imu.subscribe(nh,topic2,1);
  odom.subscribe(nh,topic3,1);

  ts_sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), LLegForceSub,imu,odom);
  ts_sync->registerCallback(&callbackf);

  while (ros::ok()){
      data_flag= false;

      ros::spinOnce();
      r.sleep();
      
      if(data_flag && WriteData){
        data << force_msg.wrench.force.x        <<"," << force_msg.wrench.force.y       <<","<< force_msg.wrench.force.z        << ","
             << force_msg.wrench.torque.x       <<"," << force_msg.wrench.torque.y      <<","<< force_msg.wrench.torque.z       << ","
             << imu_msg.linear_acceleration.x   <<"," << imu_msg.linear_acceleration.y  <<","<< imu_msg.linear_acceleration.z  << ","
             << imu_msg.angular_velocity.x      <<"," << imu_msg.angular_velocity.y     <<","<< imu_msg.angular_velocity.z     << "\n";
      }
  }

  return 0;
}
