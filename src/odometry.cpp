#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "odo_publisher");

  ros::NodeHandle node;
  tf::TransformListener listener;
  ros::Rate rate(10.0);
  ros::Publisher odo_pub = node.advertise<nav_msgs::Odometry>("odo_pub", 50);
  nav_msgs::Odometry odom;
  double roll, pitch, yaw, last_yaw;  

  //initialize current_time and last_time
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  //initialize transform and last transform
  tf::StampedTransform transform, last_transform;

  bool no_data = true;
  while(no_data){ 
    try{
      listener.lookupTransform("/fake_tf", "/world", 
                             ros::Time(0), transform);
      no_data = false;
      printf("Fetched initial tf data\n");
    }
    catch (tf::TransformException ex){
      //ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      no_data = true;
    }
  }
  last_transform = transform;

  printf("Start calculating odometry data!\n");

  //main loop
  while (node.ok()){

    //fetch tf data
    try{
      listener.lookupTransform("/fake_tf", "/world", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // calculate dt, dx, dy, dz, dth
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double dx = transform.getOrigin().x() - last_transform.getOrigin().x();
    double dy = transform.getOrigin().y() - last_transform.getOrigin().y();
    double dz = transform.getOrigin().z() - last_transform.getOrigin().z();

    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    tf::Matrix3x3(last_transform.getRotation()).getRPY(roll, pitch, last_yaw);
    double dth = yaw - last_yaw;
    
    //convert yaw to geometry_msgs quarternion
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

    //publish ros odometry topic 
    if(dt!=0){

      //set the header     
    	odom.header.stamp = current_time;
    	odom.header.frame_id = "odom";

    	//set the position
    	odom.pose.pose.position.x = transform.getOrigin().x();
    	odom.pose.pose.position.y = transform.getOrigin().y();
    	odom.pose.pose.position.z = transform.getOrigin().z();
    	odom.pose.pose.orientation = odom_quat;

    	//set the velocity
    	odom.child_frame_id = "base_link";
    	odom.twist.twist.linear.x = dx/dt;
    	odom.twist.twist.linear.y = dy/dt;
    	odom.twist.twist.linear.z = dz/dt;
    	odom.twist.twist.angular.z = dth/dt; 
      
      //publish
    	odo_pub.publish(odom);
    }

    //update data from last time
    last_transform = transform;
    last_time = current_time;
    rate.sleep();
  }
  return 0;
};
