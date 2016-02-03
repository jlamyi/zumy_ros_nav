#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

// Position initialization
float x = 1.0;
float y = 1.0;
float yaw = 0.0;
float yaw_counter = 0.0;
float dx = 0.0;
float dy = 0.0;
float dyaw = 0.0;

void drive(const geometry_msgs::Twist msg){
  if(msg.angular.z!=0) dyaw = msg.angular.z / 10;
  else {
    if(x > 0.5 && y > 0.5){
      dx = (msg.linear.x * cos(yaw) - msg.linear.y * sin(yaw)) / 10;
      dy = (msg.linear.y * cos(yaw) - msg.linear.x * sin(yaw)) / 10;
    }
    else{
      dx = 0;
      dy = 0;
    }
  }
}

int main(int argc, char** argv){
  // Node initialization
  ros::init(argc, argv, "tf_fake");
  //ros::Time::init();
  ros::NodeHandle node;
  if (argc !=1){ROS_ERROR("Error argument");return -1;};
  ros::Rate r(10);

  // For closed-loop simulation
  ros::Subscriber sub = node.subscribe("cmd_vel", 100, drive);

  // Tf variable declaration
	static tf::TransformBroadcaster br;
	tf::Transform transform;
  tf::Quaternion q;

  while(node.ok())
  {
    // Data update
    x += dx;
    y += dy;
    yaw_counter += dyaw;
    if(yaw_counter > (M_PI/2) && yaw_counter < M_PI){
      yaw = yaw_counter - M_PI;
    }
    else if(yaw_counter > M_PI){
      yaw_counter -= M_PI;
      yaw = yaw_counter;
    }
    else{
      yaw = yaw_counter;
    }
    

    // Data transmit
	  transform.setOrigin(tf::Vector3(x, y, 0.0));

	  q.setRPY(0, 0, yaw);
	  transform.setRotation(q);

	  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "fake_tf"));
	  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "fake_LaserScan"));

	  ros::spinOnce();
	  r.sleep();
  };

  return 0;
}
