#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


int main(int argc, char** argv){
  // Initialization
  ros::init(argc, argv, "fake_LaserScan");
  //ros::Time::init();
  ros::NodeHandle node;
  if (argc !=1){ROS_ERROR("Error argument");return -1;};
  ros::Rate r(10.0);
  ros::Publisher fake_LaserScan_pub = node.advertise<sensor_msgs::LaserScan>("fake_LaserScan", 50);
  sensor_msgs::LaserScan ls;

  // Main loop
  while(node.ok())
  { 
    // Fill in fake data
    ls.header.stamp = ros::Time::now();
    ls.header.frame_id = "fake_LaserScan";
    ls.angle_min = 0;
    ls.angle_max = 3;
    ls.angle_increment = 0.1;
    ls.time_increment = 0.1;                                                                    
    ls.scan_time = 0.1;
    ls.range_min = 0.1;
    ls.range_max = 1;
    //ls.ranges = 0.5; // may cause problems?? 
    //ls.intensities = 

    // Publish 
    fake_LaserScan_pub.publish(ls);

	  r.sleep();
  };

  return 0;
}
