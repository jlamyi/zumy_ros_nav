#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "tf_fake");
  //ros::Time::init();
  ros::NodeHandle node;
  if (argc !=1){ROS_ERROR("Error argument");return -1;};

  ros::Rate r(10);
  while(node.ok())
  {
	  static tf::TransformBroadcaster br;
	  
	  tf::Transform transform;
	  transform.setOrigin(tf::Vector3(1.0, 1.0, 0.0));

	  tf::Quaternion q;
	  q.setRPY(0, 0, 0);
	  transform.setRotation(q);

	  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "fake_tf"));

	  ros::spinOnce();
	  r.sleep();
  };

  return 0;
}
