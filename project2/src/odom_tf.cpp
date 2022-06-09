#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

class pub_sub{
private:
	ros::NodeHandle n; 
	ros::Subscriber sub;
	ros::Publisher pub;
	
	
public:
  	pub_sub(){
  		sub = n.subscribe("/odom", 1, &pub_sub::callback, this);
	}

	void callback(const nav_msgs::Odometry::ConstPtr& msg){
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
		transform.setRotation(tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w));
		static tf::TransformBroadcaster br;
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
	}
};

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "subscribe_and_publish");
 	pub_sub my_pub_sub;
 	ros::spin();
 	return 0;
}
