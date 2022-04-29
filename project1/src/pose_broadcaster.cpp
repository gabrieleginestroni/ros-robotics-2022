#include "ros/ros.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/PoseStamped.h"

class Pose_broadcaster{
public:
    Pose_broadcaster(){
        this->pose_sub = this->n.subscribe("robot/pose", 1000, &Pose_broadcaster::updatePose, this);
        this->poseX = 0.0;
        this->poseY = 0.0;
    }

    void main_loop() {
        ros::Rate loop_rate(10);
        while(ros::ok()){
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void updatePose(const geometry_msgs::PoseStamped::ConstPtr& msg){
        this->poseX = msg->pose.position.x;
        this->poseY = msg->pose.position.y;

        transformStamped.header = msg->header;
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "GT";

        transformStamped.transform.translation.x = this->poseX;
        transformStamped.transform.translation.y = this->poseY;
        transformStamped.transform.translation.z = 0.0;

        transformStamped.transform.rotation.x = msg->pose.orientation.x;
        transformStamped.transform.rotation.y = msg->pose.orientation.y;
        transformStamped.transform.rotation.z = msg->pose.orientation.z;
        transformStamped.transform.rotation.w = msg->pose.orientation.w;

        br.sendTransform(transformStamped);


    }


private:
    ros::NodeHandle n;
    ros::Subscriber pose_sub;

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    double poseX, poseY;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_broadcaster");
    Pose_broadcaster poseBroadcaster;
    poseBroadcaster.main_loop();
    return 0;
}