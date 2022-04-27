#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "project1/PoseVelSync.h"

class Synchronizer{
public:
    Synchronizer(){
        this->pose_sub = this->n.subscribe("robot/pose", 1000, &Synchronizer::updatePose, this);
        this->encoder_sub = this->n.subscribe("wheel_states", 1000, &Synchronizer::updateVelocity, this);
        this->pub = this->n.advertise<project1::PoseVelSync>("pose_vel_sync",1000);

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
    }

    void updateVelocity(const sensor_msgs::JointState::ConstPtr& msg){
        project1::PoseVelSync sync_msg;
        sync_msg.header = msg -> header;
        if(this->poseX != 0.0 || this->poseY != 0.0){
            sync_msg.poseX = this->poseX;
            sync_msg.poseY = this->poseY;
            sync_msg.rpm_fl = msg->velocity[0];
            sync_msg.rpm_fr = msg->velocity[1];
            sync_msg.rpm_rl = msg->velocity[2];
            sync_msg.rpm_rr = msg->velocity[3];

            this->pub.publish(sync_msg);

        }
    }

private:
    ros::NodeHandle n;
    ros::Subscriber pose_sub, encoder_sub;
    ros::Publisher pub;

    double poseX, poseY;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "synchronizer");
    Synchronizer synchronizer;
    synchronizer.main_loop();
    return 0;
}