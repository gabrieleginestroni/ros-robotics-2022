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
        this->q_x = 0.0;
        this->q_y = 0.0;
        this->q_w = 0.0;
        this->q_z = 0.0;

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
        this->q_x = msg->pose.orientation.x;
        this->q_y = msg->pose.orientation.y;
        this->q_w = msg->pose.orientation.w;
        this->q_z = msg->pose.orientation.z;

    }

    void updateVelocity(const sensor_msgs::JointState::ConstPtr& msg){
        project1::PoseVelSync sync_msg;
        sync_msg.sec = msg -> header.stamp.sec;
        sync_msg.nsec = msg -> header.stamp.nsec;
        if(this->poseX != 0.0 || this->poseY != 0.0){
            sync_msg.poseX = this->poseX;
            sync_msg.poseY = this->poseY;

            sync_msg.q_x = this->q_x;
            sync_msg.q_y = this->q_y;
            sync_msg.q_w = this->q_w;
            sync_msg.q_z = this->q_z;

            sync_msg.ticks_fl = msg->position[0];
            sync_msg.ticks_fr = msg->position[1];
            sync_msg.ticks_rl = msg->position[2];
            sync_msg.ticks_rr = msg->position[3];

            this->pub.publish(sync_msg);

        }
    }

private:
    ros::NodeHandle n;
    ros::Subscriber pose_sub, encoder_sub;
    ros::Publisher pub;

    double poseX, poseY, q_x, q_y, q_w, q_z;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "synchronizer");
    Synchronizer synchronizer;
    synchronizer.main_loop();
    return 0;
}