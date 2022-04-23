#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

class Subscriber{
    public:
        Subscriber(){
            this->pose_sub = this->n.subscribe("robot/pose", 1000, &Subscriber::addMeasure, this);
            this->odom_sub = this->n.subscribe("odom", 1000, &Subscriber::computeError, this);

            this->cumErrorX = 0.0;
            this->cumErrorY = 0.0;
            this->maxErrorX = 0.0;
            this->maxErrorY = 0.0;
        }

        void main_loop() {
            ros::Rate loop_rate(10);
            while(ros::ok()){
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        void addMeasure(const geometry_msgs::PoseStamped::ConstPtr& msg){
            this->cumErrorX =- abs(msg->pose.position.x);
            this->cumErrorY =- abs(msg->pose.position.y);
        }

        void computeError(const nav_msgs::Odometry::ConstPtr& msg){
            this->cumErrorX =+ abs(msg->pose.pose.position.x);
            this->cumErrorY =+ abs(msg->pose.pose.position.y);
            if(this->cumErrorX > this->maxErrorX)
                this->maxErrorX = this->cumErrorX;
            if(this->cumErrorY > this->maxErrorY)
                this->maxErrorY = this->cumErrorY;
            ROS_INFO("\nerror on the X: %f\nerror on the Y: %f", this->cumErrorX, this->cumErrorY);
            ROS_INFO("\nmax error on the X: %f\nmax error on the Y: %f", this->maxErrorX, this->maxErrorY);
        }

    private:
        ros::NodeHandle n;
        ros::Subscriber pose_sub, odom_sub;
        double cumErrorX, cumErrorY, maxErrorX, maxErrorY;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "error");
    Subscriber my_sub;
    my_sub.main_loop();
    return 0;
}