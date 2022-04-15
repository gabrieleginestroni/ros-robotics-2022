#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"

class Subscriber {
    public:
        Subscriber(){
            this->sub = this->n.subscribe("robot/pose", 1000, &Subscriber::compVelocity, this);
            this->pub = this->n.advertise<std_msgs::Float64>("velocities",1000);
            this->count = 0;
        }
        void main_loop() {
            ros::Rate loop_rate(10);
            while(ros::ok()){
                ros::spinOnce();
                loop_rate.sleep();
            }
        }
        void compVelocity(const geometry_msgs::PoseStamped::ConstPtr& msg) {
            float x_cur_pose,y_cur_pose;
            float velocity;
            double cur_stamp;
            float delta_x;
            float delta_y;
            std_msgs::Float64 vel_msg;

            if (this->count>0){
              x_cur_pose = msg->pose.position.x;
              y_cur_pose = msg->pose.position.y;
              cur_stamp = msg->header.stamp.nsec;
              delta_x = x_cur_pose-this->x_last_pose;
              delta_y = y_cur_pose-this->y_last_pose;
              velocity = sqrt(delta_x * delta_x + delta_y * delta_y) / ((cur_stamp -this->stamp)/ 1000000000);
              ROS_INFO("Delta x: %f",delta_x);
              ROS_INFO("Delta y: %f",delta_y);
              ROS_INFO("Delta time: %lf",(cur_stamp -this->stamp)/ 1000000000);
              vel_msg.data = velocity;
              this->pub.publish(vel_msg);
            }
            this->count++;
            this->stamp = msg->header.stamp.nsec;
            this->x_last_pose = msg->pose.position.x;
            this->y_last_pose = msg->pose.position.y;
        }

    private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        int count;
        double stamp;
        float x_last_pose;
        float y_last_pose;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "sub");
    Subscriber my_sub;
    my_sub.main_loop();
    return 0;

}
