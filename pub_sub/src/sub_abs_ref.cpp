#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"

#define B 1000000000

class Subscriber {
public:
    Subscriber(){
        this->sub_abs = this->n.subscribe("robot/pose", 1000, &Subscriber::compVelocity, this);
        this->pub_x = this->n.advertise<std_msgs::Float64>("x_abs_vel",1000);
        this->pub_y = this->n.advertise<std_msgs::Float64>("y_abs_vel",1000);

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
        double elapsed_time;
        double delta_x;
        double delta_y;
        std_msgs::Float64 vel_msg_x;
        std_msgs::Float64 vel_msg_y;

        if (this->count>0){
            if(msg->header.stamp.sec > this->stamp_sec)
                elapsed_time = (1000000000 - this->stamp_nsec) + msg->header.stamp.nsec;
            else
                elapsed_time = msg->header.stamp.nsec - this->stamp_nsec;
            vel_msg_x.data = (msg->pose.position.x - x_last_pose) * B / elapsed_time;
            vel_msg_y.data = (msg->pose.position.y - y_last_pose) * B / elapsed_time;
            this->pub_x.publish(vel_msg_x);
            this->pub_y.publish(vel_msg_y);
        }
        this->count++;
        this->stamp_nsec = msg->header.stamp.nsec;
        this->stamp_sec = msg->header.stamp.sec;
        this->x_last_pose = msg->pose.position.x;
        this->y_last_pose = msg->pose.position.y;
    }

private:
    ros::NodeHandle n;
    ros::Subscriber sub_abs;
    ros::Publisher pub_x;
    ros::Publisher pub_y;
    int count;
    int stamp_sec;
    int stamp_nsec;
    double x_last_pose;
    double y_last_pose;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "abs");
    Subscriber my_sub;
    my_sub.main_loop();
    return 0;

}
