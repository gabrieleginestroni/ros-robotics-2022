#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"


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
        ros::Duration elapsed_time;
        double time_s;
        double delta_x;
        double delta_y;
        std_msgs::Float64 vel_msg_x;
        std_msgs::Float64 vel_msg_y;

        if (this->count>0){
            elapsed_time = ros::Time::now() - stamp;
            time_s = elapsed_time.toSec();

            vel_msg_x.data = (msg->pose.position.x - x_last_pose)  / time_s;
            vel_msg_y.data = (msg->pose.position.y - y_last_pose)  / time_s;
            this->pub_x.publish(vel_msg_x);
            this->pub_y.publish(vel_msg_y);
        }
        this->count++;
        this->stamp = ros::Time::now();
        this->x_last_pose = msg->pose.position.x;
        this->y_last_pose = msg->pose.position.y;
    }

private:
    ros::NodeHandle n;
    ros::Subscriber sub_abs;
    ros::Publisher pub_x;
    ros::Publisher pub_y;
    int count;
    ros::Time stamp;
    double x_last_pose;
    double y_last_pose;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "abs");
    Subscriber my_sub;
    my_sub.main_loop();
    return 0;

}
