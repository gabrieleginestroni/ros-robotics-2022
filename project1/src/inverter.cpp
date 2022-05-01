#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "project1/RpmStamped.h"

#define T 5
#define PI 3.14159265358979323846

class Inverter {
public:
    Inverter(){
        this->sub = this->n.subscribe("cmd_vel", 1000, &Inverter::estimateRpm, this);
        this->pub = this->n.advertise<project1::RpmStamped>("wheels_rpm",1000);

        this->count = 0;

        this->n.getParam("/R", this->R);
        this->n.getParam("/LW", this->LW);

    }

    void main_loop() {
        ros::Rate loop_rate(10);
        while(ros::ok()){
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void estimateRpm(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        project1::RpmStamped rpm_message;
        double v_x,v_y,w;

        v_x = msg->twist.linear.x;
        v_y = msg->twist.linear.y;
        w = msg->twist.angular.z;

        rpm_message.header = msg->header;

        rpm_message.rpm_fl = (1 / this->R) * (v_x - v_y - this->LW * w) * T * 60;
        rpm_message.rpm_fr = (1 / this->R) * (v_x + v_y + this->LW * w) * T * 60;
        rpm_message.rpm_rl = (1 / this->R) * (v_x + v_y - this->LW * w) * T * 60;
        rpm_message.rpm_rr = (1 / this->R) * (v_x - v_y + this->LW * w) * T * 60;

        this->pub.publish(rpm_message);
        this->count++;


    }

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;

    double R, LW;

    int count;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "inverter");
    Inverter inverter;
    inverter.main_loop();
    return 0;

}
