#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "project1/RpmStamped.h"

#define N 40
#define T 5
#define PI 3.14159265358979323846
#define R 0.07
#define W 0.169
#define L 0.2

enum vel_type {RPM, TICKS};

class Vel_pub {
    public:
        Vel_pub(){
            this->sub = this->n.subscribe("wheel_states", 1000, &Vel_pub::compVelocity, this);
            this->pub = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel",1000);
            this->ticks_pub = this->n.advertise<project1::RpmStamped>("w_ticks",1000);
            this->rpm_pub = this->n.advertise<project1::RpmStamped>("w_rpm",1000);

            this->count = 0;
            this->type = RPM;
        }

        void main_loop() {
            ros::Rate loop_rate(10);
            while(ros::ok()){
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        void compVelocity(const sensor_msgs::JointState::ConstPtr& msg) {
            geometry_msgs::TwistStamped vel_msg;
            project1::RpmStamped w_ticks_msg;
            project1::RpmStamped w_rpm_msg;

            if (this->count>0){
                ros::Duration elapsed_time;
                double time_s, delta_ticks[4], w_ticks[4], w_rpm[4];

                elapsed_time = msg->header.stamp - this->stamp;
                time_s = elapsed_time.toSec();

                for (int i = 0; i < 4; i++) {
                    delta_ticks[i] = msg->position[i] - this->wheels_ticks_old[i];
                    w_ticks[i] = (delta_ticks[i] / time_s) * (2 * PI ) / (N * T);
                    w_rpm[i] = msg->velocity[i] / (60 * T);
                }

                w_ticks_msg.header = msg->header;
                w_ticks_msg.rpm_fl = w_ticks[0];
                w_ticks_msg.rpm_fr = w_ticks[1];
                w_ticks_msg.rpm_rl = w_ticks[2];
                w_ticks_msg.rpm_rr = w_ticks[3];

                w_rpm_msg.header = msg->header;
                w_rpm_msg.rpm_fl = w_rpm[0];
                w_rpm_msg.rpm_fr = w_rpm[1];
                w_rpm_msg.rpm_rl = w_rpm[2];
                w_rpm_msg.rpm_rr = w_rpm[3];

                this->ticks_pub.publish(w_ticks_msg);
                this->rpm_pub.publish(w_rpm_msg);

                vel_msg.header = msg->header;
                if(this->type == RPM){
                    vel_msg.twist.linear.x = (R / 4) * (w_rpm[0] + w_rpm[1] + w_rpm[2] + w_rpm[3]);
                    vel_msg.twist.linear.y = (R / 4) * (w_rpm[1] - w_rpm[0] + w_rpm[2] - w_rpm[3]);
                    vel_msg.twist.angular.z = (R / 4) * (w_rpm[1] + w_rpm[3] - w_rpm[0] - w_rpm[2]) / (L+W);
                } else {
                    vel_msg.twist.linear.x = (R / 4) * (w_ticks[0] + w_ticks[1] + w_ticks[2] + w_ticks[3]);
                    vel_msg.twist.linear.y = (R / 4) * (w_ticks[1] - w_ticks[0] + w_ticks[2] - w_ticks[3]);
                    vel_msg.twist.angular.z = (R / 4) * (w_ticks[1] + w_ticks[3] - w_ticks[0] - w_ticks[2]) / (L+W);
                }
                vel_msg.twist.linear.z = 0.0;
                vel_msg.twist.angular.x = 0.0;
                vel_msg.twist.angular.y = 0.0;
                this->pub.publish(vel_msg);

            }

            for (int i = 0; i < 4; i++)
                this->wheels_ticks_old[i] = msg->position[i];

            this->count++;
            this->stamp = msg->header.stamp;

        }

    private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        ros::Publisher ticks_pub;
        ros::Publisher rpm_pub;
        ros::Time stamp;

        vel_type type;

        double wheels_ticks_old[4];
        int count;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_pub");
    Vel_pub velPub;
    velPub.main_loop();
    return 0;

}
