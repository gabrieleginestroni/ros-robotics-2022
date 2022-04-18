#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"

#define B 1000000000
#define N 42
#define T 5
#define PI 3.14159265358979323846
#define R 0.07
#define W 0.169
#define L 0.2

class Subscriber {
    public:
        Subscriber(){
            this->sub = this->n.subscribe("wheel_states", 1000, &Subscriber::compVelocity, this);
            this->pub_ticks = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel_ticks",1000);
            this->pub_rpm = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel_rpm",1000);
            this->count = 0;
        }
        void main_loop() {
            ros::Rate loop_rate(10);
            while(ros::ok()){
                ros::spinOnce();
                loop_rate.sleep();
            }
        }
        void compVelocity(const sensor_msgs::JointState::ConstPtr& msg) {
            geometry_msgs::TwistStamped vel_msg_ticks;
            geometry_msgs::TwistStamped vel_msg_rpm;

            if (this->count>0){
                ros::Duration elapsed_time;
                double time_s;
                double delta_ticks[4];
                double w_ticks[4];
                double w_rpm[4];

                elapsed_time = ros::Time::now() - stamp;
                time_s = elapsed_time.toSec();

                ROS_INFO("elapsed time: %f\n", time_s);

                for (int i = 0; i < 4; i++) {
                    delta_ticks[i] = msg->position[i] - this->wheels_ticks_old[i];
                    ROS_INFO("delta ticks: %f\n", delta_ticks[i]);
                    w_ticks[i] = (delta_ticks[i] / time_s) * (2 * PI ) / (N * T);
                    w_rpm[i] = msg->velocity[i] / (60 * T);
                }

                vel_msg_ticks.twist.linear.x = (R / 4) * (w_ticks[0] + w_ticks[1] + w_ticks[2] + w_ticks[3]);
                vel_msg_ticks.twist.linear.y = (R / 4) * (w_ticks[1] - w_ticks[0] + w_ticks[2] - w_ticks[3]);
                vel_msg_ticks.twist.linear.z = 0.0;
                vel_msg_ticks.twist.angular.x = 0.0;
                vel_msg_ticks.twist.angular.y = 0.0;
                vel_msg_ticks.twist.angular.z = (R / 4) * (w_ticks[1] + w_ticks[3] - w_ticks[0] - w_ticks[2]) / (L+W);

                this->pub_ticks.publish(vel_msg_ticks);

                vel_msg_rpm.twist.linear.x = (R / 4) * (w_rpm[0] + w_rpm[1] + w_rpm[2] + w_rpm[3]);
                vel_msg_rpm.twist.linear.y = (R / 4) * (w_rpm[1] - w_rpm[0] + w_rpm[2] - w_rpm[3]);
                vel_msg_rpm.twist.linear.z = 0.0;
                vel_msg_rpm.twist.angular.x = 0.0;
                vel_msg_rpm.twist.angular.y = 0.0;
                vel_msg_rpm.twist.angular.z = (R / 4) * (w_rpm[1] + w_rpm[3] - w_rpm[0] - w_rpm[2]) / (L+W);

                this->pub_rpm.publish(vel_msg_rpm);

            }

            for (int i = 0; i < 4; i++)
                this->wheels_ticks_old[i] = msg->position[i];
            this->count++;
            this->stamp = ros::Time::now();

        }

    private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub_ticks;
        ros::Publisher pub_rpm;
        double wheels_ticks_old[4];
        ros::Time stamp;
        int count;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "sub");
    Subscriber my_sub;
    my_sub.main_loop();
    return 0;

}
