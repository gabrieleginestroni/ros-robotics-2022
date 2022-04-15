#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"

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
            this->pub = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel",1000);
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
            double cur_stamp;
            geometry_msgs::TwistStamped vel_msg_ticks;
            geometry_msgs::TwistStamped vel_msg_rpm;

            if (this->count>0){
                float elapsed_time;
                int delta_ticks[4];
                float w_ticks[4];
                float w_rpm[4];

                if(msg->header.stamp.sec > this->stamp_sec)
                    elapsed_time = ((1000000000 - this->stamp_nsec) + msg->header.stamp.nsec) / 1000000000;
                else
                    elapsed_time = (msg->header.stamp.nsec - this->stamp_nsec) / 1000000000;

                for (int i = 0; i < 4; i++) {
                    delta_ticks[i] = msg->position[i] - this->wheels_ticks_old[i];
                    w_ticks[i] = (delta_ticks[i] / elapsed_time) * (2 * PI) / (N * T);
                    w_rpm[i] = msg->velocity[i] / T;
                }

                vel_msg_ticks.twist.linear.x = (R / 4) * (w_ticks[0] + w_ticks[1] + w_ticks[2] + w_ticks[3]);
                vel_msg_ticks.twist.linear.y = (R / 4) * (w_ticks[1] - w_ticks[0] + w_ticks[2] - w_ticks[3]);
                vel_msg_ticks.twist.linear.z = 0.0;
                vel_msg_ticks.twist.angular.x = 0.0;
                vel_msg_ticks.twist.angular.y = 0.0;
                vel_msg_ticks.twist.angular.z = (R / 4) * (w_ticks[1] + w_ticks[3] - w_ticks[0] - w_ticks[2]) / (L+W);

                this->pub.publish(vel_msg_ticks);

                vel_msg_rpm.twist.linear.x = (R / 4) * (w_rpm[0] + w_rpm[1] + w_rpm[2] + w_rpm[3]);
                vel_msg_rpm.twist.linear.y = (R / 4) * (w_rpm[1] - w_rpm[0] + w_rpm[2] - w_rpm[3]);
                vel_msg_rpm.twist.linear.z = 0.0;
                vel_msg_rpm.twist.angular.x = 0.0;
                vel_msg_rpm.twist.angular.y = 0.0;
                vel_msg_rpm.twist.angular.z = (R / 4) * (w_rpm[1] + w_rpm[3] - w_rpm[0] - w_rpm[2]) / (L+W);

                this->pub.publish(vel_msg_rpm);

            }

            for (int i = 0; i < 4; i++)
                this->wheels_ticks_old[i] = msg->position[i];
            this->count++;
            this->stamp_nsec = msg->header.stamp.nsec;
            this->stamp_sec = msg->header.stamp.sec;
        }

    private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        int wheels_ticks_old[4];
        int stamp_sec;
        int stamp_nsec;
        int count;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "sub");
    Subscriber my_sub;
    my_sub.main_loop();
    return 0;

}
