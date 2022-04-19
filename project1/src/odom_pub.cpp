#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"

#include <math.h>
#include <tf/transform_broadcaster.h>

enum class integrate_type {EULER, RUNGE_KUTTA};

class Odom_pub{
    public:
        Odom_pub(){
            this->sub = this->n.subscribe("cmd_vel", 1000, &Odom_pub::compOdometry, this);
            this->pub = this->n.advertise<nav_msgs::Odometry>("odom",1000);
            this->type = integrate_type::EULER;
            this->count = 0;
            this->n.getParam("/initial_pose_x", this->x_old);
            this->n.getParam("/initial_pose_j", this->y_old);
            this->n.getParam("/initial_pose_theta", this->theta_old);
        }

        void main_loop() {
            ros::Rate loop_rate(10);
            while(ros::ok()){
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        void compOdometry(const geometry_msgs::TwistStamped::ConstPtr& msg){
            double x_new, y_new, theta_new;
            nav_msgs::Odometry msg_odometry;
            x_new = 0.0;
            y_new = 0.0;
            if (this->count>0) {
                ros::Duration elapsed_time;
                double time_s;
                elapsed_time = msg->header.stamp - this->stamp;
                time_s = elapsed_time.toSec();

                if (this->type == integrate_type::EULER) {
                    x_new = this->x_old +
                            (msg->twist.linear.x * cos(theta_old) - msg->twist.linear.y * sin(theta_old)) * time_s;
                    ROS_INFO("time=%f", time_s);
                    ROS_INFO("old_x=%f, new_x=%f\n", x_old, x_new);
                    y_new = this->y_old +
                            (msg->twist.linear.x * sin(theta_old) + msg->twist.linear.y * cos(theta_old)) * time_s;
                    ROS_INFO("old_y=%f, new_y=%f\n", y_old, y_new);
                } else {
                    x_new = this->x_old +
                            (msg->twist.linear.x * cos(theta_old + msg->twist.angular.z * time_s * 0.5) -
                             msg->twist.linear.y * sin(theta_old + msg->twist.angular.z * time_s * 0.5)) * time_s;
                    y_new = this->y_old +
                            (msg->twist.linear.x * sin(theta_old + msg->twist.angular.z * time_s * 0.5) +
                             msg->twist.linear.y * cos(theta_old + msg->twist.angular.z * time_s * 0.5)) * time_s;
                }

                theta_new = this->theta_old + msg->twist.angular.z * time_s;

                msg_odometry.header = msg->header;
                msg_odometry.header.frame_id = "odom";

                msg_odometry.child_frame_id = "base_link";

                msg_odometry.pose.pose.position.x = x_new;
                msg_odometry.pose.pose.position.y = y_new;
                msg_odometry.pose.pose.position.z = 0.0;
                msg_odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_new);

                msg_odometry.twist.twist = msg->twist;

                /*
                for (int i = 0; i < 36; i++) {
                    msg_odometry.odom.pose.covariance[i] = 0.0;
                    msg_odometry.odom.twist.covariance[i] = 0.0;
                }
                */

                this->pub.publish(msg_odometry);
            }
            this->x_old = x_new;
            this->y_old = y_new;
            this->theta_old = theta_new;
            this->stamp = msg->header.stamp;
            this->count++;

        }

    private:
        //all services missing
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        ros::Time stamp;
        integrate_type type;
        double x_old;
        double y_old;
        double theta_old;
        int count;
        tf::TransformBroadcaster transform_broadcaster;
        tf::Transform transform;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_pub");
    Odom_pub odomPub;
    odomPub.main_loop();
    return 0;
}
