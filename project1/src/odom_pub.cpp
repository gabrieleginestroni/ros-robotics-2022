#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "project1/reset_odom_to_pose.h"

#include <math.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <project1/parametersConfig.h>

enum class integration_type {EULER, RUNGE_KUTTA};

class Odom_pub{
    public:
        Odom_pub(){
            this->sub = this->n.subscribe("cmd_vel", 1000, &Odom_pub::compOdometry, this);
            this->pub = this->n.advertise<nav_msgs::Odometry>("odom",1000);

            this->n.getParam("/initial_pose_x", this->x_old);
            this->n.getParam("/initial_pose_j", this->y_old);
            this->n.getParam("/initial_pose_theta", this->theta_old);

            this->reset_odom_to_pose_service = n.advertiseService("reset_odom_to_pose",
                                                                            &Odom_pub::reset_odom_to_pose,
                                                                            this);

            dynamic_reconfigure::Server<project1::parametersConfig>::CallbackType dynRecCallback;
            dynRecCallback = boost::bind(&odom_pub::setIntegrationType, this, _1, _2);
            this->parameters_server.setCallback(dynRecCallback);
            this->integrationType = integration_type::EULER;
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
            ros::Duration elapsed_time;
            double time_s;
            nav_msgs::Odometry msg_odometry;

            elapsed_time = msg->header.stamp - this->stamp;
            time_s = elapsed_time.toSec();

            if (this->integrationType == integration_type::EULER) {
                //0 indicates that we're using Euler method
                msg_odometry.pose.covariance[0] = 0.0;

                x_new = this->x_old +
                        (msg->twist.linear.x * cos(theta_old) - msg->twist.linear.y * sin(theta_old)) * time_s;
                y_new = this->y_old +
                        (msg->twist.linear.x * sin(theta_old) + msg->twist.linear.y * cos(theta_old)) * time_s;
            } else {
                //1 indicates that we're using Runge_Kutta method
                msg_odometry.pose.covariance[0] = 1.0;

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
                msg_odometry.pose.covariance[i] = 0.0;
                msg_odometry.twist.covariance[i] = 0.0;
            }
            */

            this->pub.publish(msg_odometry);


            transform.setOrigin( tf::Vector3(x_new, y_new, 0));
            tf::Quaternion q;
            q.setRPY(0, 0, theta_new);
            transform.setRotation(q);
            transform_broadcaster.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "base_link"));

            this->x_old = x_new;
            this->y_old = y_new;
            this->theta_old = theta_new;
            this->stamp = msg->header.stamp;

        }

        bool reset_odom_to_pose(project1::reset_odom_to_pose::Request  &req,
                                    project1::reset_odom_to_pose::Response &res) {

            this->x_old = req.new_x;
            this->y_old = req.new_y;
            this->theta_old = req.new_theta;

            return true;
        }

        void setIntegrationType(project1::parametersConfig &paramServer, uint32_t level) {
            switch (paramServer.integration_method) {
                case 0:
                    this->integrationType = integration_type::EULER;
                    break;
                case 1:
                    this->integrationType = integration_type::RUNGE_KUTTA;
                    break;
            }
        }

    private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        ros::Time stamp;
        ros::ServiceServer reset_odom_to_pose_service;

        integration_type integrationType;

        double x_old;
        double y_old;
        double theta_old;

        tf::TransformBroadcaster transform_broadcaster;
        tf::Transform transform;

        dynamic_reconfigure::Server<project1::parametersConfig> parameters_server;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_pub");
    Odom_pub odomPub;
    odomPub.main_loop();
    return 0;
}
