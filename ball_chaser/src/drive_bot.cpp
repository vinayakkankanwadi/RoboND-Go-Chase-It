#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

class BallChaser{

  public:
    BallChaser(ros::NodeHandle *n)
    {
        // Inform ROS master that we will be publishing a message 
        // of type geometry_msgs::Twist on the robot actuation topic 
        // with a publishing queue size of 10
        motor_command_publisher = n->advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Drive /ball_chaser/command_robot service with 
        // a handle_drive_request callback function
        cmd_vel_service = n->advertiseService("/ball_chaser/command_robot", &BallChaser::handle_drive_request,this);
    }

    ~BallChaser(){};

     // handle_drive_request callback function that executes 
     // whenever a drive_bot service is requested
     // This function publishes the requested linear x 
     // and angular velocities to the robot wheel joints
     // After publishing the requested velocities, 
     // a message feedback is returned with the requested wheel velocities
    bool 
    handle_drive_request( ball_chaser::DriveToTarget::Request& req, 
                          ball_chaser::DriveToTarget::Response& res)
    {
        // Publish requested motor command linear x and angular velocities
        motor_command.linear.x = req.linear_x;
        motor_command.angular.z = req.angular_z;
        motor_command_publisher.publish(motor_command);

        // Respond with feedback message with requested wheel velocities
        res.msg_feedback = "Linear x: " + std::to_string(motor_command.linear.x) + 
                           " - Angular z: " + std::to_string(motor_command.angular.z);
        ROS_INFO_STREAM(res.msg_feedback);
    }

  private:
    // ROS::Publisher motor commands;
    ros::Publisher motor_command_publisher; 
    // ROS::ServiceServer command velocity;
    ros::ServiceServer cmd_vel_service;
    // geometry_msgs::Twist motor command
    geometry_msgs::Twist motor_command;
};



int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Perform BallChaser Service
    BallChaser bc = BallChaser(&n);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
