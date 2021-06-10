#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

class ProcessImage{

  public:
    ProcessImage(ros::NodeHandle *n)
    {
        // client service capable of requesting services from command_robot
        client = n->serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
        // Subscribe to /camera/rgb/image_raw topic to read the image data 
        // inside the process_image_callback function
        camera_subscriber = n->subscribe("/camera/rgb/image_raw", 10, 
                                 &ProcessImage::process_image_callback, this);
    }

    ~ProcessImage(){}

    // This function calls the command_robot service to drive the robot in the specified direction
    void drive_robot(float lin_x, float ang_z)
    {
        // Request a service and pass the velocities to it to drive the robot
        ball_chaser::DriveToTarget srv;
        srv.request.linear_x = lin_x;
        srv.request.angular_z = ang_z;

        if (!client.call(srv))
            ROS_ERROR("Failed to call service /ball_chaser/command_robot");
    }

    // This callback function continuously executes and reads the image data
    void process_image_callback(const sensor_msgs::Image img)
    {
        int white_pixel = 255;
        bool is_white_present = false;

        //Position with respect to camera
        int position;
        //Ball presence count in each position
        std::vector<int> ball_presence(3,0);
        geometry_msgs::Twist cmd_vel;

        // Loop through each pixel in the image and check if there's a bright white one
        // Then, identify if this pixel falls in the left, mid, or right side of the image
        // Depending on the white ball position, call the drive_bot function and pass velocities to it
        // Request a stop when there's no white ball seen by the camera
        for(int i = 0; i < img.height * img.step;i+=3)
        {
            if (img.data[i] == white_pixel && 
                img.data[i+1] == white_pixel && 
                img.data[i+2] == white_pixel)
            {
                is_white_present = true;
                position = i % img.step;
                if (position >= 0 && position < img.step/3 )
                    ball_presence[LEFT]++;
                else if (position >= img.step/3 && position < img.step * 2/3)
                    ball_presence[MID]++;
                else if (position >= img.step * 2/3)
                    ball_presence[RIGHT]++;
            }
        }

        if (is_white_present == false)
        {
          cmd_vel.linear.x = 0;
          cmd_vel.angular.z = 0;
        }
        else 
        if (ball_presence[MID] >= ball_presence[LEFT] && ball_presence[MID] >= ball_presence[RIGHT])
        {
          cmd_vel.linear.x = 0.5;
          cmd_vel.angular.z = 0;
        }
        else 
        if (ball_presence[LEFT] > ball_presence[MID] && ball_presence[LEFT] > ball_presence[RIGHT])
        {
          cmd_vel.linear.x = 0;
          cmd_vel.angular.z = 0.5;
        }
        else 
        if (ball_presence[RIGHT] > ball_presence[MID] && ball_presence[RIGHT] > ball_presence[LEFT])
        {
          cmd_vel.linear.x = 0;
          cmd_vel.angular.z = -0.5;
        }

        drive_robot(cmd_vel.linear.x,cmd_vel.angular.z);
    }

  private:
    // Service Client that can request services
    ros::ServiceClient client;
    // Subscriber for camera /camera/rgb/image_raw topic
    ros::Subscriber camera_subscriber;
    // Ball Presense inside the image
    enum PRESENCE{LEFT = 0, MID, RIGHT};
};

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    ProcessImage pi = ProcessImage(&n);
    
    // Handle ROS communication events
    ros::spin();

    return 0;
}
