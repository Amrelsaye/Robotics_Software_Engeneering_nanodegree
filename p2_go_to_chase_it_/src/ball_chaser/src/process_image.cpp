#include "ros/ros.h"

#include "ball_chaser/DriveToTarget.h"

#include <sensor_msgs/Image.h>



// Define a global client that can request services

ros::ServiceClient client;



// This function calls the command_robot service to drive the robot in the specified direction

void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Moving robot");

    // Request a service and pass the velocities to it to drive the robot


    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = (float)lin_x;
    srv.request.angular_z = (float)ang_z;


// This callback function continuously executes and reads the image data

 if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");

}



void process_image_callback(const sensor_msgs::Image img)

{



    int white_pixel = 255;
    bool ball_exist = 0;
    int index = 0;


    //  Loop through each pixel in the image and check if there's a bright white one

    // Then, identify if this pixel falls in the left, mid, or right side of the image

    // Depending on the white ball position, call the drive_bot function and pass velocities to it

    // Request a stop when there's no white ball seen by the camera



for (int i=0; i < img.height * img.step; i += 3)
    {
        if ((img.data[i] == 255) && (img.data[i+1] == 255) && (img.data[i+2] == 255))
        {
            index = i % img.step;
 
            if (index < img.step/3)

                drive_robot(0.5, 1);

            else if (index < (img.step/3 * 2))

                drive_robot(0.5, 0); 

            else

                drive_robot(0.5, -1);

            ball_exist = true;

            break;
         }
     }

     if (ball_exist == false)
         drive_robot(0, 0);             
  }





int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
