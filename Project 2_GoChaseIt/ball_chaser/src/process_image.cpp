#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO("Setting Robot Velocity LinearX: %1.2f AngularZ: %1.2f", lin_x, ang_z);
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x; 
    srv.request.angular_z = ang_z;
    
    if( !client.call(srv))
    {
        ROS_ERROR("Failed to call service DriveToTarget");
    }
    
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    int left = 0;
    int middle = 0;
    int right = 0;
    //Print_out statements used to understand camera image
    //ROS_INFO("IMAGE STEP: %i", img.step);
    //ROS_INFO("IMAGE HEIGHT: %i", img.height);
    //ROS_INFO("IMAGE WIDTH: %i", img.width);
    //ROS_INFO("IMAGE DETAILS: %s", img.encoding);
    for(int i = 0; i < img.height * img.step;i=i+3)
    {
        
        if ((img.data[i]== white_pixel)&& (img.data[i+1]== white_pixel) && (img.data[i+2]== white_pixel)) 
        {
            // To find location in terms of columns
            // From ROS Image Documentation, use img.width (#of columns)
            //ROS_INFO("Index of White image: %i", i);
            //ROS_INFO("Pixel Value: %i", img.data[i]); 
            int rowLoc = i % img.width;
            if (rowLoc <= img.width / 3)
            {
                left++;
            }
            else if (rowLoc <= 2*img.width / 3)
            {
                middle++;
            }
            else if (rowLoc <= img.width)
            {
                right++;
            }               
        }
    }


    //ROS_INFO("Left: %i", left);
    //ROS_INFO("Middle: %i", middle);
    //ROS_INFO("Right: %i", right);
    // Check if any white pixels were found
    if ((left != 0) || (middle != 0) || (right != 0)) 
    {
        if ((left > right) && (left > middle))
        {
            drive_robot(0.0, 0.6);
        }
        else if ((middle > left) && (middle > right))
        {
            drive_robot(0.4, 0.0);
        }
        else 
        {
            drive_robot(0.0, -0.6);
        }
    }
    else
    {
        drive_robot(0.0, 0.0);
    }
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
}


