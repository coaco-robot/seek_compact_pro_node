#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <seek/seek.h>
#define FREQ 10 

int main(int argc, char** argv)
{
    // Init ROS publisher
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    // Create an image message and advertise it to the ROS network
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("seek/image", 1);

    // ROS helper class to run the loop at a given frequency (Hz)
    ros::Rate loop_rate(FREQ);
    while (nh.ok()) {

        // Read an image from the SEEK Compact PRO camera
        cv::Mat image = cv::imread("/home/dylan/catkin_ws/src/libseek_thermal_driver/lena.jpeg", CV_LOAD_IMAGE_COLOR);
        if(image.empty()) {
            ROS_ERROR("SEEK Compact Pro camera cannot be read");
            exit(1);
        }
        // Convert the image to a readable format for OpenCV
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

        // Publish message to the ROS network
        pub.publish(msg);

        // Process callbacks from ROS, see https://answers.ros.org/question/11887/significance-of-rosspinonce/
        ros::spinOnce();

        // Sleep to comply with the given loop_rate
        loop_rate.sleep();
    }
}
