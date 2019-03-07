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

    // Create a SeekThermalPro camera
    LibSeek::SeekThermalPro seek("");
    if(!seek.open()) {
        ROS_ERROR("Unable to open SEEK Compact Pro camera");
        return -1;
    }

    // ROS helper class to run the loop at a given frequency (Hz)
    ros::Rate loop_rate(FREQ);
    while (nh.ok()) {
        // Read an image from the SEEK Compact PRO camera
        cv::Mat frame, grey_frame; // = cv::imread("/home/dylan/catkin_ws/src/libseek_thermal_driver/lena.jpeg", CV_LOAD_IMAGE_COLOR);
        if(!seek.read(frame)) {
            ROS_ERROR("SEEK Compact Pro camera cannot be read");
            return -2;
        }
        cv::normalize(frame, grey_frame, 0, 65535, cv::NORM_MINMAX);

        // Convert the image to a readable format for OpenCV
        grey_frame.convertTo(grey_frame, CV_8UC1, 1.0/256.0);
        cv::cvtColor(grey_frame, grey_frame, cv::COLOR_GRAY2BGR);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", grey_frame).toImageMsg();

        // Publish message to the ROS network
        pub.publish(msg);

        // Process callbacks from ROS, see https://answers.ros.org/question/11887/significance-of-rosspinonce/
        ros::spinOnce();

        // Sleep to comply with the given loop_rate
        loop_rate.sleep();
    }
}
