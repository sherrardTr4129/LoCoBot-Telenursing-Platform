/*
 * Author: Trevor Sherrard 
 * Course: Directed Research (Summer 2021)
 * Since: June 19, 2021
 * Purpose: This node subscribes to a given compressed image stream,
 *          Modifies the image accordingly, and then re-publishes to a
 *          new topic that can be viewed within the web interface through
 *          the use of the web_video_server package.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace cv;

// declare topic names
static const std::string SUB_TOPIC_NAME = "/camera/color/image_raw";
static const std::string PUB_TOPIC_NAME = "/camera/color/modified_stream";

// set up image publisher
image_transport::Publisher image_pub;

void imageProcCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    Mat image_to_proc = cv_ptr->image;

    // TODO: proc image

    // convert opencv image back to img message
    sensor_msgs::ImagePtr new_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_to_proc).toImageMsg();

    // try to publush image
    image_pub.publish(new_msg);
    
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


int main(int argc, char **argv)
{
    // init node
    ros::init(argc, argv, "stream_modification_engine_node");

    // set up nodehandle and image transport
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // set up image subscriber
    image_transport::Subscriber sub_image = it.subscribe(SUB_TOPIC_NAME, 1, imageProcCallback);

    // init image publisher
    image_pub = it.advertise(PUB_TOPIC_NAME, 1);

    // spin forever
    ros::spin();
}

