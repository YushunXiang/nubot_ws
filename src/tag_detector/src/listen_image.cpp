#include<ros/ros.h> 

#include<cv_bridge/cv_bridge.h> 

#include<sensor_msgs/image_encodings.h> 

#include<image_transport/image_transport.h> 

#include<opencv2/opencv.hpp> 

#include<opencv2/highgui/highgui.hpp>

#include<opencv2/imgproc/imgproc.hpp> 

#include<stdio.h> 

#include<math.h> 

#include<vector> 

void imageCalllback(const sensor_msgs::ImageConstPtr& msg) 

{

    ROS_INFO("Received \n");

    try

   {

	cv::imshow( "video", cv_bridge::toCvShare(msg, "bgr8")->image ); 

	cv::waitKey(30); 

    }

    catch( cv_bridge::Exception& e )

    {

	  ROS_ERROR( "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str() );

    }

} 



int main(int argc, char** argv) 

{

	 ros::init(argc, argv, "image_listener"); 

	 ros::NodeHandle n; cv::namedWindow("video");

	 cv::startWindowThread();

	 image_transport::ImageTransport it(n);

	 image_transport::Subscriber sub = it.subscribe( "video_image", 1, imageCalllback );

	 ros::spin();

     cv::destroyWindow("video");

 return 0;

}


