#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <in2_ins/InsInfo.h>

static const std::string OPENCV_WINDOW = "Heightmap";

ros::Subscriber ins_sub_;
in2_ins::InsInfo ins;

//double Angle;
//double leftdis;
//double rightdis;
//double leftb;
//double rightb;

void imageConvert(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	//edgemain(cv_ptr->image, &Angle, &leftdis, &rightdis, &leftb, &rightb);

	// Update GUI Window
	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	cv::waitKey(5);  
}

void transferINS(const in2_ins::InsInfo::ConstPtr &ins_msg)
{
	ins = *ins_msg;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "astar_node");
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_(nh_);
	image_transport::Subscriber image_sub_;
	ins_sub_ = nh_.subscribe("/ins",1,&transferINS);
	image_sub_ = it_.subscribe("/heightmap1000", 1, &imageConvert);
	cvStartWindowThread();
	cv::namedWindow(OPENCV_WINDOW,0);
	ros::spin();
	return 0;
}
