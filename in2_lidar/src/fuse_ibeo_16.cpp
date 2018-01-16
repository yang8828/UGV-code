#include <ros/ros.h>
//#include <common/ScanInfoV2.h>
//#include <common/InsInfo.h>
#include <in2_msgs/ScanInfoV2.h>
#include <in2_msgs/InsInfo.h>
#include <in2_msgs/RoadSurface.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int16.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvwimage.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <math.h>
#include <inttypes.h>

#define PI 3.14159265357

//Meters
#define Concave_Threshold -2.4
#define CarLeftEdge	       0.9375	
#define CarRightEdge      -0.9375
#define CarFrontEdge       3.78
#define CarBackEdge	      -1.2

using namespace cv;
using namespace std;

typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

ros::Publisher scanInfov2_pub;
ros::Publisher roadsurface_pub;

int grid_dim_;
double m_per_cell_;
long TimeStamp;
double display_range;

bool flag_16l, flag_16r, flag_ibeo;
int rec_16l=0,rec_16r=0,rec_ibeo=0;
in2_msgs::ScanInfoV2 msg_16l;
in2_msgs::ScanInfoV2 msg_16r;
in2_msgs::ScanInfoV2 msg_ibeo;
bool showimages;
bool showmessages;

class C_ServerTime
{
public:
	void C_ServerTime_Init()
	{
		StartTime = GetLocalTime() - GetLocalTime() % 86400000;
	}
	long GetServerTime()
	{
		return GetLocalTime()-StartTime;
	}
private:
	int64_t GetLocalTime()
	{
		int64_t time = int64_t((ros::Time::now().toSec())*1000.0);
		return time;
	}
	int64_t StartTime;
};
C_ServerTime my_servertime;


void transfer16l(const in2_msgs::ScanInfoV2::ConstPtr &scaninfomsg_16l)
{
	msg_16l=*scaninfomsg_16l;
	flag_16l = true;
}

void transfer16r(const in2_msgs::ScanInfoV2::ConstPtr &scaninfomsg_16r)
{
	msg_16r=*scaninfomsg_16r;
	flag_16r = true;
}

void transferibeo(const in2_msgs::ScanInfoV2::ConstPtr &scaninfomsg_ibeo)
{
	msg_ibeo=*scaninfomsg_ibeo;
	flag_ibeo = true;	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "fuselidar_node");

	ros::NodeHandle node;
	my_servertime.C_ServerTime_Init();

	node.param("/fuselidar_node/cell_size", m_per_cell_, 0.05);
	node.param("/fuselidar_node/grid_dimensions", grid_dim_, 600);
	node.param("/fuselidar_node/showmessages", showmessages, false);
	node.param("/fuselidar_node/showimages",   showimages,   true );
	node.param("/fuselidar_node/display_range", display_range, 90.0);
	
	flag_16l  = false;
	flag_16r  = false;
	flag_ibeo = false;

	ros::Subscriber scaninfo_16l;
	ros::Subscriber scaninfo_16r;
	ros::Subscriber scaninfo_ibeo;

	scaninfo_16l  = node.subscribe("ScanInfoV2_16l",  10, &transfer16l,  ros::TransportHints().tcpNoDelay(true) );
	scaninfo_16r  = node.subscribe("ScanInfoV2_16r",  10, &transfer16r,  ros::TransportHints().tcpNoDelay(true) );
	scaninfo_ibeo = node.subscribe("ScanInfoV2_ibeo", 10, &transferibeo, ros::TransportHints().tcpNoDelay(true) );

	scanInfov2_pub = node.advertise<in2_msgs::ScanInfoV2>("ScanInfoV2", 1);
	roadsurface_pub = node.advertise<in2_msgs::RoadSurface>("RoadSurface",1);

	//Mat dismap(grid_dim_,grid_dim_,CV_8UC3,Scalar(128,128,128));
	Mat dismap(grid_dim_,grid_dim_,CV_8UC3,Scalar(128,128,128));

	//Time stamp
	long int StarTime = my_servertime.GetServerTime();
	
	TimeStamp = my_servertime.GetServerTime();

	ros::Rate looprate(10);
	while(ros::ok)
	{
		//watch dog: lost 10 frames = flag->false
		if(flag_ibeo)
			rec_ibeo = 0;
		else
			rec_ibeo ++;

		if(flag_16r)
			rec_16r = 0;
		else
			rec_16r ++;

		if(flag_16l)
			rec_16l = 0;
		else
			rec_16l ++;

		//init dismap
		for(int i=0; i<600; i++)
		{
			for(int j=0; j<600; j++)
			{
				dismap.at<Vec3b>(j,i)[0] = 128;
				dismap.at<Vec3b>(j,i)[1] = 128;
				dismap.at<Vec3b>(j,i)[2] = 128;
			}
		}

		//rectangle(dismap,Point(300-8,400-38),Point(300+8,400+12),Scalar(255,255,255),1);//vehicle
		rectangle( dismap,
			   Point( grid_dim_/2 - (0.85*grid_dim_)/(display_range), grid_dim_*2/3 - (3.8*grid_dim_)/(display_range) ),
			   Point( grid_dim_/2 + (0.85*grid_dim_)/(display_range), grid_dim_*2/3 + (1.2*grid_dim_)/(display_range) ),
			   Scalar(255,255,255),
			   1
			 );//vehicle

		int offset_map_x = ( (display_range-60.0)/2 ) / ( display_range/grid_dim_ );
		int offset_map_y = ( (display_range-60.0)/3 ) / ( display_range/grid_dim_ );

		//publish scaninfov2 msgs
		in2_msgs::ScanInfoV2 scaninfov2_msg;
		scaninfov2_msg.header.stamp = ros::Time::now();
		scaninfov2_msg.header.frame_id = "ScanInfoV2";
		scaninfov2_msg.sendtime = TimeStamp;

		bool isFilled[720];
		for(int i = 0; i < 720; i++)
		{
			double range_max=200.0;
			double theta_current=(i/2-180)*PI/180;

			scaninfov2_msg.ScanInfoX[i] = 1500.0-range_max*cos(theta_current)*50.0;
			scaninfov2_msg.ScanInfoY[i] = 1000.0+range_max*sin(theta_current)*50.0;
			
			isFilled[i]=false;
		}

		double theta_increment = PI / 180.0;
		int index;
		double index_d;


		if(showmessages) ROS_INFO("/**************************/");
		if( rec_ibeo < 5 )//500ms
		{
			for(int i = 0; i < 720; i++)
			{
				double theta_ibeo = atan2(msg_ibeo.ScanInfoY[i]-1000.0, 1500.0-msg_ibeo.ScanInfoX[i]);

				index_d = theta_ibeo / theta_increment;
				index = (int)( (index_d + 180) * 2 );
				if (index < 0  ) index = 0;
				if (index > 719) index = 719;

				if(!isFilled[index])
				{
					scaninfov2_msg.ScanInfoX[index] = msg_ibeo.ScanInfoX[i];
					scaninfov2_msg.ScanInfoY[index] = msg_ibeo.ScanInfoY[i];

					isFilled[index] = true;

					circle( dismap,
				    	//Point(scaninfov2_msg.ScanInfoX[index]/5,  grid_dim_-scaninfov2_msg.ScanInfoY[index]/5),
				    	Point( offset_map_x + scaninfov2_msg.ScanInfoX[index]/(50*display_range/grid_dim_), -offset_map_y + grid_dim_ - scaninfov2_msg.ScanInfoY[index]/(50*display_range/grid_dim_)),
				    	2,
				    	Scalar(0,0,255),
				        -1
					);
				}
				else
				{
					double norm1 = scaninfov2_msg.ScanInfoX[index]*scaninfov2_msg.ScanInfoX[index]
					                +scaninfov2_msg.ScanInfoY[index]*scaninfov2_msg.ScanInfoY[index];
					double norm2 = msg_ibeo.ScanInfoX[i]*msg_ibeo.ScanInfoX[i]
									+msg_ibeo.ScanInfoY[i]*msg_ibeo.ScanInfoY[i];

					if(norm2 < norm1)
					{
						scaninfov2_msg.ScanInfoX[index] = msg_ibeo.ScanInfoX[i];
						scaninfov2_msg.ScanInfoY[index] = msg_ibeo.ScanInfoY[i];

						circle( dismap,
				    		//Point(scaninfov2_msg.ScanInfoX[index]/5,  grid_dim_-scaninfov2_msg.ScanInfoY[index]/5),
				    		Point( offset_map_x + scaninfov2_msg.ScanInfoX[i]/(50*display_range/grid_dim_), -offset_map_y + grid_dim_ - scaninfov2_msg.ScanInfoY[i]/(50*display_range/grid_dim_)),
				    		2,
				    		Scalar(0,0,255),
				            -1
						);
					}
				}
			}
		}
		else
		{
			if(showmessages) ROS_INFO("ibeo: No New Message!");
			//if(showmessages) ROS_INFO("All Lidar: No New Message!");
		}
		
		if( rec_16r < 5)//500ms
		{
			for(int i = 0; i < 720; i ++)
			{

				double theta_16r = atan2(msg_16r.ScanInfoY[i]-1000.0, 1500.0-msg_16r.ScanInfoX[i]);

				if( (theta_16r < -90*PI/180 && theta_16r > -180*PI/180) || (theta_16r >= 90*PI/180 && theta_16r < 180*PI/180) )// to be confirmed
				{
					index_d = theta_16r / theta_increment;
					index = (int)( (index_d + 180) * 2 );
					if (index < 0  ) index = 0;
					if (index > 719) index = 719;

					if(!isFilled[index])
					{
						scaninfov2_msg.ScanInfoX[index] = msg_16r.ScanInfoX[i];
						scaninfov2_msg.ScanInfoY[index] = msg_16r.ScanInfoY[i];

						circle( dismap,
				    		//Point(scaninfov2_msg.ScanInfoX[index]/5,  grid_dim_-scaninfov2_msg.ScanInfoY[index]/5),
				    		Point( offset_map_x + scaninfov2_msg.ScanInfoX[i]/(50*display_range/grid_dim_), -offset_map_y + grid_dim_ - scaninfov2_msg.ScanInfoY[i]/(50*display_range/grid_dim_)),
				    		1,
				    		Scalar(0,255,255),
				            -1
						);
					}
				}
			}
		}
		else
		{
			if(showmessages) ROS_INFO("16E Right: No New Message!");
		}

		if( rec_16l < 5 )//500ms
		{
			for(int i = 0; i < 720; i ++)
			{

				double theta_16l = atan2(msg_16l.ScanInfoY[i]-1000.0, 1500.0-msg_16l.ScanInfoX[i]);
		
				if( theta_16l >= -90*PI/180 && theta_16l < 90*PI/180 )
				{
					index_d = theta_16l / theta_increment;
					index = (int)( (index_d + 180) * 2 );
					if (index < 0  ) index = 0;
					if (index > 719) index = 719;

					if(!isFilled[index])
					{
						scaninfov2_msg.ScanInfoX[index] = msg_16l.ScanInfoX[i];
						scaninfov2_msg.ScanInfoY[index] = msg_16l.ScanInfoY[i];

						circle( dismap,
				    		//Point(scaninfov2_msg.ScanInfoX[index]/5,  grid_dim_-scaninfov2_msg.ScanInfoY[index]/5),
				    		Point( offset_map_x + scaninfov2_msg.ScanInfoX[i]/(50*display_range/grid_dim_), -offset_map_y + grid_dim_ - scaninfov2_msg.ScanInfoY[i]/(50*display_range/grid_dim_)),
				    		1,
				    		Scalar(255,255,0),
				            -1
						);
					}
				}
			}
		}
		else
		{
			if(showmessages) ROS_INFO("16E Left: No New Message!");
		}
		if(showmessages) ROS_INFO("           ");

		//filter process
		double scaninfo_norm[720];
		double filter_norm[720];

		for(int i =0; i<720; i++)
		{
			scaninfo_norm[i]=sqrt( 
				 (scaninfov2_msg.ScanInfoX[i]-1500)*(scaninfov2_msg.ScanInfoX[i]-1500)
				+(scaninfov2_msg.ScanInfoY[i]-1000)*(scaninfov2_msg.ScanInfoY[i]-1000) 
				);
		}

		filter_norm[0]=scaninfo_norm[0];
		filter_norm[719]=scaninfo_norm[719];
		for(int i =1; i<719; i++)
		{
			if(scaninfo_norm[i] <= scaninfo_norm[i+1] && scaninfo_norm[i] <= scaninfo_norm[i-1])
			{
				filter_norm[i] = scaninfo_norm[i];
			}
			else if(scaninfo_norm[i-1] <= scaninfo_norm[i+1] && scaninfo_norm[i-1] <= scaninfo_norm[i])
			{
				filter_norm[i] = scaninfo_norm[i-1];
			}
			//if(scaninfo_norm[i+1]<=scaninfo_norm[i] && scaninfo_norm[i+1] <= scaninfo_norm[i-1])
			else
			{
				filter_norm[i] = scaninfo_norm[i+1];
			}
		}
		
		for(int i = 0; i < 720; i++)
		{			
    		double theta_cur=(i/2-180)*PI/180;

			scaninfov2_msg.ScanInfoX[i] = -filter_norm[i]*cos(theta_cur)+1500;
			scaninfov2_msg.ScanInfoY[i] =  filter_norm[i]*sin(theta_cur)+1000;
		}

		for(int i = 0; i < 720; i++)
		{	


			line( dismap,
				  Point( offset_map_x + scaninfov2_msg.ScanInfoX[i]/(50*display_range/grid_dim_),     -offset_map_y + grid_dim_ - scaninfov2_msg.ScanInfoY[i]/(50*display_range/grid_dim_)),
				  Point( offset_map_x + scaninfov2_msg.ScanInfoX[(i+1)%720]/(50*display_range/grid_dim_), -offset_map_y + grid_dim_ - scaninfov2_msg.ScanInfoY[(i+1)%720]/(50*display_range/grid_dim_) ),
				  Scalar(255,0,0),
				  1
				);

			/*line( dismap,
				  Point(scaninfov2_msg.ScanInfoX[i]  /5,  grid_dim_-scaninfov2_msg.ScanInfoY[i]  /5),
				  Point(scaninfov2_msg.ScanInfoX[(i+1)%720]/5,  grid_dim_-scaninfov2_msg.ScanInfoY[(i+1)%720]/5),
				  Scalar(255,0,0),
				  1
				);*/
			/*circle( dismap,
			    Point(scaninfov2_msg.ScanInfoX[i]/5,  grid_dim_-scaninfov2_msg.ScanInfoY[i]/5),
			    1,
			    Scalar(255,0,0),
			    -1
			);*/

			/*circle( dismap,
			    Point(msg_ibeo.ScanInfoX[i]/5,  grid_dim_- msg_ibeo.ScanInfoY[i]/5),
			    1,
			    Scalar(0,0,255),
			    -1
			);*/
		}

		scanInfov2_pub.publish(scaninfov2_msg);

		in2_msgs::RoadSurface roadsurface_msg;
		roadsurface_msg.header.stamp = ros::Time::now();
		roadsurface_msg.header.frame_id = "RoadSurface";

		for(int i=0; i<720; i++)
		{
			roadsurface_msg.points[i].x = ( (float)(scaninfov2_msg.ScanInfoX[i] - 1500.0) )*60.0/3000.0;
			roadsurface_msg.points[i].y = ( (float)(scaninfov2_msg.ScanInfoY[i] - 1000.0) )*60.0/3000.0;
		}
		roadsurface_pub.publish(roadsurface_msg);

		if(showimages)
		{
			imshow("Passable Way (After fusion)",dismap);
			waitKey(1);
		}

		flag_ibeo = false;
		flag_16r  = false;
		flag_16l  = false;

		ros::spinOnce();
		looprate.sleep();
	}

	//ros::spin();
	ros::shutdown();
	return 0;
}
