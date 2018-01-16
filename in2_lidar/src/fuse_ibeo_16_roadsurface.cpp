#include <ros/ros.h>
#include <in2_msgs/ScanInfoV2.h>
#include <in2_msgs/InsInfo.h>
#include <in2_msgs/RoadSurface.h>
#include <in2_msgs/RoadSurface3D.h>
#include <in2_msgs/GisInfo.h>
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

ros::Publisher roadsurface_pub;
ros::Publisher scaninfov2_pub;

int grid_dim_;
double m_per_cell_;
long TimeStamp;
double display_range;

bool flag_16l, flag_16r, flag_ibeo;
int rec_16l=0,rec_16r=0,rec_ibeo=0;
in2_msgs::RoadSurface3D msg_16l;
in2_msgs::RoadSurface3D msg_16r;
in2_msgs::RoadSurface3D msg_ibeo;
bool showimages;
bool showmessages;
int hungedobstacle;

bool is_in_suidao = false;
bool is_in_fee = false;

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


void transfer16l(const in2_msgs::RoadSurface3D::ConstPtr &roadsurfacemsg_16l)
{
	msg_16l=*roadsurfacemsg_16l;
	flag_16l = true;
}

void transfer16r(const in2_msgs::RoadSurface3D::ConstPtr &roadsurfacemsg_16r)
{
	msg_16r=*roadsurfacemsg_16r;
	flag_16r = true;
}

void transferibeo(const in2_msgs::RoadSurface3D::ConstPtr &roadsurfacemsg_ibeo)
{
	msg_ibeo=*roadsurfacemsg_ibeo;
	flag_ibeo = true;	
}

void transferGISRect(const in2_msgs::GisInfo::ConstPtr &gisRect_msg)
{
	// suidao
	if (gisRect_msg->LaneChangeFlag == 4) is_in_suidao = true;
	else                                                              is_in_suidao = false;

	// TODO: transfer is_in_fee
	if (gisRect_msg->LaneChangeFlag == 5) is_in_fee = true;
	else                                                              is_in_fee = false;
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
	node.param("/fuselidar_node/display_range", display_range, 40.0);
	
	flag_16l  = false;
	flag_16r  = false;
	flag_ibeo = false;

	ros::Subscriber roadsurface_16l;
	ros::Subscriber roadsurface_16r;
	ros::Subscriber roadsurface_ibeo;
	ros::Subscriber gisRect_sub_ ;

	roadsurface_16l  = node.subscribe("RoadSurface_16l",  10, &transfer16l,  ros::TransportHints().tcpNoDelay(true) );
	roadsurface_16r  = node.subscribe("RoadSurface_16r",  10, &transfer16r,  ros::TransportHints().tcpNoDelay(true) );
	roadsurface_ibeo = node.subscribe("RoadSurface_ibeo", 10, &transferibeo, ros::TransportHints().tcpNoDelay(true) );
	gisRect_sub_ = node.subscribe("gis_info",10,&transferGISRect,ros::TransportHints().tcpNoDelay(true)); // earth frame

	roadsurface_pub = node.advertise<in2_msgs::RoadSurface>("RoadSurface",1);
	scaninfov2_pub = node.advertise<in2_msgs::ScanInfoV2>("ScanInfoV2",1);

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

		//publish scaninfov2 msgs
		in2_msgs::RoadSurface3D roadsurface3d_msg;


		bool isFilled[720];
		for(int i = 0; i < 720; i++)
		{
			double range_max=200.0;
			double theta_current=(i/2-180)*PI/180;

			roadsurface3d_msg.points[i].x = - range_max*cos(theta_current);
			roadsurface3d_msg.points[i].y =   range_max*sin(theta_current);
			roadsurface3d_msg.points[i].z =   0.0;
			
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
				double theta_ibeo = atan2(msg_ibeo.points[i].y, -msg_ibeo.points[i].x);

				index_d = theta_ibeo / theta_increment;
				index = (int)( (index_d + 180) * 2 );
				if (index < 0  ) index = 0;
				if (index > 719) index = 719;

				if(!isFilled[index])
				{
					roadsurface3d_msg.points[index].x = msg_ibeo.points[i].x;
					roadsurface3d_msg.points[index].y = msg_ibeo.points[i].y;
					roadsurface3d_msg.points[index].z = msg_ibeo.points[i].z;

					isFilled[index] = true;

					circle( dismap,
						    Point( roadsurface3d_msg.points[index].x/(display_range/grid_dim_)+grid_dim_/2, grid_dim_*2/3-roadsurface3d_msg.points[index].y/(display_range/grid_dim_) ),
						    2,
						    Scalar(0,0,255),
						    -1
				  		);
				}
				else
				{
					double norm1 = roadsurface3d_msg.points[index].x * roadsurface3d_msg.points[index].x
					                + roadsurface3d_msg.points[index].y * roadsurface3d_msg.points[index].y;
					double norm2 = msg_ibeo.points[i].x * msg_ibeo.points[i].x
									+ msg_ibeo.points[i].y * msg_ibeo.points[i].y;

					if(norm2 < norm1)
					{
						roadsurface3d_msg.points[index].x = msg_ibeo.points[i].x;
						roadsurface3d_msg.points[index].y = msg_ibeo.points[i].y;
						roadsurface3d_msg.points[index].z = msg_ibeo.points[i].z;

						circle( dismap,
						    Point( roadsurface3d_msg.points[index].x/(display_range/grid_dim_)+grid_dim_/2, grid_dim_*2/3-roadsurface3d_msg.points[index].y/(display_range/grid_dim_) ),
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
		}
		
		if( rec_16r < 5)//500ms
		{
			for(int i = 0; i < 720; i ++)
			{

				double theta_16r = atan2(msg_16r.points[i].y, -msg_16r.points[i].x);

				if( (theta_16r < -90*PI/180 && theta_16r > -180*PI/180) || (theta_16r >= 90*PI/180 && theta_16r < 180*PI/180) )// to be confirmed
				{
					index_d = theta_16r / theta_increment;
					index = (int)( (index_d + 180) * 2 );
					if (index < 0  ) index = 0;
					if (index > 719) index = 719;

					if(!isFilled[index])
					{
						roadsurface3d_msg.points[index].x = msg_16r.points[i].x;
						roadsurface3d_msg.points[index].y = msg_16r.points[i].y;
						roadsurface3d_msg.points[index].z = msg_16r.points[i].z;

						circle( dismap,
						    Point( roadsurface3d_msg.points[index].x/(display_range/grid_dim_)+grid_dim_/2, grid_dim_*2/3-roadsurface3d_msg.points[index].y/(display_range/grid_dim_) ),
						    1,
						    Scalar(255,255,0),
						    -1
				  		);
					}
					else if(is_in_suidao || is_in_fee)
					{
						double norm16r   = (msg_16r.points[i].x * msg_16r.points[i].x)
										 + (msg_16r.points[i].y * msg_16r.points[i].y);
						double norm_cur1 = (roadsurface3d_msg.points[index].x * roadsurface3d_msg.points[index].x)
										 + (roadsurface3d_msg.points[index].y * roadsurface3d_msg.points[index].y);

						//if(norm_cur1 < norm16r)
						//	continue;

						if(norm16r < 20.0 * 20.0)
						{
							if(msg_16r.points[i].z > 0.70  && (theta_16r > 90.0*PI/180 && theta_16r <180.0*PI/180) )
							{
								roadsurface3d_msg.points[index].x = msg_16r.points[i].x;
								roadsurface3d_msg.points[index].y = msg_16r.points[i].y;
								roadsurface3d_msg.points[index].z = msg_16r.points[i].z;

								circle( dismap,
								    Point( roadsurface3d_msg.points[index].x/(display_range/grid_dim_)+grid_dim_/2, grid_dim_*2/3-roadsurface3d_msg.points[index].y/(display_range/grid_dim_) ),
								    1,
								    Scalar(255,255,255),
								    -1
						  		);
						  		
						  		hungedobstacle++;
							}
						}
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

				double theta_16l = atan2(msg_16l.points[i].y, -msg_16l.points[i].x);
		
				if( theta_16l >= -90*PI/180 && theta_16l < 90*PI/180 )
				{
					index_d = theta_16l / theta_increment;
					index = (int)( (index_d + 180) * 2 );
					if (index < 0  ) index = 0;
					if (index > 719) index = 719;

					if(!isFilled[index])
					{
						roadsurface3d_msg.points[index].x = msg_16l.points[i].x;
						roadsurface3d_msg.points[index].y = msg_16l.points[i].y;

						circle( dismap,
						    Point( roadsurface3d_msg.points[index].x/(display_range/grid_dim_)+grid_dim_/2, grid_dim_*2/3-roadsurface3d_msg.points[index].y/(display_range/grid_dim_) ),
						    1,
						    Scalar(0,255,255),
						    -1
				  		);
					}
					else if(is_in_suidao || is_in_fee)
					{
						double norm16l   = (msg_16l.points[i].x * msg_16l.points[i].x)
										 + (msg_16l.points[i].y * msg_16l.points[i].y);
						double norm_cur2 = (roadsurface3d_msg.points[index].x * roadsurface3d_msg.points[index].x)
										 + (roadsurface3d_msg.points[index].y * roadsurface3d_msg.points[index].y);

						//if(norm_cur2 < norm16l)
						//	continue;

						if(norm16l < 10.0 * 10.0)
						{
							if(msg_16l.points[i].z > 0.70 && (theta_16l>0.0 && theta_16l <=90*PI/180) )
							{
								roadsurface3d_msg.points[index].x = msg_16l.points[i].x;
								roadsurface3d_msg.points[index].y = msg_16l.points[i].y;
								roadsurface3d_msg.points[index].z = msg_16l.points[i].z;

								circle( dismap,
								    Point( roadsurface3d_msg.points[index].x/(display_range/grid_dim_)+grid_dim_/2, grid_dim_*2/3-roadsurface3d_msg.points[index].y/(display_range/grid_dim_) ),
								    2,
								    Scalar(255,255,255),
								    -1
						  		);
						  		hungedobstacle++;
							}
						}
					}
				}
			}
		}
		else
		{
			if(showmessages) ROS_INFO("16E Left: No New Message!");
		}
		if (is_in_fee || is_in_suidao)
		{
			ROS_INFO("TOTAL HUNGED OBSTACLES: %d", hungedobstacle);
			hungedobstacle=0;
		}
		if(showmessages) ROS_INFO("           ");

		//filter process
		double roadsurface_norm[720];
		double filter_norm[720];

		for(int i =0; i<720; i++)
		{
			roadsurface_norm[i]=sqrt( 
				   roadsurface3d_msg.points[i].x * roadsurface3d_msg.points[i].x
				 + roadsurface3d_msg.points[i].y * roadsurface3d_msg.points[i].y
				);
		}

		filter_norm[0]=roadsurface_norm[0];
		filter_norm[719]=roadsurface_norm[719];
		for(int i =1; i<719; i++)
		{
			if(roadsurface_norm[i] <= roadsurface_norm[i+1] && roadsurface_norm[i] <= roadsurface_norm[i-1])
			{
				filter_norm[i] = roadsurface_norm[i];
			}
			else if(roadsurface_norm[i-1] <= roadsurface_norm[i+1] && roadsurface_norm[i-1] <= roadsurface_norm[i])
			{
				filter_norm[i] = roadsurface_norm[i-1];
			}
			//if(roadsurface_norm[i+1]<=roadsurface_norm[i] && roadsurface_norm[i+1] <= roadsurface_norm[i-1])
			else
			{
				filter_norm[i] = roadsurface_norm[i+1];
			}
		}
		
		for(int i = 0; i < 720; i++)
		{			
    		double theta_cur=(i/2-180)*PI/180;

			roadsurface3d_msg.points[i].x = -filter_norm[i]*cos(theta_cur);
			roadsurface3d_msg.points[i].y =  filter_norm[i]*sin(theta_cur);
		}


		in2_msgs::RoadSurface roadsurface_msg;
		roadsurface_msg.header.stamp = ros::Time::now();
		roadsurface_msg.header.frame_id = "RoadSurface";

		for(int i = 0; i < 720; i++)
		{			
    		roadsurface_msg.points[i].x = roadsurface3d_msg.points[i].x;
			roadsurface_msg.points[i].y = roadsurface3d_msg.points[i].y;
		}

		//roadsurface img
		for(int i = 0; i < 720; i++)
		{	
			line( dismap,
				  Point( roadsurface_msg.points[i].x/(display_range/grid_dim_)+grid_dim_/2, grid_dim_*2/3-roadsurface_msg.points[i].y/(display_range/grid_dim_) ),
				  Point( roadsurface_msg.points[(i+1)%720].x/(display_range/grid_dim_)+grid_dim_/2, grid_dim_*2/3-roadsurface_msg.points[(i+1)%720].y/(display_range/grid_dim_) ),
				  Scalar(255,0,0),
				  1
			);
		}

		roadsurface_pub.publish(roadsurface_msg);



		in2_msgs::ScanInfoV2 scaninfov2_msg;
		scaninfov2_msg.header.stamp = ros::Time::now();
		scaninfov2_msg.header.frame_id = "ScanInfoV2";

		for(int i = 0; i < 720; i++)
		{	
			scaninfov2_msg.ScanInfoX[i] = 1500 + roadsurface_msg.points[i].x/0.02;
			scaninfov2_msg.ScanInfoY[i] = 1000 + roadsurface_msg.points[i].y/0.02;
		}
		/*
		//scaninfov2 img
		int offset_map_x = ( (display_range-60.0)/2 ) / ( display_range/grid_dim_ );
		int offset_map_y = ( (display_range-60.0)/3 ) / ( display_range/grid_dim_ );
		for(int i = 0; i < 720; i++)
		{	
			line( dismap,
				  Point( offset_map_x + scaninfov2_msg.ScanInfoX[i]/(50*display_range/grid_dim_),     -offset_map_y + grid_dim_ - scaninfov2_msg.ScanInfoY[i]/(50*display_range/grid_dim_)),
				  Point( offset_map_x + scaninfov2_msg.ScanInfoX[(i+1)%720]/(50*display_range/grid_dim_), -offset_map_y + grid_dim_ - scaninfov2_msg.ScanInfoY[(i+1)%720]/(50*display_range/grid_dim_) ),
				  Scalar(255,0,0),
				  1
				);
		}*/

		scaninfov2_pub.publish(scaninfov2_msg);




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
