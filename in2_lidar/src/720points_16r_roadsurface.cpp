#include <ros/ros.h>
#include <in2_msgs/ScanInfoV2.h>
#include <in2_msgs/InsInfo.h>
#include <in2_msgs/RoadSurface.h>
#include <in2_msgs/RoadSurface3D.h>
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
#define CarLeftEdge	       0.945
#define CarRightEdge      -0.945
#define CarFrontEdge       3.875
#define CarBackEdge	      -1.22                                                                                                                                   

using namespace cv;
using namespace std;

typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
VPointCloud scan;

int grid_dim_;
double m_per_cell_;
double height_diff_threshold_;
long TimeStamp;
double display_range;

double angle_nega_coe,angle_nega_min,angle_nega_max;
double angle_posi_coe,angle_posi_min,angle_posi_max;
double height_nega_coe,height_nega_min,height_nega_max;
double height_posi_coe,height_posi_min,height_posi_max;
double deadzone_nega,deadzone_posi;

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

ros::Publisher roadsurface3d_pub;

bool init_flag = true;
bool showmessages;
bool showimages;

struct box
{
	double x[400];
	double y[400];
	double z[400];
	double norm[400];
	int PointsCount;
};
struct box boxes[720];

struct longbox
{
	double x[800];
	double y[800];
	double z[800];
	double norm[800];
	int PointsCount;
	bool isPuddle[800];
	bool isNegative[800];
	bool isPositive[800];
};
struct longbox finalbox;
struct longbox finalbox_old;

void BubbleSort(struct box *_box)
{
	int Count = _box->PointsCount;	
	double iTemp;
	for(int i=0; i < Count; i++)
	{
		int minnum=i;		
		for(int j = i; j<Count; j++)
		{
			if(_box->norm[j]<_box->norm[minnum])
			{
				minnum=j;
			}
		}
		iTemp = _box->norm[minnum];
		_box->norm[minnum] = _box->norm[i];
		_box->norm[i] = iTemp;

		iTemp = _box->x[minnum];
		_box->x[minnum] = _box->x[i];
		_box->x[i] = iTemp;

		iTemp = _box->y[minnum];
		_box->y[minnum] = _box->y[i];
		_box->y[i] = iTemp;

		iTemp = _box->z[minnum];
		_box->z[minnum] = _box->z[i];
		_box->z[i] = iTemp;
	}
}

bool isIncar(double x, double y)
{
	if( x <= CarFrontEdge && x >= CarBackEdge)
	{
		if(y >= CarRightEdge && y <= CarLeftEdge) return true;

		//back mirrors
		if(y > CarLeftEdge)
		{
			if( y < CarLeftEdge +0.40 && x > 1.775 && x < 1.775+0.35 ) return true;
		}
		if(y < CarRightEdge)
		{
			if( y > CarRightEdge-0.40 && x > 1.775 && x < 1.775+0.35 ) return true;
		}

		return false;
	}
	return false;
}

void processScan( const VPointCloud::ConstPtr &scan_msg )
{
	//Time stamp
	long int StarTime = my_servertime.GetServerTime();
	
	TimeStamp = my_servertime.GetServerTime();

    // point cloud data
	scan = *scan_msg;
		
	// initialize
	if( ! init_flag ) 
	{
		//initialize finalbox
		for(int i=0; i<720; i++)
		{
			double rang=200.0;
			double thet=(i/2-180)*PI/180;
			finalbox.x[i]=rang*sin(thet);
			finalbox.y[i]=rang*cos(thet);
			finalbox.norm[i]=rang;

			finalbox.isPuddle  [i]  =false;
			finalbox.isNegative[i]  =false;
			finalbox.isPositive[i]  =false;
		}
		
	}
		
	for(int i = 0; i < 720; i++)  boxes[i].PointsCount  =  0 ; 

    // extract from raw pointcloud
	finalbox.PointsCount = 0;
	double theta;
	int box_num;
	for(int i = 0; i < scan.points.size(); i++)
	{

		if( scan.points[i].z > 1.75 )
			continue;

		// ignore in-car range points
		/*if(  scan.points[i].x >= CarBackEdge   &&
		     scan.points[i].x <= CarFrontEdge  &&
		     scan.points[i].y <= CarLeftEdge   &&
		     scan.points[i].y >= CarRightEdge 
		  )*/
		if( isIncar(scan.points[i].x, scan.points[i].y) )
			continue;
		
        // ignore too high above ground points, max height is 0.4+2.1
		if( scan.points[i].z <= 1.75 )
		{		
			theta = atan2(scan.points[i].x, scan.points[i].y);// (-PI,PI]
			box_num = (int)((theta/PI*180.0+180.0)*2);//0 to 719
			if(box_num<0 || box_num>719) continue;
			if(boxes[box_num].PointsCount > 399) continue;//max number in each box: 399

			boxes[box_num].x[boxes[box_num].PointsCount] = scan.points[i].x;
			boxes[box_num].y[boxes[box_num].PointsCount] = scan.points[i].y;
			boxes[box_num].z[boxes[box_num].PointsCount] = scan.points[i].z;
			boxes[box_num].norm[boxes[box_num].PointsCount] = 
				sqrt( scan.points[i].x * scan.points[i].x + scan.points[i].y * scan.points[i].y);
			boxes[box_num].PointsCount++;
		}
	}

    // process data in each box
	for(int i = 0; i < 720; i++)
	{
		//sort by norm
		BubbleSort( & boxes[i] );

		//z value median filter
		double znew[boxes[i].PointsCount];

		znew[0] = boxes[i].z[0];
		for(int k = 1; k < boxes[i].PointsCount - 1; k++)
		{   
			if((boxes[i].z[k] < boxes[i].z[k - 1]) && (boxes[i].z[k] < boxes[i].z[k + 1]))
				znew[k] = (boxes[i].z[k - 1] < boxes[i].z[k + 1]) ? boxes[i].z[k - 1] : boxes[i].z[k + 1];
			else if((boxes[i].z[k] > boxes[i].z[k - 1]) && (boxes[i].z[k] > boxes[i].z[k + 1]))
				znew[k] = (boxes[i].z[k - 1] > boxes[i].z[k + 1]) ? boxes[i].z[k - 1] : boxes[i].z[k + 1];
			else
				znew[k] = boxes[i].z[k];
		}
		znew[boxes[i].PointsCount - 1] = boxes[i].z[boxes[i].PointsCount - 1];
		
		for(int k = 0; k < boxes[i].PointsCount; k++) boxes[i].z[k] = znew[k];
		
		// edge point detection
		/*if(boxes[i].z[0] < -0.2 || boxes[i].z[0]>0.2)
		{
			finalbox.x[finalbox.PointsCount] = boxes[i].x[0];
			finalbox.y[finalbox.PointsCount] = boxes[i].y[0];			
			finalbox.z[finalbox.PointsCount] = boxes[i].z[0];
			finalbox.norm[finalbox.PointsCount] = boxes[i].norm[0];

			finalbox.PointsCount++;
			continue;
		}*/

		int j;//points in i-th angle
		for(j = 1; j < boxes[i].PointsCount; j++)
		{
			double norm=sqrt(boxes[i].x[j-1]*boxes[i].x[j-1]+boxes[i].y[j-1]*boxes[i].y[j-1]);			
	        //double norm = boxes[i].norm[j-1];

			double space_lenth = sqrt(
					 (boxes[i].x[j]-boxes[i].x[j-1])*(boxes[i].x[j]-boxes[i].x[j-1])
					+(boxes[i].y[j]-boxes[i].y[j-1])*(boxes[i].y[j]-boxes[i].y[j-1])
					);
			double angle = (boxes[i].z[j]-boxes[i].z[j-1]) / space_lenth;
			
			double threshold_nega_new = 
			        - height_nega_min 
			        + (height_nega_min - height_nega_max)*(norm - deadzone_nega)/(height_nega_coe - deadzone_nega);
			if(threshold_nega_new < -height_nega_max) threshold_nega_new = -height_nega_max;
			if(threshold_nega_new > -height_nega_min) threshold_nega_new = -height_nega_min;

			double threshold_posi_new = 
			        height_posi_min 
			        + (height_posi_max - height_posi_min)*(norm - deadzone_posi)/(height_posi_coe - deadzone_posi);
			if(threshold_posi_new > height_posi_max) threshold_posi_new = height_posi_max;
			if(threshold_posi_new < height_posi_min) threshold_posi_new = height_posi_min;

			
			double angle_nega_new = 
			        -angle_nega_min 
			        + (-angle_nega_max+angle_nega_min)*(angle_nega_coe - norm)/(angle_nega_coe - deadzone_nega);
			if(angle_nega_new > -angle_nega_min) angle_nega_new = -angle_nega_min;
			if(angle_nega_new < -angle_nega_max) angle_nega_new = -angle_nega_max;
			
			double angle_posi_new = 
			        angle_posi_min 
			        + (angle_posi_max - angle_posi_min)*(angle_posi_coe - norm)/(angle_posi_coe - deadzone_posi);
			if(angle_posi_new > angle_posi_max) angle_posi_new = angle_posi_max;
			if(angle_posi_new < angle_posi_min) angle_posi_new = angle_posi_min;

			//positive obstacles
			if( angle >  angle_posi_new && boxes[i].z[j]-boxes[i].z[j-1] > threshold_posi_new )
			{
				finalbox.x[finalbox.PointsCount] = boxes[i].x[j-1];
				finalbox.y[finalbox.PointsCount] = boxes[i].y[j-1];

				if(boxes[i].z[j-1] > 0.50)
					finalbox.z[finalbox.PointsCount] = boxes[i].z[j-1];
				else
				{
					if(boxes[i].z[j] > 0.50)
						finalbox.z[finalbox.PointsCount] = boxes[i].z[j];
					else
						finalbox.z[finalbox.PointsCount] = boxes[i].z[j-1];
				}			
				
				finalbox.norm[finalbox.PointsCount] = norm;

				finalbox.isPositive[i] = true;
				finalbox.isNegative[i] = false;
				finalbox.isPuddle[i]   = false;

				finalbox.PointsCount++;
				break;
			}

			//negative obstacles
			if( angle <  angle_nega_new && boxes[i].z[j]-boxes[i].z[j-1] < threshold_nega_new )
			{
				finalbox.x[finalbox.PointsCount] = boxes[i].x[j-1];
				finalbox.y[finalbox.PointsCount] = boxes[i].y[j-1];			
				finalbox.norm[finalbox.PointsCount] = norm;

				if( boxes[i].z[j] < Concave_Threshold) 
					finalbox.z[finalbox.PointsCount] = boxes[i].z[j];
				else 
					finalbox.z[finalbox.PointsCount] = boxes[i].z[j-1];

				finalbox.isNegative[i] = true;
				finalbox.isPositive[i] = false;
				finalbox.isPuddle[i]   = false;

				finalbox.PointsCount++;
				break;
			}
				
		}

		if(j == boxes[i].PointsCount)
		{
			if(boxes[i].PointsCount<=0)	
			{
				double range=200.0;
				double thet=(i/2-180)*PI/180;
				finalbox.x[finalbox.PointsCount] = range*sin(thet);
				finalbox.y[finalbox.PointsCount] = range*cos(thet);
				finalbox.z[finalbox.PointsCount] = 0;
				finalbox.norm[finalbox.PointsCount] = range;
			}
			else
			{
				double range=200.0;
				double thet=(i/2-180)*PI/180;
				finalbox.x[finalbox.PointsCount] = range*sin(thet);
				finalbox.y[finalbox.PointsCount] = range*cos(thet);
				finalbox.z[finalbox.PointsCount] = 0;
				finalbox.norm[finalbox.PointsCount] = range;
				
				/*finalbox.x[finalbox.PointsCount] = boxes[i].x[j-1];
				finalbox.y[finalbox.PointsCount] = boxes[i].y[j-1];
				finalbox.z[finalbox.PointsCount] = boxes[i].z[j-1];
				finalbox.norm[finalbox.PointsCount] = boxes[i].norm[j-1];*/
			}
			finalbox.PointsCount++;
		}
	}


    //problem 1: 
    //       (finalbox.norm[i-1]+finalbox.norm[i+1])/2.0 != sqrt(finalbox.x[i]*finalbox.x[i]+finalbox.y[i]*finalbox.y[i])
    //mean filtering
	for(int i = 1; i < 719; i++)
	{
		if( fabs(finalbox.norm[i]-finalbox.norm[i-1])>2.0 && 
			fabs(finalbox.norm[i]-finalbox.norm[i+1])>2.0 && 
			(finalbox.norm[i]-finalbox.norm[i-1])*(finalbox.norm[i]-finalbox.norm[i+1])>0
		  )
	    {
		    finalbox.x[i] = (finalbox.x[i-1]+finalbox.x[i+1])/2.0;
		    finalbox.y[i] = (finalbox.y[i-1]+finalbox.y[i+1])/2.0;
		    finalbox.z[i] = (finalbox.z[i-1]+finalbox.z[i+1])/2.0;
		    finalbox.norm[i] = (finalbox.norm[i-1]+finalbox.norm[i+1])/2.0;
	    }
	}
	
	if( fabs(finalbox.norm[0]-finalbox.norm[719])>2.0     && 
		fabs(finalbox.norm[0]-finalbox.norm[  1])>2.0     && 
		(finalbox.norm[0]-finalbox.norm[719])*(finalbox.norm[0]-finalbox.norm[1])>0
	  )
	{
		finalbox.x[0] = (finalbox.x[719]+finalbox.x[1])/2.0;
		finalbox.y[0] = (finalbox.y[719]+finalbox.y[1])/2.0;
		finalbox.z[0] = (finalbox.z[719]+finalbox.z[1])/2.0;
		finalbox.norm[0] = (finalbox.norm[719]+finalbox.norm[1])/2.0;
	}
	
    if( fabs(finalbox.norm[719]-finalbox.norm[718])>2.0   && 
		fabs(finalbox.norm[719]-finalbox.norm[  0])>2.0   && 
		(finalbox.norm[719]-finalbox.norm[718])*(finalbox.norm[719]-finalbox.norm[0])>0
	  )
	{
		finalbox.x[719] = (finalbox.x[718]+finalbox.x[0])/2.0;
		finalbox.y[719] = (finalbox.y[718]+finalbox.y[0])/2.0;
		finalbox.z[719] = (finalbox.z[718]+finalbox.z[0])/2.0;
		finalbox.norm[719] = (finalbox.norm[718]+finalbox.norm[0])/2.0;
	}

	if(init_flag)
	{
		finalbox_old = finalbox;
		init_flag = false;
	}

	//publish roadsurface msgs
	in2_msgs::RoadSurface3D roadsurface3d_msg;
	roadsurface3d_msg.header.stamp = ros::Time::now();
	roadsurface3d_msg.header.frame_id = "RoadSurface3D";
	
	for(int i = 0; i < 720; i++)
	{
		roadsurface3d_msg.points[i].x = - finalbox.y[i];
		roadsurface3d_msg.points[i].y =   finalbox.x[i];
		roadsurface3d_msg.points[i].z =   finalbox.z[i];
	}

	Mat dismap(grid_dim_,grid_dim_,CV_8UC3,Scalar(128,128,128));
	rectangle( dismap,
			   Point( grid_dim_/2 - (0.85*grid_dim_)/(display_range), grid_dim_*2/3 - (3.8*grid_dim_)/(display_range) ),
			   Point( grid_dim_/2 + (0.85*grid_dim_)/(display_range), grid_dim_*2/3 + (1.2*grid_dim_)/(display_range) ),
			   Scalar(255,255,255),
			   1
			 );//vehicle

	for(int i = 0; i < 720; i++)
	{	
		line( dismap,
			  Point( roadsurface3d_msg.points[i].x/(display_range/grid_dim_)+grid_dim_/2, grid_dim_*2/3-roadsurface3d_msg.points[i].y/(display_range/grid_dim_) ),
			  Point( roadsurface3d_msg.points[(i+1)%720].x/(display_range/grid_dim_)+grid_dim_/2, grid_dim_*2/3-roadsurface3d_msg.points[(i+1)%720].y/(display_range/grid_dim_) ),
			  Scalar(255,0,0),
			  1
			);
		
		if(finalbox.isNegative[i])
		{
			circle( dismap,
				    Point( roadsurface3d_msg.points[i].x/(display_range/grid_dim_)+grid_dim_/2, grid_dim_*2/3-roadsurface3d_msg.points[i].y/(display_range/grid_dim_) ),
				    2,
				    Scalar(0,255,0),
				    -1
				  );
		}
		if(finalbox.isPositive[i])
		{
			circle( dismap,
				    Point( roadsurface3d_msg.points[i].x/(display_range/grid_dim_)+grid_dim_/2, grid_dim_*2/3-roadsurface3d_msg.points[i].y/(display_range/grid_dim_) ),
				    2,
				    Scalar(0,0,255),
				    -1
				  );
		}
	}

	roadsurface3d_pub.publish(roadsurface3d_msg);

	if(showimages)
	{
		imshow("16r",dismap);
		waitKey(1);
	}
		
#ifdef DISPLAYPROCESSTIME
    if(showmessages) ROS_INFO("Periscan Process End  %ldms\n",my_servertime.GetServerTime()-StarTime);
#endif
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "velodyne16r_node");

	ros::NodeHandle node;
	my_servertime.C_ServerTime_Init();

	node.param("/velodyne16r_node/cell_size", m_per_cell_, 0.05);
	node.param("/velodyne16r_node/grid_dimensions", grid_dim_, 600);
	node.param("/velodyne16r_node/display_range", display_range, 90.0);

	node.param("/velodyne16r_node/height_nega_coe", height_nega_coe, 27.0);
	node.param("/velodyne16r_node/height_nega_min", height_nega_min, 0.07);
	node.param("/velodyne16r_node/height_nega_max", height_nega_max, 0.12);

	node.param("/velodyne16r_node/height_posi_coe", height_posi_coe, 27.0);
	node.param("/velodyne16r_node/height_posi_min", height_posi_min, 0.05);
	node.param("/velodyne16r_node/height_posi_max", height_posi_max, 0.10);

	node.param("/velodyne16r_node/angle_nega_coe", angle_nega_coe, 27.0);
	node.param("/velodyne16r_node/angle_nega_min", angle_nega_min, 0.20);
	node.param("/velodyne16r_node/angle_nega_max", angle_nega_max, 0.25);

	node.param("/velodyne16r_node/angle_posi_coe", angle_posi_coe, 27.0);
	node.param("/velodyne16r_node/angle_posi_min", angle_posi_min, 0.40);
	node.param("/velodyne16r_node/angle_posi_max", angle_posi_max, 0.45);

	node.param("/velodyne16r_node/deadzone_nega", deadzone_nega, 10.0);
	node.param("/velodyne16r_node/deadzone_posi", deadzone_posi, 10.0);

	node.param("/velodyne16r_node/showimages",   showimages,   false);
	node.param("/velodyne16r_node/showmessages", showmessages, false);

	ros::Subscriber scan;
	scan = node.subscribe("rotate_16r", 10, &processScan,  ros::TransportHints().tcpNoDelay(true) );

	roadsurface3d_pub = node.advertise<in2_msgs::RoadSurface3D>("RoadSurface_16r", 1);

	ros::spin();

	ros::shutdown();
	return 0;
}
