#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud.h>
#include <in2_msgs/IbeoObjects.h>
#include <geometry_msgs/PolygonStamped.h>
#include <in2_msgs/RoadSurface.h>
#include <in2_msgs/RoadSurface3D.h>
#include "math.h"

#define PI      3.1415926
#define FRONT_IBEO_X  3.700
#define FRONT_IBEO_Z  0.310
#define REAR_IBEO_X   1.180
#define REAR_IBEO_Z   0.340

#define ACCESS_RANGE  100

//#define PUT_IBEO_ON_CENTER  //Used for test Hangle of ibeo. Usually it should be blocked.

using namespace std;

long counter=0;

bool get_front_flag=0;
bool get_rear_flag=0;


bool get_front_obj=0;
bool get_rear_obj=0;

bool get_cloud_whole=0;

int numScanpoints_rear;
int numScanpoints_front;

int numObjects_rear;
int numObjects_front;
int numObjects_whole;

sensor_msgs::PointCloud cloud_front;
sensor_msgs::PointCloud cloud_rear;
sensor_msgs::PointCloud cloud_whole;

sensor_msgs::PointCloud mask_front;
sensor_msgs::PointCloud mask_rear;
sensor_msgs::PointCloud mask_whole;
sensor_msgs::PointCloud mask_pseudo;

in2_msgs::IbeoObjects Objects_front;
in2_msgs::IbeoObjects Objects_rear;
in2_msgs::IbeoObjects Objects_whole;
//sensor_msgs::PointCloud ObjectContourPoints;

geometry_msgs::PolygonStamped MaskPolygon;
geometry_msgs::PolygonStamped MaskWhole_Polygon;

in2_msgs::RoadSurface roadSurface;
in2_msgs::RoadSurface3D roadSurface3D;

ros::Publisher  fusion_pub_;
ros::Publisher  objects_pub_;
ros::Publisher  ObjectContourPoints_pub_;
ros::Publisher  static_pointcloud_pub_;
ros::Publisher  roadSurface_pub_;
ros::Publisher  roadSurface3D_pub_;

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
long TimeStamp;

struct point_PointCloud
{
    float x;
    float y;
    float z;
    float ch0; //Preprocessing Labels
    float ch1; //Layers
		float ch2; //Hangle
    float ch3; //Distance
};

struct point_3D
{
    float x;
    float y;
    float z;
    float Hangle;
    float d;
};

void ibeo_fusion();
void get_mask_pseudo( int mask_num, int mask_front_num, int mask_rear_num );
void mask_fusion();
void fusion();
void listener_ibeo_4L(const sensor_msgs::PointCloud::ConstPtr& PointCloud_Ptr);
void listener_ibeo_8L(const sensor_msgs::PointCloud::ConstPtr& PointCloud_Ptr);
void GetDeleteArea(in2_msgs::IbeoObjects object_temp);
void object_fusion();
void listener_ibeo_4L_obj(const in2_msgs::IbeoObjects::ConstPtr& IbeoObjects_Ptr);
void listener_ibeo_8L_obj(const in2_msgs::IbeoObjects::ConstPtr& IbeoObjects_Ptr);

void get_mask_pseudo( int mask_num, int mask_front_num, int mask_rear_num )
{
  struct point_3D FrontLeft,FrontRight,RearLeft,RearRight;

  FrontRight.x = mask_whole.points[0].x;
  FrontRight.y = mask_whole.points[0].y;
  FrontRight.z = mask_whole.points[0].z;
	FrontRight.Hangle = mask_whole.channels[2].values[0];
  FrontRight.d = mask_whole.channels[3].values[0];

  FrontLeft.x = mask_whole.points[mask_front_num-1].x;
  FrontLeft.y = mask_whole.points[mask_front_num-1].y;
  FrontLeft.z = mask_whole.points[mask_front_num-1].z;
	FrontLeft.Hangle = mask_whole.channels[2].values[mask_front_num-1];
	FrontLeft.d = mask_whole.channels[3].values[mask_front_num-1];


  RearLeft.x = mask_whole.points[mask_front_num].x;
  RearLeft.y = mask_whole.points[mask_front_num].y;
  RearLeft.z = mask_whole.points[mask_front_num].z;
	RearLeft.Hangle = mask_whole.channels[2].values[mask_front_num];
	RearLeft.d = mask_whole.channels[3].values[mask_front_num];

  RearRight.x = mask_whole.points[mask_num-1].x;
  RearRight.y = mask_whole.points[mask_num-1].y;
  RearRight.z = mask_whole.points[mask_num-1].z;
	RearRight.Hangle = mask_whole.channels[2].values[mask_num-1];
	RearRight.d = mask_whole.channels[3].values[mask_num-1];

  if(mask_num <= 720) // resupply
  {
    int Diff_num = 720 - mask_num;
    int left_add_num = Diff_num/2;
    int right_add_num= Diff_num - left_add_num;

		for(int i=0;i<Diff_num;i++)
  	{
			mask_pseudo.points[mask_num+i].x = RearRight.x;
			mask_pseudo.points[mask_num+i].y = RearRight.y;
			mask_pseudo.points[mask_num+i].z = RearRight.z;
			mask_pseudo.channels[2].values[mask_num+i] = RearRight.Hangle;
			mask_pseudo.channels[3].values[mask_num+i] = RearRight.d;

		}
	  mask_pseudo.points.resize(mask_num+left_add_num+right_add_num);
  }
  else
  {
    struct point_3D check_window[3];
    struct point_3D mask_pseudo_temp[mask_whole.points.size()];
    int delete_count=0;
    int Diff_num = mask_num - 720;
    int mask_pseudo_count=0;

    for(int j=0;j<mask_whole.points.size();j++) //set value to window
    {
      mask_pseudo_temp[j].x = mask_whole.points[j].x;
      mask_pseudo_temp[j].y = mask_whole.points[j].y;
      mask_pseudo_temp[j].z = mask_whole.points[j].z;
      mask_pseudo_temp[j].Hangle = mask_whole.channels[2].values[j];
      mask_pseudo_temp[j].d = mask_whole.channels[3].values[j];
    }
    for(int i=mask_front_num;i<mask_num-2;i=i+2)
    {
      for(int j=0;j<3;j++) //set value to window
      {
        check_window[j].x = mask_pseudo_temp[i+j].x;
        check_window[j].y = mask_pseudo_temp[i+j].y;
        check_window[j].z = mask_pseudo_temp[i+j].z;
        check_window[j].Hangle = mask_pseudo_temp[i+j].Hangle;
        check_window[j].d = mask_pseudo_temp[i+j].d;
      }

      if(fabs(check_window[1].Hangle-check_window[0].Hangle)>0.24&&fabs(check_window[1].Hangle-check_window[0].Hangle)<0.26&&fabs(check_window[2].Hangle-check_window[1].Hangle)>0.24&&fabs(check_window[2].Hangle-check_window[1].Hangle)<0.26)
      {
        mask_pseudo_temp[i+1].d = 0;
        delete_count++;
      }

      if(delete_count>=Diff_num)
        break;
    }

    for(int i=0;i<mask_whole.points.size();i++)
    {
      if(mask_pseudo_temp[i].d != 0)
      {
        mask_pseudo.points[mask_pseudo_count].x = mask_pseudo_temp[i].x;
        mask_pseudo.points[mask_pseudo_count].y = mask_pseudo_temp[i].y;
        mask_pseudo.points[mask_pseudo_count].z = mask_pseudo_temp[i].z;
				mask_pseudo.channels[2].values[mask_pseudo_count] = mask_pseudo_temp[i].Hangle;
				mask_pseudo.channels[3].values[mask_pseudo_count] = mask_pseudo_temp[i].d;
        mask_pseudo_count++;
      }
    }
    mask_pseudo.points.resize(720);
  }

  roadSurface.header.stamp = ros::Time::now();
  roadSurface.header.frame_id = "RoadSurface_ibeo";

  roadSurface3D.header.stamp = ros::Time::now();
  roadSurface3D.header.frame_id = "RoadSurface3D_ibeo";

for(int i = 0;i<720;i++)
{
       roadSurface.points[i].x = -mask_pseudo.points[i].y;
       roadSurface.points[i].y = mask_pseudo.points[i].x;

	roadSurface3D.points[i].x = -mask_pseudo.points[i].y;
	roadSurface3D.points[i].y = mask_pseudo.points[i].x;
	roadSurface3D.points[i].z = mask_pseudo.points[i].z;

}  


}

void ibeo_fusion()
{
  int fusion_counter=0;
  int numScanpoints_whole = numScanpoints_rear + numScanpoints_front;

  cloud_whole.header.stamp = ros::Time::now();
  cloud_whole.header.frame_id = "base_vehicle";
  cloud_whole.points.resize(numScanpoints_whole);
  cloud_whole.channels.resize(4);
  cloud_whole.channels[0].name = "Preprocessing Labels";
  cloud_whole.channels[0].values.resize(numScanpoints_whole);
  cloud_whole.channels[1].name = "Layers";
  cloud_whole.channels[1].values.resize(numScanpoints_whole);
	cloud_whole.channels[2].name = "Hangle";
  cloud_whole.channels[2].values.resize(numScanpoints_whole);
	cloud_whole.channels[3].name = "Distance";
  cloud_whole.channels[3].values.resize(numScanpoints_whole);

  for(;fusion_counter<numScanpoints_front;fusion_counter++)
  {
#ifndef PUT_IBEO_ON_CENTER
    cloud_whole.points[fusion_counter].x = FRONT_IBEO_X+cloud_front.points[fusion_counter].x;
    cloud_whole.points[fusion_counter].y = cloud_front.points[fusion_counter].y;
    cloud_whole.points[fusion_counter].z = FRONT_IBEO_Z+cloud_front.points[fusion_counter].z;
    cloud_whole.channels[0].values[fusion_counter] = cloud_front.channels[0].values[fusion_counter];
    cloud_whole.channels[1].values[fusion_counter] = cloud_front.channels[1].values[fusion_counter];
		cloud_whole.channels[2].values[fusion_counter] = cloud_front.channels[2].values[fusion_counter];
    cloud_whole.channels[3].values[fusion_counter] = cloud_front.channels[3].values[fusion_counter];
#else
    cloud_whole.points[fusion_counter].x = cloud_front.points[fusion_counter].x;
    cloud_whole.points[fusion_counter].y = cloud_front.points[fusion_counter].y;
    cloud_whole.points[fusion_counter].z = cloud_front.points[fusion_counter].z;
    cloud_whole.channels[0].values[fusion_counter] = cloud_front.channels[0].values[fusion_counter];
    cloud_whole.channels[1].values[fusion_counter] = cloud_front.channels[1].values[fusion_counter];
		cloud_whole.channels[2].values[fusion_counter] = cloud_front.channels[2].values[fusion_counter];
    cloud_whole.channels[3].values[fusion_counter] = cloud_front.channels[3].values[fusion_counter];
#endif
  }

  for(;fusion_counter<numScanpoints_whole;fusion_counter++)
  {
#ifndef PUT_IBEO_ON_CENTER
    cloud_whole.points[fusion_counter].x = -REAR_IBEO_X-cloud_rear.points[fusion_counter-numScanpoints_front].x;
    cloud_whole.points[fusion_counter].y = -cloud_rear.points[fusion_counter-numScanpoints_front].y;
    cloud_whole.points[fusion_counter].z =  REAR_IBEO_Z+cloud_rear.points[fusion_counter-numScanpoints_front].z;
    cloud_whole.channels[0].values[fusion_counter] = cloud_rear.channels[0].values[fusion_counter-numScanpoints_front];
    cloud_whole.channels[1].values[fusion_counter] = cloud_rear.channels[1].values[fusion_counter-numScanpoints_front];
		cloud_whole.channels[2].values[fusion_counter] = cloud_rear.channels[2].values[fusion_counter-numScanpoints_front];
    cloud_whole.channels[3].values[fusion_counter] = cloud_rear.channels[3].values[fusion_counter-numScanpoints_front];
#else
    cloud_whole.points[fusion_counter].x = -cloud_rear.points[fusion_counter-numScanpoints_front].x;
    cloud_whole.points[fusion_counter].y = -cloud_rear.points[fusion_counter-numScanpoints_front].y;
    cloud_whole.points[fusion_counter].z =  cloud_rear.points[fusion_counter-numScanpoints_front].z;
    cloud_whole.channels[0].values[fusion_counter] = cloud_rear.channels[0].values[fusion_counter-numScanpoints_front];
    cloud_whole.channels[1].values[fusion_counter] = cloud_rear.channels[1].values[fusion_counter-numScanpoints_front];
		cloud_whole.channels[2].values[fusion_counter] = cloud_rear.channels[2].values[fusion_counter-numScanpoints_front];
    cloud_whole.channels[3].values[fusion_counter] = cloud_rear.channels[3].values[fusion_counter-numScanpoints_front];
#endif
  }

	get_cloud_whole=1;
}

void mask_fusion()
{
  int fusion_counter=0;
  int mask_counter=0,mask_front_counter=0,mask_rear_counter=0;

	//mask_whole is original shell
  mask_whole.header.stamp = ros::Time::now();
  mask_whole.header.frame_id = "base_vehicle";
  mask_whole.points.resize(880);  //range is 220 degrees(front 110 + rear 110), resolution is 0.25 degree
  mask_whole.channels.resize(4);
  mask_whole.channels[0].name = "Preprocessing Labels";
  mask_whole.channels[0].values.resize(880);
  mask_whole.channels[1].name = "Layers";
  mask_whole.channels[1].values.resize(880);
	mask_whole.channels[2].name = "Hangle";
  mask_whole.channels[2].values.resize(880);
  mask_whole.channels[3].name = "Distance";
  mask_whole.channels[3].values.resize(880);

	//mask_pseudo is processed shell, initialized to 880 points, will be revised to 720 points
  mask_pseudo.header.stamp = ros::Time::now();
  mask_pseudo.header.frame_id = "base_vehicle";
  mask_pseudo.points.resize(880);
  mask_pseudo.channels.resize(4);
  mask_pseudo.channels[0].name = "Preprocessing Labels";
  mask_pseudo.channels[0].values.resize(880);
  mask_pseudo.channels[1].name = "Layers";
  mask_pseudo.channels[1].values.resize(880);
	mask_pseudo.channels[2].name = "Hangle";
  mask_pseudo.channels[2].values.resize(880);
  mask_pseudo.channels[3].name = "Distance";
  mask_pseudo.channels[3].values.resize(880);

  for(;fusion_counter<440;fusion_counter++)
  {
    if(mask_front.points[fusion_counter].y!=0)
    {

#ifndef PUT_IBEO_ON_CENTER
      mask_whole.points[mask_counter].x = FRONT_IBEO_X+mask_front.points[fusion_counter].x;
      mask_whole.points[mask_counter].y = mask_front.points[fusion_counter].y;
      mask_whole.points[mask_counter].z = FRONT_IBEO_Z+mask_front.points[fusion_counter].z;
      mask_whole.channels[0].values[mask_counter] = mask_front.channels[0].values[fusion_counter];
      mask_whole.channels[1].values[mask_counter] = mask_front.channels[1].values[fusion_counter];
			mask_whole.channels[2].values[mask_counter] = mask_front.channels[2].values[fusion_counter];
      mask_whole.channels[3].values[mask_counter] = mask_front.channels[3].values[fusion_counter];

      mask_pseudo.points[mask_counter].x = FRONT_IBEO_X+mask_front.points[fusion_counter].x;
      mask_pseudo.points[mask_counter].y = mask_front.points[fusion_counter].y;
      mask_pseudo.points[mask_counter].z = FRONT_IBEO_Z+mask_front.points[fusion_counter].z;
      mask_pseudo.channels[0].values[mask_counter] = mask_front.channels[0].values[fusion_counter];
      mask_pseudo.channels[1].values[mask_counter] = mask_front.channels[1].values[fusion_counter];
			mask_pseudo.channels[2].values[mask_counter] = mask_front.channels[2].values[fusion_counter];
      mask_pseudo.channels[3].values[mask_counter] = mask_front.channels[3].values[fusion_counter];
      mask_counter++;
      mask_front_counter++;
#else
      mask_whole.points[mask_counter].x = mask_front.points[fusion_counter].x;
      mask_whole.points[mask_counter].y = mask_front.points[fusion_counter].y;
      mask_whole.points[mask_counter].z = mask_front.points[fusion_counter].z;
      mask_whole.channels[0].values[mask_counter] = mask_front.channels[0].values[fusion_counter];
      mask_whole.channels[1].values[mask_counter] = mask_front.channels[1].values[fusion_counter];
			mask_whole.channels[2].values[mask_counter] = mask_front.channels[2].values[fusion_counter];
      mask_whole.channels[3].values[mask_counter] = mask_front.channels[3].values[fusion_counter];

      mask_pseudo.points[mask_counter].x = mask_front.points[fusion_counter].x;
      mask_pseudo.points[mask_counter].y = mask_front.points[fusion_counter].y;
      mask_pseudo.points[mask_counter].z = mask_front.points[fusion_counter].z;
      mask_pseudo.channels[0].values[mask_counter] = mask_front.channels[0].values[fusion_counter];
      mask_pseudo.channels[1].values[mask_counter] = mask_front.channels[1].values[fusion_counter];
			mask_pseudo.channels[2].values[mask_counter] = mask_front.channels[2].values[fusion_counter];
      mask_pseudo.channels[3].values[mask_counter] = mask_front.channels[3].values[fusion_counter];
      mask_counter++;
      mask_front_counter++;
#endif
    }
  }

  for(;fusion_counter<880;fusion_counter++)
  {
    if(mask_rear.points[fusion_counter-440].y!=0)
    {

#ifndef PUT_IBEO_ON_CENTER
      mask_whole.points[mask_counter].x = -REAR_IBEO_X-mask_rear.points[fusion_counter-440].x;
      mask_whole.points[mask_counter].y = -mask_rear.points[fusion_counter-440].y;
      mask_whole.points[mask_counter].z =  REAR_IBEO_Z+mask_rear.points[fusion_counter-440].z;
      mask_whole.channels[0].values[mask_counter] = mask_rear.channels[0].values[fusion_counter-440];
      mask_whole.channels[1].values[mask_counter] = mask_rear.channels[1].values[fusion_counter-440];
			mask_whole.channels[2].values[mask_counter] = mask_rear.channels[2].values[fusion_counter-440];
      mask_whole.channels[3].values[mask_counter] = mask_rear.channels[3].values[fusion_counter-440];

      mask_pseudo.points[mask_counter].x = -REAR_IBEO_X-mask_rear.points[fusion_counter-440].x;
      mask_pseudo.points[mask_counter].y = -mask_rear.points[fusion_counter-440].y;
      mask_pseudo.points[mask_counter].z =  REAR_IBEO_Z+mask_rear.points[fusion_counter-440].z;
      mask_pseudo.channels[0].values[mask_counter] = mask_rear.channels[0].values[fusion_counter-440];
      mask_pseudo.channels[1].values[mask_counter] = mask_rear.channels[1].values[fusion_counter-440];
			mask_pseudo.channels[2].values[mask_counter] = mask_rear.channels[2].values[fusion_counter-440];
      mask_pseudo.channels[3].values[mask_counter] = mask_rear.channels[3].values[fusion_counter-440];
      mask_counter++;
      mask_rear_counter++;
#else
      mask_whole.points[mask_counter].x = -mask_rear.points[fusion_counter-440].x;
      mask_whole.points[mask_counter].y = -mask_rear.points[fusion_counter-440].y;
      mask_whole.points[mask_counter].z =  mask_rear.points[fusion_counter-440].z;
      mask_whole.channels[0].values[mask_counter] = mask_rear.channels[0].values[fusion_counter-440];
      mask_whole.channels[1].values[mask_counter] = mask_rear.channels[1].values[fusion_counter-440];
			mask_whole.channels[2].values[mask_counter] = mask_rear.channels[2].values[fusion_counter-440];
      mask_whole.channels[3].values[mask_counter] = mask_rear.channels[3].values[fusion_counter-440];

      mask_pseudo.points[mask_counter].x = -mask_rear.points[fusion_counter-440].x;
      mask_pseudo.points[mask_counter].y = -mask_rear.points[fusion_counter-440].y;
      mask_pseudo.points[mask_counter].z =  mask_rear.points[fusion_counter-440].z;
      mask_pseudo.channels[0].values[mask_counter] = mask_rear.channels[0].values[fusion_counter-440];
      mask_pseudo.channels[1].values[mask_counter] = mask_rear.channels[1].values[fusion_counter-440];
			mask_pseudo.channels[2].values[mask_counter] = mask_rear.channels[2].values[fusion_counter-440];
      mask_pseudo.channels[3].values[mask_counter] = mask_rear.channels[3].values[fusion_counter-440];
      mask_counter++;
      mask_rear_counter++;
#endif
    }
  }
  mask_whole.points.resize(mask_counter);
  get_mask_pseudo(mask_counter, mask_front_counter, mask_rear_counter);

}

void fusion()
{
if(get_front_flag==1 &&  get_rear_flag==1)
  {
      ibeo_fusion();
			//std::cout<<"pointcloud fusion finish."<<endl;
      mask_fusion();
			//std::cout<<"mask fusion finish."<<endl;
      fusion_pub_.publish(cloud_whole);
      roadSurface_pub_.publish(roadSurface);
      roadSurface3D_pub_.publish(roadSurface3D);


      get_front_flag=0;
      get_rear_flag=0;
			//std::cout<<"publish finish."<<endl;
  }
}


void listener_ibeo_4L(const sensor_msgs::PointCloud::ConstPtr& PointCloud_Ptr)
{
	//std::cout<<"start 4L collection."<<endl;

  numScanpoints_rear = PointCloud_Ptr->points.size();

  cloud_rear.header.stamp = ros::Time::now();
  cloud_rear.header.frame_id = "base_vehicle";
  cloud_rear.points.resize(numScanpoints_rear);
  cloud_rear.channels.resize(4);
  cloud_rear.channels[0].name = "Preprocessing Labels";
  cloud_rear.channels[0].values.resize(numScanpoints_rear);
  cloud_rear.channels[1].name = "Layers";
  cloud_rear.channels[1].values.resize(numScanpoints_rear);
	cloud_rear.channels[2].name = "Hangle";
  cloud_rear.channels[2].values.resize(numScanpoints_rear);
  cloud_rear.channels[3].name = "Distance";
  cloud_rear.channels[3].values.resize(numScanpoints_rear);

  for(int i=0; i<numScanpoints_rear;i++ )
  {
      cloud_rear.points[i].x = PointCloud_Ptr->points[i].x;
      cloud_rear.points[i].y = PointCloud_Ptr->points[i].y;
      cloud_rear.points[i].z = PointCloud_Ptr->points[i].z;
      cloud_rear.channels[0].values[i] = PointCloud_Ptr->channels[0].values[i];
      cloud_rear.channels[1].values[i] = PointCloud_Ptr->channels[1].values[i];
			cloud_rear.channels[2].values[i] = PointCloud_Ptr->channels[2].values[i];
			cloud_rear.channels[3].values[i] = PointCloud_Ptr->channels[3].values[i];
  }

	//std::cout<<"get 4L data."<<endl;
	//std::cout<<"num="<<numScanpoints_rear<<endl;
    
// set some part of the preprocessing label of pointcloud to be ground in order to reduce noise.  TO BE REVISED
// offgas processing
  for(int i=0;i<numScanpoints_rear;i++)
  {
    if(cloud_rear.channels[3].values[i]<1.5)
    {
        cloud_rear.channels[0].values[i] = 3;
    }
  }

  struct point_PointCloud mask_rear_temp[440][8]={0};
  int num_counter[440]={0};

  for(int i=0;i<numScanpoints_rear;i++)
  {
    if(cloud_rear.points[i].x==0)
      continue;

    float Hangle = cloud_rear.channels[2].values[i];

		if(Hangle>50||Hangle<-60)
		continue;

    int j=(Hangle+60.0)*4;

		if(num_counter[j]>=8)
			continue;

    mask_rear_temp[j][num_counter[j]].x    = cloud_rear.points[i].x;
    mask_rear_temp[j][num_counter[j]].y    = cloud_rear.points[i].y;
    mask_rear_temp[j][num_counter[j]].z    = cloud_rear.points[i].z;
    mask_rear_temp[j][num_counter[j]].ch0  = cloud_rear.channels[0].values[i];
    mask_rear_temp[j][num_counter[j]].ch1  = cloud_rear.channels[1].values[i];
		mask_rear_temp[j][num_counter[j]].ch2  = cloud_rear.channels[2].values[i];
    mask_rear_temp[j][num_counter[j]].ch3  = cloud_rear.channels[3].values[i];

    num_counter[j]++;

  }

	//std::cout<<"put points into cells finish."<<endl;

  for(int i=0;i<440;i++)
  {
    if(num_counter[i]>1)
    {
      for(int j=0;j<num_counter[i];j++)
      {
        for(int k=j+1;k<num_counter[i];k++)
        {
					float d1 = mask_rear_temp[i][j].ch3;
					float d2 = mask_rear_temp[i][k].ch3;

          if(d1>d2)
          {
            float temp_x = mask_rear_temp[i][j].x;
            float temp_y = mask_rear_temp[i][j].y;
            float temp_z = mask_rear_temp[i][j].z;
            float temp_ch0 = mask_rear_temp[i][j].ch0;
            float temp_ch1 = mask_rear_temp[i][j].ch1;
						float temp_ch2 = mask_rear_temp[i][j].ch2;
						float temp_ch3 = mask_rear_temp[i][j].ch3;

            mask_rear_temp[i][j].x = mask_rear_temp[i][k].x;
            mask_rear_temp[i][j].y = mask_rear_temp[i][k].y;
            mask_rear_temp[i][j].z = mask_rear_temp[i][k].z;
            mask_rear_temp[i][j].ch0 = mask_rear_temp[i][k].ch0;
            mask_rear_temp[i][j].ch1 = mask_rear_temp[i][k].ch1;
						mask_rear_temp[i][j].ch2 = mask_rear_temp[i][k].ch2;
            mask_rear_temp[i][j].ch3 = mask_rear_temp[i][k].ch3;

            mask_rear_temp[i][k].x = temp_x;
            mask_rear_temp[i][k].y = temp_y;
            mask_rear_temp[i][k].z = temp_z;
            mask_rear_temp[i][k].ch0 = temp_ch0;
            mask_rear_temp[i][k].ch1 = temp_ch1;
						mask_rear_temp[i][k].ch2 = temp_ch2;
            mask_rear_temp[i][k].ch3 = temp_ch3;
          }
        }
      }
      for(int m=0;m<num_counter[i]-1;m++)
      {
      if(mask_rear_temp[i][m].ch0!=3&&mask_rear_temp[i][num_counter[i]-1].ch0==3||mask_rear_temp[i][num_counter[i]-2].ch0==3)
      mask_rear_temp[i][m].ch0 =3;
      if(m<4&&mask_rear_temp[i][m].z>0.15)
      mask_rear_temp[i][m].ch0 =3;
      }
      
   //   for(int m=0;m<num_counter[i]-3;m++)
 //     {
   //   if(mask_rear_temp[i][m].ch0!=3&&mask_rear_temp[i][m+1].ch0==3||mask_rear_temp[i][m+2].ch0 ==3||mask_rear_temp[i][m+3].ch0 ==3 )
  //    mask_rear_temp[i][m].ch0 =3;
   //   if(m<4&&mask_rear_temp[i][m].z>0.15)
   //   mask_rear_temp[i][m].ch0 =3;
   //   }
    }
  }
	//std::cout<<"choose the one."<<endl;

  mask_rear.header.stamp = ros::Time::now();
  mask_rear.header.frame_id = "base_vehicle";
  mask_rear.points.resize(440);
  mask_rear.channels.resize(4);
  mask_rear.channels[0].name = "Preprocessing Labels";
  mask_rear.channels[0].values.resize(440);
  mask_rear.channels[1].name = "Layers";
  mask_rear.channels[1].values.resize(440);
	mask_rear.channels[2].name = "Hangle";
  mask_rear.channels[2].values.resize(440);
  mask_rear.channels[3].name = "Distance";
  mask_rear.channels[3].values.resize(440);

  for(int i=0;i<440;i++)
  {
    for(int j=0;j<num_counter[i];j++)
    {
      if(mask_rear_temp[i][j].ch0==3 ) //if ground?
      {
        if(j==num_counter[i]-1)  //if the last point?
        {
          if(mask_rear_temp[i][j].x==0)
            break;

          float Htheta = M_PI*mask_rear_temp[i][j].ch2/180.0;  //make a big distance
          mask_rear.points[i].x = ACCESS_RANGE*cos(Htheta);
          mask_rear.points[i].y = ACCESS_RANGE*sin(Htheta);
          mask_rear.points[i].z = 0;
          mask_rear.channels[0].values[i] = mask_rear_temp[i][j].ch0;
          mask_rear.channels[1].values[i] = mask_rear_temp[i][j].ch1;
					mask_rear.channels[2].values[i] = mask_rear_temp[i][j].ch2;
					mask_rear.channels[3].values[i] = mask_rear_temp[i][j].ch3;
          break;
        }
        else
        {
          continue;  //ground and not the last one.
        }
      }
      else
      {
        //the first point which is not ground
        mask_rear.points[i].x = mask_rear_temp[i][j].x;
        mask_rear.points[i].y = mask_rear_temp[i][j].y;
        mask_rear.points[i].z = mask_rear_temp[i][j].z;
        mask_rear.channels[0].values[i] = mask_rear_temp[i][j].ch0;
        mask_rear.channels[1].values[i] = mask_rear_temp[i][j].ch1;
				mask_rear.channels[2].values[i] = mask_rear_temp[i][j].ch2;
				mask_rear.channels[3].values[i] = mask_rear_temp[i][j].ch3;
        break;
      }
    }
    if(num_counter[i]==0)// if the block has no point
    {
      mask_rear.points[i].x = 0;
      mask_rear.points[i].y = 0;
      mask_rear.points[i].z = 0;
    }
  }

  get_rear_flag=1;
	//std::cout<<"get rear."<<endl;

	fusion();
}

void listener_ibeo_8L(const sensor_msgs::PointCloud::ConstPtr& PointCloud_Ptr)
{
	//std::cout<<"start 8L collection."<<endl;

  numScanpoints_front = PointCloud_Ptr->points.size();

  cloud_front.header.stamp = ros::Time::now();
  cloud_front.header.frame_id = "base_vehicle";
  cloud_front.points.resize(numScanpoints_front);
  cloud_front.channels.resize(4);
  cloud_front.channels[0].name = "Preprocessing Labels";
  cloud_front.channels[0].values.resize(numScanpoints_front);
  cloud_front.channels[1].name = "Layers";
  cloud_front.channels[1].values.resize(numScanpoints_front);
	cloud_front.channels[2].name = "Hangle";
  cloud_front.channels[2].values.resize(numScanpoints_front);
  cloud_front.channels[3].name = "Distance";
  cloud_front.channels[3].values.resize(numScanpoints_front);

  for(int i=0; i<numScanpoints_front;i++ )
  {
      cloud_front.points[i].x = PointCloud_Ptr->points[i].x;
      cloud_front.points[i].y = PointCloud_Ptr->points[i].y;
      cloud_front.points[i].z = PointCloud_Ptr->points[i].z;
      cloud_front.channels[0].values[i] = PointCloud_Ptr->channels[0].values[i];
      cloud_front.channels[1].values[i] = PointCloud_Ptr->channels[1].values[i];
			cloud_front.channels[2].values[i] = PointCloud_Ptr->channels[2].values[i];
			cloud_front.channels[3].values[i] = PointCloud_Ptr->channels[3].values[i];
  }

// set some part of the preprocessing label of pointcloud to be ground in order to reduce noise.  TO BE REVISED

  for(int i=0;i<numScanpoints_front;i++)
  {

	// if(cloud_front.channels[1].values[i]==0)
 //  	{
 //  		//if((cloud_front.channels[2].values[i]>25||cloud_front.channels[2].values[i]<-25) && (cloud_front.channels[3].values[i]>5 && cloud_front.channels[3].values[i]<10))		
 //  			cloud_front.channels[0].values[i] = 3;
 //  	}

  	if(cloud_front.channels[1].values[i]==1)
  	{
  		//if((cloud_front.channels[2].values[i]>25||cloud_front.channels[2].values[i]<-25) && (cloud_front.channels[3].values[i]>5 && cloud_front.channels[3].values[i]<10))		
  			cloud_front.channels[0].values[i] = 3;
  	}
  	if(cloud_front.channels[1].values[i]==2)
  	{
  		//if((cloud_front.channels[2].values[i]>25||cloud_front.channels[2].values[i]<-25) && (cloud_front.channels[3].values[i]>5 && cloud_front.channels[3].values[i]<20))
  			cloud_front.channels[0].values[i] = 3;
  	}
        if(cloud_front.channels[1].values[i]==3)
        {
                if((cloud_front.channels[2].values[i]>25||cloud_front.channels[2].values[i]<-25) && (cloud_front.channels[3].values[i]>5 && cloud_front.channels[3].values[i]<15))
                cloud_front.channels[0].values[i] = 3;
        }

        if(cloud_front.channels[1].values[i]==4 || cloud_front.channels[1].values[i]==5  || cloud_front.channels[1].values[i]==6 || cloud_front.channels[1].values[i]==7 )
        {
                cloud_front.channels[0].values[i] = 0;
        }

        if(cloud_front.points[i].z >1.25)
        {
                cloud_front.channels[0].values[i] = 3;
        }


  }






  struct point_PointCloud mask_front_temp[440][8]={0};
  int num_counter[440]={0};

  for(int i=0;i<numScanpoints_front;i++)
  {
    if(cloud_front.points[i].x==0)
      continue;

    float Hangle = cloud_front.channels[2].values[i];

		if(Hangle>50||Hangle<-60)
		continue;

    int j=(Hangle+60.0)*4;

		if(num_counter[j]>=8)
			continue;

    mask_front_temp[j][num_counter[j]].x    = cloud_front.points[i].x;
    mask_front_temp[j][num_counter[j]].y    = cloud_front.points[i].y;
    mask_front_temp[j][num_counter[j]].z    = cloud_front.points[i].z;
    mask_front_temp[j][num_counter[j]].ch0  = cloud_front.channels[0].values[i];
    mask_front_temp[j][num_counter[j]].ch1  = cloud_front.channels[1].values[i];
		mask_front_temp[j][num_counter[j]].ch2  = cloud_front.channels[2].values[i];
    mask_front_temp[j][num_counter[j]].ch3  = cloud_front.channels[3].values[i];

    num_counter[j]++;
  }

  for(int i=0;i<440;i++)
  {
    if(num_counter[i]>1)
    {
      for(int j=0;j<num_counter[i];j++)
      {
        for(int k=j+1;k<num_counter[i];k++)
        {
          float d1 = mask_front_temp[i][j].ch3;
					float d2 = mask_front_temp[i][k].ch3;

          if(d1>d2)
          {
            float temp_x = mask_front_temp[i][j].x;
            float temp_y = mask_front_temp[i][j].y;
            float temp_z = mask_front_temp[i][j].z;
            float temp_ch0 = mask_front_temp[i][j].ch0;
            float temp_ch1 = mask_front_temp[i][j].ch1;
						float temp_ch2 = mask_front_temp[i][j].ch2;
            float temp_ch3 = mask_front_temp[i][j].ch3;

            mask_front_temp[i][j].x = mask_front_temp[i][k].x;
            mask_front_temp[i][j].y = mask_front_temp[i][k].y;
            mask_front_temp[i][j].z = mask_front_temp[i][k].z;
            mask_front_temp[i][j].ch0 = mask_front_temp[i][k].ch0;
            mask_front_temp[i][j].ch1 = mask_front_temp[i][k].ch1;
						mask_front_temp[i][j].ch2 = mask_front_temp[i][k].ch2;
						mask_front_temp[i][j].ch3 = mask_front_temp[i][k].ch3;

            mask_front_temp[i][k].x = temp_x;
            mask_front_temp[i][k].y = temp_y;
            mask_front_temp[i][k].z = temp_z;
            mask_front_temp[i][k].ch0 = temp_ch0;
            mask_front_temp[i][k].ch1 = temp_ch1;
						mask_front_temp[i][k].ch2 = temp_ch2;
						mask_front_temp[i][k].ch3 = temp_ch3;
          }
        }
      }
    }
  }

  mask_front.header.stamp = ros::Time::now();
  mask_front.header.frame_id = "base_vehicle";
  mask_front.points.resize(440);
  mask_front.channels.resize(4);
  mask_front.channels[0].name = "Preprocessing Labels";
  mask_front.channels[0].values.resize(440);
  mask_front.channels[1].name = "Layers";
  mask_front.channels[1].values.resize(440);
	mask_front.channels[2].name = "Hangle";
  mask_front.channels[2].values.resize(440);
  mask_front.channels[3].name = "Distance";
  mask_front.channels[3].values.resize(440);

  for(int i=0;i<440;i++)
  {
    for(int j=0;j<num_counter[i];j++)
    {
      if(mask_front_temp[i][j].z<0.05) //if ground?   mask_front_temp[i][j].ch0==3
      {
        if(j==num_counter[i]-1)  //if the last point?
        {
          if(mask_front_temp[i][j].x==0)
            break;

          int Htheta = M_PI*mask_front_temp[i][j].ch2/180.0;  //make a big distance
          mask_front.points[i].x = ACCESS_RANGE*cos(Htheta);
          mask_front.points[i].y = ACCESS_RANGE*sin(Htheta);
          mask_front.points[i].z = 0;
          mask_front.channels[0].values[i] = mask_front_temp[i][j].ch0;
          mask_front.channels[1].values[i] = mask_front_temp[i][j].ch1;
					mask_front.channels[2].values[i] = mask_front_temp[i][j].ch2;
          mask_front.channels[3].values[i] = mask_front_temp[i][j].ch3;
          break;
        }
        else
        {
          continue;  //ground and not the last one.
        }
      }
      else
      {
        //the first point which is not ground
        mask_front.points[i].x = mask_front_temp[i][j].x;
        mask_front.points[i].y = mask_front_temp[i][j].y;
        mask_front.points[i].z = mask_front_temp[i][j].z;
        mask_front.channels[0].values[i] = mask_front_temp[i][j].ch0;
        mask_front.channels[1].values[i] = mask_front_temp[i][j].ch1;
				mask_front.channels[2].values[i] = mask_front_temp[i][j].ch2;
        mask_front.channels[3].values[i] = mask_front_temp[i][j].ch3;
        break;
      }
    }
    if(num_counter[i]==0)
    {
      mask_front.points[i].x = 0;
      mask_front.points[i].y = 0;
      mask_front.points[i].z = 0;
    }
  }

  get_front_flag=1;
	//std::cout<<"get front."<<endl;

	fusion();
}

void GetDeleteArea(in2_msgs::IbeoObjects object_temp)
{
	int numMovingObjects=0;
	int MovingObjectsCounter=0;

	for(int i=0;i<object_temp.Object.size();i++)
	{
		int ScalarSpeed = sqrt(object_temp.Object[i].AbsoluteVelocity.x*object_temp.Object[i].AbsoluteVelocity.x+object_temp.Object[i].AbsoluteVelocity.y*object_temp.Object[i].AbsoluteVelocity.y);
		if(ScalarSpeed > 100 && ScalarSpeed < 2700)
		{
			numMovingObjects++;
		}
	}
	in2_msgs::IbeoObjects MovingObjects;
	MovingObjects.header.stamp = ros::Time::now();
	MovingObjects.header.frame_id = "base_vehicle";
	MovingObjects.Object.resize(numMovingObjects);

	for(int i=0;i<object_temp.Object.size();i++)
	{
		int ScalarSpeed = sqrt(object_temp.Object[i].AbsoluteVelocity.x*object_temp.Object[i].AbsoluteVelocity.x+object_temp.Object[i].AbsoluteVelocity.y*object_temp.Object[i].AbsoluteVelocity.y);
		if(ScalarSpeed > 100 && ScalarSpeed < 2700)
		{
			MovingObjects.Object[MovingObjectsCounter].ContourPoints.resize(object_temp.Object[i].NumofContourPoints);
			MovingObjects.Object[MovingObjectsCounter] = object_temp.Object[i];
			MovingObjectsCounter++;
		}
	}

	float DeleteBlocks[numMovingObjects][4];

	//std::cout<<"numMovingObjects:"<<numMovingObjects<<endl;

	for(int i=0 ; i<numMovingObjects ; i++)
	{
		DeleteBlocks[i][0] = (MovingObjects.Object[i].BoundingBoxCenter.x - MovingObjects.Object[i].BoundingBoxSize.x/2)/100.0;
		DeleteBlocks[i][1] = (MovingObjects.Object[i].BoundingBoxCenter.x + MovingObjects.Object[i].BoundingBoxSize.x/2)/100.0;
		DeleteBlocks[i][2] = (MovingObjects.Object[i].BoundingBoxCenter.y - MovingObjects.Object[i].BoundingBoxSize.y/2)/100.0;
		DeleteBlocks[i][3] = (MovingObjects.Object[i].BoundingBoxCenter.y + MovingObjects.Object[i].BoundingBoxSize.y/2)/100.0;
	}

	if(get_cloud_whole == 1)
	{
		sensor_msgs::PointCloud cloud_static_temp;
		sensor_msgs::PointCloud cloud_static;

		int numPointCloud_whole = cloud_whole.points.size();

		cloud_static_temp.header.stamp = ros::Time::now();
		cloud_static_temp.header.frame_id = "base_vehicle";
		cloud_static_temp.points.resize(numPointCloud_whole);
		cloud_static_temp.channels.resize(4);
		cloud_static_temp.channels[0].name = "Preprocessing Labels";
		cloud_static_temp.channels[0].values.resize(numPointCloud_whole);
		cloud_static_temp.channels[1].name = "Layers";
		cloud_static_temp.channels[1].values.resize(numPointCloud_whole);
		cloud_static_temp.channels[2].name = "Hangle";
		cloud_static_temp.channels[2].values.resize(numPointCloud_whole);
		cloud_static_temp.channels[3].name = "Distance";
		cloud_static_temp.channels[3].values.resize(numPointCloud_whole);

		cloud_static_temp = cloud_whole;

		for(int i=0;i<numPointCloud_whole;i++)
		{
			for(int j=0;j<numMovingObjects;j++)
			{
				if((cloud_static_temp.points[i].x>DeleteBlocks[j][0]-0.2 && cloud_static_temp.points[i].x<DeleteBlocks[j][1]+0.2)||cloud_static_temp.channels[0].values[i]==3)
				{
					if((cloud_static_temp.points[i].y>DeleteBlocks[j][2]-0.2 && cloud_static_temp.points[i].y<DeleteBlocks[j][3]+0.2)||cloud_static_temp.channels[0].values[i]==3)
					{
						cloud_static_temp.points[i].x=0;
						cloud_static_temp.points[i].y=0;
						cloud_static_temp.points[i].z=0;
						break;
					}
				}
			}
		}

		cloud_static.header.stamp = ros::Time::now();
		cloud_static.header.frame_id = "base_vehicle";
		cloud_static.points.resize(numPointCloud_whole);

		double cloud_static_counter = 0;
		for(int i=0;i<numPointCloud_whole;i++)
		{
			if(cloud_static_temp.points[i].x!=0 && cloud_static_temp.points[i].y!=0)
			{
				cloud_static.points[cloud_static_counter].x = cloud_static_temp.points[i].x;
				cloud_static.points[cloud_static_counter].y = cloud_static_temp.points[i].y;
				cloud_static.points[cloud_static_counter].z = cloud_static_temp.points[i].z;
				cloud_static_counter++;
			}
		}
		cloud_static.points.resize(cloud_static_counter);

		static_pointcloud_pub_.publish(cloud_static);
		get_cloud_whole=0;
	}
}


void object_fusion()
{
	if(get_front_obj==1 && get_rear_obj==1)
	{
		int numObjects = numObjects_front + numObjects_rear;

		Objects_whole.header.stamp = ros::Time::now();
		Objects_whole.header.frame_id = "base_vehicle";
		Objects_whole.Object.resize(numObjects);

		for(int i=0;i<numObjects_front;i++)
		{
			Objects_whole.Object[i].ContourPoints.resize(Objects_front.Object[i].NumofContourPoints);
			Objects_whole.Object[i] = Objects_front.Object[i];
		}
		for(int i=0;i<numObjects_rear;i++)
		{
			Objects_whole.Object[numObjects_front+i].ContourPoints.resize(Objects_rear.Object[i].NumofContourPoints);
			Objects_whole.Object[numObjects_front+i] = Objects_rear.Object[i];
			Objects_whole.Object[numObjects_front+i].ID = Objects_rear.Object[i].ID+256;
		}
		objects_pub_.publish(Objects_whole);
		std::cout<<"numObjects:"<<Objects_whole.Object.size()<<endl;
		GetDeleteArea(Objects_whole);

		get_front_obj=0;
		get_rear_obj=0;
	}
}

void listener_ibeo_8L_obj(const in2_msgs::IbeoObjects::ConstPtr& IbeoObjects_Ptr)
{
	numObjects_front = IbeoObjects_Ptr->Object.size();
	Objects_front.header.stamp = ros::Time::now();
	Objects_front.header.frame_id = "base_ibeo_8L";
	Objects_front.Object.resize(numObjects_front);

	for(int i=0;i<numObjects_front;i++)
	{
		Objects_front.Object[i].ContourPoints.resize(IbeoObjects_Ptr->Object[i].NumofContourPoints);
		Objects_front.Object[i] = IbeoObjects_Ptr->Object[i];
		//std::cout<< Objects_front.Object[i] <<endl;
	}
	get_front_obj=1;
	object_fusion();
}

void listener_ibeo_4L_obj(const in2_msgs::IbeoObjects::ConstPtr& IbeoObjects_Ptr)
{
	numObjects_rear = IbeoObjects_Ptr->Object.size();
	Objects_rear.header.stamp = ros::Time::now();
	Objects_rear.header.frame_id = "base_ibeo_4L";
	Objects_rear.Object.resize(numObjects_rear);

	for(int i=0;i<numObjects_rear;i++)
	{
		Objects_rear.Object[i].ContourPoints.resize(IbeoObjects_Ptr->Object[i].NumofContourPoints);
		Objects_rear.Object[i] = IbeoObjects_Ptr->Object[i];
		//std::cout<< Objects_rear.Object[i] <<endl;
	}
	get_rear_obj=1;
	object_fusion();
}

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "ibeo_fusion_node");
  	ros::NodeHandle n;
  	my_servertime.C_ServerTime_Init();
	fusion_pub_ = n.advertise<sensor_msgs::PointCloud>("ibeoData_node", 10);
  	//scanInfov2_pub_ = n.advertise<in2_msgs::ScanInfoV2>("ScanInfoV2_ibeo", 10);
	objects_pub_ = n.advertise<in2_msgs::IbeoObjects>("ibeo_objects", 10);
	static_pointcloud_pub_ = n.advertise<sensor_msgs::PointCloud>("Static_PointCloud", 10);
	roadSurface_pub_ = n.advertise<in2_msgs::RoadSurface>("RoadSurface2D_ibeo",10);
        roadSurface3D_pub_ = n.advertise<in2_msgs::RoadSurface3D>("RoadSurface_ibeo",10);

  	ros::Subscriber sub_4L = n.subscribe("ibeo_4L_scan", 10, listener_ibeo_4L);
  	ros::Subscriber sub_8L = n.subscribe("ibeo_8L_scan", 10, listener_ibeo_8L);
	ros::Subscriber sub_8L_obj = n.subscribe("ibeo_8L_objects", 10, listener_ibeo_8L_obj);
	ros::Subscriber sub_4L_obj = n.subscribe("ibeo_4L_objects", 10, listener_ibeo_4L_obj);

  	ros::spin();
	return 0;

}
