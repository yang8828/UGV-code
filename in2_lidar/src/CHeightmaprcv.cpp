
#include "CHeightmaprcv.h"

/* point cloud input callback */
void CHeightmap::recieve_heightmap_16l(const sensor_msgs::ImageConstPtr &heightmap16l)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr =  cv_bridge::toCvCopy(heightmap16l, sensor_msgs::image_encodings::BGR8);
    heightmap1=cv_ptr->image;
    flag16l=true;    

    //ROS_INFO("16L RECIEVED!");
    if(showimages)
    {
    	imshow("heightmaprcv-l",heightmap1);
    	waitKey(1);
    }
} 

void CHeightmap::recieve_heightmap_16r(const sensor_msgs::ImageConstPtr &heightmap16r)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr =  cv_bridge::toCvCopy(heightmap16r, sensor_msgs::image_encodings::BGR8);
    heightmap2=cv_ptr->image;
    flag16r=true;

    //ROS_INFO("16R RECIEVED!");
    if(showimages)
    {
    	imshow("heightmaprcv-r",heightmap2);
    	waitKey(1);
    }
    
} 

void CHeightmap::recieve_heightmap_ibeo(const sensor_msgs::ImageConstPtr &heightmapibeo)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr =  cv_bridge::toCvCopy(heightmapibeo, sensor_msgs::image_encodings::BGR8);
    heightmap3=cv_ptr->image;
    flagibeo=true;

    //ROS_INFO("ibeo RECIEVED!");
    if(showimages)
    {
    	imshow("heightmaprcv-ibeo",heightmap3);
    	waitKey(1);
    }
} 


CHeightmap::CHeightmap(ros::NodeHandle node, ros::NodeHandle priv_nh)
{
	
	flag16l=false;
    flag16r=false;
    flagibeo=false;

    rcv_16l=0;
    rcv_16r=0;
    rcv_ibeo=0;

    node.param("/heightmaprcv_node/showmessages", showmessages, false);
    node.param("/heightmaprcv_node/showimages",   showimages,   false);

	hml = node.subscribe("heightmap16l", 10,
					&CHeightmap::recieve_heightmap_16l, this,
					ros::TransportHints().tcpNoDelay(true));

	hmr = node.subscribe("heightmap16r", 10,
					&CHeightmap::recieve_heightmap_16r, this,
					ros::TransportHints().tcpNoDelay(true));

	hmibeo = node.subscribe("heightmap_ibeo", 10,
					&CHeightmap::recieve_heightmap_ibeo, this,
					ros::TransportHints().tcpNoDelay(true));

	// Set up publishers  
	image_transport::ImageTransport it(node);
	hm_publisher = it.advertise("heightmap",1);

	Mat heightmap_fusion = Mat(300,300, CV_8UC3, Scalar(100,100,100));
	heightmap1_old = Mat(300,300, CV_8UC3, Scalar(100,100,100));
	heightmap2_old = Mat(300,300, CV_8UC3, Scalar(100,100,100));
	heightmap3_old = Mat(300,300, CV_8UC3, Scalar(100,100,100));

	ros::Rate looprate(10);
	while(ros::ok())
	{
		if(flag16l) rcv_16l = 0;
		else rcv_16l ++;

		if(flag16r) rcv_16r = 0;
		else rcv_16r ++;

		if(flagibeo) rcv_ibeo = 0;
		else rcv_ibeo ++;

		for(int i=0; i<300; i++)
		{
			for(int j=0; j<300; j++)
			{
				heightmap_fusion.at<Vec3b>(j,i)[0] = 100;
				heightmap_fusion.at<Vec3b>(j,i)[1] = 100;
				heightmap_fusion.at<Vec3b>(j,i)[2] = 100;
			}
		}

		if(showmessages) ROS_INFO("/***************/ %d %d %d ",rcv_16l,rcv_16r,rcv_ibeo);

		if(rcv_16l <=5)
		{
			if(flag16l)
			{
				heightmap1_old = heightmap1;
			}
			else
			{
				heightmap1 = heightmap1_old;
			}

			for(int i=0; i<300; i++)
			{
				for(int j=0; j<300; j++)
				{
					if(heightmap1.at<Vec3b>(j,i)[0]==255)
					{
						heightmap_fusion.at<Vec3b>(j,i)[0]=255;
						heightmap_fusion.at<Vec3b>(j,i)[1]=0;
						heightmap_fusion.at<Vec3b>(j,i)[2]=0;
					}
					else
					{
						if(heightmap1.at<Vec3b>(j,i)[1] == 255 && heightmap_fusion.at<Vec3b>(j,i)[0] != 255)
						{
							heightmap_fusion.at<Vec3b>(j,i)[0]=0;
							heightmap_fusion.at<Vec3b>(j,i)[1]=255;
							heightmap_fusion.at<Vec3b>(j,i)[2]=0;
						}
					}
				}
			}

		}
		else
		{
			if(showmessages) ROS_INFO("16E Left: No New Message");
		}

		if(rcv_16r <=5)
		{
			if(flag16r)
			{
				heightmap2_old = heightmap2;
			}
			else
			{
				heightmap2 = heightmap2_old;
			}


			for(int i=0; i<300; i++)
			{
				for(int j=0; j<300; j++)
				{
					if(heightmap2.at<Vec3b>(j,i)[0]==255)
					{
						heightmap_fusion.at<Vec3b>(j,i)[0]=255;
						heightmap_fusion.at<Vec3b>(j,i)[1]=0;
						heightmap_fusion.at<Vec3b>(j,i)[2]=0;
					}
					else
					{
						if(heightmap2.at<Vec3b>(j,i)[1] == 255 && heightmap_fusion.at<Vec3b>(j,i)[0] != 255)
						{
							heightmap_fusion.at<Vec3b>(j,i)[0]=0;
							heightmap_fusion.at<Vec3b>(j,i)[1]=255;
							heightmap_fusion.at<Vec3b>(j,i)[2]=0;
						}
					}
				}
			}

		}
		else
		{
			if(showmessages) ROS_INFO("16E Right: No New Message");
		}

		if(rcv_ibeo <=5)
		{
			if(flagibeo)
			{
				heightmap3_old = heightmap3;
			}
			else
			{
				heightmap3 = heightmap3_old;
			}


			for(int i=0; i<300; i++)
			{
				for(int j=0; j<300; j++)
				{
					if(heightmap3.at<Vec3b>(j,i)[0]==255)
					{
						heightmap_fusion.at<Vec3b>(j,i)[0]=255;
						heightmap_fusion.at<Vec3b>(j,i)[1]=0;
						heightmap_fusion.at<Vec3b>(j,i)[2]=0;
					}
					else
					{
						if(heightmap3.at<Vec3b>(j,i)[1] == 255 && heightmap_fusion.at<Vec3b>(j,i)[0] != 255)
						{
							heightmap_fusion.at<Vec3b>(j,i)[0]=0;
							heightmap_fusion.at<Vec3b>(j,i)[1]=255;
							heightmap_fusion.at<Vec3b>(j,i)[2]=0;
						}
					}
				}
			}

		}
		else
		{
			if(showmessages) ROS_INFO("ibeo: No New Message");
		}

		imshow("heightmaprcv-fusion",heightmap_fusion);
		waitKey(1);

		//publish heightmap topic 
		std_msgs::Header header0;
		header0.frame_id = "heightmap_fusion";
		header0.stamp = ros::Time::now();
		cv_bridge::CvImage cv_image = cv_bridge::CvImage(header0,"bgr8",heightmap_fusion);
		hm_publisher.publish(cv_image.toImageMsg());
		
		flag16l=false;
		flag16r=false;
		flagibeo=false;
		if(showmessages) ROS_INFO("/***************/");
		if(showmessages) ROS_INFO("                 ");

		ros::spinOnce();
		looprate.sleep();
	}	

}