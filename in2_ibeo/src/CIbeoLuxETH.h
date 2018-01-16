/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CIbeoLuxETH_4L_H
#define CIbeoLuxETH_4L_H

#include <ros/ros.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/utils/CClientTCPSocket.h>
//#include <pcl/point_types.h>
//#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <mrpt/hwdrivers/CGenericSensor.h>

using namespace std;


#define IBEO_4L_LUX 0
#define IBEO_8L_LUX 1

//typedef pcl::PointXYZI VPoint;
//typedef pcl::PointCloud<VPoint> VPointCloud;

//FILE *fp = fopen("ibeo_lux_data.txt","w");



//VPointCloud::Ptr cloud (new VPointCloud);
//VPointCloud cloud;
namespace mrpt
{
	namespace hwdrivers
	{
		using namespace std;
		using namespace mrpt::hwdrivers;
		using namespace mrpt::utils;
		using namespace mrpt::slam;

		/** This "software driver" implements the communication protocol for interfacing a Ibeo Lux laser scanners through an ethernet controller.
		*   This class does not need to be bind, i.e. you do not need to call C2DRangeFinderAbstract::bindIO.
		*   Connection is established when user call the turnOn() method. You can pass to the class's constructor the Lux's ip address and port.
		*   Device will NOT be configured. Configuration has to be done seperately.
		*
		* To get a laser scan you must proceed like that :
		* \code
		*     CIbeoLuxETH laser(string("192.168.0.10"), 1234);
		*     laser.turnOn();
		*     bool isOutObs, hardwareError;
		*     CObservation2DRangeScan outObs;
		*     laser.doProcessSimple(isOutObs, outObs, hardwareError);
		* \endcode
		*
		* \note This class was contributed by Adrien Barral - Robopec (France)
		* \note And modified by Jan Girlich - University of Hamburg
		* \ingroup mrpt_hwdrivers_grp
		*/
		class HWDRIVERS_IMPEXP CIbeoLuxETH : public mrpt::hwdrivers::CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CIbeoLuxETH)

		public:
		/** Constructor.
		* Note that there is default arguments, here you can customize IP Adress and TCP Port of your device.
		*/
			//CIbeoLuxETH(string _ip=string("10.152.36.93"), unsigned int _port=12002, ros::NodeHandle node, ros::NodeHandle priv_nh);
			CIbeoLuxETH(string _ip=string("10.152.36.93"), unsigned int _port=12002);
		/** Destructor.
		* Close communcation with the device, and free memory.
		*/
			virtual ~CIbeoLuxETH();
		/** This function acquire a laser scan from the device. If an error occured, hardwareError will be set to true.
		* The new laser scan will be stored in the outObservation argument.
		*
		* \exception This method throw exception if the frame received from the LMS 100 contain the following bad parameters :
		*  * Status is not OK
		*  * Data in the scan aren't DIST1 (may be RSSIx or DIST2).
		*/
			void doProcess();
			void initialize();
			void start();
			void stop();
			void makeCommandHeader(unsigned char* buffer);
			void makeStartCommand(unsigned char* buffer);
			void makeStopCommand(unsigned char* buffer);
			void makeTypeCommand(unsigned char* buffer);
			void makeFrequencyCommand(unsigned char* buffer, unsigned int freq);   //set frequency

		private :
			string			m_ip;
			unsigned int		m_port;
			CClientTCPSocket	m_client;
			unsigned int		m_scanFrequency;    // in hertz
			double			m_angleResolution;  // in degrees
			double			m_startAngle;       // degrees
			double			m_stopAngle;        // degrees
			mrpt::poses::CPose3D			m_sensorPose;
			double			m_maxRange;
			double			m_beamApperture;
			bool			m_run;
			void				dataCollection();
			mrpt::system::TThreadHandle	dataCollectionThread;
			float				convertLayerToRad(int scanlayer);
			float				convertTicksToHRad(int hticks, int hticksPerRotation);
			mrpt::poses::CPoint3D			convertToCartesian(float vrad, float hrad, float distance);
			float				vwinkel;
			vector<CObservation3DRangeScan>	m_observations;
			bool				m_newObs;
			float				m_vAngle;
			unsigned int			lastScanNumber;
			unsigned int			curScanNumber;

		protected:
		/** Load sensor pose on the robot, or keep the default sensor pose.
		*/
			void loadConfig_sensorSpecific(const mrpt::utils::CConfigFileBase &configSource,
                             const std::string	  &iniSection );
        };
    }
}

#endif
