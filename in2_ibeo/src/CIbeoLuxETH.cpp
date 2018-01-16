/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
//#include "hwdrivers-precomp.h"   // Precompiled headers

#include "CIbeoLuxETH.h" // Precompiled headers
#include <in2_msgs/IbeoObjects.h>

#include <bitset>

#define APPERTURE           4.712385    // in radian <=> 270°

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace std;

// TODO: Use enum instead
const unsigned char SearchForAF	= 0;
const unsigned char SearchForFE	= 1;
const unsigned char SearchForC0	= 2;
const unsigned char SearchForC2	= 3;
const unsigned char PacketFound	= 4;
const unsigned char SavePointCloud	= 5;
const unsigned char SaveObjects	= 6;

const unsigned char flag_valid					= 	0;
const unsigned char flag_transparent	  = 	1;
const unsigned char flag_clutter				= 	2;
const unsigned char flag_ground		  		=		3;
const unsigned char flag_dirt  					= 	4;
//const unsigned char reserved	= 5;

bool if_transparent;
bool if_clutter;
bool if_ground;
bool if_dirt;

//extern ros::NodeHandle node;
extern bool data_flag;
extern bool objects_flag;

extern sensor_msgs::PointCloud cloud_pc_upper;
extern sensor_msgs::PointCloud cloud_pc_lower;
extern in2_msgs::IbeoObjects Objects;

extern int numScanpoints_upper;
extern int numScanpoints_lower;

extern bool get_lower_flag;
extern bool get_upper_flag;

extern int IBEO_TYPE;

long watchdog_counter;

int mirrorSide;

long counter=0;

typedef struct Point2D
{
  int x;
  int y;
}Point2D;

typedef struct Size2D
{
  unsigned int x; //X
  unsigned int y;  //Y
}Size2D;

typedef struct ScanObjectData
{
  unsigned int ID,Age,PredictionAge;
  Point2D ReferencePoint;
  Point2D BoundingBoxCenter;
  unsigned int BoundingBoxWidth,BoundingBoxLength;
  Point2D ObjectBoxCenter;
  Size2D  ObjectBoxSize;
  int ObjectBoxOrientation;
  Point2D AbsoluteVelocity;
  Size2D  AbsoluteVelocitySigma;
  Point2D RelativeVelocity;
  unsigned int Classification;
  unsigned int ClassificationAge;
  unsigned int ClassificationCertainty;
  unsigned int NumofContourPoints;
  Point2D *ContourPoints;

}ScanObjectData;

IMPLEMENTS_GENERIC_SENSOR(CIbeoLuxETH,mrpt::hwdrivers)

CIbeoLuxETH::CIbeoLuxETH(string _ip, unsigned int _port):
        m_ip(_ip),
        m_port(_port),
	m_sensorPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        m_maxRange(200.0),
        m_beamApperture(.25*M_PI/180.0),
        vwinkel(0.0)
{
	//initialize();
}

CIbeoLuxETH::~CIbeoLuxETH()
{
  m_run = false;
}

short int ConvertToInt16(unsigned int input)
{
//  std::cout<<input<<"\n";
  int output;
  output = (short int)input;
//  std::cout<<output<<endl;
  return output;
}

float AccurateTo2Dec(float input)
{
  float output;
  if(input >= 0)
  {
    output = ((int)(input*100.0+0.5))/100.0;
  }
  else
  {
    output = ((int)(input*100.0-0.5))/100.0;
  }
  return output;
}


void CIbeoLuxETH::dataCollection()
{

	unsigned char state = SearchForAF;
	unsigned char msgIn[1], Header[20], ScanListHeader[44], ScanPointData[10];
	unsigned int datatype, numScanpoints, angleTicks, SPlayer, SPdistance, SPtype; // SPecho;
  	unsigned int ScanNumber,ScannerStatus, SyncPhaseOffset;
	int SPHangle;

  	unsigned char ObjectHeader[10],ObjectContent[58],ContourPointInfo[4];
  	unsigned int numScanObjects;

	m_run = true;

	while(m_run)
	{
		// watchdog_counter++;
		// std::cout<<"watchdog_counter:"<<watchdog_counter<<endl;
		
		// if(watchdog_counter>100)
		// {
		// 	std::cout<<"Ibeo ERROR!"<<endl;
		// 	initialize();
		// }
		
		std::cout<<"start"<<endl;
		switch(state)
		{
			case SearchForAF:
				std::cout<<"SearchForAF"<<endl;
				m_client.readAsync(msgIn, 1, 100, 10);
				if (msgIn[0] == 0xAF) state = SearchForFE;
				break;
			case SearchForFE:
				std::cout<<"SearchForFE"<<endl;
				m_client.readAsync(msgIn, 1, 100, 10);
				if (msgIn[0] == 0xFE) state = SearchForC0; else state = SearchForAF;
				break;
			case SearchForC0:
				std::cout<<"SearchForC0"<<endl;
				m_client.readAsync(msgIn, 1, 100, 10);
				if (msgIn[0] == 0xC0) state = SearchForC2; else state = SearchForAF;
				break;
			case SearchForC2:
				std::cout<<"SearchForC2"<<endl;
				m_client.readAsync(msgIn, 1, 100, 10);
				if (msgIn[0] == 0xC2) state = PacketFound; else state = SearchForAF;
				break;
			case PacketFound:
				std::cout<<"PacketFound"<<endl;
				m_client.readAsync(Header, 20, 100, 10);
				datatype = Header[10] * 0x100 + Header[11];
        std::cout<<"datatype:"<<datatype<<endl;
        switch(datatype)
				{
					case 0x2030:
						// do nothing
						std::cout<<"case:0x2030"<<endl;
						state = SearchForAF;
						break;
					case 0x2221:
						std::cout<<"case:0x2221"<<endl;
						state = SaveObjects;
            						//state = SearchForAF;
						break;
					case 0x2805:
            // do nothing
						std::cout<<"case:0x2805"<<endl;
						state = SearchForAF;
						break;
					case 0x2020:
						std::cout<<"case:0x2020"<<endl;
						// do nothing
						state = SearchForAF;
						break;
					case 0x2202:
						std::cout<<"case:0x2202"<<endl;
						state = SavePointCloud;
            //state = SearchForAF;
						break;
					default:
						std::cerr << "UNKNOWN packet of type " << hex << datatype << " received!!\n";
						state = SearchForAF;
				}
				break;
			case SavePointCloud:
      {
        cout<<"State: Saving pointcloud."<<endl;
				// // Create new observation object pointer
				// CObservation3DRangeScanPtr newObs = CObservation3DRangeScan::Create();
				// newObs->hasPoints3D = true;
				// newObs->maxRange = 200.00;

				m_client.readAsync(ScanListHeader, 44, 10, 10);

        ScanNumber = ScanListHeader[1] * 0x100 + ScanListHeader[0];
        ScannerStatus = ScanListHeader[3] * 0x100 + ScanListHeader[2];
        SyncPhaseOffset = ScanListHeader[5] * 0x100 + ScanListHeader[4];
        //std::cout<<"ScanNumber:"<<ScanNumber<<endl;
        //std::cout<<"ScannerStatus:"<<ScannerStatus<<endl;
        //std::cout<<"SyncPhaseOffset:"<<SyncPhaseOffset<<endl;


				numScanpoints = ScanListHeader[29] * 0x100 + ScanListHeader[28];
				angleTicks = ScanListHeader[23] * 0x100 + ScanListHeader[22];
				mirrorSide =  (ScanListHeader[43] & 0x04) >> 2;  //0 = front = lower, 1 = rear =upper
				std::cout<<"get scanNum"<<endl;
				
        //if(numScanpoints>200 && numScanpoints<2000)
          //watchdog_counter=0;

				if( mirrorSide==1 )
				{
					numScanpoints_upper=numScanpoints;

					cloud_pc_upper.header.stamp = ros::Time::now();
					cloud_pc_upper.header.frame_id = "base_laser";
					cloud_pc_upper.points.resize(numScanpoints);
					cloud_pc_upper.channels.resize(4);
					cloud_pc_upper.channels[0].name = "Preprocessing Labels";
					cloud_pc_upper.channels[0].values.resize(numScanpoints);
				  cloud_pc_upper.channels[1].name = "Layers";
					cloud_pc_upper.channels[1].values.resize(numScanpoints);
				  cloud_pc_upper.channels[2].name = "Hangle";
					cloud_pc_upper.channels[2].values.resize(numScanpoints);
				  cloud_pc_upper.channels[3].name = "Distance";
					cloud_pc_upper.channels[3].values.resize(numScanpoints);

					get_upper_flag = 1;
				}

				if( mirrorSide==0 )
				{
					numScanpoints_lower=numScanpoints;
					cloud_pc_lower.header.stamp = ros::Time::now();
					cloud_pc_lower.header.frame_id = "base_laser";
					cloud_pc_lower.points.resize(numScanpoints);
					cloud_pc_lower.channels.resize(4);
					cloud_pc_lower.channels[0].name = "Preprocessing Labels";
					cloud_pc_lower.channels[0].values.resize(numScanpoints);
					cloud_pc_lower.channels[1].name = "Layers";
					cloud_pc_lower.channels[1].values.resize(numScanpoints);
					cloud_pc_lower.channels[2].name = "Hangle";
					cloud_pc_lower.channels[2].values.resize(numScanpoints);
				  cloud_pc_lower.channels[3].name = "Distance";
					cloud_pc_lower.channels[3].values.resize(numScanpoints);

					get_lower_flag = 1;
				}

				std::cout<<counter++<<"\t scan points = "<<numScanpoints<<"\t MirrorSide: "<<mirrorSide<<endl;

				for (unsigned int i = 0; i < numScanpoints; ++i)
				{
					bool dropPacket = false;

					m_client.readAsync(ScanPointData, 10, 10, 10);
					SPlayer = ScanPointData[0] & 0x0F; // two lower bits denote layer
					// SPecho  = ScanPointData[0] >> 4; // two higher bits denote echo
					SPHangle = (char)ScanPointData[3] * 0x100 + ScanPointData[2]; // signed INT16 here
					SPdistance = ScanPointData[5] * 0x100 + ScanPointData[4];

          if_transparent   = ScanPointData[1] & 0x01;
          if_clutter = (ScanPointData[1] & 0x02)>>1;
          if_ground  = (ScanPointData[1] & 0x04)>>2;
          if_dirt    = (ScanPointData[1] & 0x08)>>3;

          if(if_ground  == 1)
          {
            SPtype = flag_ground;
          }
          else
          {
            SPtype = flag_valid;
          }

					// Sanity checks
					if (SPlayer > 4)
					{
						dropPacket = true;
					}
					if ((SPHangle < -((int)angleTicks)/2) || (SPHangle > (int)angleTicks/2))
					{
						dropPacket = true;
					}
					if ((SPdistance < 30) || (SPdistance > 20000))
					{
						dropPacket = true;
					}


					if (!dropPacket)
					{
						//TODO: Process point information correctly
						mrpt::poses::CPoint3D cartesianPoint = convertToCartesian(
							convertLayerToRad(SPlayer), // vertikal coord of scanner
							convertTicksToHRad(SPHangle, angleTicks), // horizontal coord of scanner
							SPdistance);

						// write scanpoint data to observation object
						// newObs->points3D_x.push_back(cartesianPoint.x());
						// newObs->points3D_y.push_back(cartesianPoint.y());
						// newObs->points3D_z.push_back(cartesianPoint.z());

						//ros topic
						if( mirrorSide==1 )
						{
							if(i<numScanpoints)
							{
								cloud_pc_upper.points[i].x = cartesianPoint.x();
								cloud_pc_upper.points[i].y = cartesianPoint.y();
								cloud_pc_upper.points[i].z = cartesianPoint.z();
								cloud_pc_upper.channels[0].values[i] =  SPtype;
                if(IBEO_TYPE == IBEO_8L_LUX)
								  cloud_pc_upper.channels[1].values[i] =  SPlayer+4;
                else
                  cloud_pc_upper.channels[1].values[i] =  SPlayer;
								cloud_pc_upper.channels[2].values[i] =  AccurateTo2Dec(convertTicksToHRad(SPHangle, angleTicks)*180.0/M_PI);
								cloud_pc_upper.channels[3].values[i] =  SPdistance/100.0;
							}
						}

            if( mirrorSide==0 )
						{
							if(i<numScanpoints)
							{
								cloud_pc_lower.points[i].x = cartesianPoint.x();
								cloud_pc_lower.points[i].y = cartesianPoint.y();
								cloud_pc_lower.points[i].z = cartesianPoint.z();
								cloud_pc_lower.channels[0].values[i] =  SPtype;
								cloud_pc_lower.channels[1].values[i] =  SPlayer;
                cloud_pc_lower.channels[2].values[i] =  AccurateTo2Dec(convertTicksToHRad(SPHangle, angleTicks)*180.0/M_PI);
								cloud_pc_lower.channels[3].values[i] =  SPdistance/100.0;
							}
            }
					}
				} // for

				// return observation to framework
				//appendObservation( newObs );

				state = SearchForAF;

				if( get_lower_flag == true || get_upper_flag ==true)
				data_flag = true;
				else
				data_flag = false;

				return;

				break; // SavePointCloud
      }

      case SaveObjects:
      {
      	bool dropObjPacket = false;

        cout<<"State: Saving objects."<<endl;
        m_client.readAsync(ObjectHeader, 10, 10, 10);
        numScanObjects = ObjectHeader[9] * 0x100 + ObjectHeader[8];

        if(numScanObjects>150||numScanObjects<1)
        {
        	dropObjPacket = true;
        }

        std::cout<<counter++<<"\t  numScanObjects= "<<numScanObjects<<endl;

        //watchdog_counter=0;

        Objects.header.stamp = ros::Time::now();
        Objects.header.frame_id = "base_laser";
        Objects.Object.resize(numScanObjects);

        ScanObjectData NewObjects[numScanObjects];


        for(unsigned int i=0;i<numScanObjects;i++)
        {
        	if(dropObjPacket)
        		break;

          m_client.readAsync(ObjectContent, 58, 10, 10);

          NewObjects[i].ID  = ObjectContent[1]*0x100 + ObjectContent[0];
          NewObjects[i].Age = ObjectContent[3]*0x100 + ObjectContent[2];
          NewObjects[i].PredictionAge = ObjectContent[5]*0x100 + ObjectContent[4];
          NewObjects[i].ReferencePoint.x = ConvertToInt16(ObjectContent[9]*0x0100 + ObjectContent[8]);
          NewObjects[i].ReferencePoint.y = ConvertToInt16(ObjectContent[11]*0x0100 + ObjectContent[10]);
          NewObjects[i].BoundingBoxCenter.x = ConvertToInt16(ObjectContent[21]*0x100 + ObjectContent[20]);
          NewObjects[i].BoundingBoxCenter.y = ConvertToInt16(ObjectContent[23]*0x100 + ObjectContent[22]);
          NewObjects[i].BoundingBoxWidth = ObjectContent[25]*0x100 + ObjectContent[24];
          NewObjects[i].BoundingBoxLength = ObjectContent[27]*0x100 + ObjectContent[26];
          NewObjects[i].ObjectBoxCenter.x = ConvertToInt16(ObjectContent[29]*0x100 + ObjectContent[28]);
          NewObjects[i].ObjectBoxCenter.y = ConvertToInt16(ObjectContent[31]*0x100 + ObjectContent[30]);
          NewObjects[i].ObjectBoxSize.x = ObjectContent[33]*0x100 + ObjectContent[32];
          NewObjects[i].ObjectBoxSize.y = ObjectContent[35]*0x100 + ObjectContent[34];
          NewObjects[i].ObjectBoxOrientation = ConvertToInt16(ObjectContent[37]*0x100 + ObjectContent[36]);
          NewObjects[i].AbsoluteVelocity.x = ConvertToInt16(ObjectContent[39]*0x100 + ObjectContent[38]);
          NewObjects[i].AbsoluteVelocity.y = ConvertToInt16(ObjectContent[41]*0x100 + ObjectContent[40]);
          NewObjects[i].AbsoluteVelocitySigma.x = ObjectContent[43]*0x100 + ObjectContent[42];
          NewObjects[i].AbsoluteVelocitySigma.y = ObjectContent[45]*0x100 + ObjectContent[44];
          NewObjects[i].RelativeVelocity.x = ConvertToInt16(ObjectContent[47]*0x100 + ObjectContent[46]);
          NewObjects[i].RelativeVelocity.y = ConvertToInt16(ObjectContent[49]*0x100 + ObjectContent[48]);
          NewObjects[i].Classification = ObjectContent[51]*0x100 + ObjectContent[50];
          NewObjects[i].ClassificationAge = ObjectContent[53]*0x100 + ObjectContent[52];
          NewObjects[i].ClassificationCertainty = ObjectContent[55]*0x100 + ObjectContent[54];
          NewObjects[i].NumofContourPoints = ObjectContent[57]*0x100 + ObjectContent[56];

   //       std::cout<<"Object ID:"<<NewObjects[i].ID<<endl;
   //       std::cout<<"Object Age:"<<NewObjects[i].Age<<endl;
   //       std::cout<<"Object PredictionAge:"<<NewObjects[i].PredictionAge<<endl;
   //       std::cout<<"Reference Point:"<<NewObjects[i].ReferencePoint.x<<", "<<NewObjects[i].ReferencePoint.y<<endl;
   //       std::cout<<"ObjectBoxSize:"<<NewObjects[i].ObjectBoxSize.x<<", "<<NewObjects[i].ObjectBoxSize.y<<endl;
   //       std::cout<<"Classification:"<<NewObjects[i].Classification<<endl;
   //       std::cout<<"NumofContourPoints:"<<NewObjects[i].NumofContourPoints<<endl;

          Objects.Object[i].ID = NewObjects[i].ID;
          Objects.Object[i].Age = NewObjects[i].Age;
          Objects.Object[i].PredictionAge = NewObjects[i].PredictionAge;
          Objects.Object[i].ReferencePoint.x = NewObjects[i].ReferencePoint.x;
          Objects.Object[i].ReferencePoint.y = NewObjects[i].ReferencePoint.y;
          Objects.Object[i].BoundingBoxCenter.x = NewObjects[i].BoundingBoxCenter.x;
          Objects.Object[i].BoundingBoxCenter.y = NewObjects[i].BoundingBoxCenter.y;
          Objects.Object[i].BoundingBoxSize.x = NewObjects[i].BoundingBoxLength;
          Objects.Object[i].BoundingBoxSize.y = NewObjects[i].BoundingBoxWidth;
          Objects.Object[i].ObjectBoxCenter.x = NewObjects[i].ObjectBoxCenter.x;
          Objects.Object[i].ObjectBoxCenter.y = NewObjects[i].ObjectBoxCenter.y;
          Objects.Object[i].ObjectBoxSize.x = NewObjects[i].ObjectBoxSize.x;
          Objects.Object[i].ObjectBoxSize.y = NewObjects[i].ObjectBoxSize.y;
          Objects.Object[i].ObjectBoxOrientation = NewObjects[i].ObjectBoxOrientation;
          Objects.Object[i].AbsoluteVelocity.x = NewObjects[i].AbsoluteVelocity.x;
          Objects.Object[i].AbsoluteVelocity.y = NewObjects[i].AbsoluteVelocity.y;
          Objects.Object[i].AbsoluteVelocitySigma.x = NewObjects[i].AbsoluteVelocitySigma.x;
          Objects.Object[i].AbsoluteVelocitySigma.y = NewObjects[i].AbsoluteVelocitySigma.y;
          Objects.Object[i].RelativeVelocity.x = NewObjects[i].RelativeVelocity.x;
          Objects.Object[i].RelativeVelocity.y = NewObjects[i].RelativeVelocity.y;
          Objects.Object[i].Classification = NewObjects[i].Classification;
          Objects.Object[i].ClassificationAge = NewObjects[i].ClassificationAge;
          Objects.Object[i].ClassificationCertainty = NewObjects[i].ClassificationCertainty;
          Objects.Object[i].NumofContourPoints=NewObjects[i].NumofContourPoints;

          if(Objects.Object[i].NumofContourPoints>1500)
          {
        	dropObjPacket = true;
        	break;
          }

          Objects.Object[i].ContourPoints.resize(NewObjects[i].NumofContourPoints);
          NewObjects[i].ContourPoints = new Point2D[NewObjects[i].NumofContourPoints];
          for(int j=0;j<NewObjects[i].NumofContourPoints;j++)
          {
            m_client.readAsync(ContourPointInfo, 4, 10, 10);
            NewObjects[i].ContourPoints[j].x = ConvertToInt16(ContourPointInfo[1]*0x100 + ContourPointInfo[0]);
            NewObjects[i].ContourPoints[j].y = ConvertToInt16(ContourPointInfo[3]*0x100 + ContourPointInfo[2]);
            //std::cout<<j<<"\t ContourPoints:"<<NewObjects[i].ContourPoints[j].x<<", "<<NewObjects[i].ContourPoints[j].y<<endl;
            Objects.Object[i].ContourPoints[j].x = NewObjects[i].ContourPoints[j].x;
            Objects.Object[i].ContourPoints[j].y = NewObjects[i].ContourPoints[j].y;
          }
          //std::cout<<"\n"<<endl;
          delete [] NewObjects[i].ContourPoints;
        }

        if(!dropObjPacket)
        {
        	objects_flag = true;
        }
        else
        {
        	objects_flag = false;
        }

        state = SearchForAF;
        return;
        break; //SaveObjects
      }

		}	// Switch
	}	// While
}	// dataCollection

mrpt::poses::CPoint3D CIbeoLuxETH::convertToCartesian(float vrad, float hrad, float distance)
{
	float x, y, z;
	float rho, phi, theta;

	rho = distance/100.0;
	x = rho * cos(hrad) * cos(vrad);
	y = rho * sin(hrad) * cos(vrad);
	z = rho * sin(vrad);

	mrpt::poses::CPoint3D point(x, y, z);
	return point;
}

float CIbeoLuxETH::convertTicksToHRad(int hticks, int hticksPerRotation)
{
	return M_PI*2 * hticks / hticksPerRotation;
}

float CIbeoLuxETH::convertLayerToRad(int scanlayer)
{
	float vangle;

  if(IBEO_TYPE == IBEO_8L_LUX)
  {
	   if( mirrorSide ==0 )  //lower
	  {
		    switch(scanlayer)
		    {
			       case 0:
				         vangle = -0.0488692191;	// -2.8
				         break;
			       case 1:
				         vangle = -0.034906585;	// -2.0
				        break;
			       case 2:
				         vangle =  -0.02094395103;		// -1.2
				         break;
			       case 3:
				         vangle =  -0.006981317009;	// -0.4
				         break;
			       default:
				         vangle = 0;
				         std::cerr << "Layer: " << scanlayer << "! Returning " << vangle << " as angle.\n";
				         break;
		    }
	  }

	  if( mirrorSide == 1) //upper
	  {
		    switch(scanlayer)
		    {
			       case 0:
				         vangle = 0.006981317009;	// 0.4
				         break;
             case 1:
				         vangle = 0.02094395103;	//1.2
				         break;
			       case 2:
				         vangle =  0.034906585;	//2.0
				         break;
			       case 3:
				         vangle =  0.0488692191;	//2.8
				         break;
			       default:
				         vangle = 0;
				         std::cerr << "Layer: " << scanlayer << "! Returning " << vangle << " as angle.\n";
				         break;
		    }
	  }
  }
  else if(IBEO_TYPE == IBEO_4L_LUX)
  {
    switch(scanlayer)
    {
         case 0:
             vangle = -0.02094395103;		// -1.2
             break;
         case 1:
             vangle = -0.006981317009;	// -0.4
             break;
         case 2:
             vangle =  0.006981317009;	// 0.4
             break;
         case 3:
             vangle =  0.02094395103;		// 1.2
             break;
         default:
             vangle = 0;
             std::cerr << "Layer: " << scanlayer << "! Returning " << vangle << " as angle.\n";
             break;
    }
  }
  else
  {
    std::cerr << "IBEO type ERROR!" << endl;

  }


	return vangle;
}

void CIbeoLuxETH::loadConfig_sensorSpecific( const mrpt::utils::CConfigFileBase &configSource,
                             const std::string	  &iniSection )
{
	float pose_x, pose_y, pose_z, pose_yaw, pose_pitch, pose_roll;
	bool faillNotFound = false;
	pose_x = configSource.read_float(iniSection,"pose_x",0,faillNotFound);
	if(faillNotFound) return;
	pose_y = configSource.read_float(iniSection,"pose_y",0,faillNotFound);
	if(faillNotFound) return;
	pose_z = configSource.read_float(iniSection,"pose_z",0,faillNotFound);
	if(faillNotFound) return;
	pose_yaw = configSource.read_float(iniSection,"pose_yaw",0,faillNotFound);
	if(faillNotFound) return;
	pose_pitch = configSource.read_float(iniSection,"pose_pitch",0,faillNotFound);
	if(faillNotFound) return;
	pose_roll = configSource.read_float(iniSection,"pose_roll",0,faillNotFound);
	if(faillNotFound) return;

	m_sensorPose = mrpt::poses::CPose3D( pose_x, pose_y, pose_z,
		DEG2RAD( pose_yaw ),DEG2RAD( pose_pitch ), DEG2RAD( pose_roll ));
}

void CIbeoLuxETH::makeCommandHeader(unsigned char* buffer)
{
	// Header - all big endian
	buffer[0] = 0xAF; // magic word
	buffer[1] = 0xFE;
	buffer[2] = 0xC0;
	buffer[3] = 0xC2;
	buffer[4] = 0x00; // Size of previous message, here just left to null
	buffer[5] = 0x00;
	buffer[6] = 0x00;
	buffer[7] = 0x00;
	buffer[8] = 0x00; // Size of data block
	buffer[9] = 0x00;
	buffer[10] = 0x00;
	buffer[11] = 0x00;	// to be set by the command function
	buffer[12] = 0x00;	// Reserved + source Id
	buffer[13] = 0x78;	// source ID of 0x78 as observed
	buffer[14] = 0x20;	// Data Type - 2010 = command
	buffer[15] = 0x10;
	buffer[16] = 0x00;	// 4* ntpp time (s) + 4* fractions of a second
	buffer[17] = 0x00;
	buffer[18] = 0x00;
	buffer[19] = 0x00;
	buffer[20] = 0x00;
	buffer[21] = 0x00;
	buffer[22] = 0x00;
	buffer[23] = 0x00;
}

void CIbeoLuxETH::makeStartCommand(unsigned char* buffer)
{
	// Header - all big endian
	buffer[11] = 0x04;	// Size of data block
	// Data Block - all little endian
	buffer[24] = 0x20;	// Start Measure 0x0020
	buffer[25] = 0x00;
	buffer[26] = 0x00;	// Reserved, but obligatory
	buffer[27] = 0x00;
}

void CIbeoLuxETH::makeStopCommand(unsigned char* buffer)
{
	// Header - all big endian
	buffer[11] = 0x04;	// Size of data block
	// Data Block - all little endian
	buffer[24] = 0x21;	// Stop Measure 0x0021
	buffer[25] = 0x00;
	buffer[26] = 0x00;	// Reserved, but obligatory
	buffer[27] = 0x00;
}

void CIbeoLuxETH::makeFrequencyCommand(unsigned char* buffer, unsigned int freq)
{
	// Header - all big endian
	buffer[11] = 0x08;	// Size of data block
	// Data Block - all little endian
	buffer[24] = 0x10;	// Stop Measure 0x0021
	buffer[25] = 0x00;
	buffer[26] = 0x00;	// Reserved, but obligatory
	buffer[27] = 0x00;
	buffer[28] = 0x02;
	buffer[29] = 0x11;
	if(freq == 12)
	{
		buffer[30] = 0x80;	// end value
		buffer[31] = 0x0C;
	}
	else
	{
		if(freq == 24)
		{
			buffer[30] = 0x00;	// end value
			buffer[31] = 0x19;
		}
		else
		{
			buffer[30] = 0x80;	// end value
			buffer[31] = 0x0C;			
		}
	}
}

void CIbeoLuxETH::makeTypeCommand(unsigned char* buffer)
{
	// Header - all big endian
	buffer[11] = 0x08;	// Size of data block
	// Data Block - big endian (for filter command!)
	buffer[24] = 0x00;	// Command Type - 0005 = set datatype filter
	buffer[25] = 0x05;
	buffer[26] = 0x00;	// Data type filter length
	buffer[27] = 0x02;
	buffer[28] = 0x22;	// start value
	buffer[29] = 0x00;
	buffer[30] = 0x22;	// end value
	buffer[31] = 0x10;
}

/** This method can or cannot be implemented in the derived class, depending on the need for it.
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
void CIbeoLuxETH::initialize()
{
  // Start TCP-connection to laserscanner
	m_client.connect(m_ip, m_port);

	unsigned char msg[32];

	// makeCommandHeader(msg);
	// makeFrequencyCommand(msg,12);
	// m_client.writeAsync(&msg[0], 32);


  	// Send filter command
	makeCommandHeader(msg);
	makeTypeCommand(msg);
	m_client.writeAsync(&msg[0], 32);

	// Send start command
	makeCommandHeader(msg);
	makeStartCommand(msg);
	m_client.writeAsync(&msg[0], 28);

	m_run = true;

	fprintf(stdout,"m_run=%d\r\n",m_run);
  fprintf(stdout,"Initialization finish.\r\n");
  
  //watchdog_counter=0;

}

void CIbeoLuxETH::stop()
{
  unsigned char msg[32];
  // Send stop command
  //makeCommandHeader(msg);
  //makeStopCommand(msg);
  //m_client.writeAsync(&msg[0], 28);
  //m_client.close();

  m_run = false;
  fprintf(stdout,"Destructed!\t m_run=%d\r\n",m_run);

}

void CIbeoLuxETH::doProcess()
{
	dataCollection();
	// nothing is done here
	// data is collected in the dataCollection thread
}
