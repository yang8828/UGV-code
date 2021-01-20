using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Ros_CSharp;
using XmlRpc_Wrapper;
using Messages;

namespace PathEx_Analysis
{
    public class CROS
    {
        Subscriber<Messages.common.InsInfo> ins_sub;
        Subscriber<Messages.common.TcpGeneral> vec_sub;
        //Publisher<Messages.common.GisInfo> gis_pub;
        NodeHandle nh;
        public CROS()
        {
            ROS.ROS_HOSTNAME = "inin-lab2";
            ROS.ROS_MASTER_URI = "http://192.168.0.103:11311";
            //ROS.ROS_MASTER_URI = "http://127.0.0.1:11311";
            ROS.Init(new string[0], "gis_node_2");
            nh = new NodeHandle();
            ins_sub = nh.subscribe<Messages.common.InsInfo>("/InsInfo", 10, subCallback);
            vec_sub = nh.subscribe<Messages.common.TcpGeneral>("/TREK", 10, veh_subCallback);
           // gis_pub = nh.advertise<Messages.common.GisInfo>("/gis_info", 1);
            //gis_pub = nh.advertise<Messages.std_msgs.String>("/GIS_TEST", 1);
            msg_veh.data = new int[256];
        }
        public Messages.common.TcpGeneral msg_veh = new Messages.common.TcpGeneral();
        public Messages.common.InsInfo msg_ins = new Messages.common.InsInfo();
        public bool msg_veh_ready = false;
        public bool msg_ins_ready = false;
        public void Close()
        {
            ROS.shutdown();
            ROS.waitForShutdown();
        }


        public void subCallback(Messages.common.InsInfo msg)
        {
            msg_ins = msg;
            msg_ins_ready = true;
        }

        public void veh_subCallback(Messages.common.TcpGeneral msg)
        {
            msg_veh = msg;
            msg_veh_ready = true;
        }

    //    public void PubGIS(MainForm f1, VariableDefination.DataToUDPExcludeRoadPoints DataToUDPExcludeRoadPoints, VariableDefination.RunningStatus RunningStatus, double speed, int Ready_To_Align, VariableDefination.IntersectionFlag IntersectionFlag, transfer tf, bool rdpoints_flag)
    //    {
    //        Messages.common.GisInfo GIS_info = new Messages.common.GisInfo();
    //        GIS_info.SysTime = Convert.ToInt64(DataToUDPExcludeRoadPoints.PresentTime);
    //        GIS_info.LaneNum = Convert.ToByte(DataToUDPExcludeRoadPoints.LaneNumber);
    //        GIS_info.RecommendLane = Convert.ToByte(DataToUDPExcludeRoadPoints.IndexLaneRunSuggested);
    //        GIS_info.LaneAngle = DataToUDPExcludeRoadPoints.AngleHeadingToLane;
    //        GIS_info.LaneChangeFlag = Convert.ToByte(RunningStatus.ChangeLane);
    //        GIS_info.MissionState = Convert.ToByte(RunningStatus.MatchStatus);
    //        GIS_info.CrossRoadType = Convert.ToByte(RunningStatus.RunControlWithTrafficLight);
    //        GIS_info.DistanceToNextCross = RunningStatus.DistanceToNextTrafficLightOrEndPoint;
    //        GIS_info.TrafficSignFlag = Convert.ToByte(RunningStatus.FlagWhetherTrafficSign);
    //        GIS_info.AlignToLaneCenterFlag = Convert.ToByte(Ready_To_Align);
    //        speed = f1.force_speed;
    //        GIS_info.SpeedLimit = speed;
    //        GIS_info.CurveEnterIDFlag = Convert.ToByte(IntersectionFlag.CharacterOfEnterCurve);
    //        GIS_info.CurveEnterID = IntersectionFlag.IndexStartPointInCurrentNavigationDataFrame;
    //        GIS_info.CurveExitIDFlag = Convert.ToByte(IntersectionFlag.CharacterOfOutCurve);
    //        GIS_info.CurveExitID = IntersectionFlag.IndexEndPointInCurrentNavigationDataFrame;
    //        GIS_info.RoadPoints = new Messages.common.Point2D[301];
    //        //发送绝对坐标
    //        if (rdpoints_flag)
    //        {
    //                if ((tf.end_index - tf.start_index < 301) && (tf.end_index - tf.start_index >= 0))
    //                {
    //                    for (long i = tf.start_index; i < tf.end_index; i++)
    //                    {
    //                        if (i < 1000)
    //                        {
    //                            GIS_info.RoadPoints[i - tf.start_index] = new Messages.common.Point2D();
    //                            GIS_info.RoadPoints[i - tf.start_index].x = tf.resultx[i] - f1.GetMapCoordinateAllignmentX();
    //                            GIS_info.RoadPoints[i - tf.start_index].y = tf.resulty[i] - f1.GetMapCoordinateAllignmentY();
    //                        }
    //                    }
    //                    /********** 当点数小于301个时，用最后一个点值填充后续所有点 ***********/
    //                    for (long i = tf.end_index; i < 301 + tf.start_index; i++)
    //                    {
    //                        if (i < 1000)
    //                        {
    //                            GIS_info.RoadPoints[i - tf.start_index] = new Messages.common.Point2D();
    //                            GIS_info.RoadPoints[i - tf.start_index].x = tf.resultx[tf.end_index] - f1.GetMapCoordinateAllignmentX();
    //                            GIS_info.RoadPoints[i - tf.start_index].y = tf.resulty[tf.end_index] - f1.GetMapCoordinateAllignmentY();
    //                        }
    //                    }
                    
    //                /********** 当点数大于301个时，则从开始点往后发301个点 ***********/
    //                else
    //                {
    //                    for (long i = tf.start_index; i < tf.start_index + 301; i++)
    //                    {
    //                        if (i < 1000)
    //                        {
    //                            GIS_info.RoadPoints[i - tf.start_index] = new Messages.common.Point2D();
    //                            GIS_info.RoadPoints[i - tf.start_index].x = tf.resultx[i] - f1.GetMapCoordinateAllignmentX();
    //                            GIS_info.RoadPoints[i - tf.start_index].y = tf.resulty[i] - f1.GetMapCoordinateAllignmentY();
    //                        }
    //                    }
    //                }
    //            }
    //            else
    //            {
    //                if (tf.end_index < tf.start_index)
    //                    return;
    //                if ((tf.end_index - tf.start_index < 301) && (tf.end_index - tf.start_index >= 0))
    //                {
    //                    for (long i = tf.end_index; i > tf.start_index; i--)
    //                    {
    //                        if (i < 1000)
    //                        {
    //                            GIS_info.RoadPoints[tf.end_index - i] = new Messages.common.Point2D();
    //                            GIS_info.RoadPoints[tf.end_index - i].x = tf.resultx[i] - f1.GetMapCoordinateAllignmentX();
    //                            GIS_info.RoadPoints[tf.end_index - i].y = tf.resulty[i] - f1.GetMapCoordinateAllignmentY();
    //                        }
    //                    }
    //                    /********** 当点数小于301个时，用最后一个点值填充后续所有点 ***********/
    //                    for (long i = tf.start_index; i > tf.end_index - 301; i--)
    //                    {
    //                        if (i < 1000)
    //                        {
    //                            GIS_info.RoadPoints[tf.end_index - i] = new Messages.common.Point2D();
    //                            GIS_info.RoadPoints[tf.end_index - i].x = tf.resultx[tf.start_index] - f1.GetMapCoordinateAllignmentX();
    //                            GIS_info.RoadPoints[tf.end_index - i].y = tf.resulty[tf.start_index] - f1.GetMapCoordinateAllignmentY();
    //                        }
    //                    }
    //                }
    //                /********** 当点数大于301个时，则从开始点往后发301个点 ***********/
    //                else
    //                {
    //                    for (long i = tf.end_index; i > tf.end_index - 301; i--)
    //                    {
    //                        if (i < 1000)
    //                        {
    //                            GIS_info.RoadPoints[tf.end_index - i] = new Messages.common.Point2D();
    //                            GIS_info.RoadPoints[tf.end_index - i].x = tf.resultx[i] - f1.GetMapCoordinateAllignmentX();
    //                            GIS_info.RoadPoints[tf.end_index - i].y = tf.resulty[i] - f1.GetMapCoordinateAllignmentY();
    //                        }
    //                    }
    //                }
    //            }
    //        }
    //        //发送相对坐标
    //        else
    //        {

    //            for (int i = 0; i < 301; i++)
    //            {
    //                GIS_info.RoadPoints[i] = new Messages.common.Point2D();
    //                GIS_info.RoadPoints[i].x = tf.xNavigationPointCoordianteToBeSent[i];
    //                GIS_info.RoadPoints[i].y = tf.yNavigationPointCoordianteToBeSent[i];
    //            }
    //        }
    //        gis_pub.publish(GIS_info);
    //        //Messages.std_msgs.String temp = new Messages.std_msgs.String("gis");
    //        //gis_pub.publish(temp);
    //    }
    //
    }
}
