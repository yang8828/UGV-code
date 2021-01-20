using System;
using System.Collections.Generic;
using System.Text;

namespace PathEx_Analysis
{
    public class VariableDefination
    {   //节点最大点数
        public const int JOINTIDNUMBER=10000;
        //交叉口最多路口数
        public const int INTERSECTIONNUMBER = 6000;

        public struct Init
        {
            public string String;
            public bool FlagIsBusy;    //线程是否在运行， 否 flase; 是 true
            public bool FlagInitSuccess;  //初始化是否完成， 否 flase; 是 true
        };

        //存放地图中所有道路节点的序号和坐标
        public struct JointIDCoor
        {
            public double x, y;//x-东向；y-北向
        };

        //交叉口序号和ID
        public struct IntersectionIndexID
        {
            public long Index;                          //交叉口中心点在全部路点中的序号
            public long IndexStartPointOfIntersection;  //拐弯处起点在全部路点中的序号 
            public long IndexEndPointOfIntersection;    //拐弯处终点在全部路点中的序号 
            public long IDStartPointOfIntersection;     //交叉口起点在全部路点中的序号  
            public long IDEndPointOfIntersection;       //交叉口终点在全部路点中的序号 
            
        };
        //交叉口数据
        public struct IntersectionFlag
        {
            public int IndexCrossingIntersection;   //正在通过的交叉口的序号
            public int CharacterOfEnterCurve;  //入弯的性质，0表示无效，1表示路口，2 - 有动态目标通过(当并线标志为2置为2）
            public int CharacterOfOutCurve;    //出弯的性质，0表示无效，1表示路口，2 - 有动态目标通过(当并线标志为2置为2）
            public int IndexStartPointInCurrentNavigationDataFrame;          //拐弯处起点在当前301个路点中的序号
            public int IndexEndPointInCurrentNavigationDataFrame;            //拐弯处终点在当前301个路点中的序号
            public int WholeNumberIntersection;                   //交叉口的个数            
        };      


        //从UDP中读入的数据
        public struct DataFromUDP
        {
            public double back_car_x;
            public double back_car_y;//平面坐标
            public double back_azimuth;//航向
            public int Back_gear;
            public double back_velocity;
            public long ID;
        };

        
        //地图东向和北向平面坐标校正值
        public struct MapCoordinateAllignment
        {
            public double x;
            public double y;
        };
       
        //鼠标经过点坐标值
        public struct MousePointCoordinate
        {
            public double x;
            public double y;
        };

        

        //生成路径规划点数据
        public struct PathPlanInfo
        {
            public int KeyPointNumber;//关键点个数
        };
        //向UDP发送地图匹配坐标与组合定位坐标（当前点）之间的坐标误差
        public struct DataToUDPErrorOfMapMatch
        {
            public int x, y;
        }
        //向UDP发送状态数据定义
        public struct DataToUDPExcludeRoadPoints
        {   //车道线数量LaneNumber,建立行驶的车道线的序号IndexLaneRunSuggested
            public int LaneNumber,IndexLaneRunSuggested;
            //当前航向与车道线方向的夹角（向右为正）
            public double AngleHeadingToLane;
            //车辆限速
            public int SpeedLimited;
            //入弯点和出弯点
            public int InOutTurn;
            public double PresentTime;
            public int FlagOfRoadNet;
        };
        //车的行驶状态
        public struct RunningStatus
        {   public int ChangeLane;                     //X7
            public int MatchStatus;                    //X6
            public int RunControlWithTrafficLight;      //X5
            public double DistanceToNextTrafficLightOrEndPoint;   //X4-X2
            public int FlagWhetherTrafficSign;         //X1
            public int FlagWhetherAllignToGIS;         //X0
        }
    }
}
