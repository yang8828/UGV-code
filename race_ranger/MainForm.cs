/*该版本实现所有轨迹的绘制，包括路口和路口之间的轨迹
 * 读入任务点之后可以改变任务点的坐标和两个状态
 *完成状态的赋值，包括转弯方向，限速，比赛状态，交通灯，距离，车道线数量,交通标志
 *交通标志是通过任务点属性来赋值的
 *完成timer的更新
 *将tcp通讯改为udp
 *获得拐弯轨迹的入弯点和出弯点在所发路点中的序号
 *将所发轨迹的路点中超过3000坐标系边界的点划归为一个点
 *实现速度策略
 *实现自动初始化
*/

/*********************************************************
 此文件用以建立主窗口，完成地图导入、文件读入、初始化、退出等
 *********************************************************/
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.Resources;
using System.Reflection;
using System.Runtime.InteropServices;
using SuperMapLib;
using SuperAnalystLib;
using System.Diagnostics;
using System.IO;
using Ros_CSharp;
using System.Threading;
namespace PathEx_Analysis
{
    public partial class MainForm : Form
    {   
        //定义宏变量
        //命令码定义
        public const int COMMANDCODE_ERRORXY = 0xfe;
        public const int COMMANDCODE_FLAGOFROADNET = 0x15;
        public const int Log_Count = 60;//记录的路点数，0.5米记录一个1022
        public const int BackCar_Count = 40;//倒车的路点数1022
        public const int Distance_StopCar_Send = 75;//发送停车区位置的前后距离1025
        public const int Sign_Side_StopCar = 2;//侧方停车用2来发送提示1025
        public const int Distance_Send_Turn_And_SuggestLane = 100;//发送建议车道和转向的距离1025

        public const long NAVIGATIONPOINTMAX = 100000; //10万
        public const double SPEEDMAX = 60.0;    //最高时速 40km/h
        public const double SPEEDMIN = 10.0;    //最低时速 10km/h - 路口、交通标识
        public const double SPEEDMAXBEFORESTOP = 0.0; //停车前最高时速 5km/h - 终点
        public const int ForbidPass_ErrorDetection_Distance = 100;//判断禁行路口的误检测距离

        public char YES = (char)0x01;
        public const char NO = (char)0x0;
        public const char STATUES_STARTUP = (char)0x00;      //启动状态
        public const char STATUES_STANDBY = (char)0x01;      //等待状态，等待发车命令
        public const char STATUES_MATCHING = (char)0x02;     //比赛中
        public const char STATUES_OVER = (char)0x03;         //比赛结束
        public const char GOAHEAD =(char)0x00;               //0 直行
        public const char TURNLEFT =(char)0x01;              //1 左转
        public const char TURNRIGHT =(char)0x02;             //2 右转
        public const char GOAHEAD_TRAFFICLIGHT =(char)0x03;       //3 红绿灯直行
        public const char TURNLEFT_TRAFFICLIGHT =(char)0x04;     //4 红绿灯左转
        public const char TURNRIGHT_TRAFFICLIGHT =(char)0x05;     //5 红绿灯右转
        public const char UTURN = (char)0x06;                    //6 掉头U-TURN

        public const char DYNAMICOBJECTPASS = (char)0x02;        // 有动态目标通过（if_crossline判断)
        public const char TRAFFICSIGN_MANULLY = (char)0x02;      // 人工加入交通标识(if_corner判断）
        
        /***************************** 比赛状态定义 **********************/
        public const char MATCHSTATUS_BEFOREMATCH = (char)0x01;         //比赛前,检测交通灯 
        public const char MATCHSTATUS_MATCHING = (char)0x02;            //比赛中，正常行驶（默认值）
        public const char MATCHSTATUS_OVERMATCH = (char)0x03;           //比赛即将结束

        /*********************** 任务文件中点的属性定义 ****************/
        public const int TASKPROPERTY_MAXNUMBER = 7;                    //属性定义最大数量
        public const char TASKPROPERTY_STARTPOINT = (char)0x0;          //起点
        public const char TASKPROPERTY_ENDPOINT = (char)0x7;            //终点
        public const char TASKPROPERTY_ENTERCURVERPOINT = (char)0x1;    //入弯点
        public const char TASKPROPERTY_OUTCURVERPOINT = (char)0x2;      //出弯点
        public const char TASKPROPERTY_NORMALPOINT = (char)0x3;         //普通路点
        public const char TASKPROPERTY_ENTERPARKPOINT = (char)0x4;      //泊车区入点
        public const char TASKPROPERTY_OUTPARKPOINT = (char)0x5;        //泊车区出点
        public const char TASKPROPERTY_PARKPOINT = (char)0x6;           //泊车点

        public const char TASKPROPERTY_UNCERTAINTRAFFICSIGN = (char)0x0; //不确定是否有交通标志
        public const char TASKPROPERTY_GOAHEAD = (char)0x01;             //直行
        public const char TASKPROPERTY_TURNLEFT = (char)0x02;            //左转
        public const char TASKPROPERTY_TURNRIGHT = (char)0x03;           //右转
        public const char TASKPROPERTY_UTURN = (char)0x04;               //U-turn
        public const char TASKPROPERTY_EXISTTRAFFICSIGN = (char)0x05;    //有交通标志


        
        //故障代码-500ms定时器定时更新，代码定义见 DisplayTroubleCode
        public uint TroubleCode=0x00;

        //SuperMap变量定义
        //路径规划欲经过路段端点（起点和终点）坐标等
        soPoints PathPlanMapRoadEndPoints = new soPointsClass();
        public soGeoPoint PresentPoint = new soGeoPoint();//当前点      
        soGeoPoint result = new soGeoPoint();//用于显示发送的路点        
        soRecordset Wcqy_Recordset = null;//候选路段记录集
         
        // 用户变量类的定义
        VariableDefination PublicVariable = new VariableDefination();
        //定义存放地图中所有道路节点的序号和坐标数组
        VariableDefination.JointIDCoor[] JointIDCoor = new VariableDefination.JointIDCoor[VariableDefination.JOINTIDNUMBER];
        //定义从UDP读出的数据
        public VariableDefination.DataFromUDP DataFromUDP = new VariableDefination.DataFromUDP();
        //备份从UDP读出的数据
        public VariableDefination.DataFromUDP DataFromUDPBackup = new VariableDefination.DataFromUDP();
        //定义地图东向和北向平面坐标校正值
        public VariableDefination.MapCoordinateAllignment MapCoordinateAllignment = new VariableDefination.MapCoordinateAllignment();
        //定义向UDP发送地图匹配坐标与组合定位坐标（当前点）之间的坐标误差
        public VariableDefination.DataToUDPErrorOfMapMatch DataToUDPErrorOfMapMatch = new VariableDefination.DataToUDPErrorOfMapMatch();
        //向UDP发送状态数据定义
        public VariableDefination.DataToUDPExcludeRoadPoints DataToUDPExcludeRoadPoints = new VariableDefination.DataToUDPExcludeRoadPoints();
        //车的行驶状态
        public VariableDefination.RunningStatus RunningStatus = new VariableDefination.RunningStatus();      
        //定义鼠标经过点坐标值
        VariableDefination.MousePointCoordinate MousePointCoordinate = new VariableDefination.MousePointCoordinate();
        //生成路径规划点数据
        VariableDefination.PathPlanInfo PathPlanInfo = new VariableDefination.PathPlanInfo();
        //交叉口序号和ID号
        VariableDefination.IntersectionIndexID [] IntersectionIndexID = new VariableDefination.IntersectionIndexID[VariableDefination.INTERSECTIONNUMBER];
         //交叉口标志
        VariableDefination.IntersectionFlag IntersectionFlag = new VariableDefination.IntersectionFlag();
        //初始化标志
        VariableDefination.Init Init = new VariableDefination.Init();

        //定义生成路点的类的名称NavigationPoint
        public RoadModel NavigationPoint = new RoadModel();    
   
        public RoadpointReader rpr = new RoadpointReader();//读入关键点文件类
        transfer tf = new transfer();        

        transfer._soPoint pt = new transfer._soPoint();//匹配得到的路段起点和终点
        transfer._soPoint pf = new transfer._soPoint();
        transfer._soPoint[] RealP = new transfer._soPoint[65535]; //规划得到的需要经过的点
        transfer._soPoint match_point = new transfer._soPoint();
        rd_point[] rd_points = new rd_point[NAVIGATIONPOINTMAX];//所有的路点
        bool U_turn_flag = false;//是否Uturn标志1029
        
        string wks_name =null;//保存打开的工作空间的名称     
 
        System.Threading.Thread INITThread;
        //每500ms向监管程序发送一次系统时间，未发时则被重启

        soPoint speed_point = new soPoint();
        soPoint traffic_point = new soPoint();
        int[] status;           //保存每个节点转向情况：0是直行，1是左转，2是右转 
        int[] cross_status = new int[100];      //保存每个路口的转向情况：0是直行，1是左转，2是右转 
        int[] lane_num;         //保存每条经过道路的车道线数量
        int[] lane_info;        //保存每条车道的转弯方向
        int[] traffic;          //保存每条经过道路是否有交通灯，1为有，0为没有
        int[] speedlimit;
        int[] if_corner;
        int[] if_croos_line;
        int[] if_cross;         //节点是否是十字路口点，是的话为YES，否则为NO
        int[] traffic_sign;     //存放交通标志
        soLongArray objLongArray;
        soNetworkAnalyst objNetworkAnalyst1;
        int[] Road_ID_Queue = new int[1000];//顺序保存所有经过的路段的ID
        int[] Node_ID_Queue = new int[1000];//顺序保存所有经过的节点的ID
        double[] Road_Angle = new double[1000];//保存所经道路的角度值
        int Road_ID_Match;//匹配当前路段的道路ID
        int Number_Road_Pathplan = 0;//路网下的路径规划所经路段的数量
        public double Angle_Forbid_Road = 10.0;
        public MapForm mf = new MapForm();
        public double Distance_Global = 30.0;//判断是否全局搜索的距离,初值设为30米
        public int Distance_SendWay = 30;//判断是过车身发送数据还是以GPS点为中心发送数据的误差距离
        public double Hermite_Modulus = 0.5;//赫米特插值的系数
        public int Distance_Align = 1250;//校正距离，即每隔设定的距离就校正一次惯导数据，2500*0.4即为1公里//1023
        public int Index_Distance_Align = 1;//第几次校正//1023
        public bool ListFull = false;//判断队列是否填满
        public bool Flag_Align = true;//是否添加校正量标志//1023
        List<double> DataMedianX = new List<double>(5);//x的中值滤波序列
        List<double> DataMedianY = new List<double>(5);//y的中值滤波序列
        double[] ListSort = new double[5];//中间排序数组
        List<transfer._soPoint> pass_point = new List<transfer._soPoint>(Log_Count);//用队列存储60个点，即以0.5米记录一次大约存储30米的路程
        transfer._soPoint[] pass_point_array = new transfer._soPoint[Log_Count];//数组转存
        public bool Whether_Exist_StopCarArea = false;//是否存在侧方停车区域1025
        public bool Car_Backing = false;//是否在倒车标志1022
        public int Ready_To_Align = 8;//准备校正标志，8为正常工作状态，9为到了校正距离，准备校正1027
        public bool Flag_Alignning = false;//正在校正标志
        System.Threading.Timer initTimer;
        System.Threading.Timer pubTimer;
        bool rd_point_flag = true;//发送路点坐标选项，false为相对坐标，true为绝对坐标
        public bool blnOpen = false;
        CROS cros;
        /************************************ 主程序 ***********************
         * 启动200ms定时器
         * 声明初始化程序INIT（）
         * 自动调MainForm_load
         * *****************************************************************/
        public MainForm()
        {   //200ms 定时器,启动后提示“GIS ININ”“Waiting ..",主界面加载后关闭
            MapForm mf = new MapForm(this);
            initTimer = new System.Threading.Timer(new System.Threading.TimerCallback(inittimer), null, 0, 200);
            pubTimer = new System.Threading.Timer(new System.Threading.TimerCallback(pubtimer), null, 0, 500);
            InitializeComponent();
            INITThread = new System.Threading.Thread(new System.Threading.ParameterizedThreadStart(INIT));
        }

        /********** 软件启动后在控制台Console提示初始化信息 ************/
        void inittimer(object obj)
        {
            Console.WriteLine("GIS INIT");
            Console.WriteLine("Waiting ...");
        }
        void pubtimer(object obj)
        {
            if (Init.FlagInitSuccess)
            {
                //ParameterSetBeforeSendToUDP();
                //double speed = rd_points[NavigationPoint.present_point_index].speed;
                //cros.PubGIS(this, DataToUDPExcludeRoadPoints, RunningStatus, speed, Ready_To_Align, IntersectionFlag, tf, rd_point_flag);
            }
        }
        struct rd_point
        {
            public double x;
            public double y;
            public int match_status;    //比赛状态：0 比赛前 1 发车前 2 正常行驶 3 比赛结束
            public int changedirection; //转向：1 直行 3左转，2 右转 4 掉头
            public double speed;        //实时速度
            public double distance;     //距离，此值为两种情况之一：（1）与下一个路口之间的距离；（2）与终点的距离
            public int traffic;         //是否为交通灯路点
            public int lane_num;        //车道线数量
            public int traffic_sign;    //是否有交通标志
            public int lane_info;       //车道转弯信息
            public int if_corner;       //“是YES”情况下则“GIS与实时车辆车道线数据”进行校正；否则不校正
            public int if_crossline;    //能否并线标志，YES（1）能，NO（0）不能
            public int speed_limit;
            public int Flag_OF_NoRoadNet;   //有无路网标志YES（1）-有
            public double second_;
            public int gear;
            public double velocity;
            public double yaw;
            public long ID;
        }
        rd_point[] logPoint = new rd_point[100000];
        //得到当前路点往前i个点所在路段的方向角
        private double getangle(int i)
        {
            double angle = new double();
            if (NavigationPoint.present_point_index + i < NavigationPoint.WholeNumberOfNavigationPoints)
            {
                double detax = NavigationPoint.x_new[NavigationPoint.present_point_index + i] - NavigationPoint.x_new[NavigationPoint.present_point_index - 1 + i];
                double detay = NavigationPoint.y_new[NavigationPoint.present_point_index + i] - NavigationPoint.y_new[NavigationPoint.present_point_index - 1 + i];
                angle = getangle(detax, detay);
                return angle;
            }
            else
            {
                return DataFromUDP.back_azimuth;
            }
        }

        //得到方向角
        private double getangle(double detax, double detay)
        {
            double tangle = new double();
            if (Math.Abs(detax) < 0.0001)
            {
                if (detay > 0.0)
                {
                    return 0.0;
                }
                else
                {
                    return 180.0;
                }
            }
            else
            {
                if (Math.Abs(detay) < 0.0001)
                {
                    if (detax > 0.0)
                    {
                        return 90.0;
                    }
                    else
                    {
                        return 270.0;
                    }
                }
                else
                {

                    tangle = detax / detay;
                    if (detax > 0.0)
                    {
                        if (detay > 0.0)
                        {
                            //右侧象限

                            return (Math.Atan(tangle) / Math.PI * 180.0);
                        }
                        else
                        {
                            return (Math.Atan(tangle) / Math.PI * 180.0 + 180.0);
                        }
                    }
                    else
                    {
                        if (detay > 0.0)
                        {
                            //右侧象限

                            return (Math.Atan(tangle) / Math.PI * 180.0 + 360.0);
                        }
                        else
                        {
                            return (Math.Atan(tangle) / Math.PI * 180.0 + 180.0);
                        }

                    }
                }
            }
        }
        /***************************** 坐标转换 **************************
         * 输入经度lon，纬度lat，单位：度 如39度57'=39.95
         * 输出（设置）soGeoPoint的x,y。
         * *************************************************************/
    

        //连接地图空间
        private void ConnectSuperMap()
        {
            object objHandle = this.axSuperWorkspace1.ObjectHandle;
            if (!mf.IsDisposed)
            {
                mf.axSuperMap1.Connect(objHandle);
            }
            this.axSuperAnalyst1.Connect(objHandle);
            Marshal.ReleaseComObject(objHandle);
            objHandle = null;
        }

        //窗口关闭
        private void MainForm_FormClosing(object sender, FormClosingEventArgs e)
        {   // 关闭GIS空间分析功能
            this.axSuperAnalyst1.Disconnect();
            // 关闭GIS工作空间
            this.axSuperWorkspace1.Close();
            //CloseUDP
            ROS.shutdown();
            ROS.waitForShutdown();
            base.OnClosing(e);
            pubTimer.Dispose();
            //sw_read.Close();
            //fs_read.Close();
            TimerDisplay100ms.Enabled = false;
        }

        //处理读入的关键点，得到所要经过的路段的起点和终点，为路径规划做准备
        private void processrp(int Array_index)
        {
            int NumberOfRdPoints = rpr.Number_Of_Taskpoints_InGroup[Array_index];
            int Index_Of_Startpoint = rpr.Index_Of_Startpoint_InGroup[Array_index];
            transfer._soPoint[] ptt = new transfer._soPoint[NumberOfRdPoints];
            transfer._soPoint[] ptf = new transfer._soPoint[NumberOfRdPoints];
            double distanceT = new double();
            double distanceF = new double();
            string temp_2_string = Init.String;
           
            for (int i = 0; i < NumberOfRdPoints; i++)
            {
                string temp_string = "(1/5)正匹配第" + (Array_index + 1).ToString() + "组的第" + (i + 1).ToString() + "个任务点，该组共有" + NumberOfRdPoints + "个任务点。\r\n";
                Init.String = temp_2_string + temp_string;
                soGeoPoint temp = new soGeoPoint();
                temp.x = rpr.TaskData[i + Index_Of_Startpoint].x;
                temp.y = rpr.TaskData[i + Index_Of_Startpoint].y;
                //匹配得到temp所在的路段
                match(temp,50);
                //匹配失败直接退出
                if (pt.x == 0.0)
                {
                    break;
                }
                //匹配成功，将距离较近的点加入到关键点集合
                else
                {
                    ptt[i] = pt;
                    ptf[i] = pf;
                    if (rpr.TaskData[i + Index_Of_Startpoint].status1 == TASKPROPERTY_STARTPOINT || rpr.TaskData[i + Index_Of_Startpoint].status1 == TASKPROPERTY_ENDPOINT
                        || rpr.TaskData[i + Index_Of_Startpoint].status2 == 6 || rpr.TaskData[i + Index_Of_Startpoint].status2 == 7)
                    {
                        RealP[PathPlanInfo.KeyPointNumber].x = match_point.x;
                        RealP[PathPlanInfo.KeyPointNumber].y = match_point.y;
                        PathPlanInfo.KeyPointNumber++;
                    }//如果是起点或者终点的话，直接赋值任务点
                    else
                    {
                        distanceF = tf.Getdistance(pf, temp);
                        distanceT = tf.Getdistance(pt, temp);
                        if (PathPlanInfo.KeyPointNumber == 0)
                        {
                            RealP[PathPlanInfo.KeyPointNumber].x = match_point.x;
                            RealP[PathPlanInfo.KeyPointNumber].y = match_point.y;
                            PathPlanInfo.KeyPointNumber++;
                        }
                        else
                        {
                            if (distanceF < distanceT)
                            {
                                if (Math.Abs(ptf[i].x - RealP[PathPlanInfo.KeyPointNumber - 1].x) > 2.0 || Math.Abs(ptf[i].y - RealP[PathPlanInfo.KeyPointNumber - 1].y) > 2.0)
                                {
                                    RealP[PathPlanInfo.KeyPointNumber].x = ptf[i].x;
                                    RealP[PathPlanInfo.KeyPointNumber].y = ptf[i].y;
                                    PathPlanInfo.KeyPointNumber++;//如果路段起点距离任务点更近，且与上一关键点不是同一个点，则将路段起点加入关键点序列
                                }
                            }
                            else
                            {
                                if (Math.Abs(ptt[i].x - RealP[PathPlanInfo.KeyPointNumber - 1].x) > 2.0 || Math.Abs(ptt[i].y - RealP[PathPlanInfo.KeyPointNumber - 1].y) > 2.0)
                                {
                                    RealP[PathPlanInfo.KeyPointNumber].x = ptt[i].x;
                                    RealP[PathPlanInfo.KeyPointNumber].y = ptt[i].y;
                                    PathPlanInfo.KeyPointNumber++;//如果路段终点距离任务点更近，且与上一关键点不是同一个点，则将路段终点加入关键点序列
                                }
                            }
                        }
                    }
                }
            }
        }
        //地图匹配函数
        private void match(soGeoPoint pp,int radius)
        {
            //第1步：缓冲区分析
            soGeoRegion objBffGeoRegion = pp.Buffer(radius, 100);//半径为200米的圆形误差区域，100表示由100边形模拟的圆
            soRecordset objRds = null;
            soDatasetVector dlwl = (soDatasetVector)this.axSuperWorkspace1.Datasources[wks_name].Datasets["道路网络"];
            soDatasetVector jd = (soDatasetVector)dlwl.SubDataset;//网络数据集的子数据集（节点数据集）
            Wcqy_Recordset = dlwl.QueryEx((soGeometry)objBffGeoRegion, seSpatialQueryMode.scsAreaIntersect, "");
            double wd = new double();
            if (Wcqy_Recordset == null)
            {
                pt.x = 0.0;
                pt.y = 0.0;
                pf.x = 0.0;
                pf.y = 0.0;
            }
            else
            {
                Wcqy_Recordset.MoveFirst();//指向候选路段记录集的第一条记录
                string strSQLFilter = string.Empty;
                soGeometry objGeom = null;
                soGeoPoint objGeoPtFT = new soGeoPointClass();

                soPoint GPS = new soPoint();
                GPS.x = pp.x;
                GPS.y = pp.y;


                soGeometrist objGeometrist = new soGeometrist();
                transfer._soPoint[] cz = new transfer._soPoint[Wcqy_Recordset.RecordCount];//垂足

                soPoint[] Ptf = new soPoint[Wcqy_Recordset.RecordCount];//路段起点
                soPoint[] Ptt = new soPoint[Wcqy_Recordset.RecordCount];//路段终点
                string[] Ff = new string[Wcqy_Recordset.RecordCount];
                string[] Tt = new string[Wcqy_Recordset.RecordCount];
                double xf, yf, xt, yt;//路段两端点坐标
                double distance;//GPS点到候选路段的垂直距离           
                double[] weight = new double[Wcqy_Recordset.RecordCount];//存放各候选路段权重和
                double[] weight1 = new double[Wcqy_Recordset.RecordCount];//权重排序时使用)
                int[] Road_ID_Now = new int[Wcqy_Recordset.RecordCount];//用来存储当前节点所连接路段的道路ID
                int RecordCount = Wcqy_Recordset.RecordCount;
                for (int b = 0; b < RecordCount; b++)//计算各路段的各项权重
                {


                    //=====================距离权重====================

                    Ptf[b] = new soPoint();
                    Ptt[b] = new soPoint();
                    string j = Wcqy_Recordset.GetFieldValue("SmID").ToString();//线数据集的SmUserID同其在网络数据集中的SmID
                    Road_ID_Now[b] = Convert.ToInt32(j);
                    strSQLFilter = " SmID=" + j;//选定弧段
                    objRds = dlwl.Query(strSQLFilter, true, null, "");
                    String F = objRds.GetFieldValue("SmFNode").ToString();//读取弧段起点
                    String T = objRds.GetFieldValue("SmTNode").ToString();//读取弧段终点           
                    Ff[b] = F;
                    Tt[b] = T;
                    strSQLFilter = " SmID =" + F;//读取起点坐标
                    objRds = jd.Query(strSQLFilter, true, null, "");
                    objGeom = objRds.GetGeometry();
                    objGeoPtFT = (soGeoPoint)objGeom;
                    xf = objGeoPtFT.x;
                    yf = objGeoPtFT.y;

                    strSQLFilter = " SmID =" + T;//读取终点坐标
                    objRds = jd.Query(strSQLFilter, true, null, "");//读取该节点的坐标
                    objGeom = objRds.GetGeometry();
                    objGeoPtFT = (soGeoPoint)objGeom;
                    xt = objGeoPtFT.x;
                    yt = objGeoPtFT.y;

                    Ptf[b].x = xf;
                    Ptf[b].y = yf;
                    Ptt[b].x = xt;
                    Ptt[b].y = yt;

                    soPoint temp = objGeometrist.GetPerpendicularPosition(GPS, Ptf[b], Ptt[b]);//求解垂直投影点
                    cz[b].x = temp.x;
                    cz[b].y = temp.y;

                    if (cz[b].x < Ptt[b].x && cz[b].x > Ptf[b].x || cz[b].x < Ptf[b].x && cz[b].x > Ptt[b].x || cz[b].y < Ptt[b].y && cz[b].y > Ptf[b].y || cz[b].y < Ptf[b].y && cz[b].y > Ptt[b].y)
                    {
                        distance = Math.Sqrt((GPS.x - cz[b].x) * (GPS.x - cz[b].x) + (GPS.y - cz[b].y) * (GPS.y - cz[b].y));
                    }
                    else
                    {
                        double distance1 = System.Math.Sqrt((GPS.x - Ptf[b].x) * (GPS.x - Ptf[b].x) + (GPS.y - Ptf[b].y) * (GPS.y - Ptf[b].y));
                        double distance2 = System.Math.Sqrt((GPS.x - Ptt[b].x) * (GPS.x - Ptt[b].x) + (GPS.y - Ptt[b].y) * (GPS.y - Ptt[b].y));
                        if (distance1 <= distance2)
                        {
                            distance = distance1;

                        }
                        else
                        {
                            distance = distance2;

                        }
                    }


                    wd = (250.0 - distance) / 250;
                    //距离权重表达式  

                    weight[b] = wd;
                    Wcqy_Recordset.MoveNext();

                }//候选路段循环完毕
                for (int k = 0; k < weight.GetLength(0); k++)
                {
                    weight1[k] = weight[k];
                }
                Array.Sort(weight);
                int c = Array.IndexOf(weight1, weight[weight.GetLength(0) - 1]);
                //返回匹配路段的起点和终点
                pt.x = Ptt[c].x;
                pt.y = Ptt[c].y;
                pf.x = Ptf[c].x;
                pf.y = Ptf[c].y;
                match_point = cz[c];
                Road_ID_Match = Road_ID_Now[c];//将当前路段的ID号返回
            }
        }


       

        /*********************** GIS导航路点初始化 *****************************
         * 设置正在初始化的标志
         * 删除以前初始化信息
         * 处理读入的任务点，得到关键点序列
         * 通过SuperMap软件完成路径规划
         * 设置导航点参数
         * 距离路口100米时设置相关参数
         * 终点前100米设置相关参数
         * 出发点的参数设置
         * 交通标志参数设置
         * 车的行驶状态参数设置
         * 交叉口前转向、交通灯及距离参数设置
         * 在地图上显示所有导航点
         * 设置初始化后变量参数
         * **********************************************************************/
        private void INIT(object obj)
        {   //正在进行初始化标志，此标志为true时操作初始化按钮无效
            Init.FlagIsBusy = true;
            //初始化成功标志，只有成功的情况下才发效有效数据及更新显示
            Init.FlagInitSuccess = false;

            Init.String = "初始化 ......\r\n";
            long  Last_Group_EndPoint_index = 0;
            //删除以前初始化信息
            NavigationPoint.dispose();
            IntersectionFlag.WholeNumberIntersection = 0;
            Init.String += "共有" + rpr.Number_Of_Group+"组，一共"+rpr.num+"个任务点！"+"\r\n" ;
            int Number_Of_Group = rpr.Number_Of_Group;//任务点中组的个数
            for (int j = 0; j < Number_Of_Group; j++)
            {   //无路网或居民区
                if (rpr.Flag_OF_NoRoadNet[j] == YES)          
                {   
                    Init.String += "正在对第" + (j + 1).ToString() +  "组任务点进行Hermite插值...." + "\r\n";
                    int Number_Of_Taskpoints= rpr.Number_Of_Taskpoints_InGroup[j];//第j组任务点的个数
                    int Index_Of_Startpoint = rpr.Index_Of_Startpoint_InGroup[j];//第j组任务点起始序号
                    double[] start_angle = new double[Number_Of_Taskpoints];//任务点的起始角度
                    double[] end_angle = new double [Number_Of_Taskpoints];//任务点的终止角度
                    double[] distance = new double[Number_Of_Taskpoints];//两点间的距离
                    double[] x = new double[Number_Of_Taskpoints];//存放居民区中任务点的经度，第一个点为第一个任务点在路上的匹配点，最后一个点为最后一个任务点在路上的匹配点
                    double[] y = new double[Number_Of_Taskpoints];//存放居民区中任务点的纬度，第一个点为第一个任务点在路上的匹配点，最后一个点为最后一个任务点在路上的匹配点
                    soGeoPoint temp = new soGeoPoint();
                    temp.x = rpr.TaskData[Index_Of_Startpoint].x;
                    temp.y = rpr.TaskData[Index_Of_Startpoint].y;
                    match(temp,50);//匹配第一个任务点
                    x[0] = match_point.x;
                    y[0] = match_point.y;
                    temp.x = rpr.TaskData[Index_Of_Startpoint + Number_Of_Taskpoints - 1].x;
                    temp.y = rpr.TaskData[Index_Of_Startpoint + Number_Of_Taskpoints - 1].y;
                    match(temp,50);//匹配最后一个任务点
                    x[Number_Of_Taskpoints - 1] = match_point.x;
                    y[Number_Of_Taskpoints - 1] = match_point.y;
                    //用x,y数组存放任务点的经纬度
                    for (int i = 1; i < Number_Of_Taskpoints - 1; i++)
                    {
                        x[i] = rpr.TaskData[Index_Of_Startpoint + i].x;
                        y[i] = rpr.TaskData[Index_Of_Startpoint + i].y;
                    }

                    for (int i = 0; i < Number_Of_Taskpoints-1; i++)
                    {
                        start_angle[i] = twopoints_angle(x[i + 1] - x[i], y[i + 1] - y[i]);
                    }
                    start_angle[Number_Of_Taskpoints - 1] = start_angle[Number_Of_Taskpoints - 2];
                    for (int i = 0; i < Number_Of_Taskpoints; i++)
                    {
                        if (i == 0)
                        {
                            end_angle[i] = start_angle[i];
                        }
                        else
                        {
                            end_angle[i] = (start_angle[i] + start_angle[i - 1]) / 2.0;
                            if (!(Math.Abs(end_angle[i] - start_angle[i]) < 90 || Math.Abs(end_angle[i] - start_angle[i - 1]) < 90))
                            {
                                end_angle[i] = end_angle[i] + 180;
                            }

                        }
                        if (i == Number_Of_Taskpoints - 1)
                        {
                            distance[i] = distance[i - 1];
                        }
                        else
                        {
                            distance[i] = twopoints_distance(x[i + 1], y[i + 1], x[i], y[i]);
                        }
                    }

                   
                    //赫米特插值
                    for (int i = 0; i < Number_Of_Taskpoints - 1; i++)
                    {
                        NavigationPoint.hermite(x[i], y[i], x[i + 1], y[i + 1], end_angle[i] * Math.PI / 180.0, end_angle[i + 1] * Math.PI / 180.0, Hermite_Modulus * distance[i]);
                    }
                    long TotalNumber = NavigationPoint.WholeNumberOfNavigationPoints;
                    //无路网（居民区）赋值属性点
                    for (long i = Last_Group_EndPoint_index; i < TotalNumber; i++)
                    {   
                        rd_points[i].x = NavigationPoint.x_new[i];
                        rd_points[i].y = NavigationPoint.y_new[i];
                        rd_points[i].changedirection = GOAHEAD;
                        rd_points[i].speed = SPEEDMIN;
                        rd_points[i].traffic = NO;
                        rd_points[i].match_status = MATCHSTATUS_MATCHING; //比赛中，正常行驶 （默认值）
                        rd_points[i].distance = (TotalNumber - i) * 0.4;
                        rd_points[i].lane_num = 0;
                        rd_points[i].traffic_sign = NO;//不知道有没有交通标识
                        rd_points[i].if_corner = 0;
                        rd_points[i].if_crossline = YES;  //能否并线，YES能
                        rd_points[i].speed_limit = Convert.ToInt32(SPEEDMAX);
                        rd_points[i].lane_info = 0xff;//在居民区不限制在哪条车道走
                        rd_points[i].Flag_OF_NoRoadNet = YES;   //设置无路网标志
                    }
                    Last_Group_EndPoint_index = NavigationPoint.WholeNumberOfNavigationPoints;
                    Init.String += "Hermite插值完成" + "\r\n";
                }
                //有路网
                else
                {
                    
                    
                    //处理读入的任务点，得到关键点序列
                    processrp(j);

                    Init.String += "(1/5)完成任务点匹配。\r\n";
                    Init.String += "(2/5)路径规划。。。\r\n";

                    //通过SuperMap软件完成路径规划
                    SuperMapPathPlan();

                    Init.String += "(2/5)完成路径规划，共经过" + objLongArray.Count.ToString() + "个路点。\r\n";
                    Init.String += "(3/5)路径生成。。。\r\n";

                    //根据任务点计算路径规划后各条道路的参数
                    ParameterSetOfNavigtaionPaths();
                    Init.String += "(4/5)状态赋值。。。\r\n";

                    //设置导航点参数
                    ParameterSetOfNavigationPoints(Last_Group_EndPoint_index);
                    Init.String += "(4/5)赋值车道线信息完成。。。\r\n";

                    //交叉口前转向、交通灯及距离参数设置
                    ParameterSetBeforeTheNextInterseectionOrEndPoint(Last_Group_EndPoint_index);
                    //距离路口100米时设置相关参数 
                    ParameterSet100mBeforeIntersection();


                    //出发点的参数设置
                    ParameterSetStartPoint();


                    Last_Group_EndPoint_index = NavigationPoint.WholeNumberOfNavigationPoints;
                    Init.String += "(4/5)赋值转向信息完成。。。\r\n";
                    

                    //在地图上显示所有导航点

                   
                    //设置初始化后变量参数
                    ParameterSetAfterINITSuccess();
                }

            }
            InitHermite();
            Init.String += "(3/5)完成路径生成，路径长" + (0.4 * NavigationPoint.WholeNumberOfNavigationPoints).ToString() + "米。\r\n";
           
           
            //终点前100米设置相关参数
            ParameterSetEndPoint();
            //交通标志参数设置
            ParameterSetAroundTrafficSign();
            Car_Backing = false;
            SetIndexAlign();//设置判断是否校正的序号数组1028
            Init.String += "(5/5)绘图。。。\r\n";
            DisplayNavigationPointsOnMap();
            //while (true) ;
            Init.String += "初始化完成。\r\n";
            Init.FlagInitSuccess = true;
            Init.FlagIsBusy = false;
            INITThread.Abort();
        }
        //使用Hermite插值连接LOG点
        private void InitHermite()
        {
            int Number_Of_Taskpoints = r_hermite.Task_Num;
            double[] start_angle = new double[Number_Of_Taskpoints];//任务点的起始角度
            double[] end_angle = new double[Number_Of_Taskpoints];//任务点的终止角度
            double[] distance = new double[Number_Of_Taskpoints];//两点间的距离
            double[] x = new double[Number_Of_Taskpoints];//存放居民区中任务点的经度，第一个点为第一个任务点在路上的匹配点，最后一个点为最后一个任务点在路上的匹配点
            double[] y = new double[Number_Of_Taskpoints];//存放居民区中任务点的纬度，第一个点为第一个任务点在路上的匹配点，最后一个点为最后一个任务点在路上的匹配点
            for (int i = 0; i < r_hermite.Task_Num; i++)
            {
                x[i] = r_hermite.TaskPointX[i];
                y[i] = r_hermite.TaskPointY[i];
            }

            for (int i = 0; i < Number_Of_Taskpoints - 1; i++)
            {
                start_angle[i] = twopoints_angle(x[i + 1] - x[i], y[i + 1] - y[i]);
            }
            start_angle[Number_Of_Taskpoints - 1] = start_angle[Number_Of_Taskpoints - 2];
            for (int i = 0; i < Number_Of_Taskpoints; i++)
            {
                if (i == 0)
                {
                    end_angle[i] = start_angle[i];
                }
                else
                {
                    end_angle[i] = (start_angle[i] + start_angle[i - 1]) / 2.0;
                    if (!(Math.Abs(end_angle[i] - start_angle[i]) < 90 || Math.Abs(end_angle[i] - start_angle[i - 1]) < 90))
                    {
                        end_angle[i] = end_angle[i] + 180;
                    }

                }
                if (i == Number_Of_Taskpoints - 1)
                {
                    distance[i] = distance[i - 1];
                }
                else
                {
                    distance[i] = twopoints_distance(x[i + 1], y[i + 1], x[i], y[i]);
                }
            }


            //赫米特插值
            for (int i = 0; i < Number_Of_Taskpoints - 1; i++)
            {
                //r_hermite.hermite(x[i], y[i], x[i + 1], y[i + 1], end_angle[i] * Math.PI / 180.0, end_angle[i + 1] * Math.PI / 180.0, Hermite_Modulus * distance[i]);
                r_hermite.TwoPointLine(x[i], y[i], x[i + 1], y[i + 1]);
            }
        }
        private void SetIndexAlign()//设置判断是否校正的序号数组1028
        {
            long Count = 0;//Count是赋给数组里的数
            long TotalNumber = NavigationPoint.WholeNumberOfNavigationPoints;
            for (long i = 0; i < TotalNumber; i++)
            {
                if (Count == 0)//第一个点
                {
                    NavigationPoint.Index_Align[i] = Count;
                    Count++;
                }
                else
                {
                    if ((Count % Distance_Align) == 0)//如果Count是Distance_Align的整数倍，即正好该校正的时候
                    {
                        if (rd_points[i].distance < 50.0)
                        {
                            NavigationPoint.Index_Align[i] = NavigationPoint.Index_Align[i - 1];//如果此时点的距离小于50米（即在路口内），则重复赋值上一个数，即若此时i为2500，数组一直赋值2499直到出了路口
                        }
                        else
                        {
                            NavigationPoint.Index_Align[i] = Count;//若不在路口内，则正常赋值
                            Count++;
                        }

                    }
                    else
                    {
                        NavigationPoint.Index_Align[i] = Count;
                        Count++;
                    }
                }
            }
        }
        /********************************************************************
        * 倒车初始化
        * *******************************************************************/
        private void INIT_BACK_CAR(object obj)//倒车直接连线1022
        {
            Init.FlagIsBusy = true;

            Init.FlagInitSuccess = false;

            Init.String = "初始化。。。\r\n";

            NavigationPoint.dispose();

            string strSQLFilter = string.Empty;
            soGeoPoint objGeoPtFT = new soGeoPointClass();
            soGeoPoint temp2 = new soGeoPoint();

            Init.String += "准备连接任务点。。。\r\n";

            pass_point_array = pass_point.ToArray();//将队列数据赋值到数组中

            for (int i = Log_Count - 1; i > Log_Count-BackCar_Count; i--)//两点连线
            {
                if (pass_point_array[i - 1].x == 0.0 || pass_point_array[i - 1].y == 0.0)
                    break;
                NavigationPoint.TwoPointLine(pass_point_array[i].x, pass_point_array[i].y, pass_point_array[i-1].x, pass_point_array[i-1].y);
            }
            if (NavigationPoint.WholeNumberOfNavigationPoints == 0)
            {
                MessageBox.Show("initial failure");
            }
            Init.String += "完成路径生成，路径长" + (0.4 * NavigationPoint.WholeNumberOfNavigationPoints).ToString() + "米。\r\n";

            Init.String += "绘图。。。\r\n";
            clear();
            long TotalNumber = NavigationPoint.WholeNumberOfNavigationPoints;
            for (int i = 0; i < TotalNumber; i++)
            {
                rd_points[i].x = NavigationPoint.x_new[i];
                rd_points[i].y = NavigationPoint.y_new[i];
                rd_points[i].speed = -10.0;
                rd_points[i].distance = 0;
                rd_points[i].match_status = 2;
            }

            for (long lengh_point = 0; lengh_point < TotalNumber; lengh_point ++)
            {
                temp2.x = NavigationPoint.x_new[lengh_point];
                temp2.y = NavigationPoint.y_new[lengh_point];

                show(temp2, Color.Red);
            }
            Init.String += "初始化完成。\r\n";

            Init.FlagInitSuccess = true;
            Init.FlagIsBusy = false;
            INITThread.Abort();
        }

        /*********************** 通过SuperMap软件完成路径规划 *************************
         * Real清零
         * 关键点的个数KeyPointNumber清零
         * 清除SuperMap规划的所有导航点PathPlanMapRoadEndPoints.RemoveAll
         * 设置标志位
         * 退出线程
         * *******************************************************************/
        private void SuperMapPathPlan()
        {
            //路径规划
            soNetworkAnalystEx objNetworkAnalyst = null;
            objNetworkAnalyst = this.axSuperAnalyst1.NetworkAnalyst;
            objNetworkAnalyst1 = new soNetworkAnalyst();
            soNetworkSetting objNetworkSetting = null;
            objNetworkSetting = this.axSuperAnalyst1.NetworkAnalyst.NetworkSetting;
            objNetworkSetting.NetworkDataset = (soDatasetVector)this.axSuperWorkspace1.Datasources[wks_name].Datasets["道路网络"];
            objNetworkSetting.NodeIDField = "SmID";
            objNetworkSetting.EdgeIDField = "SmID";
            objNetworkSetting.FromNodeIDField = "SmFNode";
            objNetworkSetting.ToNodeIDField = "SmTNode";
            objNetworkSetting.FTWeightField = "SmResistanceA+SmLength";
            objNetworkSetting.TFWeightField = "SmResistanceB+SmLength";
            objNetworkAnalyst.Tolerance = 0;
            int KeyNumber = PathPlanInfo.KeyPointNumber;
            for (int i = 0; i < KeyNumber; i++)
            {
                this.PathPlanMapRoadEndPoints.Add2(RealP[i].x, RealP[i].y);
            }//添加路径规划中所必须经过的点

            //SuperMap路径规划结果
            soPathResultSetting objPathResult = new soPathResultSetting();
            soPathResultInfo objPathInfo = new soPathResultInfo();

            objPathResult.NodeIDsEnabled = true;
            //进行路径规划，相关信息保存在objPathInfo中
            objNetworkAnalyst.PathEx2(this.PathPlanMapRoadEndPoints, 1, objPathResult, objPathInfo);
            objLongArray = objPathInfo.ResultNodeIDs;

            if (objLongArray == null)
            {
                MessageBox.Show("路径规划失败，可能是加了起点和终点，待调试！ 请重试！");
                Init.FlagIsBusy = false;
                INITThread.Abort();
                return;
            }
        }
        
        /*********************** 根据任务点计算路径规划后各条道路的参数 *************************
         * Real清零
         * 关键点的个数KeyPointNumber清零
         * 清除SuperMap规划的所有导航点PathPlanMapRoadEndPoints.RemoveAll
         * 设置标志位
         * 退出线程
         * *******************************************************************/
        private void ParameterSetOfNavigtaionPaths()
        {   /*********************  SuperMap 参数定义 *************************/
            //道路网络数据集
            soDatasetVector dlwl = (soDatasetVector)this.axSuperWorkspace1.Datasources[wks_name].Datasets["道路网络"];
            //网络数据集的子数据集（节点数据集） 
            soDatasetVector jd = (soDatasetVector)dlwl.SubDataset;
            soGeometry objGeom = null;
            soGeoPoint objGeoPtFT = new soGeoPointClass();
            soRecordset objRds = null;
            string strSQLFilter = string.Empty;
            //定义节点数组
            if_cross = new int[jd.RecordCount + 1];//节点是否是十字路口点，是的话为YES，否则为NO
            double[] road_width = new double[dlwl.RecordCount + 1];//道路宽度
            int[] lane_num_all = new int[dlwl.RecordCount + 1];//车道线数量
            int[] lane_info_all = new int[dlwl.RecordCount + 1];//每条车道的转向信息
            if_corner = new int[dlwl.RecordCount + 1];
            int[] if_corner_all = new int[dlwl.RecordCount + 1];
            int[] if_croos_line_all = new int[dlwl.RecordCount + 1];
            if_croos_line = new int[dlwl.RecordCount + 1];
            int[] speed_limit_all = new int[dlwl.RecordCount + 1];
            speedlimit = new int[dlwl.RecordCount + 1];
            int[] traffic_sign_all = new int[dlwl.RecordCount + 1];//存放交通标识
            traffic_sign = new int[dlwl.RecordCount + 1];
            double old_road_width = 20.0;//保存上一道路的宽度
            double[,] road_field = new double[dlwl.RecordCount + 1, 7];//7维数据分别为起点的x,y，ID和终点的x,y，ID，道路角度

            status = new int[objLongArray.Count + 1];//保存每个路口转向情况：0是直行，1是左转，2是右转 
            lane_num = new int[objLongArray.Count + 1];//保存每条经过道路的车道线数量
            lane_info = new int[objLongArray.Count + 1];//保存每条车道的转弯方向
            double[] radius = new double[objLongArray.Count + 1];
            traffic = new int[objLongArray.Count + 1];//保存每条经过道路是否有交通灯，1为有，0为没有

            int[] RoadID = new int[10];//存储每个路口所连接的道路的ID
            int road_num = 0;//存储每个节点连接道路的数量
            soSelection objselction = new soSelection();
            soRecordset RS_connected_road = null;//得到每个路口连接道路的集合
            double old_endx = new double();
            double old_endy = new double();//保存前一个道路的轨迹的终点
            //double last_old_endx = new double();
            //double last_old_endy = new double();//保存前2个道路的轨迹的终点
            double turn_radius = 4.0;
            string temp_2string = Init.String;
            //按顺序处理所有经过的节点
            for (int i = 1; i <= objLongArray.Count; i++)
            {
                int iID = objLongArray[i];//节点ID
                strSQLFilter = " SmID=" + iID.ToString();
                objRds = jd.Query(strSQLFilter, true, null, "");
                objGeom = objRds.GetGeometry();
                objGeoPtFT = (soGeoPoint)objGeom;
                JointIDCoor[iID].x = objGeoPtFT.x;
                JointIDCoor[iID].y = objGeoPtFT.y;//节点的坐标
                if_cross[iID - 1] = Convert.ToInt16(objRds.GetFieldValue("if_cross"));//道路中心点是否是十字路口
                traffic[i - 1] = Convert.ToInt32(objRds.GetFieldValue("if_traffic"));//道路中心点是否有交通灯
                //if (i != objLongArray.Count)//暂时发现没用
                //{
                //    int next_ID = objLongArray[i + 1];//节点ID
                //    strSQLFilter = " SmID=" + next_ID.ToString();
                //    objRds = jd.Query(strSQLFilter, true, null, "");
                //    objGeom = objRds.GetGeometry();
                //    objGeoPtFT = (soGeoPoint)objGeom;

                //    //道路中心点坐标 
                //    JointIDCoor[next_ID].x = objGeoPtFT.x;
                //    JointIDCoor[next_ID].y = objGeoPtFT.y;

                //    if_cross[next_ID - 1] = Convert.ToInt16(objRds.GetFieldValue("if_cross"));
                //}
                int start_index = 0;
                int end_index = 0;//起始路段和终止路段的序号

                //计算路的初始角度与终止角度
                double first_angle = getangle(RealP[0].x - JointIDCoor[objLongArray[1]].x, RealP[0].y - JointIDCoor[objLongArray[1]].y) * Math.PI / 180.0;//第一条道路的角度
                double last_angle = getangle(RealP[PathPlanInfo.KeyPointNumber - 1].x - JointIDCoor[objLongArray[objLongArray.Count]].x, RealP[PathPlanInfo.KeyPointNumber - 1].y - JointIDCoor[objLongArray[objLongArray.Count]].y) * Math.PI / 180.0;//最后一条道路的角度

                //  得到以该节点为中心的路口的道路数量和每条道路的ID，并分别存储到road_num和RoadID中
                objselction = objNetworkAnalyst1.FindConnectedEdges(dlwl, iID, sePathFindingDirection.scdBoth, 1);
                RS_connected_road = objselction.ToRecordset(false);
                road_num = RS_connected_road.RecordCount;

                for (int m = 0; m < road_num; m++)
                {
                    RoadID[m] = Convert.ToInt32(RS_connected_road.GetFieldValue("SmID"));
                    strSQLFilter = " SmID=" + RoadID[m];//选定弧段
                    objRds = dlwl.Query(strSQLFilter, true, null, "");
                    road_width[RoadID[m]] = Convert.ToDouble(objRds.GetFieldValue("width"));
                    lane_num_all[RoadID[m]] = Convert.ToInt32(objRds.GetFieldValue("lane_num"));
                    lane_info_all[RoadID[m]] = Convert.ToInt32(objRds.GetFieldValue("lane_info"));
                    if_corner_all[RoadID[m]] = Convert.ToInt32(objRds.GetFieldValue("if_corner"));
                    if_croos_line_all[RoadID[m]] = Convert.ToInt32(objRds.GetFieldValue("if_crossline"));
                    speed_limit_all[RoadID[m]] = Convert.ToInt32(objRds.GetFieldValue("speed_limit"));
                    traffic_sign_all[RoadID[m]] = Convert.ToInt32(objRds.GetFieldValue("traffic_sign"));
                    String F = objRds.GetFieldValue("SmFNode").ToString();//读取弧段起点
                    String T = objRds.GetFieldValue("SmTNode").ToString();//读取弧段终点           
                    strSQLFilter = " SmID =" + F;//读取起点坐标
                    objRds = jd.Query(strSQLFilter, true, null, "");
                    objGeom = objRds.GetGeometry();
                    objGeoPtFT = (soGeoPoint)objGeom;
                    road_field[RoadID[m], 0] = objGeoPtFT.x;
                    road_field[RoadID[m], 1] = objGeoPtFT.y;
                    road_field[RoadID[m], 2] = Convert.ToDouble(F);
                    strSQLFilter = " SmID =" + T;//读取终点坐标
                    objRds = jd.Query(strSQLFilter, true, null, "");
                    objGeom = objRds.GetGeometry();
                    objGeoPtFT = (soGeoPoint)objGeom;
                    road_field[RoadID[m], 3] = objGeoPtFT.x;
                    road_field[RoadID[m], 4] = objGeoPtFT.y;
                    road_field[RoadID[m], 5] = Convert.ToDouble(T);
                    road_field[RoadID[m], 6] = 0.0;//得到路段的角度
                    RS_connected_road.MoveNext();
                }
                NavigationPoint.n = road_num;

                NavigationPoint.x = JointIDCoor[iID].x;
                NavigationPoint.y = JointIDCoor[iID].y;
                if (i == 1)
                {
                    for (int m = 0; m < road_num; m++)
                    {
                        NavigationPoint.width[m] = road_width[RoadID[m]];
                        if ((Convert.ToInt32(road_field[RoadID[m], 5])) == iID)
                        {
                            NavigationPoint.angle[m] = getangle(road_field[RoadID[m], 0] - JointIDCoor[iID].x, road_field[RoadID[m], 1] - JointIDCoor[iID].y) * Math.PI / 180.0;
                        }
                        else
                        {
                            NavigationPoint.angle[m] = getangle(road_field[RoadID[m], 3] - JointIDCoor[iID].x, road_field[RoadID[m], 4] - JointIDCoor[iID].y) * Math.PI / 180.0;//求出每条道路的角度
                        }
                        if (Math.Abs(NavigationPoint.angle[m] - first_angle) < (5.0 * Math.PI / 180.0))
                        {
                            start_index = m;
                            Road_ID_Queue[Number_Road_Pathplan] = RoadID[start_index];//所经第一条道路的道路ID
                            Node_ID_Queue[Number_Road_Pathplan] = objLongArray[i]; 
                            Road_Angle[Number_Road_Pathplan] = NavigationPoint.angle[start_index] + Math.PI;//第一条道路的角度
                            if (Road_Angle[Number_Road_Pathplan] > (2 * Math.PI))
                            {
                                Road_Angle[Number_Road_Pathplan] = Road_Angle[Number_Road_Pathplan] - (2 * Math.PI);//角度范围都为0-360
                            }
                            Number_Road_Pathplan++;
                        }
                        if (Convert.ToInt32(road_field[RoadID[m], 5]) == objLongArray[i + 1] || Convert.ToInt32(road_field[RoadID[m], 2]) == objLongArray[i + 1])
                        {
                            end_index = m;//求出起始道路和终止道路的序号

                            Node_ID_Queue[Number_Road_Pathplan] = objLongArray[i + 1]; //下一个节点的ID
                            Road_ID_Queue[Number_Road_Pathplan] = RoadID[end_index];//下一条道路的道路ID
                            Road_Angle[Number_Road_Pathplan] = NavigationPoint.angle[end_index];//下一条道路的角度
                            Number_Road_Pathplan++;
                        }
                    }
                    old_endy = RealP[0].y;
                    old_endx = RealP[0].x;
                    //last_old_endx = old_endx;
                    //last_old_endy = old_endy;
                }
                else
                {
                    if (i == objLongArray.Count)
                    {
                        for (int m = 0; m < road_num; m++)
                        {
                            NavigationPoint.width[m] = road_width[RoadID[m]];
                            if ((Convert.ToInt32(road_field[RoadID[m], 5])) == iID)
                            {
                                NavigationPoint.angle[m] = getangle(road_field[RoadID[m], 0] - JointIDCoor[iID].x, road_field[RoadID[m], 1] - JointIDCoor[iID].y) * Math.PI / 180.0;
                            }
                            else
                            {
                                NavigationPoint.angle[m] = getangle(road_field[RoadID[m], 3] - JointIDCoor[iID].x, road_field[RoadID[m], 4] - JointIDCoor[iID].y) * Math.PI / 180.0;//求出每条道路的角度
                            }
                            if (Math.Abs(NavigationPoint.angle[m] - last_angle) < (30.0 * Math.PI / 180.0))
                            {
                                end_index = m;
                                Node_ID_Queue[Number_Road_Pathplan] = objLongArray[i + 1]; //下一个节点的ID
                                Road_ID_Queue[Number_Road_Pathplan] = RoadID[end_index];//下一条道路的ID
                                Road_Angle[Number_Road_Pathplan] = NavigationPoint.angle[end_index];//下一条道路的角度
                                Number_Road_Pathplan++;
                            }
                            if (Convert.ToInt32(road_field[RoadID[m], 2]) == objLongArray[i - 1] || Convert.ToInt32(road_field[RoadID[m], 5]) == objLongArray[i - 1])
                                start_index = m;//求出起始道路和终止道路的序号
                        }
                    }
                    else
                    {
                        for (int m = 0; m < road_num; m++)
                        {
                            NavigationPoint.width[m] = road_width[RoadID[m]];
                            if ((Convert.ToInt32(road_field[RoadID[m], 5])) == iID)
                            {
                                NavigationPoint.angle[m] = getangle(road_field[RoadID[m], 0] - JointIDCoor[iID].x, road_field[RoadID[m], 1] - JointIDCoor[iID].y) * Math.PI / 180.0;
                            }
                            else
                            {
                                NavigationPoint.angle[m] = getangle(road_field[RoadID[m], 3] - JointIDCoor[iID].x, road_field[RoadID[m], 4] - JointIDCoor[iID].y) * Math.PI / 180.0;//求出每条道路的角度
                            }
                            if (Convert.ToInt32(road_field[RoadID[m], 2]) == objLongArray[i - 1] || Convert.ToInt32(road_field[RoadID[m], 5]) == objLongArray[i - 1])
                                start_index = m;
                            if (Convert.ToInt32(road_field[RoadID[m], 5]) == objLongArray[i + 1] || Convert.ToInt32(road_field[RoadID[m], 2]) == objLongArray[i + 1])
                            {
                                end_index = m;//求出起始道路和终止道路的序号
                                Node_ID_Queue[Number_Road_Pathplan] = objLongArray[i + 1]; //下一个节点的ID
                                Road_ID_Queue[Number_Road_Pathplan] = RoadID[end_index];
                                Road_Angle[Number_Road_Pathplan] = NavigationPoint.angle[end_index];//下一条道路的角度
                                Number_Road_Pathplan++;
                            }
                        }
                    }
                }
                double rpr_angle = getangle(old_endx - NavigationPoint.x, old_endy - NavigationPoint.y) * Math.PI / 180.0;
                
                if (Math.Abs(Math.Abs(rpr_angle - NavigationPoint.angle[start_index]) - Math.PI) < Math.PI / 10.0)
                {
                    if (i > 1)
                    {
                        IntersectionIndexID[i - 1].Index = IntersectionIndexID[i - 2].Index;
                        IntersectionIndexID[i - 1].IDStartPointOfIntersection = IntersectionIndexID[i - 2].IDStartPointOfIntersection;
                        IntersectionIndexID[i - 1].IDEndPointOfIntersection = IntersectionIndexID[i - 2].IDEndPointOfIntersection;
                        lane_num[i - 1] = lane_num[i - 2];
                        lane_info[i - 1] = lane_info[i - 2];
                        if_corner[i - 1] = if_corner[i - 2];
                        if_croos_line[i - 1] = if_croos_line[i - 2];
                        speedlimit[i - 1] = speedlimit[i - 2];
                        traffic_sign[i - 1] = traffic_sign[i - 2];
                        status[i - 1] = status[i - 2];
                    }
                    continue;
                }
                //根据道路的入角与出角差判断直行、左转或右转，输出：为无灯情况下的状态
                status[i - 1] = NavigationPoint.GetStatus(NavigationPoint.angle[start_index], NavigationPoint.angle[end_index]);//保留每个道路的转向状态

                if ((if_cross[iID - 1] == 1 || if_cross[iID - 1] == 2) && status[i - 1] != GOAHEAD)//十字路口
                {
                    double start_radius = 0.0;
                    double end_radius = 0.0;
                    lane_num[i - 1] = lane_num_all[RoadID[start_index]];
                    lane_info[i - 1] = lane_info_all[RoadID[start_index]];
                    if_corner[i - 1] = if_corner_all[RoadID[start_index]];
                    if_croos_line[i - 1] = if_croos_line_all[RoadID[start_index]];
                    speedlimit[i - 1] = speed_limit_all[RoadID[start_index]];
                    traffic_sign[i - 1] = traffic_sign_all[RoadID[start_index]];
                    if (road_width[RoadID[end_index]] < turn_radius * 2.0)
                    {
                        start_radius = turn_radius;
                    }
                    else
                    {
                        if (status[i - 1] == TURNLEFT)
                        {
                            start_radius = road_width[RoadID[end_index]];
                        }
                        else if (status[i - 1] == TURNRIGHT)
                        {
                            start_radius = road_width[RoadID[end_index]] * 0.75;
                        }
                    }
                    if (road_width[RoadID[end_index]] < turn_radius * 2.0)
                    {
                        end_radius = turn_radius;
                    }
                    else
                    {
                        if (status[i - 1] == TURNLEFT)
                        {
                            end_radius = road_width[RoadID[start_index]];
                        }
                        else if (status[i - 1] == TURNRIGHT)
                        {
                            end_radius = road_width[RoadID[start_index]] * 0.75;
                        }
                    }
                    double distance = Math.Sqrt(Math.Pow((NavigationPoint.x - old_endx), 2) + Math.Pow((NavigationPoint.y - old_endy), 2));
                    if (start_radius < end_radius)
                    {
                        old_road_width = start_radius;
                    }
                    else
                    {
                        old_road_width = end_radius;
                    }
                    NavigationPoint.startx = NavigationPoint.x + old_road_width * Math.Sin(NavigationPoint.angle[start_index]);
                    NavigationPoint.starty = NavigationPoint.y + old_road_width * Math.Cos(NavigationPoint.angle[start_index]);
                    NavigationPoint.endx = NavigationPoint.x + old_road_width * Math.Sin(NavigationPoint.angle[end_index]);
                    NavigationPoint.endy = NavigationPoint.y + old_road_width * Math.Cos(NavigationPoint.angle[end_index]);
                    if (if_cross[iID - 1] == 2 && if_cross[objLongArray[i + 1] - 1] == 2 && status[i - 1] == 1)//1029若此节点的if_cross状态为2并且下一个节点的if_cross也为2，同时又是左拐
                    {
                        NavigationPoint.old_start_x = NavigationPoint.startx;//Uturn的第一个入弯点
                        NavigationPoint.old_start_y = NavigationPoint.starty;
                        NavigationPoint.old_angle = twopoints_angle(NavigationPoint.x - NavigationPoint.startx, NavigationPoint.y - NavigationPoint.starty);//Uturn第一条路的角度
                        U_turn_flag = true;
                        NavigationPoint.First_Uturn_Point_x = NavigationPoint.x;//1029
                        NavigationPoint.First_Uturn_Point_y = NavigationPoint.y;//1029
                        continue;
                    }
                    else//1029
                    {
                        if (U_turn_flag)//Uturn进行hermite插值 1029
                        {
                            double start_angle;
                            double end_angle;
                            double Distance_FPoint_And_MPoint;//Uturn入口点与中间点的直线距离
                            double Distance_MPoint_And_TPoint;//Uturn出口点与中间点的直线距离
                            NavigationPoint.startx = (NavigationPoint.First_Uturn_Point_x + NavigationPoint.x) / 2.0;//取Uturn两个拐弯点中点1029
                            NavigationPoint.starty = (NavigationPoint.First_Uturn_Point_y + NavigationPoint.y) / 2.0;//取Uturn两个拐弯点中点1029
                            start_angle = twopoints_angle(NavigationPoint.x - NavigationPoint.startx, NavigationPoint.y - NavigationPoint.starty);//Uturn横着那条路的角度
                            end_angle = twopoints_angle(NavigationPoint.endx - NavigationPoint.x, NavigationPoint.endy - NavigationPoint.y);//Uturn转过去的那条路的角度
                            NavigationPoint.TwoPointLine(old_endx, old_endy, NavigationPoint.old_start_x, NavigationPoint.old_start_y);//Uturn入弯点与上一个节点以直线方式连接
                            Distance_FPoint_And_MPoint = twopoints_distance(NavigationPoint.old_start_x, NavigationPoint.old_start_y, NavigationPoint.startx, NavigationPoint.starty);//Uturn入弯点与中间点的直线距离                            
                            NavigationPoint.hermite(NavigationPoint.old_start_x, NavigationPoint.old_start_y, NavigationPoint.startx, NavigationPoint.starty, NavigationPoint.old_angle * Math.PI / 180.0, start_angle * Math.PI / 180.0, Distance_FPoint_And_MPoint * Hermite_Modulus);
                            Distance_MPoint_And_TPoint = twopoints_distance(NavigationPoint.x_new[NavigationPoint.WholeNumberOfNavigationPoints - 1], NavigationPoint.y_new[NavigationPoint.WholeNumberOfNavigationPoints - 1], NavigationPoint.endx, NavigationPoint.endy);//Uturn出弯点与中间点的直线距离
                            NavigationPoint.hermite(NavigationPoint.x_new[NavigationPoint.WholeNumberOfNavigationPoints - 1], NavigationPoint.y_new[NavigationPoint.WholeNumberOfNavigationPoints - 1], NavigationPoint.endx, NavigationPoint.endy, start_angle * Math.PI / 180.0, end_angle * Math.PI / 180.0, Distance_MPoint_And_TPoint * Hermite_Modulus);
                            U_turn_flag = false;
                        }
                        else
                        {
                            if (distance < old_road_width)
                            {
                                NavigationPoint.WholeNumberOfNavigationPoints = NavigationPoint.WholeNumberOfNavigationPoints - Convert.ToInt32((old_road_width - distance) / NavigationPoint.step_lenth);
                                IntersectionIndexID[i - 1].Index = NavigationPoint.WholeNumberOfNavigationPoints;
                                NavigationPoint.TwopointCircle(NavigationPoint.x_new[NavigationPoint.WholeNumberOfNavigationPoints - 1], NavigationPoint.y_new[NavigationPoint.WholeNumberOfNavigationPoints - 1], NavigationPoint.endx, NavigationPoint.endy, NavigationPoint.angle[start_index], NavigationPoint.angle[end_index]);
                            }
                            else
                            {
                                NavigationPoint.TwoPointLine(old_endx, old_endy, NavigationPoint.startx, NavigationPoint.starty);//得到路口之间的轨迹
                                IntersectionIndexID[i - 1].Index = NavigationPoint.WholeNumberOfNavigationPoints;//保存每个路口在整个轨迹中的序号
                                NavigationPoint.TwopointCircle(NavigationPoint.x_new[NavigationPoint.WholeNumberOfNavigationPoints - 1], NavigationPoint.y_new[NavigationPoint.WholeNumberOfNavigationPoints - 1], NavigationPoint.endx, NavigationPoint.endy, NavigationPoint.angle[start_index], NavigationPoint.angle[end_index]);//得到路口轨迹
                            }
                        }
                    }
                    if (NavigationPoint.status == TURNLEFT || NavigationPoint.status == TURNRIGHT)
                    {
                        if (if_cross[iID - 1] == 1)
                        {
                            IntersectionIndexID[IntersectionFlag.WholeNumberIntersection].IndexStartPointOfIntersection = IntersectionIndexID[i - 1].Index;
                            IntersectionIndexID[IntersectionFlag.WholeNumberIntersection].IndexEndPointOfIntersection = NavigationPoint.WholeNumberOfNavigationPoints;
                            cross_status[IntersectionFlag.WholeNumberIntersection] = status[i - 1];
                            IntersectionFlag.WholeNumberIntersection++;
                            
                        }
                        IntersectionIndexID[i - 1].IDStartPointOfIntersection = IntersectionIndexID[i - 1].Index;
                        IntersectionIndexID[i - 1].IDEndPointOfIntersection = NavigationPoint.WholeNumberOfNavigationPoints;
                       
                    }
                    old_endy = NavigationPoint.y_new[NavigationPoint.WholeNumberOfNavigationPoints - 1];
                    old_endx = NavigationPoint.x_new[NavigationPoint.WholeNumberOfNavigationPoints - 1];//保存上一路段的坐标
                }
                else
                {
                    lane_num[i - 1] = lane_num_all[RoadID[start_index]];
                    lane_info[i - 1] = lane_info_all[RoadID[start_index]];
                    if_corner[i - 1] = if_corner_all[RoadID[start_index]];
                    if_croos_line[i - 1] = if_croos_line_all[RoadID[start_index]];
                    speedlimit[i - 1] = speed_limit_all[RoadID[start_index]];
                    traffic_sign[i - 1] = traffic_sign_all[RoadID[start_index]];
                    old_road_width = road_width[RoadID[start_index]];
                    double angle = new double();
                    if (i == 1)
                    {
                        angle = first_angle - Math.PI;
                    }
                    else
                    {
                        angle = getangle(JointIDCoor[objLongArray[i]].x - JointIDCoor[objLongArray[i - 1]].x, JointIDCoor[objLongArray[i]].y - JointIDCoor[objLongArray[i - 1]].y) * Math.PI / 180.0;
                    }
                   // NavigationPoint.TwoPointLine(old_endx, old_endy, JointIDCoor[iID].x, JointIDCoor[iID].y);
                    double distance_hemite =twopoints_distance(old_endx, old_endy, JointIDCoor[iID].x, JointIDCoor[iID].y);
                    NavigationPoint.hermite(old_endx, old_endy, JointIDCoor[iID].x, JointIDCoor[iID].y, Math.PI / 2.0 - Road_Angle[Number_Road_Pathplan - 2], Math.PI / 2.0 - Road_Angle[Number_Road_Pathplan - 1], distance_hemite * Hermite_Modulus);
                    old_endy = NavigationPoint.y_new[NavigationPoint.WholeNumberOfNavigationPoints - 1];
                    old_endx = NavigationPoint.x_new[NavigationPoint.WholeNumberOfNavigationPoints - 1];
                    IntersectionIndexID[i - 1].Index = NavigationPoint.WholeNumberOfNavigationPoints;
                    IntersectionIndexID[i - 1].IDStartPointOfIntersection = NavigationPoint.WholeNumberOfNavigationPoints;
                    IntersectionIndexID[i - 1].IDEndPointOfIntersection = NavigationPoint.WholeNumberOfNavigationPoints;
                    if (if_cross[iID - 1] == 1)
                    {
                        IntersectionIndexID[IntersectionFlag.WholeNumberIntersection].IndexStartPointOfIntersection = NavigationPoint.WholeNumberOfNavigationPoints;
                        IntersectionIndexID[IntersectionFlag.WholeNumberIntersection].IndexEndPointOfIntersection = NavigationPoint.WholeNumberOfNavigationPoints;
                        cross_status[IntersectionFlag.WholeNumberIntersection] = status[i - 1];
                        IntersectionFlag.WholeNumberIntersection++;
                        
                    }
                }

                //if(IntersectionFlag.WholeNumberIntersection==0)
                //{
                //    IntersectionFlag.WholeNumberIntersection = 1;
                //}
                
                string temp_string = "(3/5)已完成第" + (i).ToString() + "条路径生成，共" + objLongArray.Count + "条。\r\n";
                Init.String = temp_2string + temp_string;

            }
            NavigationPoint.TwoPointLine(old_endx, old_endy, RealP[PathPlanInfo.KeyPointNumber - 1].x, RealP[PathPlanInfo.KeyPointNumber - 1].y);//最后连接终点和最后一个路段
        }

        /***********************设置初始化后变量参数 *************************
         * Real清零
         * 关键点的个数KeyPointNumber清零
         * 清除SuperMap规划的所有导航点PathPlanMapRoadEndPoints.RemoveAll
         * 设置标志位
         * 退出线程
         * *******************************************************************/
        private void ParameterSetOfNavigationPoints(long Last_Group_EndPoint_index)
        {
            NavigationPoint.start_search_index = 0;
            NavigationPoint.end_search_index = NavigationPoint.WholeNumberOfNavigationPoints - 1;
            long TotalPointNumber = NavigationPoint.WholeNumberOfNavigationPoints;
            for (long i = Last_Group_EndPoint_index; i < TotalPointNumber; i++)
            {
                rd_points[i].x = NavigationPoint.x_new[i];
                rd_points[i].y = NavigationPoint.y_new[i];
                rd_points[i].changedirection = GOAHEAD;
                rd_points[i].speed = SPEEDMAX;
                rd_points[i].traffic = NO;
                rd_points[i].match_status = MATCHSTATUS_MATCHING; //比赛中，正常行驶 （默认值）
                rd_points[i].distance = 0.0;
                rd_points[i].lane_num = 0;
                rd_points[i].traffic_sign = NO;
                rd_points[i].if_corner = 0;
                rd_points[i].if_crossline = YES;  //能否并线，YES能
                rd_points[i].speed_limit = Convert.ToInt32(SPEEDMAX);

            }//为所有的路点赋初始属性
            //赋值车道线数量属性
            int TotalArrayCount = objLongArray.Count;
            for (int i = 0; i < TotalArrayCount; i++)
            {
                long number;
                if (i == 0)
                {
                    number = IntersectionIndexID[i].Index;
                    for (long j = Last_Group_EndPoint_index; j < number; j++)
                    {
                        rd_points[j].lane_num = lane_num[i];
                        rd_points[j].lane_info = lane_info[i];
                        rd_points[j].if_corner = if_corner[i];
                        rd_points[j].if_crossline = if_croos_line[i];
                        rd_points[j].speed_limit = speedlimit[i];
                        rd_points[j].traffic_sign = traffic_sign[i];
                        //当读入的限速值为0或太低时则设为最高速度
                        if (rd_points[j].speed_limit < 1.0)
                        {
                            rd_points[j].speed_limit = Convert.ToInt32(SPEEDMAX);
                        }
                    }
                }
                else
                {
                    number = IntersectionIndexID[i].Index - IntersectionIndexID[i - 1].Index;
                    long Index = IntersectionIndexID[i - 1].Index;
                    for (long j = 0; j < number; j++)
                    {
                        rd_points[j + Index].lane_num = lane_num[i];
                        rd_points[j + Index].lane_info = lane_info[i];
                        rd_points[j + Index].if_corner = if_corner[i];
                        rd_points[j + Index].if_crossline = if_croos_line[i];
                        rd_points[j + Index].speed_limit = speedlimit[i];
                        rd_points[j + Index].traffic_sign = traffic_sign[i];
                        //当读入的限速值为0或太低时则设为最高速度
                        if (rd_points[j + Index].speed_limit < 1.0)
                        {
                            rd_points[j +Index].speed_limit = Convert.ToInt32(SPEEDMAX);
                        }
                    }
                }

            }
            //将最后一个节点到终点的点的属性赋成最后一个路段的属性
            for (long j = IntersectionIndexID[TotalArrayCount - 1].Index; j < TotalPointNumber; j++)
            {
                rd_points[j].lane_num = lane_num[TotalArrayCount - 1];
                rd_points[j].lane_info = lane_info[TotalArrayCount - 1];
                rd_points[j].if_corner = if_corner[TotalArrayCount - 1];
                rd_points[j].if_crossline = if_croos_line[TotalArrayCount - 1];
                rd_points[j].speed_limit = speedlimit[TotalArrayCount - 1];
                rd_points[j].traffic_sign = traffic_sign[TotalArrayCount - 1];
                //当读入的限速值为0或太低时则设为最高速度
                if (rd_points[j].speed_limit < 1.0)
                {
                    rd_points[j].speed_limit = Convert.ToInt32(SPEEDMAX);
                }
            }
        }
        
        private void ParameterSetAfterINITSuccess()
        {
            //清理相关变量，为下一次初始化做准备
            int KeyPointNumber = PathPlanInfo.KeyPointNumber;
            for (int i = 0; i < KeyPointNumber; i++)
            {
                RealP[i].x = 0.0;
                RealP[i].y = 0.0;
            }

            PathPlanInfo.KeyPointNumber = 0;
            PathPlanMapRoadEndPoints.RemoveAll();

           
        }

        /*********************** 在地图上显示所有导航点 *************************/
        private void DisplayNavigationPointsOnMap()
        {   soGeoPoint temp1 = new soGeoPoint();

            //清除地图显示点
            clear();
            long TotalNumber = NavigationPoint.WholeNumberOfNavigationPoints;
            for (long lengh_point = 0; lengh_point < TotalNumber; lengh_point += 20)
            {
                temp1.x = rd_points[lengh_point].x;
                temp1.y = rd_points[lengh_point].y;
                //if (lengh_point % 50 == 0)
                //{
                //    show(temp1,lengh_point.ToString(),Color.Red);
                //}
                //else
                //{
                    if (rd_points[lengh_point].changedirection == GOAHEAD)
                    {
                        show(temp1, Color.Red);
                    }
                    else if (rd_points[lengh_point].changedirection == TURNRIGHT)
                    {
                        show(temp1, Color.Blue);
                    }
                    else
                    {
                        show(temp1, Color.Green);
                    }
                //}

            }
        }
        /*************** 交叉口和终点前转向、终点状态赋值、交通灯及距离参数设置 **************
         * 为每个导航点赋转向和距离信息，节点总个数=objLongArray.Count
         * 为交叉口前40米（100）的点设置“交叉口行驶标志”和“是否有交通信号灯”标志
         * 为所有导航点设置与其对应交叉口的距离
         * 计算与终点的距离，0.4m步长
         * *********************************************************************/
        private void ParameterSetBeforeTheNextInterseectionOrEndPoint(long Last_Group_EndPoint_index)
        {
            //为每个导航点赋转向和距离信息，节点总个数=objLongArray.Count
            //为交叉口前40米（100）的点设置“交叉口行驶标志”和“是否有交通信号灯”标志
            //为所有导航点设置与其对应交叉口的距离
            long rd_index = 0;
            long start_id = Last_Group_EndPoint_index;
            long last_intersection_index = 1;//上一个路口的序号1031
            int TotalCount = objLongArray.Count;
            for (int i = 1; i <= TotalCount; i++)
            {   //判断是否为交叉口
                if (if_cross[objLongArray[i] - 1] == (int)(YES))
                {   //100->40米，IntersectionIndexID[i - 1].Index ->交叉口中心点在全部路点中的序号                   
                    long number = IntersectionIndexID[i - 1].Index - Distance_Send_Turn_And_SuggestLane < 0 ? 0 : IntersectionIndexID[i - 1].Index - Distance_Send_Turn_And_SuggestLane;//1025
                    number = number > start_id ? number : start_id;//判断是否出了上一个路口，start_id保存的上个路口出弯点的序号1025
                    long IDEndPointOfIntersection = IntersectionIndexID[i - 1].IDEndPointOfIntersection;
                    for (rd_index = number; rd_index < IDEndPointOfIntersection; rd_index++)
                    {
                        //设置要求行驶车道线的序号
                        if (status[i - 1] == GOAHEAD && rd_points[rd_index].lane_info==255)
                        {
                            //所有路口处的建议车道处理为车道数除2加1，方便直行，若碰到左转或右转则在ParameterSetBeforeSendToUDP中处理
                            rd_points[rd_index].lane_info = rd_points[rd_index].lane_num / 2 + 1;
                        }
                        rd_points[rd_index].changedirection = status[i - 1];
                        rd_points[rd_index].traffic = traffic[i - 1];   //由地图数据获取是否有交通信号灯，1有，0无
                    }
                    long index_difference = IntersectionIndexID[i - 1].IDEndPointOfIntersection - IntersectionIndexID[last_intersection_index - 1].IDEndPointOfIntersection;//当前路口与上一个路口（出弯点）的序号差（为求距离）1031
                    if ((status[i - 1] == GOAHEAD) && (status[last_intersection_index - 1] == GOAHEAD) && (i > 1) && (index_difference<75))//如果连续两个路口都是直行路口并且两个路口的距离小于30米（阈值可改） 1031
                    {
                        for (long j = start_id; j < IDEndPointOfIntersection; j++)
                        {
                            rd_points[j].distance = 0.4 * (start_id - j);//负距离
                        }
                    }
                    else
                    {
                        //计算与交叉口中入弯点的距离，入弯与出弯之间的点距离为负值
                        long IDStartPointOfIntersection = IntersectionIndexID[i - 1].IDStartPointOfIntersection;
                        for (long j = start_id; j < IDEndPointOfIntersection; j++)
                        {
                            rd_points[j].distance = 0.4 * (IDStartPointOfIntersection - j);
                        }
                    }
                    start_id =IDEndPointOfIntersection;
                    last_intersection_index =i;//保存上一个路口的序号 1031
                }
            }
            
            //计算与终点的距离，0.4m步长
            //start_id目前为最后一个节点的出弯点序号，导航点的总数（也就是终点序号）NavigationPoint.WholeNumberOfNavigationPoints
            long TotalNumber = NavigationPoint.WholeNumberOfNavigationPoints;
            for (long i = start_id; i < TotalNumber; i++)
            {
                rd_points[i].distance = 0.4 * (TotalNumber - i);
            }
        }

        /**************************** 交通标识附近参数设置 *********************
         * 从导航点中搜索与交通标志对应点坐标最近点的序号->traffic_sign_index[sign_num]
         * 对【后退20米（-50）；前进40米（100）】内的所有点设置交通标志为YES
         * 窗口提示交通标志完成及其数量
         * *********************************************************************/
        private void ParameterSetAroundTrafficSign()
        {
            int sign_num = 0;//交通标志的数量
            long[] traffic_sign_index = new long[rpr.num + 1];//保存交通标志点在轨迹中的序号,rpr.num任务点的个数

            for (int i = 0; i < rpr.num; i++)
            {   
                if (rpr.TaskData[i].status2 == TASKPROPERTY_EXISTTRAFFICSIGN)
                {   //从导航点中搜索与交通标志对应点坐标最近点的序号->traffic_sign_index[sign_num]
                    NavigationPoint.start_search_index = 0;
                    NavigationPoint.end_search_index = NavigationPoint.WholeNumberOfNavigationPoints - 1;
                    NavigationPoint.find_send_points(rpr.TaskData[i].x, rpr.TaskData[i].y,2);//2代表全局找（不加航向匹配）
                    traffic_sign_index[sign_num] = NavigationPoint.present_point_index;                   
                    sign_num++;
                }

                if (rpr.TaskData[i].status1 == TASKPROPERTY_ENTERPARKPOINT)//1025
                {   //从导航点中搜索与交通标志对应点坐标最近点的序号->traffic_sign_index[sign_num]
                    NavigationPoint.start_search_index = 0;
                    NavigationPoint.end_search_index = NavigationPoint.WholeNumberOfNavigationPoints - 1;
                    NavigationPoint.find_send_points(rpr.TaskData[i].x, rpr.TaskData[i].y, 2);//2代表全局找（不加航向匹配）
                    NavigationPoint.Index_In_StopCarArea = NavigationPoint.present_point_index;//1025
                    Whether_Exist_StopCarArea = true;//1025
                }

                if (rpr.TaskData[i].status1 == TASKPROPERTY_OUTPARKPOINT)//1025
                {   //从导航点中搜索与交通标志对应点坐标最近点的序号->traffic_sign_index[sign_num]
                    NavigationPoint.start_search_index = 0;
                    NavigationPoint.end_search_index = NavigationPoint.WholeNumberOfNavigationPoints - 1;
                    NavigationPoint.find_send_points(rpr.TaskData[i].x, rpr.TaskData[i].y, 2);//2代表全局找（不加航向匹配）
                    NavigationPoint.Index_Out_StopCarArea = NavigationPoint.present_point_index;//1025
                    Whether_Exist_StopCarArea = true;//1025
                }
            }
            NavigationPoint.present_point_index = 0;
            NavigationPoint.start_search_index = 0;
            NavigationPoint.end_search_index = NavigationPoint.WholeNumberOfNavigationPoints - 1;
            for (int i = 0; i < sign_num; i++)
            {   //对【后退20米（-50）；前进40米（100）】内的所有点设置交通标志为YES
                for (int j = -50; j < 100; j++)
                {
                    if ((traffic_sign_index[i] + j > 0) && (rd_points[traffic_sign_index[i] + j].traffic_sign==0))
                    {
                        rd_points[traffic_sign_index[i] + j].traffic_sign = YES;

                    }
                }
            }
            if (Whether_Exist_StopCarArea)//1025
            {
                for (long i = NavigationPoint.Index_In_StopCarArea - Distance_StopCar_Send; i < NavigationPoint.Index_Out_StopCarArea + Distance_StopCar_Send; i++)//侧方停车前后Distance_StopCar_Send米发送Sign_Side_StopCar 1025
                {
                    if ((i >= 0) && (i < NavigationPoint.WholeNumberOfNavigationPoints))//1025
                    {
                        rd_points[i].traffic_sign = Sign_Side_StopCar;
                    }
                }
            }
                //窗口提示交通标志完成及其数量
                Init.String += "(4/5)赋值交通标志完成，共有" + sign_num + "个交通标志。。。\r\n";
        }
        private void SetSpeed(double min_speed, long IndexStartPointOfIntersection, long IndexEndPointOfIntersection)
        {
            for (long j = IndexStartPointOfIntersection - 750; j < IndexStartPointOfIntersection; j++)
            {
                if (j >= 0)
                {
                    //速度赋值： 距离路口100米至入弯点
                    double speed = rd_points[j].speed;
                    if (rd_points[j].speed > rd_points[j].speed_limit)
                    {
                        speed = rd_points[j].speed_limit;
                        rd_points[j].speed = speed;
                    }

                    if (speed > (min_speed + ((SPEEDMAX - min_speed) / 750.0) * (IndexStartPointOfIntersection - j)))
                    {
                        rd_points[j].speed = min_speed + ((SPEEDMAX - min_speed) / 750.0) * (IndexStartPointOfIntersection - j);
                        if (rd_points[j].speed > rd_points[j].speed_limit)
                        {
                            rd_points[j].speed = rd_points[j].speed_limit;
                        }
                    }
                }
            }
            //速度赋值： 入弯点至出弯点
            for (long j = IndexStartPointOfIntersection; j < IndexEndPointOfIntersection; j++)
            {

                rd_points[j].speed = min_speed;
                if (rd_points[j].speed > rd_points[j].speed_limit)
                {
                    rd_points[j].speed = rd_points[j].speed_limit;
                }
            }

            //速度赋值： 出弯点至离开路口60米 150个点*0.4=60米
            for (long j = IndexEndPointOfIntersection; j < IndexEndPointOfIntersection + 150; j++)
            {
                double speed = rd_points[j].speed;
                if (rd_points[j].speed > rd_points[j].speed_limit)
                {
                    speed = rd_points[j].speed_limit;
                }
                if (speed > min_speed + (SPEEDMAX - min_speed) / 150.0 * (j - IndexEndPointOfIntersection))
                {
                    rd_points[j].speed = min_speed + (SPEEDMAX - min_speed) / 150.0 * (j - IndexEndPointOfIntersection);
                    if (rd_points[j].speed > rd_points[j].speed_limit)
                    {
                        rd_points[j].speed = rd_points[j].speed_limit;
                    }
                }
            }
        }
        /*********************** 距离路口100米时设置相关参数 **************************
         100米计算： 路口向后250个点，两点间距0.4米，250->0.4*250=100米
         * 速度：100米内最高时速“均”减至10km/h，说明路口点行驶速度为10km/h，目前最高时速为40km/h
         * 要求行驶车道线的序号：根据交叉口行驶标志status设置序号
         * ****************************************************************************/

        private void ParameterSet100mBeforeIntersection()
        {
            int WholeNumberIntersection = IntersectionFlag.WholeNumberIntersection;
            for (int i = 0; i < WholeNumberIntersection; i++)
            {
                long IndexStartPointOfIntersection = IntersectionIndexID[i].IndexStartPointOfIntersection;
                long IndexEndPointOfIntersection = IntersectionIndexID[i].IndexEndPointOfIntersection;
                if (rd_points[IndexStartPointOfIntersection].changedirection == GOAHEAD)
                {
                    SetSpeed(30.0, IndexStartPointOfIntersection, IndexEndPointOfIntersection);
                }
                //距离路口100米至入弯点，对速度和要求行驶车道线的序号进行赋值

                else
                {
                    if (cross_status[i] == TURNLEFT && i < WholeNumberIntersection - 1 && cross_status[i + 1] == TURNLEFT)
                    {
                        SetSpeed(5.0, IndexStartPointOfIntersection, IndexEndPointOfIntersection);
                        i++;
                    }
                    else
                    {
                        SetSpeed(15.0, IndexStartPointOfIntersection, IndexEndPointOfIntersection);
                    }
                }
            }
        }
        /************************** 终点前100米设置相关参数 **************************
             100米计算： 终点向后250个点，两点间距0.4米，250->0.4*250=100米             
             * 并线：提前40米禁止并线（命令码：0x12，X7：并线标志，0 不能并线  1 （默认）能并线
             * 速度：100米内最高时速“均”减至10km/h，说明路口点行驶速度为10km/h，目前最高时速为40km/h
             * 终点前40米，更改比赛状态为“比赛即将结束”MATCHSTATUS_OVERMATCH
             * ****************************************************************************/
        private void ParameterSetEndPoint()
        {
            for (long rd_length = NavigationPoint.WholeNumberOfNavigationPoints - 250; rd_length < NavigationPoint.WholeNumberOfNavigationPoints; rd_length++)
            {   //距离终点40米，并且已过最后一个交叉口的情况下，设置禁止并线标志
                if (rd_length >= 0 && IntersectionFlag.WholeNumberIntersection>0)
                {
                    if((rd_length > IntersectionIndexID[IntersectionFlag.WholeNumberIntersection - 1].IndexEndPointOfIntersection))//更改判断所有交叉口，不单单是左拐右拐的交叉口
                    {   
                        rd_points[rd_length].if_crossline = NO;  //能否并线，NO不能
                        //终点前40米，更改比赛状态为“比赛即将结束”MATCHSTATUS_OVERMATCH
                        rd_points[rd_length].match_status = MATCHSTATUS_OVERMATCH; //比赛即将结束
                    }

                    double speed = rd_points[rd_length].speed;
                    if (rd_points[rd_length].speed > rd_points[rd_length].speed_limit)
                    {
                        speed = rd_points[rd_length].speed_limit;
                    }
                    if (speed > (((SPEEDMAX - SPEEDMAXBEFORESTOP) / 250.0) * (NavigationPoint.WholeNumberOfNavigationPoints - rd_length) + SPEEDMAXBEFORESTOP))
                    {
                        rd_points[rd_length].speed = ((SPEEDMAX - SPEEDMAXBEFORESTOP) / 250.0) * (NavigationPoint.WholeNumberOfNavigationPoints - rd_length) + SPEEDMAXBEFORESTOP;
                        if (rd_points[rd_length].speed > rd_points[rd_length].speed_limit)
                        {
                            rd_points[rd_length].speed = rd_points[rd_length].speed_limit;
                        }
                    }
                }
            }
        }

        /************************** 出发点的参数设置 **************************
             40米计算： 出发点后100个点，两点间距0.4米             
             * 并线：不对其进行特别设置，  1 （默认）能并线
             * 速度：出发点40米加速至最低限速度，目前为10km/h
                     40米至80米加速至最高速度，目前为40km/h             * 
             * ****************************************************************************/
        private void ParameterSetStartPoint()
        {   //出发点40米加速至最低限速度，目前为10km/
            for (long speed_length = 0; speed_length < 100; speed_length++)
            {
                rd_points[speed_length].speed = SPEEDMIN;
                if (rd_points[speed_length].speed > rd_points[speed_length].speed_limit)
                {
                    rd_points[speed_length].speed = rd_points[speed_length].speed_limit;
                }
            }
            //40米至80米加速至最高速度，目前为40km/h
            for (long speed_length = 100; speed_length < 300; speed_length++)
            {
                double speed = rd_points[speed_length].speed;
                if (rd_points[speed_length].speed > rd_points[speed_length].speed_limit)
                {
                    speed = rd_points[speed_length].speed_limit;
                }
                if (speed > SPEEDMIN + (SPEEDMAX - SPEEDMIN) / 200.0 * (speed_length - 100))
                {
                    rd_points[speed_length].speed = SPEEDMIN + (SPEEDMAX - SPEEDMIN) / 200.0 * (speed_length - 100);
                    if (rd_points[speed_length].speed > rd_points[speed_length].speed_limit)
                    {
                        rd_points[speed_length].speed = rd_points[speed_length].speed_limit;
                    }
                }
            }
        }

        //初始化函数
        private void button7_Click(object sender, EventArgs e)
        {
            if (INITThread.ThreadState == System.Threading.ThreadState.Running)
            {
                MessageBox.Show("INIT Running...");
            }
            else
            {
                if (blnOpen && rpr.num != 0)
                {
                    INITThread = new System.Threading.Thread(new System.Threading.ParameterizedThreadStart(INIT));
                    INITThread.Start();
                }
                else
                {
                    MessageBox.Show("未打开工作空间或未读入任务点");
                }
            }
        }
        private double getdistance(rd_point rp1, rd_point rp2)
        {
            return (Math.Sqrt((rp1.x - rp2.x) * (rp1.x - rp2.x) + (rp1.y - rp2.y) * (rp1.y - rp2.y)));
        }

        //通过方向角的改变大小来确定路点的速度
        private double getspeed(long i)
        {
            if (i < NavigationPoint.WholeNumberOfNavigationPoints - 200)
            {
                double future_angle = getangle(NavigationPoint.x_new[i + 200] - NavigationPoint.x_new[i + 100], NavigationPoint.y_new[i + 200] - NavigationPoint.y_new[i + 100]);
                double now_angle = getangle(NavigationPoint.x_new[i + 1] - NavigationPoint.x_new[i], NavigationPoint.y_new[i + 1] - NavigationPoint.y_new[i]);
                double detathta_speed = future_angle - now_angle;
                if (detathta_speed > 180.0)
                {
                    detathta_speed -= 360.0;
                }
                else if (detathta_speed < -180.0)
                {
                    detathta_speed += 360.0;
                }
                if (Math.Abs(detathta_speed) > 90)
                {
                    detathta_speed = 90;
                }
                double speed = (40 - Math.Abs(detathta_speed) * 0.33);
                return (speed);
            }
            else
            {
                return 40.0;
            }
        }

        //在地图上显示
        public void show(soPoint objgeo, Color c)
        {
            soGeoPoint temp = new soGeoPoint();
            temp.x = objgeo.x;
            temp.y = objgeo.y;
            if (!mf.IsDisposed)
            {
                soTrackingLayer objTL = mf.axSuperMap1.TrackingLayer;
                soStyle objstyle = new soStyle();
                objstyle.PenColor = (uint)ColorTranslator.ToOle(c);
                objstyle.SymbolSize = 40;
                objTL.AddEvent((soGeometry)temp, objstyle, "");
                objTL.Refresh();
            }

        }
        public void show(transfer._soPoint objgeo, Color c)
        {
            soGeoPoint temp = new soGeoPoint();
            temp.x = objgeo.x;
            temp.y = objgeo.y;
            if (!mf.IsDisposed)
            {
                soTrackingLayer objTL = mf.axSuperMap1.TrackingLayer;
                soStyle objstyle = new soStyle();
                objstyle.PenColor = (uint)ColorTranslator.ToOle(c);
                objstyle.SymbolSize = 100;
                objTL.AddEvent((soGeometry)temp, objstyle, "");
                objTL.Refresh();
            }

        }
        public void show(soGeoPoint objgeo, Color c)
        {
            if (!mf.IsDisposed)
            {
                soTrackingLayer objTL = mf.axSuperMap1.TrackingLayer;
                soStyle objstyle = new soStyle();
                objstyle.PenColor = (uint)ColorTranslator.ToOle(c);
                objstyle.SymbolSize = 30;
                objTL.AddEvent((soGeometry)objgeo, objstyle, "");
                objTL.Refresh();
            }

        }
        public void show(soGeoPoint objgeo, Color c, int size)
        {
            if (!mf.IsDisposed)
            {
                soTrackingLayer objTL = mf.axSuperMap1.TrackingLayer;
                soStyle objstyle = new soStyle();
                objstyle.PenColor = (uint)ColorTranslator.ToOle(c);
                objstyle.SymbolSize = size;
                objTL.AddEvent((soGeometry)objgeo, objstyle, "");
                objTL.Refresh();
            }

        }
        public string workspace_path = string.Empty;
        //打开工作空间
        RoadModel r_hermite = new RoadModel();
        private void bt_chooseP_Click(object sender, EventArgs e)
        {   //bit0=0
            TroubleCode &= 0x0fffffffe;

            string strOpenPath = string.Empty;
            soError objError = null;
            this.openFileDialog1.Title = "打开工作空间";
            this.openFileDialog1.InitialDirectory = @"..\..\..\..\..\SampleData\World";
            this.openFileDialog1.FileName = "";
            this.openFileDialog1.Filter = "工作空间(.smw)|*.smw";
            if (this.openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                strOpenPath = this.openFileDialog1.FileName;
                this.ConnectSuperMap();

                blnOpen = this.axSuperWorkspace1.Open(strOpenPath, "");
                wks_name = axSuperWorkspace1.Datasources[1].Alias;//保存数据源的别名
                workspace_path = strOpenPath;
                if (!blnOpen)
                {
                    TroubleCode |= 0x0001;  //bit0=1
                    objError = new soError();
                    MessageBox.Show("打开工作空间失败" + objError.LastErrorMsg);
                    return;
                }
                try
                {
                    int index = workspace_path.LastIndexOf("\\");
                    string Map_shift_file_path = workspace_path.Substring(0, index + 1) + "Map_shift.txt";
                    if (File.Exists(Map_shift_file_path))
                    {
                        FileStream fs = new FileStream(Map_shift_file_path, FileMode.Open, FileAccess.Read);
                        StreamReader sr = new StreamReader(fs);
                        string error = sr.ReadLine();
                        if (error == "" || error == null)
                        {
                            MessageBox.Show("文件为空");
                        }
                        else
                        {
                            string[] errors = error.Split(',');
                            MapCoordinateAllignment.x = Convert.ToDouble(errors[0]);
                            MapCoordinateAllignment.y = Convert.ToDouble(errors[1]);
                            sr.Close();
                            fs.Close();
                        }
                    }
                    else
                    {
                        MessageBox.Show("文件不存在");
                    }
                    string task_file_path = workspace_path.Substring(0, index + 1) + "TaskPoint.txt";
                    if (File.Exists(task_file_path))
                    {
                        FileStream fs_hermite = new FileStream(task_file_path, FileMode.Open, FileAccess.Read);
                        StreamReader sr_hermite = new StreamReader(fs_hermite);
                        string temp = sr_hermite.ReadLine();
                        while (temp != null && temp != "")
                        {
                            string[] string_array = temp.Split(',');
                            r_hermite.TaskPointX[r_hermite.Task_Num] = Convert.ToDouble(string_array[0]);
                            r_hermite.TaskPointY[r_hermite.Task_Num] = Convert.ToDouble(string_array[1]);
                            r_hermite.Task_Num++;
                            temp = sr_hermite.ReadLine();

                        }
                        sr_hermite.Close();
                        fs_hermite.Close();
                    }
                    else
                    {
                        MessageBox.Show("文件不存在");
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.ToString());
                }
               
            }
            else
            {
                return;
            }
        }

        /************************ 处理接收的UDP数据 *************************
         * 看门狗复位、重启
         * 判断是否接收到最后一帧数据（0307）03-GIS ID; 07 -命令码(最后一包数据)
         * 根据经纬度判断数据是否有效
         * 判断数据是否有跳变
         * ******************************************************************/
       
        //public void Backcar_data_OnRefresh(string str)//接收数据
        //{
        //    DataFromUDP.ID = UDP_2_Backcar.Data[1];
        //    DataFromUDP.back_car_x = UDP_2_Backcar.Data[2] / 100.0;
        //    DataFromUDP.back_car_y = UDP_2_Backcar.Data[3] / 100.0;
        //}
       public int Count_Align = 0;//1026
        /************************ 处理接收的UDP数据 *************************
         * 对当前接收坐标点进行地图匹配：输出当前点在地图数据中的序号present_point_index
         * ******************************************************************/
        void ParameterSetBeforeSendToUDP()
        {
            int WholeNumberIntersection;

            DataToUDPExcludeRoadPoints.PresentTime = cros.msg_ins.second;
            transfer._soPoint temp = new transfer._soPoint();//用于计算路点字符串

            NavigationPoint.azimuth = DataFromUDP.back_azimuth;
            r_hermite.azimuth = DataFromUDP.back_azimuth;
            PresentPoint.x = cros.msg_ins.position.x;
            PresentPoint.y = cros.msg_ins.position.y;
            //地图匹配：在地图上搜索寻当前点位置
            //输出当前点在地图数据中的序号present_point_index,延路方向前后各120米起始点和终止点的序号
            //判断惯导接收的点与之前匹配的路点之间的距离，若距离大于一定数值则全局搜索，否则按序号进行搜索

            double Distance_Between_GPS_And_MatchPoint = twopoints_distance(PresentPoint.x, PresentPoint.y, rd_points[NavigationPoint.present_point_index].x, rd_points[NavigationPoint.present_point_index].y);

            if ((Distance_Between_GPS_And_MatchPoint > Distance_Global) || (tf.start_index > tf.end_index - 5))//若当前点与匹配点的距离大于Distance_Global 1028
            {
                //MapCoordinateAllignment.x += rd_points[NavigationPoint.present_point_index].x - PresentPoint.x;//1027 强制将当前点赋值为上一个正确值的匹配点
                //MapCoordinateAllignment.y += rd_points[NavigationPoint.present_point_index].y - PresentPoint.y;//1027

                NavigationPoint.start_search_index = 0;
                NavigationPoint.end_search_index = NavigationPoint.WholeNumberOfNavigationPoints - 1;
                NavigationPoint.find_send_points(PresentPoint.x, PresentPoint.y, 2);//1代表全局找+航向匹配
                r_hermite.start_search_index = 0;
                r_hermite.end_search_index = r_hermite.WholeNumberOfNavigationPoints - 1;
                r_hermite.find_send_points(PresentPoint.x, PresentPoint.y, 2);//1代表全局找+航向匹配
            }
            else
            {
                NavigationPoint.find_send_points(PresentPoint.x, PresentPoint.y, 0);//0代表按序号进行搜索
                r_hermite.find_send_points(PresentPoint.x, PresentPoint.y, 0);
            }

            //计算匹配坐标与当前点坐标之间的误差
            DataToUDPErrorOfMapMatch.x = 0;
            DataToUDPErrorOfMapMatch.y = 0;

            
                temp.x = PresentPoint.x;
                temp.y = PresentPoint.y;
                if (Task_Plan_flag)
                {
                    tf.ComputeNavigationPointCoordinateToBeSent(temp, NavigationPoint, DataFromUDP.back_azimuth);
                }

                else
                {
                    //获取得301个要发送的导航点数据
                    tf.ComputeNavigationPointCoordinateToBeSent(temp, r_hermite, DataFromUDP.back_azimuth);
                }

            //小于16米的情况下则在所有规划的导航点中搜索当前点
            if (tf.start_index > tf.end_index - 40)
            {
                NavigationPoint.start_search_index = 0;
                NavigationPoint.end_search_index = NavigationPoint.WholeNumberOfNavigationPoints - 1;
                r_hermite.start_search_index = 0;
                r_hermite.end_search_index = r_hermite.WholeNumberOfNavigationPoints - 1;
                return;
            }
            //计算当前点与后面第10个点连线的方位角，作为车道线的方向角
            double lane_angle = getangle(10);
            DataToUDPExcludeRoadPoints.AngleHeadingToLane = lane_angle - DataFromUDP.back_azimuth;//得到车道线与航向的夹角

            //将角度范围限制在-180到+180度之间
            if (DataToUDPExcludeRoadPoints.AngleHeadingToLane > 180.0)
            {
                DataToUDPExcludeRoadPoints.AngleHeadingToLane -= 360.0;
            }
            else if (DataToUDPExcludeRoadPoints.AngleHeadingToLane < -180.0)
            {
                DataToUDPExcludeRoadPoints.AngleHeadingToLane += 360.0;
            }
            //有信号灯的情况
            if (rd_points[NavigationPoint.present_point_index].traffic == YES)
            {
                if (rd_points[NavigationPoint.present_point_index].changedirection == TURNRIGHT)
                {
                    RunningStatus.RunControlWithTrafficLight = TURNRIGHT_TRAFFICLIGHT;
                }
                else if (rd_points[NavigationPoint.present_point_index].changedirection == TURNLEFT)
                {
                    RunningStatus.RunControlWithTrafficLight = TURNLEFT_TRAFFICLIGHT;
                }
                else
                {
                    RunningStatus.RunControlWithTrafficLight = GOAHEAD_TRAFFICLIGHT;
                }
            }
            else
            {
                if (rd_points[NavigationPoint.present_point_index].changedirection == TURNRIGHT)
                {
                    RunningStatus.RunControlWithTrafficLight = TURNRIGHT;
                }
                else if (rd_points[NavigationPoint.present_point_index].changedirection == TURNLEFT)
                {
                    RunningStatus.RunControlWithTrafficLight = TURNLEFT;
                }
                else
                {
                    RunningStatus.RunControlWithTrafficLight = GOAHEAD;
                }
            }
            //赋值路点状态
            for (int i = 0; i < IntersectionFlag.WholeNumberIntersection; i++)
            {
                if (NavigationPoint.start_search_index + tf.end_index > IntersectionIndexID[i].IndexEndPointOfIntersection)
                {
                    if (NavigationPoint.start_search_index + tf.start_index < IntersectionIndexID[i].IndexEndPointOfIntersection)
                    {
                        IntersectionFlag.IndexCrossingIntersection = i;
                        break;
                    }
                }
                else
                {
                    if (NavigationPoint.start_search_index + tf.end_index > IntersectionIndexID[i].IndexStartPointOfIntersection && NavigationPoint.start_search_index + tf.start_index < IntersectionIndexID[i].IndexStartPointOfIntersection)
                    {
                        IntersectionFlag.IndexCrossingIntersection = i;
                    }
                }

            }

            WholeNumberIntersection = IntersectionFlag.IndexCrossingIntersection;
            IntersectionFlag.CharacterOfEnterCurve = NO;
            IntersectionFlag.CharacterOfOutCurve = NO;
            IntersectionFlag.IndexStartPointInCurrentNavigationDataFrame = 0;
            IntersectionFlag.IndexEndPointInCurrentNavigationDataFrame = 0;

            if (NavigationPoint.start_search_index + tf.end_index > IntersectionIndexID[WholeNumberIntersection].IndexStartPointOfIntersection && NavigationPoint.start_search_index + tf.end_index < IntersectionIndexID[WholeNumberIntersection].IndexEndPointOfIntersection)
            {
                if (NavigationPoint.start_search_index + tf.start_index < IntersectionIndexID[WholeNumberIntersection].IndexStartPointOfIntersection)
                {
                    IntersectionFlag.CharacterOfEnterCurve = YES;
                    IntersectionFlag.CharacterOfOutCurve = NO;
                    IntersectionFlag.IndexStartPointInCurrentNavigationDataFrame = Convert.ToInt32(IntersectionIndexID[WholeNumberIntersection].IndexStartPointOfIntersection - (NavigationPoint.start_search_index + tf.start_index));
                }
                else
                {
                    IntersectionFlag.CharacterOfEnterCurve = NO;
                    IntersectionFlag.CharacterOfOutCurve = NO;
                }
            }
            else if (NavigationPoint.start_search_index + tf.end_index > IntersectionIndexID[WholeNumberIntersection].IndexEndPointOfIntersection)
            {
                if (NavigationPoint.start_search_index + tf.start_index < IntersectionIndexID[WholeNumberIntersection].IndexStartPointOfIntersection)
                {
                    IntersectionFlag.CharacterOfEnterCurve = YES;
                    IntersectionFlag.CharacterOfOutCurve = YES;
                    IntersectionFlag.IndexStartPointInCurrentNavigationDataFrame = Convert.ToInt32(IntersectionIndexID[WholeNumberIntersection].IndexStartPointOfIntersection - (NavigationPoint.start_search_index + tf.start_index));
                    IntersectionFlag.IndexEndPointInCurrentNavigationDataFrame = Convert.ToInt32(IntersectionIndexID[WholeNumberIntersection].IndexEndPointOfIntersection - (NavigationPoint.start_search_index + tf.start_index));
                }
                else if (NavigationPoint.start_search_index + tf.start_index > IntersectionIndexID[WholeNumberIntersection].IndexStartPointOfIntersection && NavigationPoint.start_search_index + tf.start_index < IntersectionIndexID[WholeNumberIntersection].IndexEndPointOfIntersection)
                {
                    IntersectionFlag.CharacterOfEnterCurve = NO;
                    IntersectionFlag.CharacterOfOutCurve = YES;
                    IntersectionFlag.IndexEndPointInCurrentNavigationDataFrame = Convert.ToInt32(IntersectionIndexID[WholeNumberIntersection].IndexEndPointOfIntersection - (NavigationPoint.start_search_index + tf.start_index));
                }
                else
                {
                    IntersectionFlag.CharacterOfEnterCurve = NO;
                    IntersectionFlag.CharacterOfOutCurve = NO;
                }
            }

            //当并线标志为2时，说明有动态目标通过，置拐弯处入点、出点标志为DYNAMICOBJECTPASS
            if (rd_points[NavigationPoint.present_point_index].if_crossline == DYNAMICOBJECTPASS)
            {
                IntersectionFlag.CharacterOfEnterCurve = DYNAMICOBJECTPASS;
                IntersectionFlag.CharacterOfOutCurve = DYNAMICOBJECTPASS;
            }

            //当前点所在道路的车道线数量，若数据库中数量大于10则认为无效，置为无效数量：0xFF
            DataToUDPExcludeRoadPoints.LaneNumber = rd_points[NavigationPoint.present_point_index].lane_num;
            //建议车辆在当前点所处道路上应该行驶的车道序号，内侧为1，若车道线数量大于10或者车道序号大于车道线数量，则认为无效，置为无效数量：0xFF
            if (rd_points[NavigationPoint.present_point_index].lane_info == 255)
            {
                switch (rd_points[NavigationPoint.present_point_index].changedirection)
                {

                    case TURNLEFT:
                        DataToUDPExcludeRoadPoints.IndexLaneRunSuggested = 1;
                        break;
                    case TURNRIGHT:
                        DataToUDPExcludeRoadPoints.IndexLaneRunSuggested = rd_points[NavigationPoint.present_point_index].lane_num;
                        break;
                    default:
                        DataToUDPExcludeRoadPoints.IndexLaneRunSuggested = rd_points[NavigationPoint.present_point_index].lane_info;
                        break;
                }
            }
            else
            {
                DataToUDPExcludeRoadPoints.IndexLaneRunSuggested = rd_points[NavigationPoint.present_point_index].lane_info;
            }


            if (rd_points[NavigationPoint.present_point_index].lane_num > 10)
            {
                DataToUDPExcludeRoadPoints.LaneNumber = 0xF;
                DataToUDPExcludeRoadPoints.IndexLaneRunSuggested = 0xFF;
            }
            if (DataToUDPExcludeRoadPoints.IndexLaneRunSuggested > DataToUDPExcludeRoadPoints.LaneNumber)
            {
                DataToUDPExcludeRoadPoints.IndexLaneRunSuggested = 0xFF;
            }
            //当前位置的交通标志的代号
            RunningStatus.FlagWhetherTrafficSign = rd_points[NavigationPoint.present_point_index].traffic_sign;
            //若当前点距路口大于200米则赋值为200米，原因是因为发送距离的字节数不够
            if (rd_points[NavigationPoint.present_point_index].distance > 200.0)//1024
            {
                rd_points[NavigationPoint.present_point_index].distance = 200.0;
            }
            //赋距离信息
            RunningStatus.DistanceToNextTrafficLightOrEndPoint = rd_points[NavigationPoint.present_point_index].distance;//距离信息

            //判断是否有手工加入交通标志，若有则限速为最低速，设置交通标志位
            if (rd_points[NavigationPoint.present_point_index].if_corner == TRAFFICSIGN_MANULLY)
            {
                rd_points[NavigationPoint.present_point_index].speed = SPEEDMIN;
                rd_points[NavigationPoint.present_point_index].traffic_sign = YES;
            }
            //赋值比赛状态
            if ((NavigationPoint.present_point_index < 50) && (rd_points[NavigationPoint.present_point_index].speed > 0))
            {
                RunningStatus.MatchStatus = MATCHSTATUS_BEFOREMATCH;//起点之后20米赋值状态为比赛前，用来检测交通灯
            }
            else
            {
                RunningStatus.MatchStatus = rd_points[NavigationPoint.present_point_index].match_status;//比赛状态
            }

            //设置并线标志
            RunningStatus.ChangeLane = rd_points[NavigationPoint.present_point_index].if_crossline;

            if (rd_points[NavigationPoint.present_point_index].speed > rd_points[NavigationPoint.present_point_index].speed_limit)
            {
                rd_points[NavigationPoint.present_point_index].speed = rd_points[NavigationPoint.present_point_index].speed_limit;
            }

            //设置无路网标志
            DataToUDPExcludeRoadPoints.FlagOfRoadNet = rd_points[NavigationPoint.present_point_index].Flag_OF_NoRoadNet;

        }
        /************************ 处理接收的UDP数据 *************************
         * 看门狗复位、重启
         * 判断是否接收到最后一帧数据（0307）03-GIS ID; 07 -最后一包数据
         * ******************************************************************/
       

        public void clear()
        {
            if (!mf.IsDisposed)
            {
                soTrackingLayer objTL = mf.axSuperMap1.TrackingLayer;
                objTL.ClearEvents();
            }
        }
        //debug
       // int pan = 0;
        //DrawForm df = new DrawForm();
        //public void AddMapXY(soGeoPoint PresentP)//debug
        //{
        //    PresentP.x = PresentP.x + MapCoordinateAllignment.x;
        //    PresentP.y = PresentP.y + MapCoordinateAllignment.y;
        //}
        long send_ID = 0;
        /************************** 禁止通行 *****************************************
         * 以当前点坐标匹配路段，得到当前所在道路的ID，然后与下一条道路进行判断，
         * 若两者的角度相同，则将下一条道路禁行             
        * ****************************************************************************/
        private bool ForbidPassNextRoad()
        {
            soGeoPoint temp_current = new soGeoPoint();
            soDatasetVector dlwl = (soDatasetVector)this.axSuperWorkspace1.Datasources[wks_name].Datasets["道路网络"];
            soRecordset objRds = null;
            soSelection objselction;
            soRecordset RS_connected_road;
            bool Forbid_Flag = false;
            soNetworkAnalystEx objNetworkAnalyst = null;
            objNetworkAnalyst = this.axSuperAnalyst1.NetworkAnalyst;
            temp_current.x = cros.msg_ins.position.x;
            temp_current.y = cros.msg_ins.position.y;
            int Road_ID_ForbidPass;//存放要禁行的路段ID
            int Node_ID_ForbidPass;//存放要禁行路段的起始节点ID
            int Road_Num_Of_Node;//节点关联的道路数量
            string strSQLFilter = "";
            match(temp_current,50);
            for (int i = 0; i < Number_Road_Pathplan; i++)
            {
                //寻找节点相关联的道路数量
                Node_ID_ForbidPass = Node_ID_Queue[i];//将禁行路段的起始节点的ID提取出来
                if (Node_ID_ForbidPass == 0)
                {
                    continue;
                }
                objselction = objNetworkAnalyst1.FindConnectedEdges(dlwl, Node_ID_ForbidPass, sePathFindingDirection.scdBoth, 1);
                RS_connected_road = objselction.ToRecordset(false);
                Road_Num_Of_Node = RS_connected_road.RecordCount;

                //若找到对应ID的路段，并且此路段的角度与下一路段的角度相同，并且节点关联的道路数量大于两条，并且与禁行路口的距离小于100米（防止距离很远的时候误检，把下个路口禁行）
                if ((Road_ID_Match == Road_ID_Queue[i]) && (Math.Abs(Road_Angle[i] - Road_Angle[i + 1]) < (Angle_Forbid_Road * Math.PI / 180.0)) && (Road_Num_Of_Node > 2) && (rd_points[NavigationPoint.present_point_index].distance < ForbidPass_ErrorDetection_Distance))
                {
                    Road_ID_ForbidPass = Road_ID_Queue[i + 1];//将禁行路段的ID提取出来
                    int Road_ID_BeforeForbidPass = Road_ID_Queue[i];
                    strSQLFilter = " SmID=" + Road_ID_ForbidPass.ToString();
                    objRds = dlwl.Query(strSQLFilter, true, null, "");
                    string F = objRds.GetFieldValue("SmFNode").ToString();
                    string T = objRds.GetFieldValue("SmTNode").ToString();
                    int f_node = Convert.ToInt32(F);
                    int t_node = Convert.ToInt32(T);
                    objNetworkAnalyst.UpdateEdgeWeight(Road_ID_ForbidPass, f_node, t_node, 9999.0);
                    objNetworkAnalyst.UpdateEdgeWeight(Road_ID_ForbidPass, t_node, f_node, 9999.0);

                    //将当前匹配道路的权值置为负数，使得规划路径一定过当前道路
                    strSQLFilter = " SmID=" + Road_ID_BeforeForbidPass.ToString();
                    objRds = dlwl.Query(strSQLFilter, true, null, "");
                    F = objRds.GetFieldValue("SmFNode").ToString();
                    T = objRds.GetFieldValue("SmTNode").ToString();
                    f_node = Convert.ToInt32(F);
                    t_node = Convert.ToInt32(T);
                    objNetworkAnalyst.UpdateEdgeWeight(Road_ID_BeforeForbidPass, f_node, t_node, 1);
                    objNetworkAnalyst.UpdateEdgeWeight(Road_ID_BeforeForbidPass, t_node, f_node, 1);
                    Forbid_Flag = true;
                    return Forbid_Flag;
                }
            }
            return Forbid_Flag;
        }
        RoadModel r_log = new RoadModel();
        long back_car_index = 0;
        bool Back_car_flag = false;
        long show_index = 0;
        private string GetInsStatus(int status)
        {
            string ins_status="";
            switch(status)
            {
                case 1:
                    ins_status = "NARROW_INT";
                    break;
                case 2:
                    ins_status = "SINGLE";
                    break;
                default:
                    ins_status = "PSRDIFF";
                    break;
            }
            return ins_status;
        }
        private void TimerDisplay100ms_Tick(object sender, EventArgs e)
        {
            if (!(cros.msg_ins_ready && cros.msg_veh_ready))
            {
                return;
            }
            label6.Text = cros.msg_ins.second.ToString("F2");
            clear();
            PresentPoint.x = cros.msg_ins.position.x;
            PresentPoint.y = cros.msg_ins.position.y;
            label2.Text = PresentPoint.x.ToString("F2");
            label8.Text = PresentPoint.y.ToString("F2");
            ////显示当前点           
            if (!mf.IsDisposed && show_index > 100)
            {
                mf.axSuperMap1.CenterX = DataFromUDP.back_car_x;
                mf.axSuperMap1.CenterY = DataFromUDP.back_car_y;
                show_index = 0;
            }
            show_index++;
            label11.Text = DataToUDPExcludeRoadPoints.AngleHeadingToLane.ToString("F2");
            label10.Text = cros.msg_ins.attitude.z.ToString("F2");
            bn_initial.Enabled = !Init.FlagIsBusy;
            if (!Init.FlagInitSuccess)
            {
                if (Init.String == "")
                {
                    Console.WriteLine(Init.String);
                }
                else
                {
                    Console.WriteLine("INIT...");
                    textBox1.Text = Init.String;
                }
                Console.WriteLine("newl");
            }
            else//赋值路点状态
            {//debug2014
                //    // ParameterSetBeforeSendToUDP();
                if (r_log.WholeNumberOfNavigationPoints == 0)
                {
                    r_log.x_new[r_log.WholeNumberOfNavigationPoints] = PresentPoint.x;
                    r_log.y_new[r_log.WholeNumberOfNavigationPoints] = PresentPoint.y;
                    logPoint[r_log.WholeNumberOfNavigationPoints].x = PresentPoint.x;
                    logPoint[r_log.WholeNumberOfNavigationPoints].y = PresentPoint.y;
                    logPoint[r_log.WholeNumberOfNavigationPoints].second_ = cros.msg_ins.second;
                    logPoint[r_log.WholeNumberOfNavigationPoints].gear = cros.msg_veh.data[9];
                    logPoint[r_log.WholeNumberOfNavigationPoints].velocity = cros.msg_veh.data[2] / 1000.0;
                    logPoint[r_log.WholeNumberOfNavigationPoints].yaw = cros.msg_ins.attitude.z;
                    r_log.WholeNumberOfNavigationPoints++;
                }
                else
                {
                    if (twopoints_distance(r_log.x_new[r_log.WholeNumberOfNavigationPoints - 1], r_log.y_new[r_log.WholeNumberOfNavigationPoints - 1], PresentPoint.x, PresentPoint.y) > 2.0)
                    {
                        r_log.x_new[r_log.WholeNumberOfNavigationPoints] = PresentPoint.x;
                        r_log.y_new[r_log.WholeNumberOfNavigationPoints] = PresentPoint.y;
                        logPoint[r_log.WholeNumberOfNavigationPoints].x = PresentPoint.x;
                        logPoint[r_log.WholeNumberOfNavigationPoints].y = PresentPoint.y;
                        logPoint[r_log.WholeNumberOfNavigationPoints].second_ = cros.msg_ins.second;
                        logPoint[r_log.WholeNumberOfNavigationPoints].gear = cros.msg_veh.data[9];
                        logPoint[r_log.WholeNumberOfNavigationPoints].velocity = cros.msg_veh.data[2] / 1000.0;
                        logPoint[r_log.WholeNumberOfNavigationPoints].yaw = cros.msg_ins.attitude.z;
                        logPoint[r_log.WholeNumberOfNavigationPoints].ID = r_log.WholeNumberOfNavigationPoints;
                        r_log.WholeNumberOfNavigationPoints++;
                    }
                    else
                    {
                        logPoint[r_log.WholeNumberOfNavigationPoints - 1].second_ = cros.msg_ins.second;
                        logPoint[r_log.WholeNumberOfNavigationPoints - 1].gear = cros.msg_veh.data[9];
                        logPoint[r_log.WholeNumberOfNavigationPoints - 1].velocity = cros.msg_veh.data[2] / 1000.0;
                        logPoint[r_log.WholeNumberOfNavigationPoints - 1].yaw = cros.msg_ins.attitude.z;
                        logPoint[r_log.WholeNumberOfNavigationPoints - 1].ID = r_log.WholeNumberOfNavigationPoints - 1;

                    }
                }
                if (cros.msg_veh.data[9] == 5)
                {
                    Back_car_flag = true;
                }
                if (Back_car_flag && cros.msg_veh.data[9] != 5)
                {
                    Back_car_flag = false;
                    r_log.dispose();
                    return;
                }
                //if (r_log.WholeNumberOfNavigationPoints>1000)
                //{
                //    r_log.dispose();
                //    return;
                //}
                r_log.start_search_index = 0;
                r_log.end_search_index = r_log.WholeNumberOfNavigationPoints - 1;
                r_log.find_send_points(DataFromUDP.back_car_x, DataFromUDP.back_car_y, 0);
                back_car_index = r_log.present_point_index;

                transfer._soPoint back_car_point = new transfer._soPoint();
                back_car_point.x = DataFromUDP.back_car_x;
                back_car_point.y = DataFromUDP.back_car_y;
                this.show(back_car_point, Color.Green);
                transfer._soPoint Front_car_point = new transfer._soPoint();
                Front_car_point.x = PresentPoint.x;
                Front_car_point.y = PresentPoint.y;
                this.show(Front_car_point, Color.Red);
                //显示Log点
                transfer._soPoint[] LogPoint_array = new transfer._soPoint[r_log.WholeNumberOfNavigationPoints];
                if (r_log.WholeNumberOfNavigationPoints - 1 < 1000)
                {
                    for (int i = 0; i < r_log.WholeNumberOfNavigationPoints - 1 + 1; i++)
                    {
                        LogPoint_array[i].x = logPoint[i].x;
                        LogPoint_array[i].y = logPoint[i].y;
                    }
                }
                else
                {
                    if (r_log.WholeNumberOfNavigationPoints - 1 < 4000)
                    {
                        for (int i = 0; i < r_log.WholeNumberOfNavigationPoints - 1 + 1; i += 4)
                        {
                            LogPoint_array[i / 4].x = logPoint[i].x;
                            LogPoint_array[i / 4].y = logPoint[i].y;
                        }
                    }
                    else
                    {
                        for (int i = 0; i < r_log.WholeNumberOfNavigationPoints - 1 + 1; i += 5)
                        {
                            LogPoint_array[i / 5].x = logPoint[i].x;
                            LogPoint_array[i / 5].y = logPoint[i].y;
                        }
                    }
                }
                show(LogPoint_array, Color.Blue);
                //显示发送点
                transfer._soPoint[] SendPoint_array = new transfer._soPoint[30];
                for (int i = 0; i < 30; i++)
                {
                    long index = i + back_car_index;
                    if (index > r_log.WholeNumberOfNavigationPoints - 1)
                        index = r_log.WholeNumberOfNavigationPoints - 1;
                    SendPoint_array[i].x = logPoint[index].x;
                    SendPoint_array[i].y = logPoint[index].y;
                    if (i == 29)
                    {
                        SendPoint_array[i].x = cros.msg_ins.position.x;
                        SendPoint_array[i].y = cros.msg_ins.position.y;
                    }
                }
                show(SendPoint_array, Color.Yellow);

                textBox1.Text = "后车序号：" + back_car_index
                            + "\r\n后车速度：" + DataFromUDP.back_velocity
                            + "\r\n后车档位：" + DataFromUDP.Back_gear
                            + "\r\n后车位置：X:" + DataFromUDP.back_car_x.ToString("F2") + ",Y:" + DataFromUDP.back_car_y.ToString("F2")
                            + "\r\n后车惯导状态：" + GetInsStatus(UDP_truck.recvData.ins_status)
                            + "\r\n"
                            + "\r\n前车速度：" + cros.msg_veh.data[2] / 1000.0
                            + "\r\n前车档位：" + cros.msg_veh.data[9]
                            + "\r\n惯导状态：" + cros.msg_ins.status
                            + "\r\n前车位置：X:" + cros.msg_ins.position.x.ToString("F2") + ",Y:" + cros.msg_ins.position.y.ToString("F2")
                            + "\r\n"
                            + "前后车距离：" + twopoints_distance(cros.msg_ins.position.x, cros.msg_ins.position.y, DataFromUDP.back_car_x, DataFromUDP.back_car_y).ToString("F2");
            }
        }
        //鼠标移动相应函数
        private void axSuperMap1_MouseMoveEvent(object sender, AxSuperMapLib._DSuperMapEvents_MouseMoveEvent e)
        {
            if (!mf.IsDisposed)
            {
                MousePointCoordinate.x = mf.axSuperMap1.PixelToMapX(e.x);
                MousePointCoordinate.y = mf.axSuperMap1.PixelToMapY(e.y);
            }
        }
        //显示速度信息
        public void show_speed()
        {
            clear();
            for (int lengh_point = 0; lengh_point < NavigationPoint.WholeNumberOfNavigationPoints; lengh_point += 20)
            {
                double speed = rd_points[lengh_point].speed;
                speed_point.x = rd_points[lengh_point].x;
                speed_point.y = rd_points[lengh_point].y;
                show(speed_point, Color.FromArgb(255, Convert.ToInt16(speed * 6.0), 255 - Convert.ToInt16(speed * 6.0)));
            }
        }
       public string TaskFile_Path = "";
       public soGeoPoint convert(double lon, double lat)//坐标系转换
       {
           double WGS84_A = 6378137.0;		// major axis
           //double WGS84_B = 6356752.31424518;	// minor axis
           //double WGS84_F = 0.0033528107;		// ellipsoid flattening
           double WGS84_E = 0.0818191908;		// first eccentricity
           //double WGS84_EP = 0.0820944379;		// second eccentricity
           // UTM Parameters
           double UTM_K0 = 0.9996;			// scale factor
           double UTM_FE = 500000.0;		// false easting
           double UTM_FN_N = 0.0;           // false northing, northern hemisphere
           double UTM_FN_S = 10000000.0;    // false northing, southern hemisphere
           double UTM_E2 = (WGS84_E * WGS84_E);	// e^2
           double UTM_E4 = (UTM_E2 * UTM_E2);		// e^4
           double UTM_E6 = (UTM_E4 * UTM_E2);		// e^6
           double UTM_EP2 = (UTM_E2 / (1 - UTM_E2));	// e'^2
           double m0 = (1 - UTM_E2 / 4 - 3 * UTM_E4 / 64 - 5 * UTM_E6 / 256);
           double m1 = -(3 * UTM_E2 / 8 + 3 * UTM_E4 / 32 + 45 * UTM_E6 / 1024);
           double m2 = (15 * UTM_E4 / 256 + 45 * UTM_E6 / 1024);
           double m3 = -(35 * UTM_E6 / 3072);
           double RADIANS_PER_DEGREE = Math.PI / 180.0;
           // compute the central meridian
           double cm = ((lon >= 0.0)
                              ? ((int)lon - ((int)lon) % 6 + 3)
                              : ((int)lon - ((int)lon) % 6 - 3));
           // convert degrees into radians
           double rlat = lat * RADIANS_PER_DEGREE;
           double rlon = lon * RADIANS_PER_DEGREE;
           double rlon0 = cm * RADIANS_PER_DEGREE;

           // compute trigonometric functions
           double slat = Math.Sin(rlat);
           double clat = Math.Cos(rlat);
           double tlat = Math.Tan(rlat);

           // decide the false northing at origin
           double fn = (lat > 0) ? UTM_FN_N : UTM_FN_S;

           double T = tlat * tlat;
           double C = UTM_EP2 * clat * clat;
           double A = (rlon - rlon0) * clat;
           double M = WGS84_A * (m0 * rlat + m1 * Math.Sin(2 * rlat)
                     + m2 * Math.Sin(4 * rlat) + m3 * Math.Sin(6 * rlat));
           double V = WGS84_A / Math.Sqrt(1 - UTM_E2 * slat * slat);

           // compute the easting-northing coordinates
           DataFromUDP.back_car_x = UTM_FE + UTM_K0 * V * (A + (1 - T + C) * Math.Pow(A, 3) / 6
                           + (5 - 18 * T + T * T + 72 * C - 58 * UTM_EP2) * Math.Pow(A, 5) / 120);
           DataFromUDP.back_car_y = fn + UTM_K0 * (M + V * tlat * (A * A / 2
                               + (5 - T + 9 * C + 4 * C * C) * Math.Pow(A, 4) / 24
                               + ((61 - 58 * T + T * T + 600 * C - 330 * UTM_EP2)
                              * Math.Pow(A, 6) / 720)));
           soGeoPoint p1 = new soGeoPoint();
           p1.x = DataFromUDP.back_car_x + MapCoordinateAllignment.x;
           p1.y = DataFromUDP.back_car_y + MapCoordinateAllignment.y;
           return p1;
       }


        //手工操作读入任务点
        private void button12_Click(object sender, EventArgs e)
        {   //清除bit1故障--任务点
            TroubleCode &= 0xfffffffd;

            //string strOpenPath = string.Empty;
            this.openFileDialog2.Title = "读入任务文件";
            this.openFileDialog2.InitialDirectory = @"..\..\..\..\..\SampleData\World";
            this.openFileDialog2.FileName = "";
            this.openFileDialog2.Filter = "任务文件(.txt)|*.txt";
            if (this.openFileDialog2.ShowDialog() == DialogResult.OK)
            {
                TaskFile_Path = this.openFileDialog2.FileName;

            }
            else
            {
                return;
            }
            rpr.dispose();
            rpr.Read(TaskFile_Path, this);//读入任务点
            if (rpr.num == 0)
            {
                TroubleCode |= 0x0002;  //bit1=1
            }

            clear();
            //显示任务点
            if (blnOpen)
            {
                for (int length = 0; length < rpr.num; length++)
                {
                    soGeoPoint temp = new soGeoPoint();

                    temp = convert(rpr.TaskData[length].longitude, rpr.TaskData[length].latitude);//x-经度 ，y-纬度

                    rpr.TaskData[length].x = temp.x - MapCoordinateAllignment.x;
                    rpr.TaskData[length].y = temp.y - MapCoordinateAllignment.y;
                    temp.x = rpr.TaskData[length].x;
                    temp.y = rpr.TaskData[length].y;
                    if (rpr.TaskData[length].status1 == TASKPROPERTY_STARTPOINT)
                    {
                        show(temp, (length+1).ToString(), Color.Blue);
                    }
                    else if (rpr.TaskData[length].status1 == TASKPROPERTY_ENDPOINT)
                    {
                        show(temp, (length + 1).ToString(), Color.Purple);
                    }
                    else if((rpr.TaskData[length].status1==TASKPROPERTY_ENTERPARKPOINT)||(rpr.TaskData[length].status1==TASKPROPERTY_OUTPARKPOINT))//1025
                    {
                        show(temp, (length + 1).ToString(), Color.Green);
                    }
                    else if (rpr.TaskData[length].status1 == TASKPROPERTY_PARKPOINT)
                    {
                        show(temp, (length + 1).ToString(), Color.HotPink);
                    }
                    else
                    {
                        show(temp, (length + 1).ToString(), Color.Red);
                    }
                }
            }
            else
            {
                MessageBox.Show("未打开工作空间");
            }
        }

        //显示交通灯信息
        public void show_traffic()
        {
            clear();
            for (int lengh_point = 0; lengh_point < NavigationPoint.WholeNumberOfNavigationPoints; lengh_point += 20)
            {
                traffic_point.x = rd_points[lengh_point].x;
                traffic_point.y = rd_points[lengh_point].y;
                if (rd_points[lengh_point].traffic == 1)
                {
                    show(traffic_point, Color.Red);
                }
                else
                {
                    show(traffic_point, Color.Green);
                }
            }
        }
        public void show(transfer._soPoint[] objgeo, Color c)
        {
            if (mf.IsDisposed)
                return;
            soGeoPoint temp = new soGeoPoint();
            soTrackingLayer objTL = mf.axSuperMap1.TrackingLayer;
            soStyle objstyle = new soStyle();
            objstyle.PenColor = (uint)ColorTranslator.ToOle(c);
            objstyle.SymbolSize = 30;
            for (int i = 0; i < objgeo.Length; i++)
            {
                temp.x = objgeo[i].x;
                temp.y = objgeo[i].y;
                objTL.AddEvent((soGeometry)temp, objstyle, "");


            }
            objTL.Refresh();
        }
        //弹出改变关键点窗口
        private void change_key_point_Click(object sender, EventArgs e)
        {
            Form1 f1 = new Form1(this);
            clear();
            f1.Show();
            for (int length = 0; length < rpr.num; length++)
            {
                soGeoPoint temp = new soGeoPoint();
                temp.x = rpr.TaskData[length].x;
                temp.y = rpr.TaskData[length].y;
                if (rpr.TaskData[length].status1 == TASKPROPERTY_STARTPOINT)
                {
                    show(temp, Color.Blue, 50);
                }
                else if (rpr.TaskData[length].status1 == TASKPROPERTY_ENDPOINT)
                {
                    show(temp, Color.Purple, 50);
                }
                else
                {
                    show(temp, Color.Red, 50);
                }
            }
            //if (DataFromUDP.Flag_Align == 19)
            //{
            //    DataFromUDP.Flag_Align = 0;
            //}
            //else
            //{
            //    DataFromUDP.Flag_Align = 19;
            //}
        }
        TRUCK_UDP UDP_truck = new TRUCK_UDP();
        //C_UDP UDP_2_Backcar;
        /***************************** 主界面加载程序 ***********************
         * 参数初始化
         * 网口初始化
         * 显示更新100ms定时器TimerDisplay100ms使能有效
         * 读地图东向与北向偏移量
         * 运行初始化程序INIT（）
         *
         * *****************************************************************/
        private void MainForm_Load(object sender, EventArgs e)
        {   //路径规划关键点个数初始化为0
            PathPlanInfo.KeyPointNumber = 0;
            //车道线数量 -ffff为无效
            DataToUDPExcludeRoadPoints.LaneNumber = 0x0f;
            //车的行驶状态 初始化为-1 为未定义状态
            //X7：并线标志，0 不能并线  1 （默认）能并线
            RunningStatus.ChangeLane = YES;
            DataToUDPExcludeRoadPoints.FlagOfRoadNet = YES; //是否有路网标志，1（YES）-有（默认值）；0-无
            RunningStatus.DistanceToNextTrafficLightOrEndPoint = 0.0;
            RunningStatus.FlagWhetherAllignToGIS = YES;//if_corner
            RunningStatus.FlagWhetherTrafficSign = NO;
            RunningStatus.MatchStatus = MATCHSTATUS_MATCHING; //比赛中
            RunningStatus.RunControlWithTrafficLight = GOAHEAD;
            //网口初始化
            textBox2.Text = ROS.ROS_IP;
            //显示更新100ms定时器
            TimerDisplay100ms.Enabled = true;

            //关闭200ms 开机提示定时器
            initTimer.Dispose();

            transfer._soPoint Temp_DataFU;
            Temp_DataFU.x = 0.0;
            Temp_DataFU.y = 0.0;//队列灌初始值

            for (int i = 0; i < Log_Count; i++)
            {
                pass_point.Add(Temp_DataFU);
            }
            cros = new CROS();
            //cros.OnInsRefresh+=new CROS.InsRefreshEventHandler(cros_OnInsRefresh);
            //cros.OnVehicleRefresh += new CROS.VehicleRefreshEventHandler(cros_OnVehicleRefresh);
            //UDP_2_Backcar = new C_UDP("127.0.0.1", 40001, 1);
            //UDP_2_Backcar.OnRefresh += new C_UDP.RefreshEventHandler(Backcar_data_OnRefresh);
            //UDP_truck.OnUDPRefresh += new TRUCK_UDP.UDPRefreshEventHandler(Frontcar_data_OnRefresh);
            UDP_truck.OnUDPRefresh += UDP_truck_OnUDPRefresh;
        }

        //每500ms向监管程序发送系统时间；监管程序超过500ms未收到此时间则重启GIS程序
        //故障显示
     

        private void button1_Click_1(object sender, EventArgs e)
        {
            mf = new MapForm(this);
            ConnectSuperMap();
            mf.axSuperMap1.OpenMap("道路");
            mf.delx = MapCoordinateAllignment.x;
            mf.dely = MapCoordinateAllignment.y;
            mf.Show();
            //df.Show();///////////////////////////////debug
        }

        private void textBox2_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (e.KeyChar == 13)
            {
                try
                {   //设置服务器IP地址textBox2.Text、端口号4003及本程序模块对应的ID=03号
                   // PublicVariable.my_client = new C_UDP(textBox2.Text, 4003, 03, this);
                   // PublicVariable.my_client.OnRefresh += new C_UDP.RefreshEventHandler(my_client_OnRefresh);
                    ROS.ROS_IP =  textBox2.Text;
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message, "UDP初始化失败");
                }
                textBox2.Text = ROS.ROS_IP;
            }
        }
        public int force_speed = 0;
        //变更传递(封装处理)
        public void SetMapCoordinateAllignmentX(double x)
        {
            MapCoordinateAllignment.x = x;
        }
        public void SetMapCoordinateAllignmentY(double y)
        {
            MapCoordinateAllignment.y = y;
        }
        public double GetMapCoordinateAllignmentX()
        {
            return MapCoordinateAllignment.x;
        }
        public double GetMapCoordinateAllignmentY()
        {
            return MapCoordinateAllignment.y;
        }
        public double GetRPR_X()
        {
            return mf.rpr_x;
        }
        public double GetRPR_Y()
        {
            return mf.rpr_y;
        }
        public double GetAngleForbidRoad()
        {
            return Angle_Forbid_Road;
        }
        public double GetDistanceGlobal()
        {
            return Distance_Global;
        }
        public double GetDistanceSendway()
        {
            return Distance_SendWay;
        }
        public double GetHermiteModulus()
        {
            return Hermite_Modulus;
        }
        public int GetDistanceAlign()//1023
        {
            return Distance_Align;//1023
        }
        //显示任务点及其序号
        public void show(soGeoPoint objgeo,string s,Color c)
        {
            if (mf != null)
            {
                soTextPart objtextpart = new soTextPart();
                soGeoText objgeotext = new soGeoText();
                soTextStyle objtextstyle = new soTextStyle();
                objtextstyle.FixedSize = true;
                objtextstyle.FixedTextSize = 80;
                objtextstyle.Color = (uint)ColorTranslator.ToOle(Color.Red);
                soTrackingLayer objTL = mf.axSuperMap1.TrackingLayer;
                soStyle objstyle = new soStyle();
                objstyle.PenColor = (uint)ColorTranslator.ToOle(c);
                objstyle.SymbolSize = 30;
                objTL.AddEvent((soGeometry)objgeo, objstyle, "");
                objtextpart.Text = s;
                soGeoPoint objgeopoint = (soGeoPoint)objgeo;
                objtextpart.x = objgeopoint.x;
                objtextpart.y = objgeopoint.y;
                objgeotext.AddPart(objtextpart);
                objgeotext.TextStyle = objtextstyle;
                objTL.AddEvent((soGeometry)objgeotext, objstyle, "");
                objTL.Refresh();
            }
        }
        private double twopoints_angle(double detax, double detay)//前一点的角度，detax为后一点减前一点的X轴的差值
        {
            double tangle = new double();
            if (Math.Abs(detax) < 0.0001)
            {
                if (detay > 0.0)
                {
                    return 90.0;
                }
                else
                {
                    return 270.0;
                }
            }
            else
            {
                if (Math.Abs(detay) < 0.0001)
                {
                    if (detax > 0.0)
                    {
                        return 0.0;
                    }
                    else
                    {
                        return 180.0;
                    }
                }
                else
                {

                    tangle = detay / detax;
                    if (detax > 0.0)
                    {
                        if (detay > 0.0)
                        {
                            //第一象限

                            return (Math.Atan(tangle) / Math.PI * 180);
                        }
                        else//第四象限
                        {
                            return (Math.Atan(tangle) / Math.PI * 180 + 360.0);
                        }
                    }
                    else
                    {
                        if (detay > 0.0)
                        {
                            //第二象限

                            return (Math.Atan(tangle) / Math.PI * 180 + 180.0);
                        }
                        else//第三象限
                        {
                            return (Math.Atan(tangle) / Math.PI * 180 + 180.0);
                        }

                    }
                }
            }
        }
        private double twopoints_distance(double x1, double y1, double x2, double y2)//求两个任务点之间的距离
        {
            return (Math.Sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
        }

        private void textBox3_TextChanged(object sender, EventArgs e)
        {
            if (!Int32.TryParse(textBox3.Text, out Index_Distance_Align))
            {
                MessageBox.Show("请输入正确的区间号！");
                return;
            }
            else
            {
                Index_Distance_Align = Convert.ToInt32(textBox3.Text);//1029
            }
        }


        private void rb_earth_CheckedChanged(object sender, EventArgs e)
        {
            rd_point_flag = !rd_point_flag;
        }
        bool Task_Plan_flag = false;
        private void rb_log_CheckedChanged(object sender, EventArgs e)
        {
            Task_Plan_flag = !Task_Plan_flag;
        }
        double del_length = 0.0;
        private void numericUpDown1_ValueChanged(object sender, EventArgs e)
        {
            del_length = Convert.ToInt32(numericUpDown1.Value);
        }

        private void timer_for_send_data_Tick(object sender, EventArgs e)
        {
            if (r_log.WholeNumberOfNavigationPoints > 0)
            {
                TRUCK_UDP.CAR_TO_TRUCK_MESSAGE message = new TRUCK_UDP.CAR_TO_TRUCK_MESSAGE();
                message.send_to_truck = new TRUCK_UDP.TRUCK_TO_CAR_MESSAGE[30];
                double x, y, speed;
                int gear;
                long ID;
                if (!cros.msg_veh_ready)
                    return;
                    for (int i = 0; i < 30; i++)
                    {
                        long index = i + back_car_index;
                        if (index > r_log.WholeNumberOfNavigationPoints - 1)
                            index = r_log.WholeNumberOfNavigationPoints - 1;
                        x = logPoint[index].x;
                        y = logPoint[index].y;
                        gear = logPoint[index].gear;
                        ID = logPoint[index].ID;
                        speed = logPoint[index].velocity;
                        message.send_to_truck[i].gear = gear;
                        message.send_to_truck[i].x = (x + del_length * Math.Cos(logPoint[index].yaw * Math.PI / 180.0)) * 100.0;
                        message.send_to_truck[i].y = (y - del_length * Math.Sin(logPoint[index].yaw * Math.PI / 180.0)) * 100.0;
                        message.send_to_truck[i].velocity = speed;
                        message.send_to_truck[i].id = ID;
                        if (i == 29)
                        {
                            message.send_to_truck[i].gear = cros.msg_veh.data[9];
                            message.send_to_truck[i].x = (cros.msg_ins.position.x + del_length * Math.Cos(cros.msg_ins.attitude.z * Math.PI / 180.0)) * 100.0;
                            message.send_to_truck[i].y = (cros.msg_ins.position.y - del_length * Math.Sin(cros.msg_ins.attitude.z * Math.PI / 180.0)) * 100.0;                        
                            message.send_to_truck[i].velocity = cros.msg_veh.data[2] / 1000.0;
                            message.send_to_truck[i].id = r_log.WholeNumberOfNavigationPoints - 1;
                        }

                    }
            
                UDP_truck.dataSend(message);
            }
        }

        void UDP_truck_OnUDPRefresh(TRUCK_UDP.TRUCK_TO_CAR_MESSAGE udp_msg)//接收数据
        {
            DataFromUDP.back_car_x = UDP_truck.recvData.x / 100.0;
            DataFromUDP.back_car_y = UDP_truck.recvData.y / 100.0;
            DataFromUDP.ID = UDP_truck.recvData.id;
            DataFromUDP.Back_gear = UDP_truck.recvData.gear;
            DataFromUDP.back_velocity = UDP_truck.recvData.velocity;
        }


    }
}                                                  