using System;
using System.Collections.Generic;
using System.Text;
using System.IO;
using SuperMapLib;
using System.Windows.Forms;
namespace PathEx_Analysis
{
    public class RoadpointReader
    {
        public struct TaskFileData
        {
            public double longitude;
            public double latitude;
            public double altitude;
            public double x;
            public double y;
            public int status1;
            public int status2;
        }
        //读入的任务点个数
        public TaskFileData[ ] TaskData = new TaskFileData[65536];
        //根据是需要差值还是路径规划将任务点分组，Number_Of_Group为组的个数，Number_Of_Taskpoints_InGroup为每组中任务点的个数,
        //Index_Of_Startpoint_InGroup为每组的起始点在整个任务点中的序号，Index_Of_Endpoint_InGroup为每组的终止点在整个任务点中的序号
        //Flag_OF_NoRoadNet表示组的属性，1表示此组记录的是居民区的任务点，0表示正常城市道路的任务点
        public int Number_Of_Group = 1;
        public int[] Number_Of_Taskpoints_InGroup = new int[10];
        public int[] Index_Of_Startpoint_InGroup = new int[10];
        public int[] Index_Of_Endpoint_InGroup = new int[10];
        public int[] Flag_OF_NoRoadNet = new int[10];
        //总的任务点个数
        public long num = 0;
        public TaskFileData endpoint = new TaskFileData();
        public void dispose()
        {
            for (int i = 0; i < num; i++)
            {
                
                    TaskData[i].longitude = 0.0;
                    TaskData[i].latitude = 0.0;
                    TaskData[i].altitude = 0.0;
                    TaskData[i].status1 = 0;
                    TaskData[i].status2 = 0;
            }
            for (int i = 0; i < Number_Of_Group; i++)
            {
                Number_Of_Taskpoints_InGroup[i] = 0;
            }
            Number_Of_Group = 1;
            num = 0;
        }
        public void Read(string FileName, MainForm MainForm)
        {
            //读文件，更新points和num
            string[] str_buff = new string[12000];
            //任务文件数据有效性标志
            bool FlagTaskValid = true;
            bool FlagTaskStartPoint = false;
            bool FlagTaskDestinationPoint = false;
            if (!File.Exists(FileName))
            {
                MessageBox.Show("打开任务文件失败");
                return;
            }
            try
            {
                FileStream fs = new FileStream(FileName, FileMode.Open, FileAccess.Read);
                StreamReader sr = new StreamReader(fs);
                sr.BaseStream.Seek(0, SeekOrigin.Begin);
                int j = 0;
                str_buff[j] = sr.ReadLine();
                while (str_buff[j] != null)
                {
                    j++;
                    str_buff[j] = sr.ReadLine();

                }
                for (int i = 0; i < j; i++)
                {
                    string[] s = str_buff[i].Split('\t');
                    TaskData[i].longitude = Convert.ToDouble(s[1]);
                    TaskData[i].latitude = Convert.ToDouble(s[2]);
                    TaskData[i].altitude = Convert.ToDouble(s[3]);
                    TaskData[i].status1 = Convert.ToInt32(s[4]);
                    TaskData[i].status2 = Convert.ToInt32(s[5]);
                    Console.WriteLine("debug初始读入任务点经度" + TaskData[i].longitude);

                    //判断是不是居民区入口点，在此假设属性2的数值为6时代表为居民区入口点,7的时候代表出口点，其间的点为居民区内的点
                    if ((TaskData[i].status2 == 6) || (TaskData[i].status2 == 7))
                    {
                        if (TaskData[i].status2 == 6)//居民区入口点
                        {
                            if (i != 0)
                            {
                                Index_Of_Endpoint_InGroup[Number_Of_Group - 1] = i;//存储居民区之前正常城市道路的终止点在整个任务点的序号
                                Number_Of_Taskpoints_InGroup[Number_Of_Group - 1] = Index_Of_Endpoint_InGroup[Number_Of_Group - 1] - Index_Of_Startpoint_InGroup[Number_Of_Group - 1] + 1;//存储上一组的任务点数量
                                Flag_OF_NoRoadNet[Number_Of_Group - 1] = 0;//表示这一组为正常城市道路
                                //遇6点则组的数量加1
                                Number_Of_Group++;
                            }
                            Index_Of_Startpoint_InGroup[Number_Of_Group - 1] = i;//存储居民区入口点在整个任务点的序号
                        }
                        if (TaskData[i].status2 == 7)//居民区出口点
                        {
                            Index_Of_Endpoint_InGroup[Number_Of_Group - 1] = i;//存储居民区出口点在整个任务点的序号
                            Number_Of_Taskpoints_InGroup[Number_Of_Group - 1] = Index_Of_Endpoint_InGroup[Number_Of_Group - 1] - Index_Of_Startpoint_InGroup[Number_Of_Group - 1] + 1;//存储居民区的任务点数量
                            Flag_OF_NoRoadNet[Number_Of_Group - 1] = 1;//表示这一组为居民区
                            //如果不是最后一个点则组的数量加1
                            if (i != (j - 1))
                            {
                                Number_Of_Group++;
                                Index_Of_Startpoint_InGroup[Number_Of_Group - 1] = i;//存储居民区之后正常城市道路的起始点点在整个任务点的序号
                            }
                        }
                    }
                    else
                    {
                        if (i == 0)
                        {
                            Index_Of_Startpoint_InGroup[Number_Of_Group - 1] = i;//记下初始路段的起始点
                        }

                        if (i == (j - 1))
                        {
                            Index_Of_Endpoint_InGroup[Number_Of_Group - 1] = i;//记下最后路段的终止点
                            Number_Of_Taskpoints_InGroup[Number_Of_Group - 1] = Index_Of_Endpoint_InGroup[Number_Of_Group - 1] - Index_Of_Startpoint_InGroup[Number_Of_Group - 1] + 1;//存储当前组的任务点数量
                            Flag_OF_NoRoadNet[Number_Of_Group - 1] = 0;//表示这一组为正常城市道路
                        }
                    }
                    num++;//总的任务点数量

                    //判断任务数据的有效性，无效为false
                    //0. 判断是否有起点和终点，无效为false
                    if (TaskData[i].status1 == MainForm.TASKPROPERTY_STARTPOINT)
                    {
                        FlagTaskStartPoint = true;
                    }
                    if (TaskData[i].status1 == MainForm.TASKPROPERTY_ENDPOINT)
                    {
                        FlagTaskDestinationPoint = true;
                    }

                    //1. 经度
                    if ((TaskData[i].longitude < 0.0) || (TaskData[i].longitude > 180.0))
                    {
                        FlagTaskValid = false;
                    }
                    //2. 纬度
                    if ((TaskData[i].latitude < 0.0) || (TaskData[i].latitude > 90.0))
                    {
                        FlagTaskValid = false;
                    }
                    //3. 高程
                    if ((TaskData[i].altitude < -500.0) || (TaskData[i].altitude > 10000.0))
                    {
                        FlagTaskValid = false;
                    }
                    //4. 判断任务点的属性是否在有效范围内MainForm.TASKPROPERTY_STARTPOINT至MainForm.TASKPROPERTY_MAXNUMBER
                    if ((TaskData[i].status1 < 0) || (TaskData[i].status1 > MainForm.TASKPROPERTY_MAXNUMBER))
                    {
                        FlagTaskValid = false;
                    }

                    //5. 状态2 表示点的属性，可扩展，不判断
                }

                if ((num == 0) || (FlagTaskValid == false) || (FlagTaskDestinationPoint == false) || (FlagTaskStartPoint == false))
                {
                    num = 0;
                    MessageBox.Show("任务文件数据无效");
                    return;
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }
    }
}
