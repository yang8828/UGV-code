using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using SuperMapLib;
using System.IO;
namespace PathEx_Analysis
{
    public partial class MapForm : Form
    {
        public MapForm(MainForm mf1)
        {
            this.mf1 = mf1;
            InitializeComponent();
        }

        public MapForm()
        {
            InitializeComponent();
        }
        public MainForm mf1;
        public double rpr_x = 0.0;
        public double rpr_y = 0.0;
        public double delx = 0.0;
        public double dely = 0.0;
        private void button5_Click(object sender, EventArgs e)
        {
            axSuperMap1.Action = seAction.scaPan ;
        }

        private void button4_Click(object sender, EventArgs e)
        {
            axSuperMap1.Action = seAction.scaNull;
            soTrackingLayer objTL = axSuperMap1.TrackingLayer;
            objTL.ClearEvents();
        }

        private void button2_Click(object sender, EventArgs e)
        {
            axSuperMap1.Action = seAction.scaZoomIn;
        }

        private void button3_Click(object sender, EventArgs e)
        {
            axSuperMap1.Action = seAction.scaZoomOut;
        }
        private void axSuperMap1_MouseDownEvent(object sender, AxSuperMapLib._DSuperMapEvents_MouseDownEvent e)
        {
            rpr_x = axSuperMap1.PixelToMapX(e.x);
            rpr_y = axSuperMap1.PixelToMapY(e.y);
            //mf1.DataFromUDP.x = rpr_x ;//debug
            //mf1.DataFromUDP.y = rpr_y ;

        }

        private void bn_show_speed_Click(object sender, EventArgs e)
        {
            mf1.show_speed();
        }
               

        private void button13_Click(object sender, EventArgs e)
        {
            mf1.show_traffic();
        }

        private void 修改位置_Click(object sender, EventArgs e)
        {
            int index;
            if (!Int32.TryParse(tb_point_index.Text,out index))
            {
                MessageBox.Show("请输入正确的ID");
                return;
            }
            else
            {
                index = Convert.ToInt32(tb_point_index.Text) - 1;//要改变位置的点的序号
                
                if (!this.IsDisposed)
                {
                    mf1.rpr.TaskData[index].x = rpr_x;
                    mf1.rpr.TaskData[index].y = rpr_y;//将鼠标按下的位置赋给要改变的点
                }
                else
                {
                    MessageBox.Show("请打开地图窗口");
                    return;
                }
                mf1.clear();
                for (int length = 0; length < mf1.rpr.num; length++)
                {
                    soGeoPoint temp = new soGeoPoint();
                    temp.x = mf1.rpr.TaskData[length].x;
                    temp.y = mf1.rpr.TaskData[length].y;
                    if (mf1.rpr.TaskData[length].status1 == MainForm.TASKPROPERTY_STARTPOINT)
                    {
                        mf1.show(temp, Color.Blue, 50);
                    }
                    else if (mf1.rpr.TaskData[length].status1 == MainForm.TASKPROPERTY_ENDPOINT)
                    {
                        mf1.show(temp, Color.Purple, 50);                         
                    }
                    else
                    {
                        mf1.show(temp, Color.Red, 50);
                    }
                }
                SaveModifyTaskFile(); 
            }
        }
        /**************************** 存储修改后的任务文件 ***************************
         * 1. 对新任务点进行坐标反变换（x,y)->(B,L)
         * 2. 将修改后的任务点写到文件“改后任务文件.txt”中
         ****************************************************************************/
        CoordinateConversion cc = new CoordinateConversion();
        private void SaveModifyTaskFile()
        {
            double longitude = mf1.rpr.TaskData[0].longitude;
            //对修改的任务点进行反变换
            int index;
            for (index = 0; index < mf1.rpr.num; index++)
            {
                double map_X = mf1.rpr.TaskData[index].x;
                double map_Y = mf1.rpr.TaskData[index].y;
                //计算带号

                int Zone = (int)(longitude / 6.0) + 31; //北京输出50
                //int Zone = 50;
                cc.UTMtoLatLon(map_X, map_Y, false, Zone);
                mf1.rpr.TaskData[index].latitude = cc.latitude;
                mf1.rpr.TaskData[index].longitude = cc.lontitude;
            }
            //2. 将修改后的任务点写到文件“改后任务文件.txt”中。            
            StreamWriter sw;            
            
            string TaskFile_PathPath;
            TaskFile_PathPath = mf1.TaskFile_Path.Substring(0, mf1.TaskFile_Path.LastIndexOf('\\'));

            FileStream fs = new FileStream(TaskFile_PathPath + "\\改后任务文件.txt", FileMode.Create, FileAccess.Write);

            sw = new StreamWriter(fs);
            sw.BaseStream.Seek(0, SeekOrigin.Begin);
            string string_temp;

            for (index = 0; index < mf1.rpr.num; index++)
            {
                string_temp = (index + 1) + "\t" + mf1.rpr.TaskData[index].longitude.ToString() + "\t"
                            + mf1.rpr.TaskData[index].latitude.ToString() + "\t" + mf1.rpr.TaskData[index].altitude.ToString()
                            + "\t" + mf1.rpr.TaskData[index].status1.ToString() + "\t" + mf1.rpr.TaskData[index].status2.ToString();
                sw.WriteLine(string_temp);
            }
            sw.Close();
            fs.Close();

        }

        private void axSuperMap1_MouseMoveEvent(object sender, AxSuperMapLib._DSuperMapEvents_MouseMoveEvent e)
        {
            label1.Text = "X: " + axSuperMap1.PixelToMapX(e.x).ToString("F2") + "\r\nY: " + axSuperMap1.PixelToMapY(e.y).ToString("F2");
        }

        private void MapForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            axSuperMap1.Disconnect();
            delx = mf1.GetMapCoordinateAllignmentX();
            dely = mf1.GetMapCoordinateAllignmentY();
            try
            {
                int index = mf1.workspace_path.LastIndexOf("\\");
                string file_path = mf1.workspace_path.Substring(0, index + 1) + "Map_shift.txt";
                FileStream fs = new FileStream(file_path, FileMode.Create, FileAccess.Write);
                StreamWriter sw = new StreamWriter(fs);
                sw.WriteLine(delx.ToString("F2") + "," + dely.ToString("F2"));
                sw.Close();
                fs.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
           
        }

        private void numericUpDownEast_ValueChanged(object sender, EventArgs e)
        {
            mf1.SetMapCoordinateAllignmentX ( Convert.ToDouble(numericUpDownEast.Value));

        }

        private void numericUpDownNorth_ValueChanged(object sender, EventArgs e)
        {
            mf1.SetMapCoordinateAllignmentY ( Convert.ToDouble(numericUpDownNorth.Value));
        }

        private void MapForm_Load(object sender, EventArgs e)
        {
            double MapCoordinateAllignmentX = mf1.GetMapCoordinateAllignmentX();
            double MapCoordinateAllignmentY = mf1.GetMapCoordinateAllignmentY();
            numericUpDownEast.Value = (decimal)(MapCoordinateAllignmentX);
            numericUpDownNorth.Value = (decimal)(MapCoordinateAllignmentY);
        }
    }
}
