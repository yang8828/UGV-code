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
    public partial class Form1 : Form
    {
        public const char TASKPROPERTY_STARTPOINT = (char)0x0;          //起点
        public const char TASKPROPERTY_ENDPOINT = (char)0x7;            //终点
        MainForm f1;
        public RoadpointReader rpr1;
        public Form1(MainForm f1)
        {
            this.f1 = f1;
            rpr1 = f1.rpr;
            InitializeComponent();
            
        }   
        public Form1()
        {
            InitializeComponent();
        }      
        private void Form1_Load(object sender, EventArgs e)
        {
            int index = 0;
            for (int i = 0; i < f1.rpr.num; i++)
            {
                index = this.dataGridView1.Rows.Add();
                //cc.UTMtoLatLon(rpr1.TaskData[i].longitude, rpr1.TaskData[i].latitude, false, 50);
                dataGridView1.Rows[index].Cells[0].Value = i + 1;
                dataGridView1.Rows[index].Cells[1].Value = rpr1.TaskData[i].longitude.ToString("F7");
                dataGridView1.Rows[index].Cells[2].Value = rpr1.TaskData[i].latitude.ToString("F7");
                dataGridView1.Rows[index].Cells[3].Value = rpr1.TaskData[i].altitude.ToString("F2");
                dataGridView1.Rows[index].Cells[4].Value = rpr1.TaskData[i].status1;
                dataGridView1.Rows[index].Cells[5].Value = rpr1.TaskData[i].status2;
            }
            textBox2.Text = f1.GetAngleForbidRoad().ToString();
            textBox3.Text = f1.GetDistanceGlobal().ToString();
            textBox4.Text = f1.GetDistanceSendway().ToString();
            textBox5.Text = f1.NavigationPoint.Angle_For_Turning.ToString();
            textBox6.Text = f1.GetHermiteModulus().ToString();
            textBox7.Text = f1.GetDistanceAlign().ToString();//1023
            textBox8.Text = f1.NavigationPoint.Number_of_Overlap_Road.ToString();//1024
        }
     
        private void show_Click(object sender, EventArgs e)
        {
            soGeoPoint TaskPointAfterChange = new soGeoPoint();
            FileStream fs;
            StreamWriter sw;
            fs = new FileStream(f1.TaskFile_Path, FileMode.Create, FileAccess.Write);
            sw = new StreamWriter(fs);
            sw.BaseStream.Seek(0, SeekOrigin.Begin);
            string string_temp;
            for (int i = 0; i < f1.rpr.num; i++)
            {
                string_temp = (i+1) + "\t" + dataGridView1.Rows[i].Cells[1].Value.ToString() + "\t"
                            + dataGridView1.Rows[i].Cells[2].Value.ToString() + "\t" + dataGridView1.Rows[i].Cells[3].Value.ToString()
                            + "\t" +dataGridView1.Rows[i].Cells[4].Value.ToString() + "\t" + dataGridView1.Rows[i].Cells[5].Value.ToString();
                sw.WriteLine(string_temp);
            }
            sw.Close();
            fs.Close();
            rpr1.dispose();
            rpr1.Read(f1.TaskFile_Path,f1);
            f1.clear();
            //显示任务点
            if (f1.blnOpen)
            {
                for (int length = 0; length < rpr1.num; length++)
                {
                    soGeoPoint temp = new soGeoPoint();

                   // temp = f1.convert(rpr1.TaskData[length].longitude, rpr1.TaskData[length].latitude);//x-经度 ，y-纬度

                    rpr1.TaskData[length].x = temp.x - f1.GetMapCoordinateAllignmentX();
                    rpr1.TaskData[length].y = temp.y - f1.GetMapCoordinateAllignmentY(); 
                    temp.x = rpr1.TaskData[length].x;
                    temp.y = rpr1.TaskData[length].y;
                    if (rpr1.TaskData[length].status1 == TASKPROPERTY_STARTPOINT)
                    {
                        f1.show(temp, (length + 1).ToString(), Color.Blue);
                    }
                    else if (rpr1.TaskData[length].status1 == TASKPROPERTY_ENDPOINT)
                    {
                        f1.show(temp, (length + 1).ToString(), Color.Purple);
                    }
                    else
                    {
                        f1.show(temp, (length + 1).ToString(), Color.Red);
                    }
                }
            }
            else
            {
                MessageBox.Show("未打开工作空间");
            }
            this.Close();
        }
        CoordinateConversion cc = new CoordinateConversion();
        //插入任务点的序号
        private void button1_Click(object sender, EventArgs e)
        {
            int InsertIndex;
            if (!Int32.TryParse(textBox1.Text, out InsertIndex))
            {
                MessageBox.Show("请输入正确的序号！");
                return;
            }
            else
            {
                InsertIndex = Convert.ToInt32(textBox1.Text);//读入插入点序号;
            }
            //从插入点序号处统一往后移一位
            for (int i = (int)f1.rpr.num - 1; i > InsertIndex-2; i--)
            {
                dataGridView1.Rows[i + 1].Cells[0].Value = dataGridView1.Rows[i].Cells[0].Value;
                dataGridView1.Rows[i + 1].Cells[1].Value = dataGridView1.Rows[i].Cells[1].Value;
                dataGridView1.Rows[i + 1].Cells[2].Value = dataGridView1.Rows[i].Cells[2].Value;
                dataGridView1.Rows[i + 1].Cells[3].Value = dataGridView1.Rows[i].Cells[3].Value;
                dataGridView1.Rows[i + 1].Cells[4].Value = dataGridView1.Rows[i].Cells[4].Value;
                dataGridView1.Rows[i + 1].Cells[5].Value = dataGridView1.Rows[i].Cells[5].Value;
            }
            double map_X = f1.GetRPR_X();
            double map_Y = f1.GetRPR_Y();
            cc.UTMtoLatLon(map_X, map_Y, false, 50);
            dataGridView1.Rows[InsertIndex - 1].Cells[1].Value = cc.lontitude.ToString("F7");
            dataGridView1.Rows[InsertIndex - 1].Cells[2].Value = cc.latitude.ToString("F7");
            f1.rpr.num++;
        }

        private void button2_Click(object sender, EventArgs e)
        {
            dataGridView1.Rows.RemoveAt(Convert.ToInt32(dataGridView1.CurrentRow.Index));
            f1.rpr.num--;
        }

        private void button3_Click(object sender, EventArgs e)
        {
            if (!double.TryParse(textBox2.Text, out f1.Angle_Forbid_Road))
            {
                MessageBox.Show("请输入正确的角度！");
                return;
            }
            else
            {
                f1.Angle_Forbid_Road = Convert.ToDouble(textBox2.Text);
            }
            if (!double.TryParse(textBox3.Text, out f1.Distance_Global))
            {
                MessageBox.Show("请输入正确的距离！");
                return;
            }
            else
            {
                f1.Distance_Global = Convert.ToDouble(textBox3.Text);
            }
            if (!Int32.TryParse(textBox4.Text, out f1.Distance_SendWay))
            {
                MessageBox.Show("请输入正确的距离！");
                return;
            }
            else
            {
                f1.Distance_SendWay = Convert.ToInt32(textBox4.Text);
            }
            if (!double.TryParse(textBox5.Text, out f1.NavigationPoint.Angle_For_Turning))
            {
                MessageBox.Show("请输入正确的角度！");
                return;
            }
            else
            {
                f1.NavigationPoint.Angle_For_Turning = Convert.ToDouble(textBox5.Text);
            }
            if (!double.TryParse(textBox6.Text, out f1.Hermite_Modulus))
            {
                MessageBox.Show("请输入正确的系数！");
                return;
            }
            else
            {
                f1.Hermite_Modulus = Convert.ToDouble(textBox6.Text);
            }
            if (!Int32.TryParse(textBox7.Text, out f1.Distance_Align))
            {
                MessageBox.Show("请输入正确的序号！");
                return;
            }
            else
            {
                f1.Distance_Align = Convert.ToInt32(textBox7.Text);//1023
            }
            if (!Int32.TryParse(textBox8.Text, out f1.NavigationPoint.Number_of_Overlap_Road))
            {
                MessageBox.Show("请输入正确的圈数！");
                return;
            }
            else
            {
                f1.NavigationPoint.Number_of_Overlap_Road = Convert.ToInt32(textBox8.Text);//1024
            }
            if (radioButton1.Checked)//1023
            {
                f1.Flag_Align = true;//1023
            }
            if (radioButton2.Checked)//1023
            {
                f1.Flag_Align = false;//1023
            }
           
        }

        



    }
}
