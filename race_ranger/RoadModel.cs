using System;
using System.Collections.Generic;
using System.Text;

namespace PathEx_Analysis
{
    public class RoadModel
    {
        public double x;
        public double y;//道路中心点坐标
        public double[] angle = new double[10];//每条道路的方向角
        public double[] width = new double[10];//每条道路的宽度
        public int n;//此路口包含的道路数量
        public double[] x_new = new double[MainForm.NAVIGATIONPOINTMAX];
        public double[] y_new = new double[MainForm.NAVIGATIONPOINTMAX];//生成的行驶线路坐标
        public double[] TaskPointX = new double[MainForm.NAVIGATIONPOINTMAX];
        public double[] TaskPointY = new double[MainForm.NAVIGATIONPOINTMAX];
        public int Task_Num;
        public double startx;
        public double starty;//行驶轨迹起点的坐标
        public double endx;
        public double endy;//行驶轨迹终点的坐标
        public long WholeNumberOfNavigationPoints = 0;    //所有导航点的个数
        public double step_lenth = 0.4;
        public int status = MainForm.GOAHEAD;//路口的行驶情况，0是直行，1是左转，2是右转
        public int[] index = new int[10];//保存按angle的大小排序的结果，即index[0]是angle最小的那条路的序号
        public int[] reverse_index = new int[10];//reverse_index[0]为angle[0]的大小序号，若为1即为第二小的道路
        public long start_search_index = 0;
        public long end_search_index = 0;//确定当前位置的范围
        public double First_Uturn_Point_x;//Uturn第一个拐弯点1029
        public double First_Uturn_Point_y;//1029
        public double old_start_x = 0.0;//存储Uturn入口点x,y1029
        public double old_start_y = 0.0;//1029
        public double old_angle = 0.0;//存储Uturn入口角度1029
        public double azimuth;
        public double Angle_For_Turning = 20.0;//判断直行还是转向的角度阈值
        public int Number_of_Overlap_Road=1;//1024 如果出现重复路段，该变量选择当前处在第几圈，1代表第一圈  目前只支持2圈重合
        public long Index_In_StopCarArea;//进入停车区的点的序号1025
        public long Index_Out_StopCarArea;//驶出停车区的点的序号1025
        public long[] Index_Align = new long[MainForm.NAVIGATIONPOINTMAX];//判断是否校正的序号1028
        public long present_point_index = 0;
        long last_present_point_index = 0;
        double distance_for_search_index = 60.0;//在当前点前后左右各60米找寻发送的点，并确认start_search_index，end_search_index
        int index_difference_for_present_index = 100;//按序号找点方法的区间大小
        int min_index;
        int second_min_index;
        //清空相应变量
        
        public void dispose()
        {
            if (WholeNumberOfNavigationPoints > MainForm.NAVIGATIONPOINTMAX)
            {
                WholeNumberOfNavigationPoints = MainForm.NAVIGATIONPOINTMAX;
            }
            for (int i = 0; i < WholeNumberOfNavigationPoints; i++)
            {
                x_new[i] = 0.0;
                y_new[i] = 0.0;
            }
            for (int i = 0; i < n; i++)
            {
                angle[i] = 0.0;
                width[i] = 0.0;
            }
            x = 0.0;
            y = 0.0;
            n = 0;
            WholeNumberOfNavigationPoints = 0;
            start_search_index = 0;
            end_search_index = 0;
            present_point_index = 0;
        }
        //生成环岛轨迹，x,y为路口坐标，r为环岛半径
        public int GetStatus(double anglestart, double angleend )
        {
            double ang_temp = angleend -anglestart;
            //判断是否为直行
            if (Math.Abs(Math.Abs(ang_temp) - Math.PI) < (Angle_For_Turning*Math.PI/180.0))   //出角与入角差值在180度左右（45度-阈值）
            {
                return MainForm.GOAHEAD;
            }
            else
            {
                if (ang_temp < 0)
                    ang_temp = ang_temp + 2 * Math.PI;//将azimuth转换到[0,2pi]
                if (ang_temp < Math.PI)
                    return  MainForm.TURNLEFT ;   //1 左转
                else
                    return MainForm.TURNRIGHT;   //右转
            }
        }
        //通过两个点的插值产生环岛导航点，目前未用
        public void TwopointRoundabout(double x, double y, double r, double anglestart, double angleend)
        {
            
            double angle_temp1 = anglestart - angleend;
            if (angle_temp1 < 0)
            {
                angle_temp1 += 2 * Math.PI;
            }
            int k = Convert.ToInt32(Math.Abs(angle_temp1) * r / 0.4);
            double y_temp, x_temp;
            for (int j = 0; j < k + 1; j++)
            {
                y_temp = y + r * Math.Cos(anglestart - j * 0.4 / r);
                x_temp = x + r * Math.Sin(anglestart - j * 0.4 / r);
                x_new[WholeNumberOfNavigationPoints] = x_temp;
                y_new[WholeNumberOfNavigationPoints] = y_temp;
                WholeNumberOfNavigationPoints++;
            }
        }
        //生成道路的出点和入点，startroad是起始道路的序号，endroad是终止道路的序号
        public void create(int startroad, int endroad)
        {
            double[] xl = new double[n];
            double[] yl = new double[n];
            double[] xr = new double[n];
            double[] yr = new double[n];
            double[] a = new double[n];
            double[] b = new double[n];
            double[] cr = new double[n];
            double[] cl = new double[n];
            double[] xedger = new double[n];
            double[] yedger = new double[n];
            double[] xedgel = new double[n];
            double[] yedgel = new double[n];
            double[] xroadl = new double[n];
            double[] yroadl = new double[n];
            double[] xroadr = new double[n];
            double[] yroadr = new double[n];
            //分别将道路中心点沿着道路方向左右移动width；
            sort();
            for (int i = 0; i < n; i++)
            {
                xl[i] = x + width[index[i]] * Math.Cos(angle[index[i]]);
                yl[i] = y - width[index[i]] * Math.Sin(angle[index[i]]);
                xr[i] = x - width[index[i]] * Math.Cos(angle[index[i]]);
                yr[i] = y + width[index[i]] * Math.Sin(angle[index[i]]);
            }
            for (int i = 0; i < n; i++)
            {
                a[i] = Math.Cos(angle[index[i]]);
                b[i] = -Math.Sin(angle[index[i]]);
                cl[i] = Math.Cos(angle[index[i]]) * xl[i] - Math.Sin(angle[index[i]]) * yl[i];
                cr[i] = Math.Cos(angle[index[i]]) * xr[i] - Math.Sin(angle[index[i]]) * yr[i];
            }
            //得到道路的边界点
            for (int i = 0; i < n; i++)
            {
                int ia = i - 1;
                int ib = i + 1;
                if (ia < 0)
                    ia = n - 1;
                if (ib > n - 1)
                    ib = 0;
                if (Math.Abs(a[ib] * b[i] - a[i] * b[ib]) < 0.001)
                {
                    xedgel[i] = (cl[i] * b[ia] - cl[ia] * b[i]) / (a[i] * b[ia] - a[ia] * b[i]);
                    yedgel[i] = (cl[i] * a[ia] - cl[ia] * a[i]) / (a[ia] * b[i] - a[i] * b[ia]);
                }
                else
                {
                    xedgel[i] = (cr[ib] * b[i] - cl[i] * b[ib]) / (a[ib] * b[i] - a[i] * b[ib]);
                    yedgel[i] = (cr[ib] * a[i] - cl[i] * a[ib]) / (a[i] * b[ib] - a[ib] * b[i]);
                }

                if (Math.Abs(a[ia] * b[i] - a[i] * b[ia]) < 0.001)
                {
                    xedger[i] = (cr[i] * b[ib] - cr[ib] * b[i]) / (a[i] * b[ib] - a[ib] * b[i]);
                    yedger[i] = (cr[i] * a[ib] - cr[ib] * a[i]) / (a[ib] * b[i] - a[i] * b[ib]);
                }
                else
                {
                    xedger[i] = (cl[ia] * b[i] - cr[i] * b[ia]) / (a[ia] * b[i] - a[i] * b[ia]);
                    yedger[i] = (cl[ia] * a[i] - cr[i] * a[ia]) / (a[i] * b[ia] - a[ia] * b[i]);
                }

            }
            //得到道路的进入点和出点
            for (int i = 0; i < n; i++)
            {
                xroadl[i] = (xedgel[i] * 3 + xedger[i]) / 4;
                yroadl[i] = (yedgel[i] * 3 + yedger[i]) / 4;
                xroadr[i] = (xedgel[i] + xedger[i] * 3) / 4;
                yroadr[i] = (yedgel[i] + yedger[i] * 3) / 4;
            }
            if (startroad >= 0 && startroad < n && endroad >= 0 && endroad < n)
            {
                startx = xroadr[reverse_index[startroad]];
                starty = yroadr[reverse_index[startroad]];
                endx = xroadl[reverse_index[endroad]];
                endy = yroadl[reverse_index[endroad]];
            }
            else
            {
                startx = -3;
                starty = -3;
                endx = -3;
                endy = -3;
            }
        }
        public void hermite(double startx, double starty, double endx, double endy, double anglestart, double angleend, double distance)//赫米特插值
        {
            int steps = Convert.ToInt32(Math.Sqrt((endx - startx) * (endx - startx) + (endy - starty) * (endy - starty)) / step_lenth);
            if (steps == 0)
            {
                return;
            }
            double s, h1, h2, h3, h4;
            for (int t = 0; t < steps; t++)
            {
                s = (double)t / (double)steps;    //scale s to go from 0 to 1
                h1 = 2 * s * s * s - 3 * s * s + 1;          //calculate basis function 1
                h2 = -2 * s * s * s + 3 * s * s;              //calculate basis function 2
                h3 = s * s * s - 2 * s * s + s;           //calculate basis function 3
                h4 = s * s * s - s * s;                //calculate basis function 4
                x_new[WholeNumberOfNavigationPoints] = h1 * startx + h2 * endx + h3 * distance * Math.Cos(anglestart) + h4 * distance * Math.Cos(angleend);
                y_new[WholeNumberOfNavigationPoints] = h1 * starty + h2 * endy + h3 * distance * Math.Sin(anglestart) + h4 * distance * Math.Sin(angleend);
                WholeNumberOfNavigationPoints++;
            }
        }
        //按照angle的大小对道路进行排序，并将顺序序号保存在index中
        private void sort()
        {
            double[] angle_temp = new double[n];
            double temp = 0.0;
            int temp_int = 0;
            for (int i = 0; i < n; i++)
            {
                angle_temp[i] = angle[i];
                index[i] = i;
            }
            for (int i = 0; i < n; i++)
            {
                for (int j = i+1; j < n; j++)
                {
                    if (angle_temp[i] > angle_temp[j])
                    {
                        temp = angle_temp[i];
                        angle_temp[i] = angle_temp[j];
                        angle_temp[j] = temp;
                        temp_int = index[i];
                        index[i] = index[j];
                        index[j] = temp_int;
                    }
                }
            }
            for (int i = 0; i < n; i++)
            {
                reverse_index[index[i]] = i;
            }
        }
        //通过入弯点和出弯点生成圆弧导航路径点
        public void TwopointCircle(double startx, double starty, double endx, double endy, double anglestart, double angleend)//构造行驶轨迹
        {

            if (Math.Abs(anglestart - angleend) < (Angle_For_Turning * Math.PI / 180.0) || Math.Abs(Math.Abs(anglestart - angleend) - Math.PI) < (Angle_For_Turning*Math.PI/180.0))//两条道路平行，则行驶轨迹为连接起点和终点的直线
            {
                if (Math.Abs(Math.Abs(anglestart - angleend) - Math.PI) < (Angle_For_Turning * Math.PI / 180.0))
                {
                    TwoPointLine(startx, starty, endx, endy);
                    status = MainForm.GOAHEAD;
                }
                else
                {
                    TwoPointLine(startx, starty, endx, endy);
                    status = MainForm.TURNLEFT;
                }
            }
            else
            {
                double a_start = Math.Cos(anglestart);
                double b_start = -Math.Sin(anglestart);
                double a_end = Math.Cos(angleend);
                double b_end = -Math.Sin(angleend);
                double c_start = a_start * startx + b_start * starty;
                double c_end = a_end * endx + b_end * endy;
                double x = (c_start * b_end - c_end * b_start) / (a_start * b_end - a_end * b_start);
                double y = (c_start * a_end - c_end * a_start) / (a_end * b_start - a_start * b_end);//求得两条路的交点
                double l1 = Math.Sqrt((x - startx) * (x - startx) + (y - starty) * (y - starty));//交点到起点的距离
                double l2 = Math.Sqrt((x - endx) * (x - endx) + (y - endy) * (y - endy));//交点到终点的距离
                double ang_temp = anglestart - angleend;
                double r, xl, yl, x1, y1;//内切圆的半径和圆心的坐标
                double circle_length;//圆弧轨迹点之间的角度间隔
                if (ang_temp < 0)
                    ang_temp = -ang_temp;
                if (ang_temp > Math.PI)
                    ang_temp = 2 * Math.PI - ang_temp;//将ang_temp转换到[0,pi];
                double azimuth = angleend - anglestart;
                if (azimuth < 0)
                    azimuth = azimuth + 2 * Math.PI;//将azimuth转换到[0,2pi]
                if (azimuth < Math.PI)
                    status = MainForm.TURNLEFT;  //左转
                else
                    status = MainForm.TURNRIGHT;//右转
                if (l1 <= l2)
                {

                    r = l1 * Math.Tan(ang_temp / 2.0);//圆的半径
                    xl = x + l1 * Math.Sin(angleend);
                    yl = y + l1 * Math.Cos(angleend);
                    c_start = b_start * startx - a_start * starty;
                    c_end = b_end * xl - a_end * yl;
                    y1 = (c_start * b_end - c_end * b_start) / (a_end * b_start - a_start * b_end);
                    x1 = (c_start * a_end - c_end * a_start) / (a_end * b_start - a_start * b_end);//圆心坐标
                    circle_length = step_lenth /r;
                    if (status == MainForm.TURNLEFT)//左转
                    {
                        int k = Convert.ToInt32((Math.PI - ang_temp) / circle_length);
                        double y_temp, x_temp;
                        for (int j = 0; j < k + 1; j++)
                        {
                            y_temp = y1 + r * Math.Cos(anglestart - Math.PI / 2.0 - j * circle_length);
                            x_temp = x1 + r * Math.Sin(anglestart - Math.PI / 2.0 - j * circle_length);
                            if (WholeNumberOfNavigationPoints < MainForm.NAVIGATIONPOINTMAX)
                            {
                                x_new[WholeNumberOfNavigationPoints] = x_temp;
                                y_new[WholeNumberOfNavigationPoints] = y_temp;
                                WholeNumberOfNavigationPoints++;
                                
                            }
                            else
                                break;
                        }
                        TwoPointLine(x_new[WholeNumberOfNavigationPoints - 1], y_new[WholeNumberOfNavigationPoints - 1], endx, endy);
                    }
                    else//右转
                    {
                        int k = Convert.ToInt32((Math.PI - ang_temp) / circle_length);
                        double y_temp, x_temp;
                        for (int j = 0; j < k + 1; j++)
                        {
                            y_temp = y1 + r * Math.Cos(anglestart + Math.PI / 2.0 + j *  circle_length);
                            x_temp = x1 + r * Math.Sin(anglestart + Math.PI / 2.0 + j *  circle_length);
                            if (WholeNumberOfNavigationPoints < MainForm.NAVIGATIONPOINTMAX)
                            {
                                x_new[WholeNumberOfNavigationPoints] = x_temp;
                                y_new[WholeNumberOfNavigationPoints] = y_temp;
                                WholeNumberOfNavigationPoints++;                                
                            }
                            else
                                break;
                        }
                        TwoPointLine(x_new[WholeNumberOfNavigationPoints - 1], y_new[WholeNumberOfNavigationPoints - 1], endx, endy);
                    }
                }
                else
                {
                    r = l2 * Math.Tan(ang_temp / 2.0);
                    xl = x + l2 * Math.Sin(anglestart);
                    yl = y + l2 * Math.Cos(anglestart);
                    c_start = b_start * xl - a_start * yl;
                    c_end = b_end * endx - a_end * endy;
                    y1 = (c_start * b_end - c_end * b_start) / (a_end * b_start - a_start * b_end);
                    x1 = (c_start * a_end - c_end * a_start) / (a_end * b_start - a_start * b_end);
                    circle_length = step_lenth / r;
                    if (status == MainForm.TURNLEFT)//左转
                    {
                        int k = Convert.ToInt32((Math.PI - ang_temp) / circle_length);
                        double y_temp, x_temp;
                        TwoPointLine(startx, starty, r * Math.Sin(anglestart - Math.PI / 2.0) + x1, r * Math.Cos(anglestart - Math.PI / 2.0) + y1);
                        for (int j = 0; j < k + 1; j++)
                        {
                            y_temp = y1 + r * Math.Cos(anglestart - Math.PI / 2.0 - j * circle_length);
                            x_temp = x1 + r * Math.Sin(anglestart - Math.PI / 2.0 - j * circle_length);
                            if (WholeNumberOfNavigationPoints < MainForm.NAVIGATIONPOINTMAX)
                            {
                                x_new[WholeNumberOfNavigationPoints] = x_temp;
                                y_new[WholeNumberOfNavigationPoints] = y_temp;
                                WholeNumberOfNavigationPoints++;                                
                            }
                            else
                                break;
                        }

                    }
                    else//右转
                    {
                        int k = Convert.ToInt32((Math.PI - ang_temp) /circle_length);
                        double y_temp, x_temp;
                        TwoPointLine(startx, starty, r * Math.Sin(anglestart + Math.PI / 2.0) + x1, r * Math.Cos(anglestart + Math.PI / 2.0) + y1);
                        for (int j = 0; j < k + 1; j++)
                        {
                            y_temp = y1 + r * Math.Cos(anglestart + Math.PI / 2.0 + j * circle_length);
                            x_temp = x1 + r * Math.Sin(anglestart + Math.PI / 2.0 + j * circle_length);
                            if (WholeNumberOfNavigationPoints < MainForm.NAVIGATIONPOINTMAX)
                            {
                                x_new[WholeNumberOfNavigationPoints] = x_temp;
                                y_new[WholeNumberOfNavigationPoints] = y_temp;
                                WholeNumberOfNavigationPoints++;                                
                            }
                            else
                                break;
                        }

                    }
                }
            }

        }
        private double TwoPointLine_y(double x0, double x1, double y0, double y1, double x)
        {
            double y = (x - x0) / (x1 - x0) * (y1 - y0) + y0;
            return y;
        }

        private double TwoPointLine_x(double x0, double x1, double y0, double y1, double y)
        {
            double x = (y - y0) / (y1 - y0) * (x1 - x0) + x0;
            return x;
        }
        //两点直线插入导航点
        public void TwoPointLine(double startx,double starty,double endx,double endy)//构造两点直线
        {   
            double lenth = Math.Sqrt((startx - endx) * (startx - endx) + (starty - endy) * (starty - endy));
            double k = lenth / step_lenth;

            if (Math.Abs(k) >= 1.0)
            {
                if (Math.Abs(startx - endx) > Math.Abs(starty - endy))
                {
                    for (int j = 1; j < k; j++)
                    {
                        double x_temp = startx + j / k * (endx - startx);
                        double y_temp = TwoPointLine_y(startx, endx, starty, endy, x_temp);
                        if (WholeNumberOfNavigationPoints < MainForm.NAVIGATIONPOINTMAX)
                        {
                            x_new[WholeNumberOfNavigationPoints] = x_temp;
                            y_new[WholeNumberOfNavigationPoints] = y_temp;
                            WholeNumberOfNavigationPoints++;
                        }
                        else
                            break;
                    }
                }
                else
                {
                    for (int j = 1; j < k; j++)
                    {
                        double y_temp = starty + j / k * (endy - starty);
                        double x_temp = TwoPointLine_x(startx, endx, starty, endy, y_temp);
                        if (WholeNumberOfNavigationPoints < MainForm.NAVIGATIONPOINTMAX)
                        {
                            x_new[WholeNumberOfNavigationPoints] = x_temp;
                            y_new[WholeNumberOfNavigationPoints] = y_temp;
                            WholeNumberOfNavigationPoints++;
                        }
                        else
                            break;
                    }
                }
            }
            else
            {
                x_new[WholeNumberOfNavigationPoints] = startx;
                y_new[WholeNumberOfNavigationPoints] = starty;
                WholeNumberOfNavigationPoints++;
            }

        }
        
        /********************************************************************
         * 全局+航相匹配搜索
         * 功能：找离当前点最近和次近的点（为了防止重复路段），
         * 用Number_of_Overlap_Road来选择是哪一段路段上的点
        ********************************************************************/
        private int find_present_index(double present_x, double present_y)//1024
        {
            double[] distance = new double[WholeNumberOfNavigationPoints];
            double [] copy_distance=new double [WholeNumberOfNavigationPoints];
            for (int k = 0; k < WholeNumberOfNavigationPoints; k++)
            {
                double angle = getangle(x_new[(k + 1) % WholeNumberOfNavigationPoints] - x_new[(k) % WholeNumberOfNavigationPoints], y_new[(k + 1) % WholeNumberOfNavigationPoints] - y_new[(k) % WholeNumberOfNavigationPoints]);
                if (Math.Abs(Math.Abs(angle - azimuth) - 180.0) < 30.0)//如果不满足航向匹配给个9999
                    distance[k] = 9999;
                else
                    distance[k] = Math.Sqrt((present_x - x_new[k]) * (present_x - x_new[k]) + (present_y - y_new[k]) * (present_y - y_new[k]));
                copy_distance[k] = distance[k];
            }
            Array.Sort(distance);
            min_index = 0;
            for (int i = 0; i < WholeNumberOfNavigationPoints; i++)
            {
                if (Math.Abs(copy_distance[i] - distance[0]) < 0.0001)
                {
                    min_index = i;
                    break;
                }
            }
            second_min_index = min_index;
            int index=1;
            while (Math.Abs(second_min_index - min_index) < 10)
            {
                double d = distance[index];
                for (int j = 0; j < WholeNumberOfNavigationPoints; j++)
                {
                    if (Math.Abs(copy_distance[j] - d) < 0.0001)
                    {
                        second_min_index = j;

                        break;
                    }
                }
                index++;
            }
            if (Number_of_Overlap_Road == 1)
            {
                int temp = min_index < second_min_index ? min_index : second_min_index;
                return temp;
            }
            else
            {
                int temp = min_index > second_min_index ? min_index : second_min_index;
                return temp;
            }
        }
        
        public void find_send_points(double present_x, double present_y,int Find_Whole)//找寻要发送的点
        {
            long start_index = 0;
            long end_index = 0;
            if (Find_Whole == 1)//1024
            {
                present_point_index = find_present_index(present_x, present_y);//1024
            }
            else
            {
                present_point_index = find_present_point(start_search_index, end_search_index, present_x, present_y, Find_Whole);
            }
            last_present_point_index = present_point_index;
            //向前找
            long i = present_point_index;
            if (WholeNumberOfNavigationPoints == 0)
            {
                return;
            }
            while ((x_new[(i + WholeNumberOfNavigationPoints) % WholeNumberOfNavigationPoints] > x_new[present_point_index] - distance_for_search_index) &&
                (x_new[(i + WholeNumberOfNavigationPoints) % WholeNumberOfNavigationPoints] < x_new[present_point_index] + distance_for_search_index) &&
                (y_new[(i + WholeNumberOfNavigationPoints) % WholeNumberOfNavigationPoints] > y_new[present_point_index] - distance_for_search_index) &&
                (y_new[(i + WholeNumberOfNavigationPoints) % WholeNumberOfNavigationPoints] < y_new[present_point_index] + distance_for_search_index) && i > 0)
            {
                start_index = i;
                //start_index = (i + WholeNumberOfNavigationPoints) % WholeNumberOfNavigationPoints;
                i--;
            }
            //向后找
            i = present_point_index;
            while ((x_new[(i + WholeNumberOfNavigationPoints) % WholeNumberOfNavigationPoints] > x_new[present_point_index] - distance_for_search_index) &&
                (x_new[(i + WholeNumberOfNavigationPoints) % WholeNumberOfNavigationPoints] < x_new[present_point_index] + distance_for_search_index) &&
                (y_new[(i + WholeNumberOfNavigationPoints) % WholeNumberOfNavigationPoints] > y_new[present_point_index] - distance_for_search_index) &&
                (y_new[(i + WholeNumberOfNavigationPoints) % WholeNumberOfNavigationPoints] < y_new[present_point_index] + distance_for_search_index) && i < WholeNumberOfNavigationPoints)
            {
                end_index = i % WholeNumberOfNavigationPoints; 
                //end_index = (i + WholeNumberOfNavigationPoints) % WholeNumberOfNavigationPoints;
                i++;
            }
            if (present_point_index == WholeNumberOfNavigationPoints)
            {
                start_search_index = 0;
                end_search_index = 600;
            }
            else
            {
                start_search_index = start_index;
                end_search_index = end_index;
            }
        }
        //bool find_points = false;
        //找寻当前点的序号,Find_Whole三种状态，0代表按序号找寻，1代表全局找+航向匹配，2代表全局找（没有航向匹配）
        private long find_present_point(long start, long end, double present_x, double present_y, int Find_Whole)
        {

            long _present_point_index = start;
            double d = Math.Sqrt((present_x - x_new[start]) * (present_x - x_new[start]) + (present_y - y_new[start]) * (present_y - y_new[start]));
            double d_min = d;
            //find_points = false;
            if (WholeNumberOfNavigationPoints != 0)
            {
                for (long i = 0; i < (end - start + WholeNumberOfNavigationPoints) % WholeNumberOfNavigationPoints; i++)
                {
                    d = Math.Sqrt((present_x - x_new[(i + start + 1) % WholeNumberOfNavigationPoints]) * (present_x - x_new[(i + start + 1) % WholeNumberOfNavigationPoints]) + (present_y - y_new[(i + start + 1) % WholeNumberOfNavigationPoints]) * (present_y - y_new[(i + start + 1) % WholeNumberOfNavigationPoints]));
                    double angle = getangle(x_new[(i + start + 1) % WholeNumberOfNavigationPoints] - x_new[(i + start) % WholeNumberOfNavigationPoints], y_new[(i + start + 1) % WholeNumberOfNavigationPoints] - y_new[(i + start) % WholeNumberOfNavigationPoints]);
                    
                    if (Find_Whole==1)
                    {
                        //航向匹配
                        //if (Math.Abs(Math.Abs(angle - azimuth) - 180.0) < 30.0 || i + start + 1 == WholeNumberOfNavigationPoints)
                        //    continue;
                        if (d < d_min)
                        {
                            d_min = d;
                            _present_point_index = (i + start + 1) % WholeNumberOfNavigationPoints;
                            //find_points = true;
                        }
                    }
                    if (Find_Whole == 2)
                    {
                        if (d < d_min)
                        {
                            d_min = d;
                            _present_point_index = (i + start + 1) % WholeNumberOfNavigationPoints;
                            //find_points = true;
                        }
                    }
                    if(Find_Whole==0)
                    {
                        //1025
                        //if (Math.Abs(Math.Abs(angle - azimuth) - 180.0) < 30.0 || i + start + 1 == WholeNumberOfNavigationPoints)
                        //    continue;
                        if (d < d_min && Math.Abs(i + start + 1 - last_present_point_index) < index_difference_for_present_index)
                        {
                            d_min = d;
                            _present_point_index = (i + start + 1) % WholeNumberOfNavigationPoints;
                            //find_points = true;
                        }
                    }
                    ///////deug
                }
            }
            //if (find_points)
            //{
           
            return _present_point_index;
            //}
            //else
            //{
            //    return present_point_index;
            //}
        }
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

                            return (Math.Atan(tangle) /Math.PI * 180);
                        }
                        else
                        {
                            return (Math.Atan(tangle) / Math.PI * 180 + 180.0);
                        }
                    }
                    else
                    {
                        if (detay > 0.0)
                        {
                            //右侧象限

                            return (Math.Atan(tangle) / Math.PI * 180 + 360.0);
                        }
                        else
                        {
                            return (Math.Atan(tangle) / Math.PI * 180 + 180.0);
                        }

                    }
                }
            }
        }
    }  
}
