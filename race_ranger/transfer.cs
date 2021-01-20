using System;
using System.Collections.Generic;
using System.Text;
using SuperMapLib;
using System.Windows.Forms;
namespace PathEx_Analysis
{
    public class transfer
    {
        public double[] resultx = new double[1000];
        public double[] resulty = new double[1000];
        public double[] arrx = new double[1000];
        public double[] arry = new double[1000];
        public int[] xNavigationPointCoordianteToBeSent = new int[301];
        public int[] yNavigationPointCoordianteToBeSent = new int[301];
        public int[] x_temp = new int[1000];
        public int[] y_temp = new int[1000];
        public _soPoint[] rds = new _soPoint[1300];
        public int length = 0;
        private bool FlagPointOut60L60WFrame;
        private _soPoint temp_point;
        public long start_index;
        public long end_index;
        public double Getdistance(_soPoint goal, _soPoint V)
        {
            double deltax = goal.x - V.x;
            double deltay = goal.y - V.y;
            return Math.Sqrt(deltax * deltax + deltay * deltay);//得到距离

        }
        public double Getdistance(double x1,double y1, RoadpointReader.TaskFileData V)
        {
            double deltax = x1 - V.x;
            double deltay = y1 - V.y;
            return Math.Sqrt(deltax * deltax + deltay * deltay);//得到距离

        }
        public double Getdistance(_soPoint goal, soGeoPoint V)
        {
            double deltax = goal.x - V.x;
            double deltay = goal.y - V.y;
            return Math.Sqrt(deltax * deltax + deltay * deltay);//得到距离
        }
        public double Getdistance(RoadpointReader.TaskFileData goal, soGeoPoint V)
        {
            double deltax = goal.x - V.x;
            double deltay = goal.y - V.y;
            return Math.Sqrt(deltax * deltax + deltay * deltay);//得到距离
        }
        public struct _soPoint
        {
            public double x;
            public double y;
        }
        /**************  根据当前点以及航向角计算载体坐标系的导航点坐标 ****************
         * 输入：(1) 当前点CurrentPoint (2) 导航点NavigationPoints (3) 航向
         * 输出：x[301],y[301] 共301个点，车中心点坐标（30*50，20*50），左下为（0，0），右上为（60*50，60*50）；
         * *************************************************************/
        public void ComputeNavigationPointCoordinateToBeSent(_soPoint CurrentPoint, RoadModel NavigationPoints, double azimuth)
        {
            long j = 0;
            start_index = 0;
            end_index = 0;
            FlagPointOut60L60WFrame = false;
            long index = 0;
            //int road_index = 0;
            if (NavigationPoints.WholeNumberOfNavigationPoints == 0)
            {
                return;
            }

            long sum = (NavigationPoints.end_search_index + NavigationPoints.WholeNumberOfNavigationPoints - NavigationPoints.start_search_index) % (NavigationPoints.WholeNumberOfNavigationPoints);
            for (long i = 0; i < sum; i++)
            {
                if (j < 1000)//数据定义为1000
                {
                    resultx[j] = NavigationPoints.x_new[(i + NavigationPoints.start_search_index) % (NavigationPoints.WholeNumberOfNavigationPoints)];
                    resulty[j] = NavigationPoints.y_new[(i + NavigationPoints.start_search_index) % (NavigationPoints.WholeNumberOfNavigationPoints)];
                    j++;
                }
                else
                {
                    break;
                }
            }
            /********** 对当前点向车后方的点进行赋值 ***********/
            //index-当前在result中的序号
            index = (NavigationPoints.present_point_index - NavigationPoints.start_search_index + NavigationPoints.WholeNumberOfNavigationPoints) % (NavigationPoints.WholeNumberOfNavigationPoints);
            //对未出60*60矩形框的点进行转换
            while ((FlagPointOut60L60WFrame==false) && index > -1&&index<1000)
            {   //根据车辆航向角将输入坐标相对车辆坐标的坐标差值（大地坐标）转化成载体坐标系下的坐标
                temp_point = ComputeBodyRelativeCoorAccordintCoordiateDiffToUGVCenter(resultx[index] - CurrentPoint.x, resulty[index] - CurrentPoint.y, azimuth);
                x_temp[index] = Convert.ToInt32(temp_point.x);
                y_temp[index] = Convert.ToInt32(temp_point.y);
                index--;
            }
            if (index > 0)
            {
                start_index = index + 1;
            }
            else
            {
                start_index = 0;
            }
            /********** 对当前点向车前方的点进行赋值 ***********/
            index = (NavigationPoints.present_point_index - NavigationPoints.start_search_index + NavigationPoints.WholeNumberOfNavigationPoints) % (NavigationPoints.WholeNumberOfNavigationPoints);
            FlagPointOut60L60WFrame = false;
            while ((!FlagPointOut60L60WFrame) && index < (NavigationPoints.end_search_index - NavigationPoints.start_search_index + NavigationPoints.WholeNumberOfNavigationPoints) % NavigationPoints.WholeNumberOfNavigationPoints&&index<1000)
            {
                temp_point = ComputeBodyRelativeCoorAccordintCoordiateDiffToUGVCenter(resultx[index] - CurrentPoint.x, resulty[index] - CurrentPoint.y, azimuth);
                x_temp[index] = Convert.ToInt32(temp_point.x);
                y_temp[index] = Convert.ToInt32(temp_point.y);
                index++;
            }
            end_index = index-1;


            if ((end_index - start_index < 301) && (end_index - start_index >= 0))
            {
                for (long i = start_index; i < end_index; i++)
                {
                    if (i < 1000)
                    {
                        xNavigationPointCoordianteToBeSent[i - start_index] = x_temp[i];
                        yNavigationPointCoordianteToBeSent[i - start_index] = y_temp[i];
                    }
                }
                /********** 当点数小于300个时，用最后一个点值填充后续所有点 ***********/
                for (long i = end_index; i < 301 + start_index; i++)
                {
                    if (i < 1000)
                    {
                        xNavigationPointCoordianteToBeSent[i - start_index] = x_temp[end_index];
                        yNavigationPointCoordianteToBeSent[i - start_index] = y_temp[end_index];
                    }
                }
            }
            else
            {
                for (long i = start_index; i < start_index + 301; i++)
                {
                    if (i < 1000)
                    {
                        xNavigationPointCoordianteToBeSent[i - start_index] = x_temp[i];
                        yNavigationPointCoordianteToBeSent[i - start_index] = y_temp[i];
                    }
                }
            }
        }

        /********* 根据车辆航向角将大地坐标差值转化成载体坐标系下的坐标 ************
         * 输入：导航点大地坐标与车辆大地坐标的坐标差，航向角（度为单位）
         * 输出：在下面的载体坐标系的导航点的坐标（3000*3000）
         * 载体坐标定义：车辆左下为(0,0),右上为(60,60)，车辆中心点(30,20)
         * **************************************************************************/
        public _soPoint ComputeBodyRelativeCoorAccordintCoordiateDiffToUGVCenter(double x, double y, double azimuth)
        {
            double resultX, resultY;
            long finalX, finalY;
            double cosa = Math.Cos(azimuth * Math.PI / 180);
            double sina = Math.Sin(azimuth * Math.PI / 180);
            //坐标旋转（大地->载体）载体中心坐标（0，0）
            resultX = x * cosa - y * sina;
            resultY = x * sina + y * cosa;//坐标转换
            //将坐标转换为载体中心坐标（30，20），并将（60，60）转换为（3000，3000）
            finalX = (long)(resultX * 50.0 + 1500);
            finalY = (long)(resultY * 50.0 + 1000);

            FlagPointOut60L60WFrame = false;
            if (finalX < 1)
            {
                finalX = 1;
                FlagPointOut60L60WFrame = true;
            }
            else if (finalX > 2999)
            {
                finalX = 2999;
                FlagPointOut60L60WFrame = true;
            }
            if (finalY < 1)
            {
                finalY = 1;
                FlagPointOut60L60WFrame = true;
            }
            else if (finalY > 2999)
            {
                finalY = 2999;
                FlagPointOut60L60WFrame = true;
            }

            _soPoint temp=new _soPoint();
            temp.x=finalX;
            temp.y=finalY;
            return temp;
        }

        //将平面坐标转换为车体坐标系
        public _soPoint tranfers(double x, double y, double azimuth)
        {
            double resultX, resultY;
            long finalX, finalY;
            double cosa = Math.Cos(azimuth * Math.PI / 180);
            double sina = Math.Sin(azimuth * Math.PI / 180);
            resultX = x * cosa - y * sina;
            resultY = x * sina + y * cosa;
            finalX = (long)(resultX * 50.0 + 1500);
            finalY = (long)(resultY * 50.0 + 1000);
            _soPoint temp = new _soPoint();
            temp.x = finalX;
            temp.y = finalY;
            return temp;
        }
    }
}
