using System;
using System.Collections.Generic;
using System.Text;

namespace PathEx_Analysis
{
    class CRoadWayCreater
    {

        public double[] x = new double[65535];
        public double[] y = new double[65535];
        public int num = 0;
        public bool flag = false;
        public double[] x_new = new double[100000];
        public double[] y_new = new double[100000];
        public long num_new = 0;
        public double all_lenth = 0;
        public double step_lenth = 0.4;
        static double p = Math.PI;
        public long start_search_index = 0;
        public long end_search_index = 0;
        public double azimuth = 0.0;
        public long[] index_in_new = new long[65535];
        public long old_index = 0;
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

        public void creat(double present_x, double present_y)
        {
           

            for (int i = 0; i < num - 1; i++)
            {
                double lenth = Math.Sqrt((x[i + 1] - x[i]) * (x[i + 1] - x[i]) + (y[i + 1] - y[i]) * (y[i + 1] - y[i]));
                double k = lenth / step_lenth;
                if (Math.Abs(x[i + 1] - x[i]) > Math.Abs(y[i + 1] - y[i]))
                {
                    for (int j = 0; j < k; j++)
                    {
                        double x_temp = x[i] + j / k * (x[i + 1] - x[i]);
                        double y_temp = TwoPointLine_y(x[i], x[i + 1], y[i], y[i + 1], x_temp);
                        x_new[num_new] = x_temp;
                        y_new[num_new] = y_temp;
                        num_new++;
                    }
                }
                else
                {
                    for (int j = 0; j < k; j++)
                    {
                        double y_temp = y[i] + j / k * (y[i + 1] - y[i]);
                        double x_temp = TwoPointLine_x(x[i], x[i + 1], y[i], y[i + 1], y_temp);
                        x_new[num_new] = x_temp;
                        y_new[num_new] = y_temp;
                        num_new++;
                    }
                }
                index_in_new[i+1] = num_new;
                all_lenth += lenth;
            }
            start_search_index = 0;
            end_search_index = num_new;
            find_send_points(present_x, present_y);
        }
        public void dispose()
        {
            for (int i = 0; i < num; i++)
            {
                x[i] = 0.0;
                y[i] = 0.0;
                index_in_new[i] = 0;
            }
            for (int i = 0; i < num_new; i++)
            {
                x_new[i] = 0.0;
                y_new[i] = 0.0;

            }
            num = 0;
            num_new = 0;
            start_search_index = 0;
            all_lenth = 0.0;
            end_search_index = 0;
            present_point_index = 0;

        }
        bool find_points = false;
        private long find_present_point(long start, long end, double present_x, double present_y)
        {
            
            long _present_point_index = start;
            double d = Math.Sqrt((present_x - x_new[start]) * (present_x - x_new[start]) + (present_y - y_new[start]) * (present_y - y_new[start]));
            double d_min = d;
            double angle = new double();
            double deta_anle = new double();
            find_points = false;
            for (long i = start + 1; i < end; i++)
            {
                d = Math.Sqrt((present_x - x_new[i]) * (present_x - x_new[i]) + (present_y - y_new[i]) * (present_y - y_new[i]));
                if (d < d_min)
                {
                    if (!flag)
                    {
                        d_min = d;
                        _present_point_index = i;
                        find_points=true;
                    }
                    else
                    {
                        angle = getangle(x_new[i + 1] - x_new[i], y_new[i + 1] - y_new[i]);
                        deta_anle = angle - azimuth;
                        if (deta_anle > 180.0)
                        {
                            deta_anle -= 360.0;
                        }
                        else if (deta_anle < -180.0)
                        {
                            deta_anle += 360.0;
                        }
                        if (Math.Abs(deta_anle) < 90.0)
                        {
                            find_points = true;
                            d_min = d;
                            _present_point_index = i;
                        }
                    }
                    }
                    
            }
            if (find_points)
            {
                return _present_point_index;
            }
            else
            {
                return present_point_index;
            }
        }

        private double getangle(double detax, double detay)
        {
            double tangle = new double();
            tangle = detax / detay;
            if (Math.Abs(detax) < 0.0001)
            {
                if (detay > 0)
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
                    if (detax > 0)
                    {
                        if (detay > 0)
                        {
                            //右侧象限

                            return (Math.Atan(tangle) / p * 180);
                        }
                        else
                        {
                            return (Math.Atan(tangle) / p * 180 + 180.0);
                        }
                    }
                    else
                    {
                        if (detay > 0)
                        {
                            //右侧象限

                            return (Math.Atan(tangle) / p * 180 + 360.0);
                        }
                        else
                        {
                            return (Math.Atan(tangle) / p * 180 + 180.0);
                        }

                    }
                }
            }
        }

        public long present_point_index = 0;
        public void find_send_points(double present_x, double present_y)
        {
            long start_index = 0;
            long end_index = 0;
            present_point_index = find_present_point(start_search_index, end_search_index, present_x, present_y);
            //向前找
            long i = present_point_index;
            while ((x_new[i] > x_new[present_point_index] - 60.0) &&
                (x_new[i] < x_new[present_point_index] + 60.0) &&
                (y_new[i] > y_new[present_point_index] - 60.0) &&
                (y_new[i] < y_new[present_point_index] + 60.0) &&
                i > 0)
            {
                start_index = i;
                i--;
            }
            //向后找
            i = present_point_index;
            while ((x_new[i] > x_new[present_point_index] - 60.0) &&
                (x_new[i] < x_new[present_point_index] + 60.0) &&
                (y_new[i] > y_new[present_point_index] - 60.0) &&
                (y_new[i] < y_new[present_point_index] + 60.0) &&
                i < num_new)
            {
                end_index = i;
                i++;
            }
            start_search_index = start_index;
            end_search_index = end_index;
        }

    }
}
