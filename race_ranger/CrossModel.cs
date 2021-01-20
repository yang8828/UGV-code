using System;
using System.Collections.Generic;
using System.Text;

namespace PathEx_Analysis
{
    class CrossModel
    {
        public double x;
        public double y;//道路中心点坐标
        public double[] angle = new double[10];//每条道路的方向角
        public double[] width = new double[10];//每条道路的宽度
        public int n;//此路口包含的道路数量
        public double[] x_new = new double[10000];
        public double[] y_new = new double[10000];//生成的行驶线路坐标
        public int n_new = 0;

        //生成行驶路线，crossdirection表示转弯方向，startroad是起始道路的序号，endroad是终止道路的序号
        public void CRoadWayCreater(int crossdirection,int startroad,int endiroad)
        {
            double[] xl = new double[n];
            double[] yl = new double[n];
            double[] xr = new double[n];
            double[] yr = new double[n];
            //分别将道路中心点沿着道路方向左右移动width；
            for (int i = 0; i < n; i++)
			{
                xl[i] = x + width[i] * Math.Cos(angle[i]);
                yl[i] = y - width[i] * Math.Sin(angle[i]); 
                xr[i] = x - width[i] * Math.Cos(angle[i]);
                yr[i] = y + width[i] * Math.Sin(angle[i]);
			}
            

        }
    }
}
