using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;

namespace PathEx_Analysis
{
    public partial class INITForm : Form
    {
        System.Timers.Timer timer = new System.Timers.Timer();
        public INITForm()
        {
            InitializeComponent();
        }

        private void INITForm_Load(object sender, EventArgs e)
        {
            timer.Interval = 200;
            timer.Elapsed += new System.Timers.ElapsedEventHandler(timer_Elapsed);
            timer.Enabled = true;
        }

        void timer_Elapsed(object sender, System.Timers.ElapsedEventArgs e)
        {
            Console.WriteLine("GIS INIT...");
            Console.WriteLine("newl");
        }
    }
}
