using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;

namespace PathEx_Analysis
{
    public partial class MessageBoxTimer : Form
    {
        string header = "";
        string message = "";
        System.Timers.Timer timer = new System.Timers.Timer(1000);
        int counter = 0;
        int timerN = 3;
        public MessageBoxTimer(string Header, string Message, int TimerN = 3)
        {
            CheckForIllegalCrossThreadCalls = false;
            InitializeComponent();
            header = Header;
            message = Message;
            timerN = TimerN;
        }

        private void MessageBoxTimer_Load(object sender, EventArgs e)
        {
            this.Text = header;
            label1.Text = message;
            timer.Elapsed += new System.Timers.ElapsedEventHandler(timer_Elapsed);
            timer.Enabled = true;
            button1.Text = "Yes = " + (timerN - counter).ToString();
        }

        void timer_Elapsed(object sender, System.Timers.ElapsedEventArgs e)
        {
            counter++;
            button1.Text = "Yes = " + (timerN - counter).ToString();
            if (counter == timerN)
            {
                this.DialogResult = System.Windows.Forms.DialogResult.Yes;
            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            this.DialogResult = System.Windows.Forms.DialogResult.Yes;

        }

        private void button2_Click(object sender, EventArgs e)
        {
            this.DialogResult = System.Windows.Forms.DialogResult.No;
        }
    }
}
