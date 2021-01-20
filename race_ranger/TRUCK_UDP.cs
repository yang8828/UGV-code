using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

//本段代码中需要新增加的命名空间
using System.Net.Sockets;
using System.Net;
using System.Threading;

namespace PathEx_Analysis
{
    class TRUCK_UDP
    {
        public struct TRUCK_TO_CAR_MESSAGE
        {
            public long id;
            public double x;
            public double y;
            public int gear;
            public double velocity;
            public int ins_status;
        };

        public struct CAR_TO_TRUCK_MESSAGE
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 30)]
            public TRUCK_TO_CAR_MESSAGE[] send_to_truck;
        };

        private string local_ip = "192.168.0.102";
        private string remote_ip = "192.168.0.252";

        private UdpClient udpcSend;
        private UdpClient udpcRecv;
        public TRUCK_TO_CAR_MESSAGE recvData;

        public delegate void UDPRefreshEventHandler(TRUCK_TO_CAR_MESSAGE udp_msg);
        public event UDPRefreshEventHandler OnUDPRefresh;
        
        public TRUCK_UDP()
        {
            dataRecv();
            IPEndPoint localIpep = new IPEndPoint(
                IPAddress.Parse(local_ip), 12345); // 本机IP，指定的端口号
            udpcSend = new UdpClient(localIpep);
        }

        ///发送数据
        public void dataSend(CAR_TO_TRUCK_MESSAGE message)
        {
            // 实名发送
            

            // 填写数据
            byte[] sendbytes = struct2Bytes(message);

            IPEndPoint remoteIpep = new IPEndPoint(
                IPAddress.Parse(remote_ip), 41001); // 发送到的IP地址和端口号
            udpcSend.Send(sendbytes, sendbytes.Length, remoteIpep);
           
        }

        // 结构体序列化
        public static byte[] struct2Bytes(CAR_TO_TRUCK_MESSAGE structObj)
        {
            int size = Marshal.SizeOf(structObj);
            byte[] bytes = new byte[size];
            IntPtr structPtr = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(structObj, structPtr, false);
            Marshal.Copy(structPtr, bytes, 0, size);
            Marshal.FreeHGlobal(structPtr);
            return bytes;       
        }

        // 结构体解序列化
        public static object byte2ToStruct(byte[] bytes, Type type)
        {
            int size = Marshal.SizeOf(type);
            if (size > bytes.Length)
            {
                return null;
            }
            IntPtr structPtr = Marshal.AllocHGlobal(size);
            Marshal.Copy(bytes, 0, structPtr, size);
            object obj = Marshal.PtrToStructure(structPtr, type);
            Marshal.FreeHGlobal(structPtr);
            return obj;
        }

        /// 开关：在监听UDP报文阶段为true，否则为false
        bool IsUdpcRecvStart = false;
        /// 线程：不断监听UDP报文
        Thread thrRecv;

        /// 按钮：接收数据开关
        public void dataRecv()
        {
            if (!IsUdpcRecvStart) // 未监听的情况，开始监听
            {
                IPEndPoint localIpep = new IPEndPoint(
                    IPAddress.Parse(local_ip), 41000); // 本机IP和监听端口号

                udpcRecv = new UdpClient(localIpep);

                thrRecv = new Thread(ReceiveMessage);
                thrRecv.Start();

                IsUdpcRecvStart = true;
            }
            else // 正在监听的情况，终止监听
            {
                thrRecv.Abort(); // 必须先关闭这个线程，否则会异常
                udpcRecv.Close();

                IsUdpcRecvStart = false;
            }
        }

        /// 接收数据
        public void ReceiveMessage(object obj)
        {
            IPEndPoint remoteIpep = new IPEndPoint(IPAddress.Any, 0);
            while (true)
            {
                    byte[] bytRecv = udpcRecv.Receive(ref remoteIpep);
                    recvData = (TRUCK_TO_CAR_MESSAGE)byte2ToStruct(bytRecv, typeof(TRUCK_TO_CAR_MESSAGE));

                    if (OnUDPRefresh != null)
                    {
                        OnUDPRefresh(recvData);
                    }
            }
        }
        public void Close()
        {
            udpcSend.Close();
        }
    }
}
