using System;
using System.Text;
using Microsoft.Azure.Kinect.Sensor;
using Microsoft.Azure.Kinect.BodyTracking;
using System.IO;
using System.Numerics;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Timers;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;

namespace headlemmethod
{
    class Program
    {



        static (float roll, float pitch, float yaw) QuaternionToEulerAngles(double q0, double q1, double q2, double q3)
        {
            double q0q0 = q0 * q0;
            double q0q1 = q0 * q1;
            double q0q2 = q0 * q2;
            double q0q3 = q0 * q3;
            double q1q1 = q1 * q1;
            double q1q2 = q1 * q2;
            double q1q3 = q1 * q3;
            double q2q2 = q2 * q2;
            double q2q3 = q2 * q3;
            double q3q3 = q3 * q3;
            double roll = Math.Atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3);
            double pitch = Math.Asin(2.0 * (q0q2 - q1q3));
            double yaw = Math.Atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
            return ((float)roll, (float)pitch, (float)yaw);
        }

        static void Main(string[] args)
        {
            var que = new Queue<double> {};
            var quetime = new Queue<double> { };
            var sw1 = new System.Diagnostics.Stopwatch();
            int k = 0;


            Console.WriteLine("ESC:STOP");
            String now = DateTime.Now.ToString("MMddHHmmss");
            Console.WriteLine("Start Body Tracking App!");
            using (var server = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp))
            {
                server.Bind(new IPEndPoint(IPAddress.Any, 9999));
                server.Listen(20);
                // コンソールに出力
                Console.WriteLine("Server Start... Listen port 9999...");


                try
                {
                    //接続されている機器の数をチェック
                    if (Device.GetInstalledCount() == 0)
                    {
                        Console.WriteLine("No k4a devices attached!");
                        Console.ReadKey();
                        return;
                    }

                    //■デバイスを開いてカメラを起動する
                    Device device = null;
                    // Open the first plugged in Kinect device
                    try
                    {
                        //1台目に接続
                        device = Device.Open(0);
                    }
                    catch (AzureKinectOpenDeviceException ex)
                    {
                        Console.WriteLine("Failed to open k4a device!!");
                        Console.WriteLine(ex.Message);
                        Console.WriteLine(ex.StackTrace.ToString());
                        Console.ReadKey();
                        return;
                    }
                    // Start camera. Make sure depth camera is enabled.
                    var deviceConfig = new DeviceConfiguration();
                    deviceConfig.DepthMode = DepthMode.NFOV_Unbinned;
                    deviceConfig.ColorResolution = ColorResolution.Off;
                    try
                    {
                        device.StartCameras(deviceConfig);
                    }
                    catch (AzureKinectStartCamerasException ex)
                    {
                        Console.WriteLine("Failed to open k4a device!!");
                        Console.WriteLine(ex.Message);
                        Console.WriteLine(ex.StackTrace.ToString());
                        device.Dispose();
                        Console.ReadKey();
                        return;
                    }
                    using (StreamWriter file = new StreamWriter(@"C:/Users/hackathon/Desktop/" + now + ".csv", true))
                    {
                        file.Write("Time[ms],HEAD,PELVIS,WHILLcmd\n");
                    }



                    //■トラッカーを作成する
                    var calibration = device.GetCalibration(deviceConfig.DepthMode, deviceConfig.ColorResolution);
                    var trackerConfig = new TrackerConfiguration();
                    trackerConfig.ProcessingMode = TrackerProcessingMode.Gpu;       //GPUがない場合はCpuを指定
                    trackerConfig.SensorOrientation = SensorOrientation.Default;
                    device.StartImu();
                    using (var tracker = Tracker.Create(calibration, trackerConfig))
                    {
                        var wantExit = false;
                        var imu = device.GetImuSample();
                        double newpitch = Math.Atan(imu.AccelerometerSample.X / Math.Sqrt(imu.AccelerometerSample.Y * imu.AccelerometerSample.Y + imu.AccelerometerSample.Z * imu.AccelerometerSample.Z));
                        double newroll = Math.Atan2(imu.AccelerometerSample.Z, imu.AccelerometerSample.Y) + Math.PI / 2;
                        //Console.BackgroundColor = ConsoleColor.Yellow;
                        Console.Write("STARTTTTTTTTTTTTTTTTTTTT!!!!!!!!!!!!!!!!!!!!\n");

                        Socket client = server.Accept();
                        while (!wantExit)
                        {

                            //■Azure Kinect デバイスからキャプチャを取得する
                            // Capture a depth frame
                            using (Capture sensorCapture = device.GetCapture())
                            {
                                // Queue latest frame from the sensor.
                                tracker.EnqueueCapture(sensorCapture);
                            }

                            // Try getting latest tracker frame.
                            using (Frame frame = tracker.PopResult(TimeSpan.Zero, throwOnTimeout: false))
                            {

                                if (frame != null)
                                {
                                    if (frame.NumberOfBodies > 0)
                                    {
                                        var skeleton = frame.GetBodySkeleton(0);
                                        Quaternion imuq = Quaternion.CreateFromYawPitchRoll(-(float)newpitch - 6, 0, 0);

                                        Quaternion calihead = imuq * skeleton.GetJoint(26).Quaternion;
                                        Vector3 heador;
                                        (heador.X, heador.Y, heador.Z) = QuaternionToEulerAngles(calihead.W, calihead.X, calihead.Y, calihead.Z);
                                        double imuheadx = Math.Atan2(Math.Sin(heador.X + 90), Math.Cos(heador.X + 90)) * 180 / Math.PI;

                                        Quaternion calipelv = imuq * skeleton.GetJoint(0).Quaternion;
                                        Vector3 pelvor;
                                        (pelvor.X, pelvor.Y, pelvor.Z) = QuaternionToEulerAngles(calipelv.W, calipelv.X, calipelv.Y, calipelv.Z);
                                        double imupelvx = Math.Atan2(Math.Sin(pelvor.X + 90), Math.Cos(pelvor.X + 90)) * 180 / Math.PI;
                                        using (StreamWriter file = new StreamWriter(@"C:/Users/hackathon/Desktop/" + now + ".csv", true))
                                        {
                                            TimeSpan sw = frame.DeviceTimestamp;
                                            file.Write(sw.TotalMilliseconds + ",");
                                            file.Write(imuheadx + "," + imupelvx + "," + k + "\n");


                                            que.Enqueue(imuheadx);
                                            quetime.Enqueue(sw.TotalMilliseconds);
                                            
                                            if (k == 0)
                                            {
                                                if (sw.TotalMilliseconds - quetime.Peek() >= 300)
                                                {
                                                    if (((imuheadx - que.Peek()) * Math.PI / 180) / (sw.TotalMilliseconds - quetime.Peek()) < -0.0006)
                                                    {
                                                        k = 1;
                                                        que.Clear();
                                                        quetime.Clear();
                                                    }
                                                    else if (((imuheadx - que.Peek()) * Math.PI / 180) / (sw.TotalMilliseconds - quetime.Peek()) > 0.0006)
                                                    {
                                                        k =2;
                                                        que.Clear();
                                                        quetime.Clear();
                                                    }
                                                    else
                                                    {
                                                        que.Dequeue();
                                                        quetime.Dequeue();
                                                    }
                                                }
                                            }
                                            else if (k == 1)
                                            {

                                                if (sw.TotalMilliseconds - quetime.Peek() >= 600)
                                                {
                                                    Console.Write(((imuheadx - que.Peek()) * Math.PI / 180) / (sw.TotalMilliseconds - quetime.Peek()) + "\n");
                                                    double t = ((imuheadx - que.Peek()) * Math.PI / 180) / (sw.TotalMilliseconds - quetime.Peek());
                                                    if (t > 0.00000 && t< 0.000030)
                                                    {
                                                        Console.Write("STOP!!!!!\n");
                                                        k = 0;
                                                        Thread.Sleep(1000);
                                                        que.Clear();
                                                        quetime.Clear();
                                                    }
                                                    else
                                                    {
                                                        que.Dequeue();
                                                        quetime.Dequeue();
                                                    }
                                                }

                                            }
                                            else if (k == 2)
                                            {

                                                if (sw.TotalMilliseconds - quetime.Peek() >= 550)
                                                {
                                                    Console.Write(((imuheadx - que.Peek()) * Math.PI / 180) / (sw.TotalMilliseconds - quetime.Peek()) + "\n");
                                                    double t = ((imuheadx - que.Peek()) * Math.PI / 180) / (sw.TotalMilliseconds - quetime.Peek());
                                                    if (t < 0.00000 && t >- 0.000030)
                                                    {
                                                        Console.Write("STOP!!!!!\n");
                                                        k = 0;
                                                        Thread.Sleep(1500);
                                                        que.Clear();
                                                        quetime.Clear();
                                                    }
                                                    else
                                                    {
                                                        que.Dequeue();
                                                        quetime.Dequeue();
                                                    }
                                                }

                                            }

                                            var data = new byte[1];
                                            var msg = k.ToString();
                                            // データをUTF8エンコードでbyte形式で変換する。
                                            data = Encoding.UTF8.GetBytes(msg);
                                            // データを転送する。
                                            client.Send(data, data.Length, SocketFlags.None);


                                        }
                                    }

                                }
                            }
                            //Escキーで終了
                            if (Console.KeyAvailable)
                            {
                                var outChar = Console.ReadKey().Key.ToString();
                                if (outChar == "Escape")
                                {
                                    Console.Write("FINISHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH!!!!!!!!!!!!!!!!!!!!\n");
                                    wantExit = true;
                                    return;
                                }
                            }
                        }
                    }

                    Console.ResetColor();
                    device.StopCameras();
                    device.Dispose();
                }
                catch (Exception e)
                {
                    Console.WriteLine(e);
                }
            }
        }
    }
    //ref(https://devlog.arksystems.co.jp/2020/07/14/13673/)
}

