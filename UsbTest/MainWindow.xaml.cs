using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Forms;
using System.IO;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
using LibUsbDotNet;
using LibUsbDotNet.Main;
using LibUsbDotNet.Info;
using OpenCvSharp;

namespace UsbTest
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    /// 

    internal class FilePath
    {
        private string _path = string.Empty;
        public string Path
        {
            get
            {
                if (_path.Length == 0)
                {
                    _path = "";
                }
                return _path;
            }
            set => _path = value;
        }
    }

    public enum DataState
    {
        Started, Stoped
    }

    public partial class MainWindow : System.Windows.Window
    {
        private UsbDevice MyUsbDevice;
        private UsbDeviceFinder MyUsbFinder = new UsbDeviceFinder(1155, 22315);
        private UsbRegDeviceList regList;
        private UsbEndpointReader reader;
        private byte[] recvBuf = new byte[8640];
        private float[] iniImage = new float[2160];
        private Mat dst = new Mat();
        private Mat recover = new Mat();
        private Mat im_gray_result1 = new Mat();
        private Mat im_gray_result2 = new Mat();
        private Mat im_gray = new Mat(54, 80, MatType.CV_32F);
        private string filename;
        private FileStream _filestream;
        private BinaryWriter _sw;
        private FilePath filePath = new FilePath();
        private DataState dataState = DataState.Stoped;

        private delegate void ShowMsg();
        private delegate void UpdateBytesDelegate(byte[] data);

        private void Window_Loaded(object sender, EventArgs e)
        {
            Cv2.NamedWindow("Demo", WindowMode.AutoSize);
            UsbFind();
            for (int i = 0; i < im_gray.Height / 2; i++)
            {
                for (int j = 0; j < im_gray.Width; j++)
                {
                    im_gray.Set<float>(i, j, 0);
                }
            }
        }

        private void Window_Closing(object sender, EventArgs e)
        {
            if (MyUsbDevice != null)
            {
                if (MyUsbDevice.IsOpen)
                {
                    try
                    {
                        IUsbDevice wholeUsbDevice = MyUsbDevice as IUsbDevice;
                        if (!(wholeUsbDevice is null))
                        {
                            wholeUsbDevice.ReleaseInterface(0);
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.Write(ex.ToString());
                    }
                }
                MyUsbDevice = null;
                UsbDevice.Exit();
            }
        }

        public MainWindow()
        {
            InitializeComponent();
        }

        private void OnRxEndPointData(object sender, EndpointDataEventArgs e)
        {
            AddMsg($" > {e.Count} data received");
            Array.Copy(e.Buffer, recvBuf, e.Buffer.Length);

            for (int i = 0; i < 8640; i = i + 4)
            {
                iniImage[i / 4] = BitConverter.ToSingle(recvBuf, i);
            }

            for (int i = im_gray.Height / 2; i < im_gray.Height; i++)
            {
                for (int j = 0; j < im_gray.Width; j++)
                {
                    float temp = iniImage[(i - 27) + j * 27] * 255 / 50000;
                    //float temp = iniImage[i - 27 + (j + 8) * 27] * iniImage[i - 27 + (j + 8) * 27] * iniImage[i - 27 + (j + 8) * 27];
                    im_gray.Set<float>(i, j, temp);
                }
            }

            Cv2.Resize(im_gray, im_gray_result2, new OpenCvSharp.Size(600, 300));
            //Cv2.Normalize(im_gray_result2, im_gray_result2, 0, 255, NormTypes.MinMax);
            Cv2.ConvertScaleAbs(im_gray_result2, im_gray_result1);
            //Cv2.ConvertScaleAbs(im_gray, im_gray_result1);

            Point2f point2F = new Point2f(im_gray_result1.Width / 2, im_gray_result1.Height);
            Cv2.LinearPolar(im_gray_result1, recover, point2F, 300, InterpolationFlags.WarpInverseMap);
            Cv2.ApplyColorMap(recover, dst, ColormapTypes.Jet);

            Cv2.ImShow("Demo", dst);
            //Cv2.ImShow("Demo", im_gray_result1);

            if (dataState == DataState.Started)
            {
                Dispatcher.Invoke(new UpdateBytesDelegate(SaveData), e.Buffer);
            }

        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            UsbFind();
        }

        private void UsbFind()
        {
            lbxDev.Items.Clear();
            regList = UsbDevice.AllDevices.FindAll(MyUsbFinder);
            if (regList.Count == 0)
            {
                System.Windows.MessageBox.Show("Device Not Found");
            }

            foreach(UsbRegistry regDevice in regList)
            {
                lbxDev.Items.Add(regDevice.FullName);
            }

        }

        private void SaveData(byte[] bytes)
        {

            if (filePath.Path != "")
            {
                _sw.Write(bytes);
                _sw.Flush();
            }

        }

        private void AddMsg(string msg)
        {
            Dispatcher.BeginInvoke(DispatcherPriority.Normal, (ShowMsg)delegate () {
                if (lbxMsg.Items.Count > 50)
                {
                    lbxMsg.Items.RemoveAt(0);
                }
                lbxMsg.Items.Add(msg);
            });
        }

        private void ConnectbuttonClick(object sender, RoutedEventArgs e)
        {

            if ((bool)connectbutton.IsChecked)
            {
                if(MyUsbDevice != null)
                {

                }
                else
                {
                    lbxDev.Items.Clear();
                    foreach (UsbRegistry regDevice in regList)
                    {
                        lbxDev.Items.Add(regDevice.FullName);
                    }

                    MyUsbDevice = UsbDevice.OpenUsbDevice(MyUsbFinder);
                    IUsbDevice wholeUsbDevice = MyUsbDevice as IUsbDevice;

                    if (!(wholeUsbDevice is null))
                    {
                        wholeUsbDevice.SetConfiguration(1);
                        wholeUsbDevice.ClaimInterface(0);
                    }

                    reader = MyUsbDevice.OpenEndpointReader(ReadEndpointID.Ep01);
                    reader.DataReceived += OnRxEndPointData;
                    reader.ReadBufferSize = 8640;
                    //reader.ReadBufferSize = 5120;
                    reader.Reset();
                    reader.DataReceivedEnabled = true;
                }
            }
            else
            {
                if(MyUsbDevice != null)
                {

                    reader.DataReceivedEnabled = false;
                    reader.DataReceived -= OnRxEndPointData;

                    lbxDev.Items.RemoveAt(0);
                    AddMsg($" > Close Device");

                    if (MyUsbDevice.IsOpen)
                    {
                        try
                        {
                            IUsbDevice wholeUsbDevice = MyUsbDevice as IUsbDevice;
                            if(!(wholeUsbDevice is null))
                            {
                                wholeUsbDevice.ReleaseInterface(0);
                            }
                        }
                        catch(Exception ex)
                        {
                            Console.Write(ex.ToString());
                        }
                    }
                    MyUsbDevice = null;
                    UsbDevice.Exit();
                }
            }
        }

        private void Button_Click_1(object sender, RoutedEventArgs e)
        {
            var btn = (System.Windows.Controls.Button)sender;
            if(filePath.Path == "")
            {
                var mDialog = new FolderBrowserDialog();
                DialogResult result = mDialog.ShowDialog();

                if(result == System.Windows.Forms.DialogResult.Cancel)
                {
                    return;
                }

                filePath.Path = mDialog.SelectedPath.Trim();
            }

            if(dataState == DataState.Started)
            {
                dataState = DataState.Stoped;
                btn.Content = "开始保存";
                _sw.Dispose();
                _filestream.Dispose();
            }
            else
            {
                filename = DateTime.Now.ToString("MM-dd") + "保存数据";
                dataState = DataState.Started;
                btn.Content = "停止保存";
                string path = filePath.Path + "\\" + filename;
                _filestream = new FileStream(@path, FileMode.Create, FileAccess.Write);
                _sw = new BinaryWriter(_filestream);
            }
        }
    }
}
