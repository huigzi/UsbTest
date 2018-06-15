using LibUsbDotNet;
using LibUsbDotNet.Main;
using OpenCvSharp;
using System;
using System.IO;
using System.Windows;
using System.Windows.Forms;
using System.Windows.Threading;

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
        private UsbDevice _myUsbDevice;
        private readonly UsbDeviceFinder _myUsbFinder;
        private UsbRegDeviceList _regList;
        private UsbEndpointReader _reader;
        private byte[] _recvBuf = new byte[17280];
        private float[] _iniImage = new float[4320];
        private Mat _imGray = new Mat(54, 80, MatType.CV_32F);
        private Mat _imGray2 = new Mat(54, 160, MatType.CV_32F);
        private string _filename;
        private FileStream _filestream;
        private BinaryWriter _sw;
        private readonly FilePath _filePath;
        private DataState _dataState = DataState.Stoped;

        private Mat dst = new Mat();
        private Mat recover = new Mat();
        private Mat imGrayResult1 = new Mat();
        private Mat imGrayResult2 = new Mat();

        private delegate void ShowMsg();
        private delegate void UpdateBytesDelegate(byte[] data);

        private void Window_Loaded(object sender, EventArgs e)
        {
            Cv2.NamedWindow("Demo", WindowMode.AutoSize);
            UsbFind();

            for (int i = 0; i < _imGray.Height / 2; i++)
            {
                for (int j = 0; j < _imGray.Width; j++)
                {
                    _imGray.Set<float>(i, j, 0);
                }
            }

            for (int i = 0; i < _imGray.Height / 2; i++)
            {
                for (int j = 0; j < _imGray.Width; j++)
                {
                    _imGray2.Set<float>(i, j, 0);
                }
            }

        }

        private void Window_Closing(object sender, EventArgs e)
        {
            if (_myUsbDevice == null) return;
            if (_myUsbDevice.IsOpen)
            {
                try
                {
                    if (_myUsbDevice is IUsbDevice wholeUsbDevice)
                    {
                        wholeUsbDevice.ReleaseInterface(0);
                    }
                }
                catch (Exception ex)
                {
                    Console.Write(ex.ToString());
                }
            }
            _myUsbDevice = null;
            UsbDevice.Exit();
        }

        public MainWindow()
        {
            InitializeComponent();

            _myUsbFinder = new UsbDeviceFinder(1155, 22315);
            _filePath = new FilePath();
        }

        private void OnRxEndPointData(object sender, EndpointDataEventArgs e)
        {
            AddMsg($" > {e.Count} data received");
            Array.Copy(e.Buffer, _recvBuf, e.Buffer.Length);

            Dispatcher.Invoke(new ShowMsg(CreatImg));

            if (_dataState == DataState.Started)
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
            LbxDev.Items.Clear();
            _regList = UsbDevice.AllDevices.FindAll(_myUsbFinder);
            if (_regList.Count == 0)
            {
                System.Windows.MessageBox.Show("找不到设备");
            }

            foreach(UsbRegistry regDevice in _regList)
            {
                LbxDev.Items.Add(regDevice.FullName);
            }

        }

        private void SaveData(byte[] bytes)
        {
            if (_filePath.Path == "") return;
            _sw.Write(bytes);
            _sw.Flush();

        }

        private void AddMsg(string msg)
        {
            Dispatcher.BeginInvoke(DispatcherPriority.Normal, (ShowMsg)delegate () {
                if (LbxMsg.Items.Count > 50)
                {
                    LbxMsg.Items.RemoveAt(0);
                }
                LbxMsg.Items.Add(msg);
            });
        }

        private void CreatImg()
        {

            if (Filter.SelectedIndex == 0)
            {

                for (int i = 0; i < 8640; i = i + 4)
                {
                    _iniImage[i / 4] = BitConverter.ToSingle(_recvBuf, i);
                }

                for (int i = _imGray.Height / 2; i < _imGray.Height; i++)
                {
                    for (int j = 0; j < _imGray.Width; j++)
                    {
                        float temp = _iniImage[(i - 27) + j * 27] * 255 / 50000;
                        //float temp = iniImage[i - 27 + (j + 8) * 27] * iniImage[i - 27 + (j + 8) * 27] * iniImage[i - 27 + (j + 8) * 27];
                        _imGray.Set<float>(i, j, temp);
                    }
                }

                Cv2.Resize(_imGray, imGrayResult2, new OpenCvSharp.Size(600, 300));
            }
            else
            {
                for (int i = 0; i < 17280; i = i + 4)
                {
                    _iniImage[i / 4] = BitConverter.ToSingle(_recvBuf, i);
                }

                for (int i = _imGray2.Height / 2; i < _imGray2.Height; i++)
                {
                    for (int j = 0; j < _imGray2.Width; j++)
                    {
                        float temp = _iniImage[(i - 27) + j * 27] * 255 / 10000;

                        _imGray2.Set<float>(i, j, temp);
                    }
                }

                Cv2.Resize(_imGray2, imGrayResult2, new OpenCvSharp.Size(1200, 600));
            }

            Cv2.ConvertScaleAbs(imGrayResult2, imGrayResult1);

            Point2f point2F = new Point2f(imGrayResult1.Width / 2, imGrayResult1.Height);
            Cv2.LinearPolar(imGrayResult1, recover, point2F, 600, InterpolationFlags.WarpInverseMap);
            Cv2.ApplyColorMap(recover, dst, ColormapTypes.Jet);

            Cv2.ImShow("Demo", dst);
        }

        private void ConnectbuttonClick(object sender, RoutedEventArgs e)
        {

            if (Connectbutton.IsChecked != null && (bool)Connectbutton.IsChecked)
            {
                if(_myUsbDevice != null)
                {

                }
                else
                {
                    LbxDev.Items.Clear();
                    foreach (UsbRegistry regDevice in _regList)
                    {
                        LbxDev.Items.Add(regDevice.FullName);
                    }

                    _myUsbDevice = UsbDevice.OpenUsbDevice(_myUsbFinder);

                    if (_myUsbDevice is IUsbDevice wholeUsbDevice)
                    {
                        wholeUsbDevice.SetConfiguration(1);
                        wholeUsbDevice.ClaimInterface(0);
                    }

                    _reader = _myUsbDevice.OpenEndpointReader(ReadEndpointID.Ep01);
                    _reader.DataReceived += OnRxEndPointData;

                    _reader.ReadBufferSize = Filter.SelectedIndex == 0 ? 8640 : 17280;

                    _reader.Reset();
                    _reader.DataReceivedEnabled = true;
                }
            }
            else
            {
                if (_myUsbDevice == null) return;
                _reader.DataReceivedEnabled = false;
                _reader.DataReceived -= OnRxEndPointData;

                LbxDev.Items.RemoveAt(0);
                AddMsg($" > Close Device");

                if (_myUsbDevice.IsOpen)
                {
                    try
                    {
                        IUsbDevice wholeUsbDevice = _myUsbDevice as IUsbDevice;
                        wholeUsbDevice?.ReleaseInterface(0);
                    }
                    catch(Exception ex)
                    {
                        Console.Write(ex.ToString());
                    }
                }
                _myUsbDevice = null;
                UsbDevice.Exit();
            }
        }

        private void Button_Click_1(object sender, RoutedEventArgs e)
        {
            var btn = (System.Windows.Controls.Button)sender;
            if(_filePath.Path == "")
            {
                var mDialog = new FolderBrowserDialog();
                DialogResult result = mDialog.ShowDialog();

                if(result == System.Windows.Forms.DialogResult.Cancel)
                {
                    return;
                }

                _filePath.Path = mDialog.SelectedPath.Trim();
            }

            if(_dataState == DataState.Started)
            {
                _dataState = DataState.Stoped;
                btn.Content = "开始保存";
                _sw.Dispose();
                _filestream.Dispose();
            }
            else
            {
                _filename = DateTime.Now.ToString("MM-dd") + "保存数据";
                _dataState = DataState.Started;
                btn.Content = "停止保存";
                string path = _filePath.Path + "\\" + _filename;
                _filestream = new FileStream(@path, FileMode.Create, FileAccess.Write);
                _sw = new BinaryWriter(_filestream);
            }
        }
    }
}
