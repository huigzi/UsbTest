using LibUsbDotNet;
using LibUsbDotNet.Main;
using OpenCvSharp;
using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Forms;
using System.Windows.Media;
using System.Windows.Threading;
using InteractiveDataDisplay.WPF;
using MathNet.Numerics.LinearAlgebra;
using UsbTest.algorithm;

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
        private UsbEndpointWriter _writer;
        private byte[] _recvBuf = new byte[13760];
        private float[] _iniImage = new float[2160];
        private float[] _trackImage = new float[2160];
        private byte[] adcBuf = new byte[5120];
        private Int16[] timerPlotBuf = new Int16[320];
        private double[] xTemp = new double[1000];
        private double[] yTemp = new double[1000];
        private Mat _imGray = new Mat(54, 80, MatType.CV_32F);
        private string _filename;
        private FileStream _filestream;
        private BinaryWriter _sw;
        private readonly FilePath _filePath;
        private DataState _dataState = DataState.Stoped;
        private readonly LineGraph _lineGraphs;
        private int trackNum = 0;

        private Mat dst = new Mat();
        private Mat recover = new Mat();
        private Mat imGrayResult1 = new Mat();
        private Mat imGrayResult2 = new Mat();

        private KalmanFiltering kalmanFiltering = new KalmanFiltering();

        private Matrix<double> _matrixDataTemp;

        private delegate void ShowMsg();

        private delegate void UpdateBytesDelegate(byte[] data);

        private delegate void UpdatePlot(Int16[] plotBuf);

        private delegate void UpdateTrack(double[] plotBuf);

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

            var matrixBuider = Matrix<double>.Build;

            //_myUsbFinder = new UsbDeviceFinder(1155, 22399);
            _myUsbFinder = new UsbDeviceFinder(0x0484, 0x573d);
            _filePath = new FilePath();

            _matrixDataTemp = matrixBuider.Dense(72, 27);

            _lineGraphs = new LineGraph()
            {
                Stroke = new SolidColorBrush(Color.FromArgb(255, 0, 0, 0)),
                Description = $"跟踪图样",
                StrokeThickness = 2
            };

            Tlines.Children.Add(_lineGraphs);
        }

        private void OnRxEndPointData(object sender, EndpointDataEventArgs e)
        {
            AddMsg($" > {e.Count} data received");
            Array.Copy(e.Buffer, 0, _recvBuf, 0, 8640);
            Array.Copy(e.Buffer, 8640, adcBuf, 0, 5120);
            int k = 0;

            for (int i = 0; i < 5120 - 16; i = i + 16)
            {
                timerPlotBuf[k] = BitConverter.ToInt16(adcBuf, i);
                // timerPlotBuf[k] = BitConverter.ToInt16(e.Buffer, i);
                k++;
            }

            Dispatcher.Invoke(new UpdatePlot(AddPlot), timerPlotBuf);
            //Dispatcher.Invoke(new ShowMsg(CreatImg));

            for (int i = 0; i < 8640; i = i + 4)
            {
                _trackImage[i / 4] = BitConverter.ToSingle(_recvBuf, i);
            }

            for (int i = 8; i < 80; i++)
            {
                for (int j = 0; j < 27; j++)
                {
                    _matrixDataTemp[i - 8, j] = _trackImage[j + i * 27];
                }
            }

            var result = kalmanFiltering.TraceTableEstablishment(_matrixDataTemp);

            if (result.Count > 0)
            {
                if (kalmanFiltering.StartFlag == 0)
                {
                    var temp = kalmanFiltering.TrackingIni(result);

                    for (int i = 0; i < 6; i++)
                    {
                        kalmanFiltering.PredictPosition[i] = temp[i];
                    }

                    trackNum = 0;
                }
                else
                {
                    kalmanFiltering.Tracking(result);
                    xTemp[trackNum] = kalmanFiltering.CurrentPosition[0] * 100;
                    yTemp[trackNum] = kalmanFiltering.CurrentPosition[3] * 100;
                    trackNum++;
                }

            }
            else
            {

                if (kalmanFiltering.TrackCount > 0)
                {
                    kalmanFiltering.MissCount++;
                }

                if (kalmanFiltering.MissCount > 50)
                {
                    double[] plotTemp = new double[kalmanFiltering.TrackCount * 2];
                    for (int i = 0; i < kalmanFiltering.TrackCount; i++)
                    {
                        plotTemp[i] = xTemp[i];
                        plotTemp[i + kalmanFiltering.TrackCount] = yTemp[i];
                    }

                    //Dispatcher.Invoke(new UpdateTrack(AddTrack), plotTemp);
                    trackNum = 0;
                    kalmanFiltering.TrackCount = 0;

                    kalmanFiltering.StartFlag = 0;
                }
                
            }


            if (_dataState == DataState.Started)
            {
                Dispatcher.Invoke(new UpdateBytesDelegate(SaveData), adcBuf);
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

            foreach (UsbRegistry regDevice in _regList)
            {
                LbxDev.Items.Add(regDevice.FullName);
            }

        }

        private void AddPlot(Int16[] plotBuf)
        {
            _lineGraphs.PlotY(plotBuf);
        }

        private void AddTrack(double[] plotBuf)
        {
            var xTemp = new double[plotBuf.Length / 2];
            var yTemp = new double[plotBuf.Length / 2];

            for (int i = 0; i < plotBuf.Length / 2; i++)
            {
                xTemp[i] = plotBuf[i];
                yTemp[i] = plotBuf[i + plotBuf.Length / 2];
                _lineGraphs.Plot(xTemp, yTemp);
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
            Dispatcher.BeginInvoke(DispatcherPriority.Normal, (ShowMsg) delegate()
            {
                if (LbxMsg.Items.Count > 50)
                {
                    LbxMsg.Items.RemoveAt(0);
                }

                LbxMsg.Items.Add(msg);
            });
        }

        private void CreatImg()
        {

            if (Filter.SelectedIndex == 0 || Filter.SelectedIndex == -1)
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
                for (int i = 0; i < 8640; i = i + 4)
                {
                    _iniImage[i / 4] = BitConverter.ToSingle(_recvBuf, i);
                }

                for (int i = _imGray.Height / 2; i < _imGray.Height; i++)
                {
                    for (int j = 0; j < _imGray.Width; j++)
                    {
                        float temp = _iniImage[(i - 27) + j * 27] * 255 / 10000;

                        _imGray.Set<float>(i, j, temp);
                    }
                }

                Cv2.Resize(_imGray, imGrayResult2, new OpenCvSharp.Size(600, 300));
            }

            Cv2.ConvertScaleAbs(imGrayResult2, imGrayResult1);

            Point2f point2F = new Point2f(imGrayResult1.Width / 2, imGrayResult1.Height);
            Cv2.LinearPolar(imGrayResult1, recover, point2F, 300, InterpolationFlags.WarpInverseMap);
            Cv2.ApplyColorMap(recover, dst, ColormapTypes.Jet);

            Cv2.ImShow("Demo", dst);
        }

        private void ConnectbuttonClick(object sender, RoutedEventArgs e)
        {

            if (Connectbutton.IsChecked != null && (bool) Connectbutton.IsChecked)
            {
                if (_myUsbDevice != null)
                {

                }
                else
                {
                    if (_regList.Count != 0)
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
                        _writer = _myUsbDevice.OpenEndpointWriter(WriteEndpointID.Ep01);

                        _reader.DataReceived += OnRxEndPointData;

                        _reader.ReadBufferSize = 13760;

                        _reader.Reset();
                        _reader.DataReceivedEnabled = true;

                        ScanButton.IsEnabled = false;
                        SaveButton.IsEnabled = true;
                        Filter.IsEnabled = true;
                    }
                    else
                    {
                        AddMsg($" > 找不到设备");
                        Connectbutton.IsChecked = false;
                        Filter.IsEnabled = false;
                    }
                }
            }
            else
            {
                if (_myUsbDevice == null) return;
                _reader.DataReceivedEnabled = false;
                _reader.DataReceived -= OnRxEndPointData;

                LbxDev.Items.RemoveAt(0);
                AddMsg($" > 关闭设备");
                System.Windows.MessageBox.Show($"再次开启设备需要重新上电");

                if (_myUsbDevice.IsOpen)
                {
                    try
                    {
                        IUsbDevice wholeUsbDevice = _myUsbDevice as IUsbDevice;
                        wholeUsbDevice?.ReleaseInterface(0);
                        _myUsbDevice.Close();
                    }
                    catch (Exception ex)
                    {
                        Console.Write(ex.ToString());
                    }
                }

                _myUsbDevice = null;
                UsbDevice.Exit();

                ScanButton.IsEnabled = true;
                SaveButton.IsEnabled = false;
                Filter.IsEnabled = true;
            }
        }

        private void Button_Click_1(object sender, RoutedEventArgs e)
        {
            var btn = (System.Windows.Controls.Button) sender;
            if (_filePath.Path == "")
            {
                var mDialog = new FolderBrowserDialog();
                DialogResult result = mDialog.ShowDialog();

                if (result == System.Windows.Forms.DialogResult.Cancel)
                {
                    return;
                }

                _filePath.Path = mDialog.SelectedPath.Trim();
            }

            if (_dataState == DataState.Started)
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

        private void Filter_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
        {
            byte[] command1 = {0x0a, 0x0b, 0x0c, 0x0d};
            byte[] command2 = {0x01, 0x02, 0x03, 0x04};
            int bytesWritten;
            LibUsbDotNet.Main.ErrorCode ec = LibUsbDotNet.Main.ErrorCode.None;

            try
            {
                if (Filter.SelectedIndex == 0)
                {
                    ec = _writer.Write(command1, 1000, out bytesWritten);
                    if (ec != LibUsbDotNet.Main.ErrorCode.None)
                        throw new Exception(UsbDevice.LastErrorString);
                }

                if (Filter.SelectedIndex == 1)
                {
                    ec = _writer.Write(command2, 1000, out bytesWritten);
                    if (ec != LibUsbDotNet.Main.ErrorCode.None)
                        throw new Exception(UsbDevice.LastErrorString);
                }
            }
            catch(Exception ex)
            {
                Console.WriteLine();
                Console.WriteLine((ec != LibUsbDotNet.Main.ErrorCode.None ? ec + ":" : string.Empty) + ex.Message);
            }
        }
    }
}
