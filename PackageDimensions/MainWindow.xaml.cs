using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;
using System.Linq;
using System.Collections.Generic;
using System.Threading.Tasks;



namespace PackageDimensions
{

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for depth frames
        /// </summary>
        private DepthFrameReader depthFrameReader = null;

        /// <summary>
        /// Description of the data contained in the depth frame
        /// </summary>
        private FrameDescription depthFrameDescription = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap PackagedepthBitmap = null;
        private WriteableBitmap CalibrationdepthBitmap = null;
        private WriteableBitmap DifferencedepthBitmap = null;

        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        private byte[] CalibrationdepthPixels = null;
        private byte[] PackagedepthPixels = null;
        private byte[] DiffernecedepthPixels = null;

        ushort[] PackageFrameData;
        ushort[] CalibrationFrameData;
        ushort[] DifferenceFrame;

        int PackageDepth;
        int PackageWidth;
        int PackageHeight;
        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;


        int CalibrationWidth = 512;
        int CalibrationHeight = 424;
        bool IsCalibrate;
        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // open the reader for the depth frames
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();

            // wire handler for frame arrival
            this.depthFrameReader.FrameArrived += this.Reader_FrameArrived;

            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // allocate space to put the pixels being received and converted
            this.PackagedepthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            this.CalibrationdepthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            this.DiffernecedepthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

            // create the bitmap to display
            this.PackagedepthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);
            this.CalibrationdepthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);
            this.DifferencedepthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource PackageImageSource
        {
            get
            {
                return this.PackagedepthBitmap;
            }
        }
        public ImageSource CalibrationImageSource
        {
            get
            {
                return this.CalibrationdepthBitmap;
            }
        }

        public ImageSource DifferenceImageSource
        {
            get
            {
                return this.DifferencedepthBitmap;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.depthFrameReader != null)
            {
                // DepthFrameReader is IDisposable
                this.depthFrameReader.Dispose();
                this.depthFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }



        /// <summary>
        /// Handles the depth frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            bool depthFrameProcessed = false;

            using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    // the fastest way to process the body index data is to directly access 
                    // the underlying buffer
                    using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {
                        // verify data and write the color data to the display bitmap
                        if (((this.depthFrameDescription.Width * this.depthFrameDescription.Height) == (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel)) &&
                            (this.depthFrameDescription.Width == this.CalibrationdepthBitmap.PixelWidth) && (this.depthFrameDescription.Height == this.CalibrationdepthBitmap.PixelHeight))
                        {
                            // Note: In order to see the full range of depth (including the less reliable far field depth)
                            // we are setting maxDepth to the extreme potential depth threshold
                            ushort maxDepth = ushort.MaxValue;

                            // If you wish to filter by reliable depth distance, uncomment the following line:
                            //// maxDepth = depthFrame.DepthMaxReliableDistance

                            this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth);
                            depthFrameProcessed = true;
                        }
                    }
                }
            }

            if (depthFrameProcessed)
                this.RenderDepthPixels();

        }

        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        /// <param name="minDepth">The minimum reliable depth value for the frame</param>
        /// <param name="maxDepth">The maximum reliable depth value for the frame</param>
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {

            int X1 = (depthFrameDescription.Width - CalibrationWidth) / 2;
            int X2 = depthFrameDescription.Width - X1;
            int Y1 = (depthFrameDescription.Height - CalibrationHeight) / 2;
            int Y2 = depthFrameDescription.Height - Y1;
            if (IsCalibrate||CalibrationCheckBox.IsChecked.Value)
            {
                // depth frame data is a 16 bit value
                CalibrationFrameData = new ushort[(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel)]; // (ushort*)depthFrameData;
                // convert depth to a visual representation
                for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
                {
                    // Get the depth for this pixel

                    int ColumnIdex = i % depthFrameDescription.Width;
                    int RowIndex = (i - ColumnIdex) / depthFrameDescription.Width;

                    ushort depth = ((ushort*)depthFrameData)[i];
                    if (depth < 500 || depth > 4000 || ColumnIdex < X1 || ColumnIdex > X2 || RowIndex < Y1 || RowIndex > Y2)
                        depth = 0;


                    CalibrationFrameData[i] = depth;
                    // To convert to a byte, we're mapping the depth value to the byte range.
                    // Values outside the reliable depth range are mapped to 0 (black).
                    this.CalibrationdepthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
                }
                IsCalibrate = false;
            }
            else
            {
                // depth frame data is a 16 bit value
                PackageFrameData = new ushort[(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel)];
                DifferenceFrame = new ushort[(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel)];

                // convert depth to a visual representation
                for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
                {
                    int ColumnIdex = i % depthFrameDescription.Width;
                    int RowIndex = (i - ColumnIdex) / depthFrameDescription.Width;
                    // Get the depth for this pixel
                    ushort depth = ((ushort*)depthFrameData)[i];
                    if (depth < 500 || depth > 4000 || ColumnIdex < X1 || ColumnIdex > X2 || RowIndex < Y1 || RowIndex > Y2)
                        depth = 0;
                    PackageFrameData[i] = depth;
                    // To convert to a byte, we're mapping the depth value to the byte range.
                    // Values outside the reliable depth range are mapped to 0 (black).
                    this.PackagedepthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);


                    //Difference 
                    if (PackageFrameData != null && CalibrationFrameData != null)
                    {
                        int Difference;
                        if (CalibrationFrameData[i] < 500 || PackageFrameData[i] < 500 || CalibrationFrameData[i] > 4000 || PackageFrameData[i] > 4000 || ColumnIdex < X1 || ColumnIdex > X2 || RowIndex < Y1 || RowIndex > Y2)
                            Difference = 0;
                        else
                            Difference = (CalibrationFrameData[i] - PackageFrameData[i]);

                        // remove difference smaller than 100
                        Difference = (ushort)((Difference > 100) ? Difference : 0);
                        // var LayerDifference = (ushort)(Difference + Difference % 10);
                        DifferenceFrame[i] = (ushort)Difference;
                        int differencePixel = (Difference / MapDepthToByte) * 10;
                        this.DiffernecedepthPixels[i] = (byte)((differencePixel > 255) ? 255 : differencePixel);

                    }
                }

                if (PackageFrameData != null && CalibrationFrameData != null)
                {
                    int TopY = 0;
                    int DownY = 0;
                    int RightX = 0;
                    int LeftX = 0;
                    for (int RowIndex = 0; RowIndex < depthFrameDescription.Height; RowIndex++)
                    {
                        int Topcount = 0;
                        int DownCount = 0;
                        for (int ColumnIndex = 0; ColumnIndex < depthFrameDescription.Width; ColumnIndex++)
                        {
                            var TopIndex = ColumnIndex + (RowIndex * depthFrameDescription.Width);
                            var DownIndex = ColumnIndex + (((depthFrameDescription.Height - RowIndex) - 1) * depthFrameDescription.Width);

                            if (DifferenceFrame[TopIndex] > 0 && TopY == 0)
                                Topcount++;

                            if (DifferenceFrame[DownIndex] > 0 && DownY == 0)
                                DownCount++;

                        }
                        if (Topcount > 10 && TopY == 0)
                        {
                            TopY = RowIndex;
                            Topcount = 0;
                        }
                        if (DownCount > 10 && DownY == 0)
                        {
                            DownY = depthFrameDescription.Height - RowIndex - 1;
                            DownCount = 0;
                        }
                        if (DownY > 0 && TopY > 0)
                            break;

                    }

                    for (int ColumnIndex = 0; ColumnIndex < depthFrameDescription.Width; ColumnIndex++)
                    {
                        int Leftcount = 0;
                        int RightCount = 0;
                        for (int RowIndex = 0; RowIndex < depthFrameDescription.Height; RowIndex++)
                        {
                            var LeftIndex = ColumnIndex + (RowIndex * depthFrameDescription.Width);
                            var RightIndex = (depthFrameDescription.Width - ColumnIndex - 1) + (RowIndex * depthFrameDescription.Width);

                            if (DifferenceFrame[LeftIndex] > 0 && LeftX == 0)
                                Leftcount++;

                            if (DifferenceFrame[RightIndex] > 0 && RightX == 0)
                                RightCount++;

                        }
                        if (Leftcount > 10 && LeftX == 0)
                        {
                            LeftX = ColumnIndex;
                            Leftcount = 0;
                        }
                        if (RightCount > 10 && RightX == 0)
                        {
                            RightX = depthFrameDescription.Width - ColumnIndex - 1;
                            RightCount = 0;
                        }
                        if (LeftX > 0 && RightX > 0)
                            break;

                    }


                    for (int ColumnIndex = LeftX; ColumnIndex < RightX; ColumnIndex++)
                    {
                        var TopRowIndex = ColumnIndex + (TopY * depthFrameDescription.Width);
                        var DownRowIndex = ColumnIndex + (DownY * depthFrameDescription.Width);
                        this.DiffernecedepthPixels[TopRowIndex] = 255;
                        this.DiffernecedepthPixels[DownRowIndex] = 255;
                    }

                    for (int RowIndex = TopY; RowIndex < DownY; RowIndex++)
                    {
                        var LeftIndex = LeftX + (RowIndex * depthFrameDescription.Width);
                        var RightIndex = RightX + (RowIndex * depthFrameDescription.Width);
                        this.DiffernecedepthPixels[LeftIndex] = 255;
                        this.DiffernecedepthPixels[RightIndex] = 255;

                    }

                    SortedDictionary<ushort, int> DifferentDepthWeight = new SortedDictionary<ushort, int>();
                    for (int RowIndex = TopY; RowIndex < DownY; RowIndex++)
                    {
                        for (int ColumnIndex = LeftX; ColumnIndex < RightX; ColumnIndex++)
                        {
                            var Index = ColumnIndex + (RowIndex * depthFrameDescription.Width);

                            if (DifferentDepthWeight.ContainsKey(DifferenceFrame[Index]))
                                DifferentDepthWeight[DifferenceFrame[Index]]++;
                            else
                                DifferentDepthWeight.Add(DifferenceFrame[Index], 1);
                        }
                    }

                    SortedDictionary<ushort, int> DepthWeight = new SortedDictionary<ushort, int>();
                    for (int RowIndex = TopY; RowIndex < DownY; RowIndex++)
                    {
                        for (int ColumnIndex = LeftX; ColumnIndex < RightX; ColumnIndex++)
                        {
                            var Index = ColumnIndex + (RowIndex * depthFrameDescription.Width);

                            if (DepthWeight.ContainsKey(PackageFrameData[Index]))
                                DepthWeight[PackageFrameData[Index]]++;
                            else if (PackageFrameData[Index] != 0)
                                DepthWeight.Add(PackageFrameData[Index], 1);
                        }
                    }
                    int DepthFromCamera = 0;
                    if (DepthWeight.Count > 0)
                        DepthFromCamera = DepthWeight.OrderBy(p => p.Value).Last().Key;

                    // Kinect has a depth image resolution of 512 x 424 pixels with a fov of 70.6 x 60 degrees resulting in an average of about 7 x 7 pixels per degree

                    PackageDepth = (DifferentDepthWeight.LastOrDefault(p => p.Value > 4).Key) / 10;
                    PackageWidth = (int)(((2 * DepthFromCamera * Math.Tan(35.3 * Math.PI / 180) * (RightX - LeftX)) / depthFrameDescription.Width) / 10);
                    PackageHeight = (int)(((2 * DepthFromCamera * Math.Abs(Math.Tan(30 * Math.PI / 180)) * (DownY - TopY)) / depthFrameDescription.Height) / 10);
                }




            }


        }

        /// <summary>
        /// Renders color pixels into the writeableBitmap.
        /// </summary>
        private void RenderDepthPixels()
        {
            if (PackageFrameData != null)
            {
                this.PackagedepthBitmap.WritePixels(
                    new Int32Rect(0, 0, this.PackagedepthBitmap.PixelWidth, this.PackagedepthBitmap.PixelHeight),
                    PackagedepthPixels,
                    this.PackagedepthBitmap.PixelWidth,
                    0);

                this.DifferencedepthBitmap.WritePixels(
                  new Int32Rect(0, 0, this.DifferencedepthBitmap.PixelWidth, this.DifferencedepthBitmap.PixelHeight),
                  DiffernecedepthPixels,
                  this.DifferencedepthBitmap.PixelWidth,
                  0);

                Depth.Text = PackageDepth.ToString();
                Width.Text = PackageWidth.ToString();
                Height.Text = PackageHeight.ToString();

            }
            if (CalibrationFrameData != null)
            {
                this.CalibrationdepthBitmap.WritePixels(
                   new Int32Rect(0, 0, this.CalibrationdepthBitmap.PixelWidth, this.CalibrationdepthBitmap.PixelHeight),
                   CalibrationdepthPixels,
                   this.CalibrationdepthBitmap.PixelWidth,
                   0);
            }




        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        private void CalibrationWidth_Changed(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (CalibrationImage == null)
                return;
            CalibrationWidth = (int)e.NewValue;
            int RectWidth = (int)(CalibrationWidth * CalibrationImage.ActualWidth) / depthFrameDescription.Width;
            int Left = (int)(CalibrationImage.ActualWidth - RectWidth) / 2;
            int Right = Left;
            CalibrationRect.Margin = new Thickness(Left, CalibrationRect.Margin.Top, Right, CalibrationRect.Margin.Bottom);
        }

        private void CalibrationHeightChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (CalibrationImage == null)
                return;
            CalibrationHeight = (int)e.NewValue;
            int RectHeight = (int)(CalibrationHeight * CalibrationImage.ActualHeight) / depthFrameDescription.Height;
            int Top = (int)(CalibrationImage.ActualHeight - RectHeight) / 2;
            int Bottom = Top;
            CalibrationRect.Margin = new Thickness(CalibrationRect.Margin.Left, Top, CalibrationRect.Margin.Right, Bottom);
        }

        private void Calibrate(object sender, RoutedEventArgs e)
        {
            IsCalibrate = true;
        }

    }
}
