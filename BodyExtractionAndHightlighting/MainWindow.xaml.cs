using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

using Microsoft.Kinect;
using System.ComponentModel;
using System.Runtime.InteropServices;

namespace BodyExtractionAndHightlighting
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        KinectSensor sensor = null;

        /// <summary>
        /// instead of creating readers for each stream, combine them using multiSourceFrameReader
        /// Allows the app to get a matched set of frames from multiple sources on a single event
        /// Caveat: Delivers frames at the lowest FPS of the selected sources
        /// No access to audio (because of framerate reduction)
        /// </summary>
        MultiSourceFrameReader reader;

        private bool showFps = false;
        private bool isTouchPositionEnabled = false;
        private Point touchPosition;

        // coordinate mapper
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        Body[] bodies;

        // body-index
        byte[] biDataSource;
        uint[] biImageBuffer; //tmp storage for frame data converted to color
        WriteableBitmap biBitmap;

        ColorSpacePoint[] depthIntoColorSpace = null;
        DepthSpacePoint[] colorIntoDepthSpace = null;

        private static readonly uint[] BodyColor = 
        {
            0xFF00FF04,
            0xFFFF0021,
            0xFFFF4082,
            0xFFF2FF21,
            0xFF40F0CF,
            0xFF8080FF
        };

        // color
        WriteableBitmap colorBitmap;
        FrameDescription fdColor;

        // depth
        ushort[] depthDataSource;
        FrameDescription fdDepth;

        // combined color - bodyIndex
        ushort[] stencilBuffer;
        byte[] combiColorBuffer1080p;
        WriteableBitmap combiBitmap1080p;
        // ------
        // for low res: 512x424
        byte[] combiColorBuffer;
        WriteableBitmap combiBitmap;

        //hand extraction
        byte[] handBuffer;

        //check performance in ticks
        const long TICKS_PER_SECOND = 10000000; //according to msdn
        double fps = 0;
        long prevTick = 0, ticksNow = 0;

        //gui logic
        bool isFullHD = false;
        enum BackgroundType { Black, White, Custom };
        BackgroundType bgType = BackgroundType.Custom;
        private bool extendArm = false; //value set via checkbox in GUI

        // right lower arm detection for scaling
        bool isArmDetected = false;
        IReadOnlyDictionary<JointType, Joint> joints;
        Dictionary<JointType, Point> armJointPoints = new Dictionary<JointType, Point>();

        private double sumFps = 0;
        private int counter = 0;

        public MainWindow()
        {
            InitializeComponent();
            this.Loaded += MainWindow_Loaded;
            this.Closing += MainWindow_Closing;
        }

        void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            sensor = KinectSensor.GetDefault();
            bodies = new Body[6];

            // color
            fdColor = sensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
            colorBitmap = new WriteableBitmap(fdColor.Width, fdColor.Height, 96.0, 96.0, PixelFormats.Bgra32, null);
            //imageColor.Source = colorBitmap; //color img only

            // body-index
            FrameDescription fdBi = sensor.BodyIndexFrameSource.FrameDescription;
            biDataSource = new byte[fdBi.LengthInPixels];
            biImageBuffer = new uint[fdBi.LengthInPixels];
            biBitmap = new WriteableBitmap(fdBi.Width, fdBi.Height, 96, 96, PixelFormats.Bgra32, null);
            //imageBi.Source = biBitmap; //body index img only

            // depth (same resolution as body index)
            fdDepth = sensor.DepthFrameSource.FrameDescription;
            depthDataSource = new ushort[fdDepth.LengthInPixels];

            // combination 1080p
            stencilBuffer = new ushort[fdBi.LengthInPixels];
            combiColorBuffer1080p = new byte[fdColor.LengthInPixels * 4];
            combiBitmap1080p = new WriteableBitmap(fdColor.Width, fdColor.Height, 96, 96, PixelFormats.Bgra32, null);

            //combination 512x424 (depth resolution)
            combiColorBuffer = new byte[fdDepth.LengthInPixels * 4];
            combiBitmap = new WriteableBitmap(fdDepth.Width, fdDepth.Height, 96, 96, PixelFormats.Bgra32, null);
            imageCombi.Source = combiBitmap; //img with 512x424-color of body index frame;

            // get the coordinate mapper
            this.coordinateMapper = this.sensor.CoordinateMapper;

            depthIntoColorSpace = new ColorSpacePoint[depthDataSource.Length];
            colorIntoDepthSpace = new DepthSpacePoint[fdColor.LengthInPixels];

            if (sensor != null)
            {
                reader = sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Body | FrameSourceTypes.BodyIndex);
                reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;

                sensor.Open();
            }
        }

        private void calculateFps()
        {
            ticksNow = DateTime.Now.Ticks;
            fps = ((float)TICKS_PER_SECOND) / (DateTime.Now.Ticks - prevTick); // 1 / ((ticksNow - prevTick) / TICKS_PER_SECOND);
            //Console.Out.WriteLine("fps: " + fps);
            Console.Out.WriteLine("fps: " + (int)(fps + 0.5));
            prevTick = ticksNow;

            //calc mean
            sumFps += fps;
            counter++;
        }

        void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs args)
        {
            MultiSourceFrame reference = args.FrameReference.AcquireFrame();

            if (reference == null)
            {
                return;
            }

            using (var cFrame = reference.ColorFrameReference.AcquireFrame())
            using (var dFrame = reference.DepthFrameReference.AcquireFrame())
            using (var biFrame = reference.BodyIndexFrameReference.AcquireFrame())
            using (var bodyFrame = reference.BodyFrameReference.AcquireFrame())
            {
                if ((cFrame == null) || (dFrame == null) || (biFrame == null) || (bodyFrame == null))
                {
                    return;
                }

                //showFps = true;
                //check performance
                if (showFps)
                {
                    this.calculateFps();
                    //return;
                }

                // writes depth values from frame into an array
                dFrame.CopyFrameDataToArray(depthDataSource);
                // writes body index values from frame into an array
                biFrame.CopyFrameDataToArray(biDataSource);


                // -- body frame --
                if (this.bodies == null)
                {
                    //number of bodies the system can track (always 6 for Kinect v2)
                    this.bodies = new Body[bodyFrame.BodyCount];
                }
                // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                // As long as those body objects are not disposed and not set to null in the array,
                // those body objects will be re-used.
                bodyFrame.GetAndRefreshBodyData(this.bodies);

                // -- Color Frame --
                //FrameDescription fdColor = cFrame.FrameDescription;
                if (cFrame.RawColorImageFormat == ColorImageFormat.Bgra)
                {
                    cFrame.CopyRawFrameDataToArray(combiColorBuffer1080p);
                }
                else
                {
                    cFrame.CopyConvertedFrameDataToArray(combiColorBuffer1080p, ColorImageFormat.Bgra);
                }

                //########### Get Right Arm ###########
                isArmDetected = this.GetRightArmJointPoints();
                //#####################################
                ImageProcessor imgProcessor = new ImageProcessor(fdDepth.Width, fdDepth.Height, fdColor.Width, fdColor.Height);

                #region --- 512x424 ---
                if (!isFullHD)
                {
                    sensor.CoordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthIntoColorSpace);
                    Array.Clear(combiColorBuffer, 0, combiColorBuffer.Length);

                    imgProcessor.PropUserTransparency = (byte)this.userTransparency.Value;

                    if (isArmDetected && isTouchPositionEnabled)
                    {
                        Point pElbow = armJointPoints[JointType.ElbowRight];
                        Point pWrist = armJointPoints[JointType.WristRight];
                        Point pHandTip = armJointPoints[JointType.HandTipRight];
                        Point pTouch = this.GetKinectCoordinates(this.touchPosition);
                        imgProcessor.ComputeTransformedImage(biDataSource, combiColorBuffer1080p, combiColorBuffer, depthIntoColorSpace, pElbow, pWrist, pHandTip, pTouch);
                    }
                    else
                    {
                        imgProcessor.ComputeSimpleImage(biDataSource, combiColorBuffer1080p, combiColorBuffer, depthIntoColorSpace);
                    }

                    //===========
                    combiBitmap.Lock();
                    Marshal.Copy(combiColorBuffer, 0, combiBitmap.BackBuffer, combiColorBuffer.Length);
                    combiBitmap.AddDirtyRect(new Int32Rect(0, 0, (int)combiBitmap.Width, (int)combiBitmap.Height));
                    combiBitmap.Unlock();
                }
                #endregion
                #region --- FullHD: Arm NOT Detected or NO extension ---
                else if (isFullHD && !isArmDetected)
                {
                    sensor.CoordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorIntoDepthSpace);

                    //after the loop, only color pixels with a body index value that can be mapped to a depth value will remain in the buffer
                    for (int i = 0; i < fdColor.LengthInPixels; i++)
                    {
                        //where color map has no corresponding value in the depth map due to resolution/sensor position, the pixels are set to black
                        if (Single.IsInfinity(colorIntoDepthSpace[i].Y) || Single.IsInfinity(colorIntoDepthSpace[i].X))
                        {
                            combiColorBuffer1080p[i * 4] = 255; //b
                            combiColorBuffer1080p[i * 4 + 1] = 255; //g
                            combiColorBuffer1080p[i * 4 + 2] = 255; //r
                            combiColorBuffer1080p[i * 4 + 3] = 0; //a
                        }
                        else
                        {
                            //if bodyIndex pixel has no body assigned, draw a black pixel to the corresponding color pixel
                            int idx = (int)(sensor.BodyIndexFrameSource.FrameDescription.Width * colorIntoDepthSpace[i].Y + colorIntoDepthSpace[i].X); //2D to 1D
                            if ((biDataSource[idx] > 5) || (biDataSource[idx] < 0))
                            {
                                combiColorBuffer1080p[i * 4] = 255;
                                combiColorBuffer1080p[i * 4 + 1] = 255;
                                combiColorBuffer1080p[i * 4 + 2] = 255;
                                combiColorBuffer1080p[i * 4 + 3] = 0;
                            }
                            else
                            {
                                combiColorBuffer1080p[i * 4 + 3] = (byte)userTransparency.Value; //alpha of person set by gui-slider
                            }
                        }
                    } // for loop

                    //combiColorBuffer1080p contains all required information
                    combiBitmap1080p.WritePixels(new Int32Rect(0, 0, this.combiBitmap1080p.PixelWidth, this.combiBitmap1080p.PixelHeight), combiColorBuffer1080p, combiBitmap1080p.PixelWidth * sizeof(int), 0);
                }
                #endregion
                #region --- FullHD and Arm Detected and Arm Extension ---
                else if (isFullHD && isArmDetected)
                {
                    //TODO implement
                }
                #endregion
                else
                {
                    Console.Out.WriteLine("Something went terribly wrong! Frame is disposed.");
                    cFrame.Dispose();
                    dFrame.Dispose();
                    biFrame.Dispose();
                    bodyFrame.Dispose();
                    return;
                }
            } // using Frames
        }

        private bool GetRightArmJointPoints()
        {
            bool armDetected = false;

            if (!extendArm)
            {
                return false;
            }

            foreach (Body body in this.bodies)
            {
                if (!body.IsTracked)
                {
                    continue;
                }

                joints = body.Joints;

                // convert the joint points to depth (display) space
                Joint wristRight = joints[JointType.WristRight];
                Joint elbowRight = joints[JointType.ElbowRight];
                Joint handTipRight = joints[JointType.HandTipRight];

                CameraSpacePoint[] camSpacePosJoints = { wristRight.Position, elbowRight.Position, handTipRight.Position };

                if (camSpacePosJoints[0].Z < 0)
                    camSpacePosJoints[0].Z = InferredZPositionClamp;

                if (camSpacePosJoints[1].Z < 0)
                    camSpacePosJoints[1].Z = InferredZPositionClamp;

                if (camSpacePosJoints[2].Z < 0)
                    camSpacePosJoints[2].Z = InferredZPositionClamp;

                if ((wristRight.TrackingState == TrackingState.Tracked) && (elbowRight.TrackingState == TrackingState.Tracked) && (handTipRight.TrackingState == TrackingState.Tracked))
                {
                    DepthSpacePoint[] depthSpacePosJoints = new DepthSpacePoint[3];
                    this.coordinateMapper.MapCameraPointsToDepthSpace(camSpacePosJoints, depthSpacePosJoints);

                    DepthSpacePoint wrist = depthSpacePosJoints[0];
                    DepthSpacePoint elbow = depthSpacePosJoints[1];
                    DepthSpacePoint handTip = depthSpacePosJoints[2];

                    this.armJointPoints[JointType.WristRight] = new Point(wrist.X, wrist.Y);
                    this.armJointPoints[JointType.ElbowRight] = new Point(elbow.X, elbow.Y);
                    this.armJointPoints[JointType.HandTipRight] = new Point(handTip.X, handTip.Y);

                    armDetected = true;
                }
            }

            return armDetected;
        }

        #region private Methods

        private Point GetKinectCoordinates(Point touchpoint)
        {
            int Kx = (int)touchpoint.X * 512 / (int)this.imageCanvas.ActualWidth;
            int Ky = 424 * (int)touchpoint.Y / (int)this.imageCanvas.ActualHeight;

            return new Point(Kx, Ky);
        }

        #endregion

        #region GUI properties

        private void nonFullHD_Checked(object sender, RoutedEventArgs e)
        {
            isFullHD = false;
            if (imageCombi != null)
            {
                imageCombi.Source = combiBitmap; //img with 512x424-color of body index frame;
            }
        }

        private void fullHD_Checked(object sender, RoutedEventArgs e)
        {
            isFullHD = true;
            if (imageCombi != null)
            {
                imageCombi.Source = combiBitmap1080p; //img with 1080p-color of body index frame;
            }
        }

        private void BlackBG_Checked(object sender, RoutedEventArgs e)
        {
            bgType = BackgroundType.Black;
            this.Background = new SolidColorBrush(Colors.Black);
        }

        private void WhiteBG_Checked(object sender, RoutedEventArgs e)
        {
            bgType = BackgroundType.White;
            this.Background = new SolidColorBrush(Colors.White);
        }

        private void CustomBG_Checked(object sender, RoutedEventArgs e)
        {
            bgType = BackgroundType.Custom;
            this.Background = new SolidColorBrush(Colors.Transparent);
        }

        private void checkBoxExtendArm_Checked(object sender, RoutedEventArgs e)
        {
            extendArm = true;
        }

        private void checkBoxExtendArm_Unchecked(object sender, RoutedEventArgs e)
        {
            extendArm = false;
        }

        private void checkBoxShowFps_Unchecked(object sender, RoutedEventArgs e)
        {
            showFps = false;
        }

        private void checkBoxShowFps_Checked(object sender, RoutedEventArgs e)
        {
            showFps = true;
        }

        private void Window_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            Console.Out.WriteLine("Canvas Pos: " + Mouse.GetPosition(imageCanvas).ToString());
            Console.Out.WriteLine("RightButtonDown: " + e.GetPosition(this).ToString());
            isTouchPositionEnabled = true;
            //touchPosition = e.GetPosition(this);
            touchPosition = Mouse.GetPosition(imageCanvas);
        }

        private void Window_MouseRightButtonUp(object sender, MouseButtonEventArgs e)
        {
            isTouchPositionEnabled = false;
        }

        #endregion

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            Console.Out.WriteLine("mean fps:" + (sumFps / (double)counter));

            if (this.sensor != null)
            {
                this.sensor.Close();
                this.sensor = null;
            }
        }

    } //  public partial class MainWindow : Window
} //namespace BodyExtractionAndHightlighting
