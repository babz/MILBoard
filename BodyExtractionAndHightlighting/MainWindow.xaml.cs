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
        private bool hasTouchOccurred = false;
        private Point touchPosition = new Point(0, 0);

        // coordinate mapper
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        Body[] bodies;

        // body-index
        byte[] bodyIndexSensorBuffer;
        uint[] biImageBuffer; //tmp storage for frame data converted to color
        WriteableBitmap biBitmap;

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
        byte[] colorSensorBuffer;
        byte[] imageBufferHD;
        WriteableBitmap writeableBitmapHD;
        
        // for low res: 512x424
        byte[] imageBufferLowRes;
        WriteableBitmap writeableBitmapLowRes;

        //check performance in ticks
        const long TICKS_PER_SECOND = 10000000; //according to msdn
        double fps = 0;
        long prevTick = 0, ticksNow = 0;

        //gui logic
        bool isFullHD = false;
        enum BackgroundType { Black, White, Custom };
        enum GUIPointerType { Arm, Hand, Symbol };
        GUIPointerType guiPointerType = GUIPointerType.Arm;
        BackgroundType bgType = BackgroundType.Custom;

        // right lower arm detection for scaling
        IReadOnlyDictionary<JointType, Joint> joints;
        Dictionary<JointType, Point> armJointPoints = new Dictionary<JointType, Point>();

        //checkbox values
        bool armScaleOnly = false;
        bool armRotateOnly = false;

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
            bodyIndexSensorBuffer = new byte[fdBi.LengthInPixels];
            biImageBuffer = new uint[fdBi.LengthInPixels];
            biBitmap = new WriteableBitmap(fdBi.Width, fdBi.Height, 96, 96, PixelFormats.Bgra32, null);
            //imageBi.Source = biBitmap; //body index img only

            // depth (same resolution as body index)
            fdDepth = sensor.DepthFrameSource.FrameDescription;
            depthDataSource = new ushort[fdDepth.LengthInPixels];

            // combination 1080p
            stencilBuffer = new ushort[fdBi.LengthInPixels];
            colorSensorBuffer = new byte[fdColor.LengthInPixels * 4];

            imageBufferHD = new byte[fdColor.LengthInPixels * 4];
            writeableBitmapHD = new WriteableBitmap(fdColor.Width, fdColor.Height, 96, 96, PixelFormats.Bgra32, null);

            //combination 512x424 (depth resolution)
            imageBufferLowRes = new byte[fdDepth.LengthInPixels * 4];
            writeableBitmapLowRes = new WriteableBitmap(fdDepth.Width, fdDepth.Height, 96, 96, PixelFormats.Bgra32, null);

            imageCombi.Source = writeableBitmapLowRes; //img with 512x424-color of body index frame;

            // get the coordinate mapper
            this.coordinateMapper = this.sensor.CoordinateMapper;

            if (sensor != null)
            {
                reader = sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Body | FrameSourceTypes.BodyIndex);
                reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;

                sensor.Open();
            }
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
                biFrame.CopyFrameDataToArray(bodyIndexSensorBuffer);


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
                    cFrame.CopyRawFrameDataToArray(colorSensorBuffer);
                }
                else
                {
                    cFrame.CopyConvertedFrameDataToArray(colorSensorBuffer, ColorImageFormat.Bgra);
                }

                //########################## Start processing ##########################


                ImageProcessor imgProcessor = new ImageProcessor(fdDepth.Width, fdDepth.Height, fdColor.Width, fdColor.Height, bodyIndexSensorBuffer, colorSensorBuffer, sensor, depthDataSource);
                imgProcessor.PropUserTransparency = (byte)this.userTransparency.Value;

                // --- 512x424 ---
                if (!isFullHD)
                {
                    Array.Clear(imageBufferLowRes, 0, imageBufferLowRes.Length);
                    //########### Get Right Arm Joint-Points ###########
                    bool armDetected = this.DetectArm();

                    //normal image writethrough
                    if (!hasTouchOccurred || !armDetected)
                    {
                        imgProcessor.processImageSimple_LowRes(imageBufferLowRes);
                    }
                    else
                    {
                        bool isHDTest = false;                        
                        Point pTouch = this.GetKinectCoordinates(this.touchPosition);
                        ArmExtensionManager armProcessor = new ArmExtensionManager(fdDepth.Width, fdDepth.Height, fdColor.Width, fdColor.Height, bodyIndexSensorBuffer, colorSensorBuffer, sensor, depthDataSource, armJointPoints, pTouch);
                        armProcessor.PropUserTransparency = (byte)this.userTransparency.Value;
                        //arm operation
                        if (guiPointerType == GUIPointerType.Arm)
                        {
                            if (armScaleOnly)
                            {
                                armProcessor.processImage_scaleOnly(imageBufferLowRes);
                            }
                            else if (armRotateOnly)
                            {
                                armProcessor.processImage_rotationOnly(imageBufferLowRes);
                            }
                            else if (isHDTest)
                            {
                                // HD test
                                armProcessor.processImageLowRes_HD_test(imageBufferLowRes);
                            }
                            else
                            {
                                // normal
                                armProcessor.processImageLowRes(imageBufferLowRes);
                            }
                        }
                        else if (guiPointerType == GUIPointerType.Hand) 
                        {

                        }
                        else if (guiPointerType == GUIPointerType.Symbol)
                        {

                        }
                        else
                        {
                            throw new ApplicationException("Error: undefined pointer state");
                        }
                    } 
                    
                    //===========
                    writeableBitmapLowRes.Lock();
                    Marshal.Copy(imageBufferLowRes, 0, writeableBitmapLowRes.BackBuffer, imageBufferLowRes.Length);
                    writeableBitmapLowRes.AddDirtyRect(new Int32Rect(0, 0, (int)writeableBitmapLowRes.Width, (int)writeableBitmapLowRes.Height));
                    writeableBitmapLowRes.Unlock();
                }
                // --- FullHD 1920 x 1080 ---
                else if (isFullHD)
                {
                    Array.Clear(imageBufferHD, 0, imageBufferHD.Length);

                    if ((guiPointerType == GUIPointerType.Arm) && hasTouchOccurred)
                    {
                        Point pTouch = this.GetKinectCoordinates(this.touchPosition);

                        ArmExtensionManager aemProcessor = new ArmExtensionManager(fdDepth.Width, fdDepth.Height, fdColor.Width, fdColor.Height, bodyIndexSensorBuffer, colorSensorBuffer, sensor, depthDataSource, armJointPoints, pTouch);

                        aemProcessor.processImageHD(imageBufferHD);
                    }
                    else
                    {
                        //normal image writethrough
                        imgProcessor.processImageSimple_HD(imageBufferHD);
                    }

                    //===========
                    //combiColorBuffer1080p contains all required information
                    writeableBitmapHD.WritePixels(new Int32Rect(0, 0, this.writeableBitmapHD.PixelWidth, this.writeableBitmapHD.PixelHeight), imageBufferHD, writeableBitmapHD.PixelWidth * sizeof(int), 0);
                }
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
        

        #region private Methods

        private Point GetKinectCoordinates(Point touchpoint)
        {
            if (armRotateOnly || armScaleOnly)
            {
                touchpoint = new Point(800.0, 550.0);
            }

            int touchpointX = (int)(touchpoint.X + 0.5) * 512;
            int touchpointY = (int)(touchpoint.Y + 0.5) * 424;
            int Kx = touchpointX / (int)(this.imageCanvas.ActualWidth + 0.5);
            int Ky = touchpointY / (int)(this.imageCanvas.ActualHeight + 0.5);

            return new Point(Kx, Ky);
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

        private bool DetectArm()
        {
            bool armDetected = false;

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

        #endregion


        #region GUI properties

        private void nonFullHD_Checked(object sender, RoutedEventArgs e)
        {
            isFullHD = false;
            if (imageCombi != null)
            {
                imageCombi.Source = writeableBitmapLowRes; //img with 512x424-color of body index frame;
            }
        }

        private void fullHD_Checked(object sender, RoutedEventArgs e)
        {
            isFullHD = true;
            if (imageCombi != null)
            {
                imageCombi.Source = writeableBitmapHD; //img with 1080p-color of body index frame;
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

        private void GUIArmPtr_Checked(object sender, RoutedEventArgs e)
        {
            guiPointerType = GUIPointerType.Arm;
        }

        private void GUIHandPtr_Checked(object sender, RoutedEventArgs e)
        {
            guiPointerType = GUIPointerType.Hand;
        }

        private void GUISymbolPtr_Checked(object sender, RoutedEventArgs e)
        {
            guiPointerType = GUIPointerType.Symbol;
        }

        private void checkBoxShowFps_Unchecked(object sender, RoutedEventArgs e)
        {
            showFps = false;
        }

        private void checkBoxShowFps_Checked(object sender, RoutedEventArgs e)
        {
            showFps = true;
        }

        private void checkBoxRotateOnly_Unchecked(object sender, RoutedEventArgs e)
        {
            armRotateOnly = false;
            hasTouchOccurred = false;
        }

        private void checkBoxRotateOnly_Checked(object sender, RoutedEventArgs e)
        {
            armRotateOnly = true;
            armScaleOnly = false;
            hasTouchOccurred = true;
        }

        private void checkBoxScaleOnly_Unchecked(object sender, RoutedEventArgs e)
        {
            armScaleOnly = false;
            hasTouchOccurred = false;
        }

        private void checkBoxScaleOnly_Checked(object sender, RoutedEventArgs e)
        {
            armScaleOnly = true;
            armRotateOnly = false;
            hasTouchOccurred = true;
        }

        private void Window_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            Console.Out.WriteLine("Canvas Pos: " + Mouse.GetPosition(imageCanvas).ToString());
            Console.Out.WriteLine("RightButtonDown: " + e.GetPosition(this).ToString());
            hasTouchOccurred = true;
            //touchPosition = e.GetPosition(this);
            touchPosition = Mouse.GetPosition(imageCanvas);
        }

        private void Window_MouseRightButtonUp(object sender, MouseButtonEventArgs e)
        {
            hasTouchOccurred = false;
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
