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
        [DllImport("msvcrt.dll", EntryPoint = "memset", CallingConvention = CallingConvention.Cdecl, SetLastError = false)]
        public static extern IntPtr MemSet(IntPtr dest, int valueToSet, int length);


        KinectSensor sensor = null;

        /// <summary>
        /// instead of creating readers for each stream, combine them using multiSourceFrameReader
        /// Allows the app to get a matched set of frames from multiple sources on a single event
        /// Caveat: Delivers frames at the lowest FPS of the selected sources
        /// No access to audio (because of framerate reduction)
        /// </summary>
        MultiSourceFrameReader reader;

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
        //WriteableBitmap colorBitmap;

        // depth
        ushort[] depthSensorBuffer;

        // combined color - bodyIndex
        ushort[] stencilBuffer;
        byte[] colorSensorBuffer;
        //byte[] imageBufferHD;
        WriteableBitmap writeableBitmapHD;
        
        // for low res: 512x424
        //byte[] imageBufferLowRes;
        WriteableBitmap writeableBitmapLowRes;

        //check performance in ticks
        const long TICKS_PER_SECOND = 10000000; //according to msdn
        double fps = 0;
        long prevTick = 0, ticksNow = 0;

        //gui logic
        enum BackgroundType { Black, White, Custom };
        enum GUIPointerType { Arm, Hand, Symbol };
        //default settings
        bool isFullHD = false;
        GUIPointerType guiPointerType = GUIPointerType.Arm;
        BackgroundType bgType = BackgroundType.White;

        IImgProcessorFactory imgProcessor = null;

        // right lower arm detection for scaling
        IReadOnlyDictionary<JointType, Joint> joints;
        Dictionary<JointType, Point> armJointPointsDepth = new Dictionary<JointType, Point>();

        //checkbox values
        bool armScaleOnly = false;
        bool armRotateOnly = false;
        private bool showFps = false;
        private bool hasTouchOccurred = false;
        private Point touchPosition = new Point(0, 0);

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
            FrameDescription fdColor = sensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
            //colorBitmap = new WriteableBitmap(fdColor.Width, fdColor.Height, 96.0, 96.0, PixelFormats.Bgra32, null);
            //imageColor.Source = colorBitmap; //color img only

            // body-index
            FrameDescription fdBi = sensor.BodyIndexFrameSource.FrameDescription;
            bodyIndexSensorBuffer = new byte[fdBi.LengthInPixels];
            biImageBuffer = new uint[fdBi.LengthInPixels];
            biBitmap = new WriteableBitmap(fdBi.Width, fdBi.Height, 96, 96, PixelFormats.Bgra32, null);
            //imageBi.Source = biBitmap; //body index img only

            // depth (same resolution as body index)
            FrameDescription fdDepth = sensor.DepthFrameSource.FrameDescription;
            depthSensorBuffer = new ushort[fdDepth.LengthInPixels];

            // combination 1080p
            stencilBuffer = new ushort[fdBi.LengthInPixels];
            colorSensorBuffer = new byte[fdColor.LengthInPixels * 4];

            //imageBufferHD = new byte[fdColor.LengthInPixels * 4];
            writeableBitmapHD = new WriteableBitmap(fdColor.Width, fdColor.Height, 96, 96, PixelFormats.Bgra32, null);

            //combination 512x424 (depth resolution)
            //imageBufferLowRes = new byte[fdDepth.LengthInPixels * 4];
            writeableBitmapLowRes = new WriteableBitmap(fdDepth.Width, fdDepth.Height, 96, 96, PixelFormats.Bgra32, null);

            // get the coordinate mapper
            CoordinateMapper coordinateMapper = this.sensor.CoordinateMapper;

            //inits
            Constants.initConstants(fdDepth.Width, fdDepth.Height, fdColor.Width, fdColor.Height, coordinateMapper, false);
            if (isFullHD)
            {
                imgProcessor = new ImgProcessorFactoryHD();
                imageCombi.Source = writeableBitmapHD;
            }
            else
            {
                imgProcessor = new ImgProcessorFactoryLowRes();
                imageCombi.Source = writeableBitmapLowRes; //img with 512x424-color of body index frame;
            }


            if (sensor != null)
            {
                reader = sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Body | FrameSourceTypes.BodyIndex);
                reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;

                sensor.Open();
            }
        }

        void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs args)
        {
            //ISSUES
            //hd mode only works if first switched to low res, then back to hd
            //hand manager low res does not work, hd only flickering
            //arm extension manager: hd does not work
            //                       low res works if hand is above button
            //                       infinite loop in Z 226 when checked if there is a body along the right normal vector (check direction of increment)
            //read wouterverweirder

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
                dFrame.CopyFrameDataToArray(depthSensorBuffer);
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

                //TODO Draw skeleton
                //DrawBodies();

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
                //byte[] imageBuffer = null;
                WriteableBitmap writeableBitmap = null;
                if (isFullHD)
                {
                    //imageBuffer = imageBufferHD;
                    writeableBitmap = writeableBitmapHD;
                }
                else
                {
                    //imageBuffer = imageBufferLowRes;
                    writeableBitmap = writeableBitmapLowRes;
                }
                //Array.Clear(imageBuffer, 0, imageBuffer.Length);

                byte userTransparency = (byte)this.userTransparency.Value;

                /*
                    * Get Right Arm Joint-Points DEPTH SPACE
                    * @return true if right elbow, right wrist AND right handTip are tracked by the sensor
                    * */
                bool armTracked = this.GetArmJointPoints();

                writeableBitmap.Lock();
                //clear backbuffer
                MemSet(writeableBitmap.BackBuffer, 0, (int)(writeableBitmap.Width) * (int)(writeableBitmap.Height));

                /*
                * Normal image writethrough
                * @hasTouchOccurred is true if GUIPointerType == (Hand || Symbol) OR Mode == (rotate only || scale only) || right mouse button pressed
                * */
                if (!hasTouchOccurred || !armTracked)
                {
                    imgProcessor.createBasicManager(bodyIndexSensorBuffer, colorSensorBuffer, depthSensorBuffer, userTransparency).processImage(writeableBitmap.BackBuffer);
                }
                else
                {
                    /*
                    * Touchpoint coordinates
                    * @GUIPointerType.Hand: Button5 (1200.0, 550.0)
                    * @GUIPointerType.Symbol: Button5 (1200.0, 550.0)
                    * @Mode == Rotate only: Button5 (1200.0, 550.0)
                    * @Mode == Scale only: Button5 (1200.0, 550.0)
                    * @MousePressed: Actual mouse position
                    * */
                    Point pTouch = this.GetKinectCoordinates(this.touchPosition);
                        
                    //arm operation
                    if (guiPointerType == GUIPointerType.Arm)
                    {
                        IArmExtensionManager armExtensionManager = imgProcessor.createArmExtensionManager(bodyIndexSensorBuffer, colorSensorBuffer, depthSensorBuffer, armJointPointsDepth, pTouch, userTransparency);

                        if (armScaleOnly)
                        {
                            armExtensionManager.processImage_scaleOnly(null);
                        }
                        else if (armRotateOnly)
                        {
                            armExtensionManager.processImage_rotationOnly(null);
                        }
                        else
                        {
                            // normal: scale + rotation
                            armExtensionManager.processImage(null);
                        }
                    }
                    else if (guiPointerType == GUIPointerType.Hand) 
                    {
                        imgProcessor.createHandManager(bodyIndexSensorBuffer, colorSensorBuffer, depthSensorBuffer, armJointPointsDepth, pTouch, userTransparency).processImage(writeableBitmap.BackBuffer);
                    }
                    else if (guiPointerType == GUIPointerType.Symbol)
                    {
                        pointerSymbol.Visibility = Visibility.Visible;
                        imgProcessor.createSymbolManager(bodyIndexSensorBuffer, colorSensorBuffer, depthSensorBuffer, pTouch, userTransparency, pointerSymbol).processImage(null);
                    }
                    else
                    {
                        throw new ApplicationException("Error: undefined gui pointer state");
                    }
                } 
                    
                //===========
                //TODO
                //http://stackoverflow.com/questions/17549123/c-sharp-performance-using-unsafe-pointers-instead-of-intptr-and-marshal
                //Marshal.Copy(imageBuffer, 0, writeableBitmap.BackBuffer, imageBuffer.Length);
                writeableBitmap.AddDirtyRect(new Int32Rect(0, 0, (int)writeableBitmap.Width, (int)writeableBitmap.Height));

                //NOTE alternative; performance unknown
                //writeableBitmapHD.WritePixels(new Int32Rect(0, 0, this.writeableBitmapHD.PixelWidth, this.writeableBitmapHD.PixelHeight), imageBufferHD, writeableBitmapHD.PixelWidth * sizeof(int), 0);
                writeableBitmap.Unlock();

                //NOTE save for later when UIThread is implemented
                //Console.Out.WriteLine("Something went terribly wrong! Frame is disposed.");
                //cFrame.Dispose();
                //dFrame.Dispose();
                //biFrame.Dispose();
                //bodyFrame.Dispose();

            } // using Frames
        }

        #region private Methods

        private Point GetKinectCoordinates(Point touchpoint)
        {
            if (armRotateOnly || armScaleOnly || (guiPointerType == GUIPointerType.Hand))
            {
                touchpoint = new Point(1200.0, 550.0);
            }

            if (guiPointerType == GUIPointerType.Symbol)
            {
                return new Point(1200.0, 550.0);
            }

            //considers the discrepancy btw position of canvas and actual image;
            //canvas relative point is translated to image relative point
            Point relativeTouchpoint = this.imageCanvas.TranslatePoint(touchpoint, imageCombi);

            //source image buffer is smaller than image on screen
            //source image is either 512x424 or fullHD
            int touchpointX = (int)(relativeTouchpoint.X * imageCombi.Source.Width + 0.5);
            int touchpointY = (int)(relativeTouchpoint.Y * imageCombi.Source.Height + 0.5);
            int Kx = touchpointX / (int)(this.imageCombi.ActualWidth + 0.5);
            int Ky = touchpointY / (int)(this.imageCombi.ActualHeight + 0.5);

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

        private unsafe bool GetArmJointPoints()
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
                Joint handRight = joints[JointType.HandRight];
                Joint shoulderRight = joints[JointType.ShoulderRight];

                CameraSpacePoint[] camSpacePosJoints = { wristRight.Position, elbowRight.Position, handTipRight.Position, handRight.Position, shoulderRight.Position };
                uint noOfJoints = (uint)camSpacePosJoints.Length;
                DepthSpacePoint[] depthSpacePosJoints = new DepthSpacePoint[noOfJoints];
                ColorSpacePoint[] colorSpacePosJoints = new ColorSpacePoint[noOfJoints];

                //IntPtr ptrCameraSpacePosJoints = camSpacePosJoints;
                //fixed (CameraSpacePoint* ptrCameraSpacePosJoints = camSpacePosJoints) 
                //fixed (DepthSpacePoint* ptrDepthSpacePosJoints = depthSpacePosJoints)
                //fixed (ColorSpacePoint* ptrColorSpacePosJoints = colorSpacePosJoints)
                //{
                //    uint* ptrCameraSpacePosJointInt = (uint*)ptrCameraSpacePosJoints;
                //    uint* ptrColorSpacePosJointsInt = (uint*)ptrColorSpacePosJoints;

                //    Constants.GetCoordinateMapper().MapCameraPointsToColorSpaceUsingIntPtr(ptrCameraSpacePosJointInt, noOfJoints, ptrColorSpacePosJointsInt, noOfJoints);
                //}
                
                //wristRight
                if (camSpacePosJoints[0].Z < 0)
                    camSpacePosJoints[0].Z = InferredZPositionClamp;
                //elbowRight
                if (camSpacePosJoints[1].Z < 0)
                    camSpacePosJoints[1].Z = InferredZPositionClamp;
                //handTipRight
                if (camSpacePosJoints[2].Z < 0)
                    camSpacePosJoints[2].Z = InferredZPositionClamp;
                //handRight
                if (camSpacePosJoints[3].Z < 0)
                    camSpacePosJoints[3].Z = InferredZPositionClamp;
                //shoulderRight
                if (camSpacePosJoints[4].Z < 0)
                    camSpacePosJoints[4].Z = InferredZPositionClamp;

                if ((wristRight.TrackingState == TrackingState.Tracked) && (elbowRight.TrackingState == TrackingState.Tracked) && (handTipRight.TrackingState == TrackingState.Tracked) && (handRight.TrackingState == TrackingState.Tracked) && (shoulderRight.TrackingState == TrackingState.Tracked))
                {
                    //==joints for depth space
                    //DepthSpacePoint[] depthSpacePosJoints = new DepthSpacePoint[5];
                    //TODO
                    //Constants.GetCoordinateMapper().MapCameraPointsToDepthSpace(camSpacePosJoints, depthSpacePosJoints);

                    DepthSpacePoint d_wrist = depthSpacePosJoints[0];
                    DepthSpacePoint d_elbow = depthSpacePosJoints[1];
                    DepthSpacePoint d_handTip = depthSpacePosJoints[2];
                    DepthSpacePoint d_hand = depthSpacePosJoints[3];
                    DepthSpacePoint d_shoulder = depthSpacePosJoints[4];

                    this.armJointPointsDepth[JointType.WristRight] = new Point(d_wrist.X, d_wrist.Y);
                    this.armJointPointsDepth[JointType.ElbowRight] = new Point(d_elbow.X, d_elbow.Y);
                    this.armJointPointsDepth[JointType.HandTipRight] = new Point(d_handTip.X, d_handTip.Y);
                    this.armJointPointsDepth[JointType.HandRight] = new Point(d_hand.X, d_hand.Y);
                    this.armJointPointsDepth[JointType.ShoulderRight] = new Point(d_shoulder.X, d_shoulder.Y);

                    //==joints for color space
                    //TODO do for color
                    //ColorSpacePoint[] colorSpacePosJoints = new ColorSpacePoint[5];
                    //Constants.GetCoordinateMapper().MapCameraPointsToColorSpaceUsingIntPtr

                    armDetected = true;
                    //Console.Out.WriteLine("============Arm detected! Body: " + body.TrackingId);
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
            imgProcessor = new ImgProcessorFactoryLowRes();
        }

        private void fullHD_Checked(object sender, RoutedEventArgs e)
        {
            isFullHD = true;
            if (imageCombi != null)
            {
                imageCombi.Source = writeableBitmapHD; //img with 1080p-color of body index frame;
            }
            imgProcessor = new ImgProcessorFactoryHD();
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
            hasTouchOccurred = false;
            if (pointerSymbol != null)
            {
                pointerSymbol.Visibility = Visibility.Hidden;
            }
        }

        private void GUIHandPtr_Checked(object sender, RoutedEventArgs e)
        {
            guiPointerType = GUIPointerType.Hand;
            hasTouchOccurred = true;
            if (pointerSymbol != null)
            {
                pointerSymbol.Visibility = Visibility.Hidden;
            }
        }

        private void GUISymbolPtr_Checked(object sender, RoutedEventArgs e)
        {
            guiPointerType = GUIPointerType.Symbol;
            hasTouchOccurred = true;
        }

        private void checkBoxShowFps_Unchecked(object sender, RoutedEventArgs e)
        {
            showFps = false;
        }

        private void checkBoxShowFps_Checked(object sender, RoutedEventArgs e)
        {
            showFps = true;
        }

        private void checkBoxSkeleton_Unchecked(object sender, RoutedEventArgs e)
        {
            Constants.IsSkeletonShown = false;
        }

        private void checkBoxSkeleton_Checked(object sender, RoutedEventArgs e)
        {
            Constants.IsSkeletonShown = true;
        }

        private void checkBoxRotateOnly_Unchecked(object sender, RoutedEventArgs e)
        {
            armRotateOnly = false;
            hasTouchOccurred = false;
        }

        private void checkBoxRotateOnly_Checked(object sender, RoutedEventArgs e)
        {
            if (armScaleOnly)
            {
                checkBoxScaleOnly.IsChecked = false;
                armScaleOnly = false;
            }
            armRotateOnly = true;
            hasTouchOccurred = true;
        }

        private void checkBoxScaleOnly_Unchecked(object sender, RoutedEventArgs e)
        {
            armScaleOnly = false;
            hasTouchOccurred = false;
        }

        private void checkBoxScaleOnly_Checked(object sender, RoutedEventArgs e)
        {
            if (armRotateOnly)
            {
                checkBoxRotateOnly.IsChecked = false;
                armRotateOnly = false;
            }
            armScaleOnly = true;
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
