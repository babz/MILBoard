﻿using System;
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

        // coordinate mapper
        private CoordinateMapper coordinateMapper = null;

        // body
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        private readonly Pen highlightRightArmPenRed = new Pen(Brushes.Red, 12);
        private readonly Pen highlightRightArmPenYellow = new Pen(Brushes.Yellow, 12);

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;
        Body[] bodies;

        // body-index
        //BodyIndexFrameReader biReader = null; //based on depth img; info abt which depth/IR pixels belong to tracked people and which are bg
        byte[] biDataSource;
        uint[] biImageBuffer; //tmp storage for frame data converted to color
        WriteableBitmap biBitmap;

        //-----XAML variables-----
        //private bool isFullHD = true;

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
        //ColorFrameReader colorReader = null;
        WriteableBitmap colorBitmap;

        // depth
        ushort[] depthDataSource;

        // combined color - bodyIndex
        ushort[] stencilBuffer;
        byte[] combiColorBuffer1080p;
        WriteableBitmap combiBitmap1080p;
        // ------
        byte[] combiColorBuffer;
        WriteableBitmap combiBitmap;

        //check performance in ticks
        const long TICKS_PER_SECOND = 10000000;
        long prevTick = 0;

        //gui logic
        bool isFullHD = false;
        enum BackgroundType { Black, White, Custom };
        BackgroundType bgType = BackgroundType.Custom;
        private bool extendArm = false;

        // right lower arm detection for scaling
        bool isArmDetected = false;
        IReadOnlyDictionary<JointType, Joint> joints;
        Dictionary<JointType, Point> armJointPoints = new Dictionary<JointType, Point>();

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
            colorBitmap = new WriteableBitmap(fdColor.Width, fdColor.Height, 96.0, 96.0, PixelFormats.Bgra32, null);
            //imageColor.Source = colorBitmap; //color img only

            // body-index
            FrameDescription fdBi = sensor.BodyIndexFrameSource.FrameDescription;
            biDataSource = new byte[fdBi.LengthInPixels];
            biImageBuffer = new uint[fdBi.LengthInPixels];
            biBitmap = new WriteableBitmap(fdBi.Width, fdBi.Height, 96, 96, PixelFormats.Bgra32, null);
            //imageBi.Source = biBitmap; //body index img only

            // depth (same resolution as body index)
            FrameDescription fdDepth = sensor.DepthFrameSource.FrameDescription;
            depthDataSource = new ushort[fdDepth.LengthInPixels];

            // combination 1080p
            stencilBuffer = new ushort[fdBi.LengthInPixels];
            combiColorBuffer1080p = new byte[fdColor.LengthInPixels * 4];
            combiBitmap1080p = new WriteableBitmap(fdColor.Width, fdColor.Height, 96, 96, PixelFormats.Bgra32, null);

            //combination 512x424 (depth resolution)
            combiColorBuffer = new byte[fdDepth.LengthInPixels * 4];
            combiBitmap = new WriteableBitmap(fdDepth.Width, fdDepth.Height, 96, 96, PixelFormats.Bgra32, null);
            imageCombi.Source = combiBitmap; //img with 512x424-color of body index frame;

            //############################################
            // SKELETON

            // get the coordinate mapper
            this.coordinateMapper = this.sensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.sensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            //displayWidth = frameDescription.Width;
            //displayHeight = frameDescription.Height;
            displayWidth = this.sensor.ColorFrameSource.FrameDescription.Width;
            displayHeight = this.sensor.ColorFrameSource.FrameDescription.Height;


            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // TODO draws skeleton
            //imageBody.Source = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            depthIntoColorSpace = new ColorSpacePoint[depthDataSource.Length];
            colorIntoDepthSpace = new DepthSpacePoint[fdColor.LengthInPixels];


            //############################################

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

                //-----------------------
                //check performance
                long ticksNow = DateTime.Now.Ticks;
                float fps = ((float)TICKS_PER_SECOND) / (ticksNow - prevTick); // 1 / ((ticksNow - prevTick) / TICKS_PER_SECOND);
                Console.Out.WriteLine("fps: " + fps);
                // store current tick as prevTick for next computation
                prevTick = ticksNow;
                //-----------------------
                //return;


                dFrame.CopyFrameDataToArray(depthDataSource);
                biFrame.CopyFrameDataToArray(biDataSource);


                // body frame
                if (this.bodies == null)
                {
                    this.bodies = new Body[bodyFrame.BodyCount];
                }
                // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                // As long as those body objects are not disposed and not set to null in the array,
                // those body objects will be re-used.
                bodyFrame.GetAndRefreshBodyData(this.bodies);

                DrawBodies();

                //######################################### Get Right Arm ###############################################
                if (extendArm)
                {
                    foreach (Body body in this.bodies)
                    {
                        if (body.IsTracked)
                        {
                            //IReadOnlyDictionary<JointType, Joint> joints = body.Joints;
                            joints = body.Joints;
                            // convert the joint points to depth (display) space


                            Joint wristRight = joints[JointType.WristRight];
                            CameraSpacePoint positionWrist = wristRight.Position;
                            if (positionWrist.Z < 0)
                            {
                                positionWrist.Z = InferredZPositionClamp;
                            }
                            DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(positionWrist);
                            if (wristRight.TrackingState == TrackingState.Tracked)
                            {
                                this.armJointPoints[JointType.WristRight] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            Joint elbowRight = joints[JointType.ElbowRight];
                            CameraSpacePoint positionElbow = elbowRight.Position;
                            if (positionElbow.Z < 0)
                            {
                                positionElbow.Z = InferredZPositionClamp;
                            }

                            depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(positionElbow);
                            if (elbowRight.TrackingState == TrackingState.Tracked)
                            {
                                this.armJointPoints[JointType.ElbowRight] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            Console.Out.WriteLine("arm joint points: x =" + depthSpacePoint.X + ", y =" + depthSpacePoint.Y);
                            isArmDetected = true;

                        }
                    }
                }
                //##############################


                //-------------------------------------------------------------------
                // Color Frame
                FrameDescription fdColor = cFrame.FrameDescription;

                // cut out color frame according to stencil mask
                if (cFrame.RawColorImageFormat == ColorImageFormat.Bgra)
                {
                    cFrame.CopyRawFrameDataToArray(combiColorBuffer1080p);
                }
                else
                {
                    cFrame.CopyConvertedFrameDataToArray(combiColorBuffer1080p, ColorImageFormat.Bgra);
                }

                if (!isFullHD)
                {
                    sensor.CoordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthIntoColorSpace);
                    Array.Clear(combiColorBuffer, 0, combiColorBuffer.Length);


                    unsafe
                    {
                        fixed (byte* ptrCombiColorBuffer = combiColorBuffer)
                        fixed (byte* ptrCombiColorBuffer1080p = combiColorBuffer1080p)
                        {
                            bool stretchEnabled = false;
                            int xElbow = 0;
                            int yElbow = 0;
                            int xWrist = 0;
                            int yWrist = 0;

                            /* // could be used for better start / end conditions                            
                            // normal vector of the right lower arm 
                            float xNormalVector = 0;
                            float yNormalVector = 0;
                            */
                            double normalizedAngle = 0.0;

                            if (isArmDetected && extendArm)
                            {
                                if ((joints[JointType.WristRight].TrackingState == TrackingState.Tracked) && (joints[JointType.ElbowRight].TrackingState == TrackingState.Tracked))
                                {
                                    stretchEnabled = true;
                                    Point pWrist = armJointPoints[JointType.WristRight];
                                    Point pElbow = armJointPoints[JointType.ElbowRight];
                                    int depthWidth = sensor.DepthFrameSource.FrameDescription.Width;
                                    int depthHeight = sensor.DepthFrameSource.FrameDescription.Height;
                                    //Console.Out.WriteLine("pElbow: x =" + pElbow.X + ", y =" + pElbow.Y);
                                    int indexWrist = ((int)pWrist.Y) * depthWidth + ((int)pWrist.X);
                                    int indexElbow = ((int)pElbow.Y) * depthWidth + ((int)pElbow.X);
                                    xElbow = (int)pElbow.X;
                                    yElbow = (int)pElbow.Y;
                                    xWrist = (int)pWrist.X;
                                    yWrist = (int)pWrist.Y;

                                    double deltaX = pWrist.X - pElbow.X;
                                    double deltaY = pWrist.Y - pElbow.Y;
                                    // gradient is positive for vectors going up
                                    if (deltaX == 0) { deltaX = 0.01f; } // hot-fix to avoid division by zero 
                                    double gradient = deltaY / deltaX; // Steigung
                                    double angle = Math.Atan(gradient) * 180.0f / Math.PI; // value between -90 and 90 degree

                                    // we don't care about + or - sign --> and also we want to have a normalized direction (0 - 1)
                                    normalizedAngle = Math.Abs(angle / 180.0f);

                                    // computes the left hand side normal vector
                                    // --> the arm is on the right hand side of this normal vector
                                    //xNormalVector = (deltaY) * -1;
                                    //yNormalVector = deltaX;


                                    // compute the angle (]-\pi/2;\pi/2[) --> -90 to 90 degree
                                }
                            }

                            //after the loop, only color pixels with a body index value that can be mapped to a depth value will remain in the buffer
                            for (int i = 0; i < depthIntoColorSpace.Length; i++)
                            {
                                bool extendedHandDrawn = false;

                                //always stretch in x-dir
                                if (stretchEnabled)
                                {
                                    // Determine the current 2 dimensional coordinate in the image (x, y)
                                    int w = sensor.DepthFrameSource.FrameDescription.Width;
                                    int x = i % w;
                                    int y = i / w;

                                    // Check if the point is on the right hand side of the normal vector
                                    if ((x > xElbow)) //&& (x < xWrist))
                                    {
                                        extendedHandDrawn = true; // hack

                                        int offsetX = x - xElbow;
                                        //int lookupX = xElbow + (offsetX / 2);
                                        int lookupX = (int) (xElbow + (offsetX / (2.0 - normalizedAngle)));

                                        int offsetY = y - yElbow;
                                        //int lookupY = y;
                                        int lookupY = (int) (yElbow + (offsetY / (1.0 + normalizedAngle)));


                                        int colorPointX_stretch = (int)(depthIntoColorSpace[w * lookupY + lookupX].X + 0.5);
                                        int colorPointY_stretch = (int)(depthIntoColorSpace[w * lookupY + lookupX].Y + 0.5); //stays the same
                                        uint* intPtr_stretch = (uint*)(ptrCombiColorBuffer + i * 4); //stays the same

                                        if ((biDataSource[w * lookupY + lookupX] != 0xff) &&
                                            (colorPointY_stretch < fdColor.Height) && (colorPointX_stretch < fdColor.Width) &&
                                            (colorPointY_stretch >= 0) && (colorPointX_stretch >= 0))
                                        {
                                            uint* intPtr1080p = (uint*)(ptrCombiColorBuffer1080p + (colorPointY_stretch * fdColor.Width + colorPointX_stretch) * 4); // corresponding pixel in the 1080p image
                                            *intPtr_stretch = *intPtr1080p; // assign color value (4 bytes)
                                            *(((byte*)intPtr_stretch) + 3) = (byte)userTransparency.Value; // overwrite the alpha value                                            
                                        }
                                    }
                                }
                                if (extendedHandDrawn)
                                {
                                    continue;
                                }

                                int colorPointX = (int)(depthIntoColorSpace[i].X + 0.5);
                                int colorPointY = (int)(depthIntoColorSpace[i].Y + 0.5);
                                uint* intPtr = (uint*)(ptrCombiColorBuffer + i * 4);

                                if ((biDataSource[i] != 0xff) &&
                                    (colorPointY < fdColor.Height) && (colorPointX < fdColor.Width) &&
                                    (colorPointY >= 0) && (colorPointX >= 0))
                                {
                                    uint* intPtr1080p = (uint*)(ptrCombiColorBuffer1080p + (colorPointY * fdColor.Width + colorPointX) * 4); // corresponding pixel in the 1080p image
                                    *intPtr = *intPtr1080p; // assign color value (4 bytes)
                                    *(((byte*)intPtr) + 3) = (byte)userTransparency.Value; // overwrite the alpha value
                                    /*
                                    combiColorBuffer[i * 4] = combiColorBuffer1080p[(colorPointY * fdColor.Width + colorPointX) * 4]; //b
                                    combiColorBuffer[i * 4 + 1] = combiColorBuffer1080p[(colorPointY * fdColor.Width + colorPointX) * 4 + 1]; //g
                                    combiColorBuffer[i * 4 + 2] = combiColorBuffer1080p[(colorPointY * fdColor.Width + colorPointX) * 4 + 2]; //r
                                    combiColorBuffer[i * 4 + 3] = (byte)userTransparency.Value; //alpha of person
                                    */
                                }


                            } // for loop                            
                        }
                    } // unsafe

                    //combiColorBuffer1080p contains all required information
                    combiBitmap.Lock();
                    //combiBitmap.WritePixels(new Int32Rect(0, 0, this.combiBitmap.PixelWidth, this.combiBitmap.PixelHeight), combiColorBuffer, combiBitmap.PixelWidth * sizeof(int), 0);
                    Marshal.Copy(combiColorBuffer, 0, combiBitmap.BackBuffer, combiColorBuffer.Length);
                    combiBitmap.AddDirtyRect(new Int32Rect(0, 0, (int)combiBitmap.Width, (int)combiBitmap.Height));
                    combiBitmap.Unlock();
                }
                else
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
                                //combiColorBuffer1080p[i * 4 + 3] = 255; //alpha of person
                                combiColorBuffer1080p[i * 4 + 3] = (byte)userTransparency.Value;
                            }
                        }
                    } // for loop
                    //combiColorBuffer1080p contains all required information
                    combiBitmap1080p.WritePixels(new Int32Rect(0, 0, this.combiBitmap1080p.PixelWidth, this.combiBitmap1080p.PixelHeight), combiColorBuffer1080p, combiBitmap1080p.PixelWidth * sizeof(int), 0);
                } // full HD
                // Color Frame
                //-------------------------------------------------------------------


            } // using Frames
        }

        void DrawBodies()
        {
            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                Color tmpColor = new Color();
                tmpColor.R = 0;
                tmpColor.G = 0;
                tmpColor.B = 0;
                tmpColor.A = 0;


                Brush tmpBrush = new SolidColorBrush(tmpColor);
                dc.DrawRectangle(tmpBrush, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                int penIndex = 0;
                foreach (Body body in this.bodies)
                {
                    Pen drawPen = this.bodyColors[penIndex++];

                    if (body.IsTracked)
                    {
                        this.DrawClippedEdges(body, dc);

                        IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                        // convert the joint points to depth (display) space
                        Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                        foreach (JointType jointType in joints.Keys)
                        {
                            // sometimes the depth(Z) of an inferred joint may show as negative
                            // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                            CameraSpacePoint position = joints[jointType].Position;
                            if (position.Z < 0)
                            {
                                position.Z = InferredZPositionClamp;
                            }

                            //DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                            //jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            ColorSpacePoint colorSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(position);
                            jointPoints[jointType] = new Point(colorSpacePoint.X, colorSpacePoint.Y);

                        }

                        this.DrawBody(joints, jointPoints, dc, drawPen);

                        this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                        this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
            } // using DrawingContext

        }


        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;


                if (((joint0.JointType == JointType.ElbowRight) && (joint1.JointType == JointType.WristRight)) ||
                    ((joint1.JointType == JointType.ElbowRight) && (joint0.JointType == JointType.WristRight)))
                {
                    if (drawPen.Brush == Brushes.Red)
                    {
                        drawPen = highlightRightArmPenYellow;
                    }
                    else
                    {
                        drawPen = highlightRightArmPenRed;
                    }
                }

                drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
            }
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            /*
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
             * */
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {

            if (this.sensor != null)
            {
                this.sensor.Close();
                this.sensor = null;
            }
        }

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

    }
}
