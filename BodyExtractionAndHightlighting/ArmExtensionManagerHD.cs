﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;
using System.Threading;

namespace BodyExtractionAndHightlighting
{
    public class ArmExtensionManagerHD : HDManager, IArmExtensionManager
    {
        private byte[] bodyIndexSensorBuffer, colorSensorBuffer;
        private ushort[] depthDataSource;
        private DepthSpacePoint[] colorToDepthSpaceMapper = null;

        private Point pTouch;
        private int xElbow, yElbow, xWrist, yWrist, xHand, yHand, xHandTip, yHandTip;
        private int xTranslationOffset, yTranslationOffset;

        private Vector vElbowToWristOrig, vElbowToWristOrigNorm, vHalfShoulderWrist_NormRight;

        private Helper helper;

        public ArmExtensionManagerHD(IntPtr ptrBackbuffer, byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, IReadOnlyDictionary<JointType, Joint> bodyJoints, byte userTransparency, Point pTouch)
            : base(ptrBackbuffer, bodyJoints, userTransparency, depthDataSource)
        {
            this.bodyIndexSensorBuffer = bodyIndexSensorBuffer;
            this.colorSensorBuffer = colorSensorBuffer;

            this.colorToDepthSpaceMapper = new DepthSpacePoint[colorImageLength];
            this.depthDataSource = depthDataSource;
            
            this.pTouch = pTouch;
            this.helper = Helper.getInstance();
        }

        public unsafe void processImage()
        {
            //coordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);
            coordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorToDepthSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            //fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
            fixed (DepthSpacePoint* ptrColorToDepthSpaceMapper = colorToDepthSpaceMapper)
            {
                this.ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;
                base.SetRequiredBuffers(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt);
                base.SetCoordinateMapper(ptrColorToDepthSpaceMapper);

                bool rightArmTracked = base.isRightArmTracked();
                if (rightArmTracked)
                {
                    Dictionary<JointType, ColorSpacePoint> rightArmJoints = base.GetRightArmJointsColorSpace();

                    this.xElbow = (int)(rightArmJoints[JointType.ElbowRight].X + 0.5);
                    this.yElbow = (int)(rightArmJoints[JointType.ElbowRight].Y + 0.5);
                    this.xWrist = (int)(rightArmJoints[JointType.WristRight].X + 0.5);
                    this.yWrist = (int)(rightArmJoints[JointType.WristRight].Y + 0.5);
                    //start point for floodfill 
                    this.xHand = (int)(rightArmJoints[JointType.HandRight].X + 0.5);
                    this.yHand = (int)(rightArmJoints[JointType.HandRight].Y + 0.5);
                    this.xHandTip = (int)(rightArmJoints[JointType.HandTipRight].X + 0.5);
                    this.yHandTip = (int)(rightArmJoints[JointType.HandTipRight].Y + 0.5);
                    int xShoulder = (int)(rightArmJoints[JointType.ShoulderRight].X + 0.5);
                    int yShoulder = (int)(rightArmJoints[JointType.ShoulderRight].Y + 0.5);
                    //offset for hand translation
                    this.xTranslationOffset = (int)(pTouch.X - xHandTip + 0.5);
                    this.yTranslationOffset = (int)(pTouch.Y - yHandTip + 0.5);

                    //vector elbow to wrist, also normal vector of wrist
                    this.vElbowToWristOrig = new Vector((xWrist - xElbow), (yWrist - yElbow));
                    this.vElbowToWristOrigNorm = vElbowToWristOrig;
                    this.vElbowToWristOrigNorm.Normalize();

                    //lower boundary: half vector of shoulder and wrist
                    Vector vElbowToShoulder = new Vector((xShoulder - xElbow), (yShoulder - yElbow));
                    // H = (S+W)
                    Vector vHalfShoulderWrist = new Vector((vElbowToShoulder.X + vElbowToWristOrig.X), (vElbowToShoulder.Y + vElbowToWristOrig.Y));
                    //vHalfShoulderWrist.Normalize();
                    this.vHalfShoulderWrist_NormRight = new Vector(-vHalfShoulderWrist.Y, vHalfShoulderWrist.X);

                    this.drawStretchedRightLowerArm();

                    //==draw a translated right hand duplicate
                    Thread threadTranslateHand = new Thread(() => drawTranslatedRightHand(xHand, yHand), Constants.STACK_SIZE_HD);
                    //thread.Priority = ThreadPriority.AboveNormal;
                    threadTranslateHand.Start();

                    Thread threadBody = new Thread(() => bodyFloodFill(xElbow, yElbow), Constants.STACK_SIZE_HD);
                    threadBody.Start();
                    threadBody.Join();
                    //sequential approach
                    //this.drawBodyWithoutRightHand();

                    threadTranslateHand.Join();
                }
                else
                {
                    //==write-through
                    base.drawFullBody();
                }

            } //end fixed
        }

        //public unsafe void processImage_rotationOnly()
        //{
        //    coordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);
        //    coordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorToDepthSpaceMapper);

        //    fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
        //    fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
        //    fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
        //    fixed (DepthSpacePoint* ptrColorToDepthSpaceMapper = colorToDepthSpaceMapper)
        //    {
        //        this.ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;
        //        base.SetRequiredBuffers(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt);
        //        base.SetCoordinateMapper(ptrColorToDepthSpaceMapper);

        //        float xElbow = (float)pElbow.X;
        //        float yElbow = (float)pElbow.Y;
        //        float xWrist = (float)pWrist.X;
        //        float yWrist = (float)pWrist.Y;
        //        float xHandTip = (float)pHandTip.X;
        //        float yHandTip = (float)pHandTip.Y;
        //        float xTouch = (float)pTouch.X;
        //        float yTouch = (float)pTouch.Y;

        //        uint* ptrImageBufferHDInt = (uint*)ptrBackbuffer;
        //        uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

        //        this.transform_HD_rotationOnly(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferHDInt, ptrDepthToColorSpaceMapper, ptrColorToDepthSpaceMapper, xElbow, yElbow, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

        //    } //end fixed
        //}

        //public unsafe void processImage_scaleOnly()
        //{
        //    coordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);
        //    coordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorToDepthSpaceMapper);

        //    fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
        //    fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
        //    fixed (byte* ptrImageBufferHD = imageBufferHD)
        //    fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
        //    fixed (DepthSpacePoint* ptrColorToDepthSpaceMapper = colorToDepthSpaceMapper)
        //    {
        //        float xElbow = (float)pElbow.X;
        //        float yElbow = (float)pElbow.Y;
        //        float xWrist = (float)pWrist.X;
        //        float yWrist = (float)pWrist.Y;
        //        float xHandTip = (float)pHandTip.X;
        //        float yHandTip = (float)pHandTip.Y;
        //        float xTouch = (float)pTouch.X;
        //        float yTouch = (float)pTouch.Y;

        //        uint* ptrImageBufferHDInt = (uint*)ptrBackbuffer;
        //        uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

        //        this.transform_HD_scaleOnly(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferHDInt, ptrDepthToColorSpaceMapper, ptrColorToDepthSpaceMapper, xElbow, yElbow, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

        //    } //end fixed
        //}

        private unsafe void bodyFloodFill(int xStart, int yStart)
        {
            if ((xStart >= colorSensorBufferWidth) || (xStart < 0) || (yStart >= colorSensorBufferHeight) || (yStart < 0))
            {
                return;
            }

            int idxCurrColorPixel = yStart * colorSensorBufferWidth + xStart;
            if (idxCurrColorPixel >= colorImageLength) // colorToDepthSpaceMapper.Length = colorSensorBufferWidth * colorSensorBufferHeight
            {
                return;
            }

            int vPointElbowX = (int)(xStart - xElbow + 0.5);
            int vPointElbowY = (int)(yStart - yElbow + 0.5);
            int sigPointElbow = ((int)(vHalfShoulderWrist_NormRight.X + 0.5)) * vPointElbowX + ((int)(vHalfShoulderWrist_NormRight.Y + 0.5)) * vPointElbowY;
            //point is not drawn if p > Elbow
            if (sigPointElbow > 0)
            {
                return;
            }

            //pixel already visited
            uint* ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxCurrColorPixel;
            if ((*ptrColorSensorBufferPixelInt) == 0xFF000000)
            {
                return;
            }

            float xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->X;
            float yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->Y;
            int idxDepthPixel = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
            if (Single.IsInfinity(xDepthPixel) || Single.IsInfinity(yDepthPixel) || (*(ptrBodyIndexSensorBuffer + idxDepthPixel) == 0xff))
            {
                return;
            }
            else
            {
                int vPointWristX = xStart - xWrist;
                int vPointWristY = yStart - yWrist;
                int sig = ((int)(vElbowToWristOrig.X + 0.5)) * vPointWristX + ((int)(vElbowToWristOrig.Y + 0.5)) * vPointWristY;
                //point ON line counts to hand
                int xTranslatedColorSpace = xStart;// +xTranslationOffset;
                int yTranslatedColorSpace = yStart;// +yTranslationOffset;

                // point to current pixel in image buffer
                uint* ptrImgBufferPixelInt = ptrBackbuffer + (yTranslatedColorSpace * colorSensorBufferWidth + xTranslatedColorSpace);

                // assign color value (4 bytes)
                *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                // overwrite the alpha value (last byte)
                *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency);

                //do not visit same pixel twice
                *ptrColorSensorBufferPixelInt = 0xFF000000;
            }

            //4-way neighbourhood to visit all pixels of hand (can have background pixel btw fingers)
            this.bodyFloodFill((xStart + 1), yStart);
            this.bodyFloodFill((xStart - 1), yStart);
            this.bodyFloodFill(xStart, (yStart + 1));
            this.bodyFloodFill(xStart, (yStart - 1));
        }

        private unsafe void drawBodyWithoutRightHand()
        {
            //with the cast to int, step size is 4 bytes
            uint* ptrImgBufferPixelInt = null;
            uint* ptrColorSensorBufferPixelInt = null;

            //save computing power by incrementing x, y without division/modulo
            int xColorSpace = 0;
            int yColorSpace = 0;

            //==draw whole body without manipulation
            int lengthColorBuffer = colorToDepthSpaceMapper.Length; // = colorSensorBufferHeight * colorSensorBufferWidth;
            //after the loop, only color pixels with a body index value that can be mapped to a depth value will remain in the buffer
            for (int idxColorSpace = 0; idxColorSpace < lengthColorBuffer; idxColorSpace++)
            {
                //where the color img cannot be mapped to the depth image, there are infinity values
                if (!Single.IsInfinity((ptrColorToDepthSpaceMapper + idxColorSpace)->Y) && !Single.IsInfinity((ptrColorToDepthSpaceMapper + idxColorSpace)->X))
                {
                    int idxDepthSpace = (int)(bodyIndexSensorBufferWidth * (ptrColorToDepthSpaceMapper + idxColorSpace)->Y + (ptrColorToDepthSpaceMapper + idxColorSpace)->X); //2D to 1D

                    // bodyIndex can be 0, 1, 2, 3, 4, or 5
                    if (*(ptrBodyIndexSensorBuffer + idxDepthSpace) != 0xff)
                    {
                        //omit right lower hand pixel; use half vector btw shoulder and wrist as termination criteria
                        int vPointElbowX = (int)(xColorSpace - xElbow + 0.5);
                        int vPointElbowY = (int)(yColorSpace - yElbow + 0.5);
                        int sigPointElbow = ((int)(vHalfShoulderWrist_NormRight.X + 0.5)) * vPointElbowX + ((int)(vHalfShoulderWrist_NormRight.Y + 0.5)) * vPointElbowY;
                        //point is not drawn if p > Elbow
                        if (sigPointElbow <= 0)
                        {
                            ptrImgBufferPixelInt = ptrBackbuffer + idxColorSpace;

                            //with the cast to int, 4 bytes are copied
                            *ptrImgBufferPixelInt = *(ptrColorSensorBufferInt + idxColorSpace);
                            // overwrite the alpha value
                            *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                        }
                    }
                }

                //increment counter
                if (++xColorSpace == colorSensorBufferWidth)
                {
                    xColorSpace = 0;
                    yColorSpace++;
                }
            } // for loop
        }

        private unsafe void drawStretchedRightLowerArm()
        {
            uint* ptrImgBufferPixelInt = null; // new position where the color is written into
            uint* ptrColorSensorBufferPixelInt = null; // color pixel position in color frame

            // v = (x, y)
            float vOrigArmLength = (float)vElbowToWristOrig.Length;

            //NOTE vector normals different as coordinate system origin (0,0) is upper left corner!
            //v_nleft = (y, -x)
            Vector vNormLeftOrigArm = new Vector(vElbowToWristOrigNorm.Y, -(vElbowToWristOrigNorm.X));
            //v_nright = (-y, x)
            Vector vNormRightOrigArm = new Vector(-(vElbowToWristOrigNorm.Y), vElbowToWristOrigNorm.X);

            //move arm so that it fits to the shifted hand
            Vector offsetNewArmWrist = new Vector((xHandTip - xWrist), (yHandTip - yWrist));

            Vector vNewArm = new Vector((pTouch.X - ((int)(offsetNewArmWrist.X + 0.5)) - xElbow), (pTouch.Y - ((int)(offsetNewArmWrist.Y + 0.5)) - yElbow));
            float vNewArmLength = (float)vNewArm.Length;
            vNewArm.Normalize();

            Vector vNormLeftNewArm = new Vector(vNewArm.Y, -(vNewArm.X));
            Vector vNormRightNewArm = new Vector(-(vNewArm.Y), vNewArm.X);

            //sampling rate
            float totalSteps;
            if (vOrigArmLength < vNewArmLength)
                totalSteps = (float)(vNewArmLength * 2.2);
            else
                totalSteps = (float)(vOrigArmLength * 2.2);
            float stepSizeOrigArm = vOrigArmLength / totalSteps;
            float stepSizeNewArm = vNewArmLength / totalSteps;

            float xCurrOrigArm = xElbow;
            float yCurrOrigArm = yElbow;
            float xCurrNewArm = xElbow;
            float yCurrNewArm = yElbow;
            for (int i = 0; i < totalSteps; i++)
            {
                //write color pixel into position of new arm
                if ((xCurrNewArm < colorSensorBufferWidth) && (yCurrNewArm < colorSensorBufferHeight) && (xCurrNewArm >= 0) && (yCurrNewArm >= 0))
                {
                    int newPositionX = (int)(xCurrNewArm + 0.5);
                    int newPositionY = (int)(yCurrNewArm + 0.5);
                    //position of new position pixel
                    ptrImgBufferPixelInt = ptrBackbuffer + (newPositionY * colorSensorBufferWidth + newPositionX);

                    // assign color value
                    if (Constants.IsSkeletonShown)
                    {
                        //draw line where new stretched vector is
                        *ptrImgBufferPixelInt = 0xFF00FFFF;

                        //draw red line where orig hand vector is
                        ptrImgBufferPixelInt = ptrBackbuffer + (int)((int)(yCurrOrigArm + 0.5) * colorSensorBufferWidth + xCurrOrigArm + 0.5);
                        *ptrImgBufferPixelInt = 0xFFFF0000; //ARGB in storage

                        *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                    }
                    else
                    {
                        //color of original pixel
                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (int)((int)(yCurrOrigArm + 0.5) * colorSensorBufferWidth + xCurrOrigArm + 0.5);

                        // assign color value (4 bytes)
                        *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                        // overwrite the alpha value (last byte)
                        *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                    }
                }

                #region left normal vector of elbow-wrist vector

                //calculate for left normal of vector; all variables in color space unless defined otherwise
                float xNormLeft = xCurrOrigArm + (float)vNormLeftOrigArm.X;
                float yNormLeft = yCurrOrigArm + (float)vNormLeftOrigArm.Y;
                int idxCurrColorPixel = (int)((int)(yNormLeft + 0.5) * colorSensorBufferWidth + xNormLeft + 0.5);
                float xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->X;
                float yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->Y;
                int idxDepthPixel = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
                float newPosNormLeftX = xCurrNewArm + (float)vNormLeftNewArm.X;
                float newPosNormLeftY = yCurrNewArm + (float)vNormLeftNewArm.Y;

                while (!Single.IsInfinity(xDepthPixel) && !Single.IsInfinity(yDepthPixel) && (*(ptrBodyIndexSensorBuffer+idxDepthPixel) != 0xff))
                {
                    //check image boundaries
                    if ((xNormLeft >= colorSensorBufferWidth) || (xNormLeft < 0) || (yNormLeft >= colorSensorBufferHeight) || (yNormLeft < 0) || (newPosNormLeftX >= colorSensorBufferWidth) || (newPosNormLeftX < 0) || (newPosNormLeftY >= colorSensorBufferHeight) || (newPosNormLeftY < 0))
                    {
                        break;
                    }

                    //normal might point inside body
                    //lower boundary: half vector of shoulder and wrist (upper boundary implicitly given through touch point)
                    int vPointElbowX = (int)(xNormLeft - xElbow + 0.5);
                    int vPointElbowY = (int)(yNormLeft - yElbow + 0.5);
                    int sigPointElbow = ((int)(vHalfShoulderWrist_NormRight.X + 0.5)) * vPointElbowX + ((int)(vHalfShoulderWrist_NormRight.Y + 0.5)) * vPointElbowY;
                    //point is not drawn if p < Elbow
                    if (sigPointElbow < 0)
                    {
                        break;
                    }
                    
                    //position of new arm
                    ptrImgBufferPixelInt = ptrBackbuffer + (int)((int)(newPosNormLeftY + 0.5) * colorSensorBufferWidth + newPosNormLeftX + 0.5);

                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxCurrColorPixel;
                    //write color pixel into position of left norm of new arm
                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;

                    //increment to move along left normal vector
                    xNormLeft += (float)vNormLeftOrigArm.X;
                    yNormLeft += (float)vNormLeftOrigArm.Y;
                    newPosNormLeftX += (float)vNormLeftNewArm.X;
                    newPosNormLeftY += (float)vNormLeftNewArm.Y;
                    idxCurrColorPixel = (int)((int)(yNormLeft + 0.5) * colorSensorBufferWidth + xNormLeft + 0.5);
                    xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->X;
                    yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->Y;
                }

                #endregion

                #region right normal vector of elbow-wrist vector

                //calculate for right normal of vector; all variables in color space unless defined otherwise
                float xNormRight = xCurrOrigArm + (float)vNormRightOrigArm.X;
                float yNormRight = yCurrOrigArm + (float)vNormRightOrigArm.Y;
                idxCurrColorPixel = (int)((int)(yNormRight + 0.5) * colorSensorBufferWidth + xNormRight + 0.5);
                if (idxCurrColorPixel >= colorToDepthSpaceMapper.Length)
                    return; 

                xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->X;
                yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->Y;
                float newPosNormRightX = xCurrNewArm + (float)vNormRightNewArm.X;
                float newPosNormRightY = yCurrNewArm + (float)vNormRightNewArm.Y;
                while (!Single.IsInfinity(xDepthPixel) && !Single.IsInfinity(yDepthPixel) && (*(ptrBodyIndexSensorBuffer + ((int)((int)(yDepthPixel + 0.5) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5))) != 0xff))
                {
                    //normal might point inside body;
                    if ((xNormRight >= colorSensorBufferWidth) || (xNormRight < 0) || (yNormRight >= colorSensorBufferHeight) || (yNormRight < 0) || (newPosNormRightX >= colorSensorBufferWidth) || (newPosNormRightX < 0) || (newPosNormRightY >= colorSensorBufferHeight) || (newPosNormRightY < 0))
                    {
                        break;
                    }

                    //normal might point inside body
                    //lower boundary: half vector of shoulder and wrist (upper boundary implicitly given through touch point)
                    int vPointElbowX = (int)(xNormRight - xElbow + 0.5);
                    int vPointElbowY = (int)(yNormRight - yElbow + 0.5);
                    int sigPointElbow = ((int)(vHalfShoulderWrist_NormRight.X + 0.5)) * vPointElbowX + ((int)(vHalfShoulderWrist_NormRight.Y + 0.5)) * vPointElbowY;
                    //point is not drawn if p < Elbow
                    if (sigPointElbow < 0)
                    {
                        break;
                    }
                    
                    //position of new arm
                    ptrImgBufferPixelInt = ptrBackbuffer + (int)((int)(newPosNormRightY + 0.5) * colorSensorBufferWidth + newPosNormRightX + 0.5);

                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxCurrColorPixel;
                    //write color pixel into position of left norm of new arm
                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;

                    //increment to move along left normal vector
                    xNormRight += (float)vNormRightOrigArm.X;
                    yNormRight += (float)vNormRightOrigArm.Y;
                    newPosNormRightX += (float)vNormRightNewArm.X;
                    newPosNormRightY += (float)vNormRightNewArm.Y;
                    idxCurrColorPixel = (int)((int)(yNormRight + 0.5) * colorSensorBufferWidth + xNormRight + 0.5);
                    xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->X;
                    yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->Y;
                }
                #endregion

                //increment to move along vector; use normal vectors for increment
                xCurrOrigArm += (float)(vElbowToWristOrigNorm.X) * stepSizeOrigArm;
                yCurrOrigArm += (float)(vElbowToWristOrigNorm.Y) * stepSizeOrigArm;
                xCurrNewArm += (float)(vNewArm.X) * stepSizeNewArm;
                yCurrNewArm += (float)(vNewArm.Y) * stepSizeNewArm;

            }
            
        }

        private unsafe void drawTranslatedRightHand(int xStart, int yStart)
        {
            if ((xStart >= colorSensorBufferWidth) || (xStart < 0) || (yStart >= colorSensorBufferHeight) || (yStart < 0))
            {
                return;
            }

            int idxCurrColorPixel = yStart * colorSensorBufferWidth + xStart;
            if (idxCurrColorPixel >= colorToDepthSpaceMapper.Length) // colorToDepthSpaceMapper.Length = colorSensorBufferWidth * colorSensorBufferHeight
            {
                return;
            }

            //boundary: normal vector of wrist; no normalization necessary!! (only signum is of value)
            int vPointToWristX = xStart - xWrist;
            int vPointToWristY = yStart - yWrist;
            int sig = ((int)(vElbowToWristOrig.X + 0.5)) * vPointToWristX + ((int)(vElbowToWristOrig.Y + 0.5)) * vPointToWristY;
            //point ON line counts to hand
            if (sig < 0)
            {
                return;
            }

            //pixel already visited
            uint* ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxCurrColorPixel;
            if ((*ptrColorSensorBufferPixelInt) == 0xFF000000)
            {
                return;
            }

            float xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->X;
            float yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->Y;
            int idxDepthPixel = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
            if (Single.IsInfinity(xDepthPixel) || Single.IsInfinity(yDepthPixel) || (*(ptrBodyIndexSensorBuffer + idxDepthPixel) == 0xff))
            {
                return;
            }
            else
            {
                int xTranslatedColorSpace = xStart + xTranslationOffset;
                int yTranslatedColorSpace = yStart + yTranslationOffset;
                if ((yTranslatedColorSpace < colorSensorBufferHeight) && (xTranslatedColorSpace < colorSensorBufferWidth) &&
                            (yTranslatedColorSpace >= 0) && (xTranslatedColorSpace >= 0))
                {
                    // point to current pixel in image buffer
                    uint* ptrImgBufferPixelInt = ptrBackbuffer + (yTranslatedColorSpace * colorSensorBufferWidth + xTranslatedColorSpace);
                    // assign color value (4 bytes)
                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency);

                    //do not visit same pixel twice
                    *ptrColorSensorBufferPixelInt = 0xFF000000;
                }
                else
                {
                    return;
                }
            }

            //8-way neighbourhood to visit all pixels of hand (can have background pixel btw fingers)
            this.drawTranslatedRightHand((xStart + 1), yStart);
            this.drawTranslatedRightHand((xStart - 1), yStart);
            this.drawTranslatedRightHand(xStart, (yStart + 1));
            this.drawTranslatedRightHand(xStart, (yStart - 1));
        }


        //private unsafe void transform_HD_scaleOnly(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferHDInt, ColorSpacePoint* ptrDepthToColorSpaceMapper, DepthSpacePoint* ptrColorToDepthSpaceMapper, float xElbow, float yElbow, float xWrist, float yWrist, float xHandTip, float yHandTip, float xTouch, float yTouch)
        //{
        //    double normalizedAngle = 0.5; // todo helper.CalculateNormalizedAngleToXaxis(xElbow, yElbow, xWrist, yWrist);

        //    int idxElbowDepthSpace = ((int)(yElbow + 0.5)) * bodyIndexSensorBufferWidth + ((int)(xElbow + 0.5));
        //    float xElbowColorSpace = (ptrDepthToColorSpaceMapper + idxElbowDepthSpace)->X;
        //    float yElbowColorSpace = (ptrDepthToColorSpaceMapper + idxElbowDepthSpace)->Y;

        //    bool isElbowOutOfBound = false;
        //    where the color img cannot be mapped to the depth image, there are infinity values
        //    if (Single.IsInfinity(xElbowColorSpace) || Single.IsInfinity(yElbowColorSpace))
        //    {
        //        isElbowOutOfBound = true;
        //    }

        //    uint* ptrImgBufferHDPixelInt = null; // this is where we want to write the pixel
        //    uint* ptrColorSensorBufferPixelInt = null;

        //    save computing power by incrementing x, y without division/modulo
        //    int xColorSpace = 0;
        //    int yColorSpace = 0;

        //    int colorSpaceSize = colorSensorBufferHeight * colorSensorBufferWidth;
        //    for (int idxColorSpace = 0; idxColorSpace < colorSpaceSize; idxColorSpace++)
        //    {
        //        ptrColorSensorBufferPixelInt = null;

        //        point to current pixel in target imgBuffer
        //        ptrImgBufferHDPixelInt = ptrImageBufferHDInt + idxColorSpace;

        //        if (!isElbowOutOfBound && (xColorSpace >= xElbowColorSpace)) // right arm
        //        {
        //            float offsetX = xColorSpace - xElbowColorSpace;
        //            int lookupX = (int)(xElbowColorSpace + (offsetX / (2.0 - normalizedAngle)) + 0.5);

        //            float offsetY = yColorSpace - yElbowColorSpace;
        //            int lookupY = (int)(yElbowColorSpace + (offsetY / (1.0 + normalizedAngle)) + 0.5);

        //            if ((lookupY < colorSensorBufferHeight) && (lookupX < colorSensorBufferWidth) &&
        //                    (lookupY >= 0) && (lookupX >= 0))
        //            {
        //                int idxColorSpaceStreched = lookupY * colorSensorBufferWidth + lookupX;
        //                float xDepthSpace = (ptrColorToDepthSpaceMapper + idxColorSpaceStreched)->X;
        //                float yDepthSpace = (ptrColorToDepthSpaceMapper + idxColorSpaceStreched)->Y;

        //                where the color img cannot be mapped to the depth image, there are infinity values
        //                if (!Single.IsInfinity(xDepthSpace) && !Single.IsInfinity(yDepthSpace))
        //                {
        //                    int idxDepthSpace = ((int)(yDepthSpace + 0.5)) * bodyIndexSensorBufferWidth + ((int)(xDepthSpace + 0.5));
        //                    if (*(ptrBodyIndexSensorBuffer + idxDepthSpace) != 0xff)
        //                    {
        //                         corresponding pixel color sensor buffer (source)
        //                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxColorSpaceStreched; // corresponding pixel in the 1080p image
        //                    }
        //                }
        //            }
        //        } //end: right arm
        //        else // normal pixel
        //        {
        //            float xDepthSpace = (ptrColorToDepthSpaceMapper + idxColorSpace)->X;
        //            float yDepthSpace = (ptrColorToDepthSpaceMapper + idxColorSpace)->Y;

        //            where the color img cannot be mapped to the depth image, there are infinity values
        //            if (!Single.IsInfinity(xDepthSpace) && !Single.IsInfinity(yDepthSpace))
        //            {
        //                corrresponding pixel in the bodyIndexBuffer; mapper returns pixel in depth space
        //                int idxDepthSpace = ((int)(yDepthSpace + 0.5)) * bodyIndexSensorBufferWidth + ((int)(xDepthSpace + 0.5)); //2D to 1D
        //                if (*(ptrBodyIndexSensorBuffer + idxDepthSpace) != 0xff)
        //                {
        //                     corresponding pixel color sensor buffer (source)
        //                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (yColorSpace * this.colorSensorBufferWidth + xColorSpace);
        //                }
        //            }
        //        } // end: normal pixel

        //        if (ptrColorSensorBufferPixelInt != null)
        //        {
        //             assign color value (4 bytes); dereferencing + assigning writes into imgBuffer
        //            *ptrImgBufferHDPixelInt = *ptrColorSensorBufferPixelInt;
        //             overwrite the alpha value
        //            *(((byte*)ptrImgBufferHDPixelInt) + 3) = this.userTransparency;
        //        }

        //        increment counter
        //        if (++xColorSpace == colorSensorBufferWidth)
        //        {
        //            xColorSpace = 0;
        //            yColorSpace++;
        //        }
        //    } //end for
        //}

        //private unsafe void transform_HD_rotationOnly(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferHDInt, ColorSpacePoint* ptrDepthToColorSpaceMapper, DepthSpacePoint* ptrColorToDepthSpaceMapper, float xElbow, float yElbow, float xWrist, float yWrist, float xHandTip, float yHandTip, float xTouch, float yTouch)
        //{
        //    double rotationAngleInRad = helper.CalculateRotationAngle(xElbow, yElbow, xWrist, yWrist, xTouch, yTouch);
        //    double cos = Math.Cos(rotationAngleInRad);
        //    double sin = Math.Sin(rotationAngleInRad);

        //    int idxElbowDepthSpace = ((int)(yElbow + 0.5)) * bodyIndexSensorBufferWidth + ((int)(xElbow + 0.5));
        //    float xElbowColorSpace = (ptrDepthToColorSpaceMapper + idxElbowDepthSpace)->X;
        //    float yElbowColorSpace = (ptrDepthToColorSpaceMapper + idxElbowDepthSpace)->Y;

        //    bool isElbowOutOfBound = false;
        //    where the color img cannot be mapped to the depth image, there are infinity values
        //    if (Single.IsInfinity(xElbowColorSpace) || Single.IsInfinity(yElbowColorSpace))
        //    {
        //        isElbowOutOfBound = true;
        //    }

        //    uint* ptrImgBufferHDPixelInt = null; // this is where we want to write the pixel
        //    uint* ptrColorSensorBufferPixelInt = null;

        //    Vector areaOffset = new Vector((xWrist - xElbow), (yWrist - yElbow));
        //    int handOffset = (int)areaOffset.Length / 3;
        //    int handTipBoundaryX = (int)(xHandTip + 0.5);// +handOffset;
        //    int handTipBoundaryY = (int)(yHandTip + 0.5);// -handOffset;

        //    save computing power by incrementing x, y without division/modulo
        //    int xColorSpace = 0;
        //    int yColorSpace = 0;

        //    int colorSpaceSize = colorSensorBufferHeight * colorSensorBufferWidth;
        //    for (int idxColorSpace = 0; idxColorSpace < colorSpaceSize; idxColorSpace++)
        //    {
        //        ptrColorSensorBufferPixelInt = null;

        //        float xDepthSpace = (ptrColorToDepthSpaceMapper + idxColorSpace)->X;
        //        float yDepthSpace = (ptrColorToDepthSpaceMapper + idxColorSpace)->Y;
        //        int idxDepthSpace = (int)(yDepthSpace + 0.5) * bodyIndexSensorBufferWidth + (int)(xDepthSpace + 0.5);

        //        if (!Single.IsInfinity(xDepthSpace) && !Single.IsInfinity(yDepthSpace))
        //        {
        //            if (*(ptrBodyIndexSensorBuffer + idxDepthSpace) != 0xff)
        //            {
        //                 right arm
        //                if (!isElbowOutOfBound && (xColorSpace >= xElbowColorSpace))
        //                {

        //                     compute rotation (in target image buffer)
        //                    int xRotatedColorSpace = (int)(cos * (xColorSpace - xElbowColorSpace) - sin * (yColorSpace - yElbowColorSpace) + xElbowColorSpace + 0.5);
        //                    int yRotatedColorSpace = (int)(sin * (xColorSpace - xElbowColorSpace) + cos * (yColorSpace - yElbowColorSpace) + yElbowColorSpace + 0.5);

        //                    if ((yRotatedColorSpace < colorSensorBufferHeight) && (xRotatedColorSpace < colorSensorBufferWidth) &&
        //                        (yRotatedColorSpace >= 0) && (xRotatedColorSpace >= 0))
        //                    {
        //                        ptrImgBufferHDPixelInt = ptrImageBufferHDInt + (yRotatedColorSpace * colorSensorBufferWidth + xRotatedColorSpace);
        //                    }
        //                }
        //                else //rest of body
        //                {
        //                    point to current pixel in target imgBuffer
        //                    ptrImgBufferHDPixelInt = ptrImageBufferHDInt + idxColorSpace;
        //                }
        //            } //if body
        //        } //if idxDepth not infinity

        //        if (ptrImgBufferHDPixelInt != null)
        //        {
        //            check boundaries
        //            if ((yColorSpace < colorSensorBufferHeight) && (xColorSpace < colorSensorBufferWidth) &&
        //                    (yColorSpace >= 0) && (xColorSpace >= 0))
        //            {
        //                ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (yColorSpace * colorSensorBufferWidth + xColorSpace);
        //                 assign color value (4 bytes)
        //                *ptrImgBufferHDPixelInt = *ptrColorSensorBufferPixelInt;
        //                 overwrite the alpha value (last byte)
        //                *(((byte*)ptrImgBufferHDPixelInt) + 3) = this.userTransparency;
        //            }
        //        }

        //        increment counter
        //        if (++xColorSpace == colorSensorBufferWidth)
        //        {
        //            xColorSpace = 0;
        //            yColorSpace++;
        //        }
        //    } //for loop
        //}

    }
}
