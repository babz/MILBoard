using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;
using System.Threading;
using System.Windows.Media.Imaging;
using System.Runtime.InteropServices;

namespace BodyExtractionAndHightlighting
{
    public class HandManagerHD : HDManager, IHandManager
    {
        private byte[] bodyIndexSensorBuffer, colorSensorBuffer;
        private ushort[] depthDataSource;
        private DepthSpacePoint[] colorToDepthSpaceMapper = null;

        private Point pTouch;
        private int xElbow, yElbow, xWrist, yWrist, xHand, yHand;
        private int xTranslationOffset, yTranslationOffset;
        
        private Vector vElbowToWristOrig;

        public HandManagerHD(IntPtr ptrBackbuffer, byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, IReadOnlyDictionary<JointType, Joint> bodyJoints, byte userTransparency, Point pTouch)
            : base(ptrBackbuffer, bodyJoints, userTransparency)
        {
            this.bodyIndexSensorBuffer = bodyIndexSensorBuffer;
            this.colorSensorBuffer = colorSensorBuffer;

            this.colorToDepthSpaceMapper = new DepthSpacePoint[colorImageLength];
            this.depthDataSource = depthDataSource;

            this.pTouch = pTouch;
        }

        public unsafe void processImage()
        {
            coordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorToDepthSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
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
                    int xHandTip = (int)(rightArmJoints[JointType.HandTipRight].X + 0.5);
                    int yHandTip = (int)(rightArmJoints[JointType.HandTipRight].Y + 0.5);
                    //offset for hand translation
                    this.xTranslationOffset = (int)(pTouch.X - xHandTip + 0.5);
                    this.yTranslationOffset = (int)(pTouch.Y - yHandTip + 0.5);

                    //vector elbow to wrist, also normal vector of wrist
                    this.vElbowToWristOrig = new Vector((xWrist - xElbow), (yWrist - yElbow));

                    //==draw a translated right hand duplicate
                    Thread threadTranslateHand = new Thread(() => translateHand(xHand, yHand), Constants.STACK_SIZE);
                    threadTranslateHand.Start();

                    //==write-through
                    base.drawFullBody();
                    threadTranslateHand.Join();
                }
                else
                {
                    //==write-through
                    base.drawFullBody();
                }
                
                if (Constants.IsSkeletonShown)
                {
                    this.drawWristVector();
                }

            } //end fixed
        }

        private unsafe void drawWristVector()
        {
            //==== Elbow-Wrist
            vElbowToWristOrig.Normalize();

            float xCurrOrigArm = xWrist;
            float yCurrOrigArm = yWrist;
            int stepsOnVector = 50;
            for (int i = 0; i < stepsOnVector; i++)
            {
                if ((xCurrOrigArm < colorSensorBufferWidth) && (xCurrOrigArm >= 0) && (yCurrOrigArm < colorSensorBufferHeight) && (yCurrOrigArm >= 0))
                {
                    uint* ptrImgBufferPixelInt = ptrBackbuffer + (((int)(yCurrOrigArm + 0.5)) * colorSensorBufferWidth + ((int)(xCurrOrigArm + 0.5)));
                    *ptrImgBufferPixelInt = 0xFF00FFFF;
                    *(((byte*)ptrImgBufferPixelInt) + 3) = 255;
                }

                xCurrOrigArm += (float)vElbowToWristOrig.X;
                yCurrOrigArm += (float)vElbowToWristOrig.Y;
            }

            //==== WRIST
            //wrist vector is normal to elbowToWrist vector
            Vector vWristRight = new Vector(-vElbowToWristOrig.Y, vElbowToWristOrig.X);
            vWristRight.Normalize();

            //start at 50 pixel right on wrist vector
            float vWrist_posX = (float)(xWrist - vWristRight.X * 50);
            float vWrist_posY = (float)(yWrist - vWristRight.Y * 50);
            stepsOnVector = 100;
            for (int i = 0; i < stepsOnVector; i++)
            {
                if ((vWrist_posX < colorSensorBufferWidth) && (vWrist_posX >= 0) && (vWrist_posY < colorSensorBufferHeight) && (vWrist_posY >= 0))
                {
                    // new position where the color is written into
                    uint* ptrImgBufferPixelInt = ptrBackbuffer + (((int)(vWrist_posY + 0.5)) * colorSensorBufferWidth + ((int)(vWrist_posX + 0.5)));
                    *ptrImgBufferPixelInt = 0xFFFF0000;
                    *(((byte*)ptrImgBufferPixelInt) + 3) = 255;
                }

                vWrist_posX += (float)vWristRight.X;
                vWrist_posY += (float)vWristRight.Y;
            }
        }

        // important: always stop if an error occurs (out of bounds, pixel already visited)
        // NOTE stack too small for recursive impl
        private unsafe void translateHand(int xStart, int yStart)
        {
            if ((xStart >= colorSensorBufferWidth) || (xStart < 0) || (yStart >= colorSensorBufferHeight) || (yStart < 0))
            {
                return;
            }

            int idxCurrColorPixel = yStart * colorSensorBufferWidth + xStart;
            if (idxCurrColorPixel >= colorImageLength) 
            {
                return;
            }

            //boundary: normal vector of wrist; no normalization necessary!! (only signum is of value)
            int vPointWristX = xStart - xWrist;
            int vPointWristY = yStart - yWrist;
            int sig = ((int)(vElbowToWristOrig.X + 0.5)) * vPointWristX + ((int)(vElbowToWristOrig.Y + 0.5)) * vPointWristY;
            //point ON line counts to hand
            if (sig < 0)
            {
                return;
            }

            //pixel already visited
            uint* ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxCurrColorPixel;

            float xDepthPixel = (ptrColorToDepthSpaceMapper+idxCurrColorPixel)->X;
            float yDepthPixel = (ptrColorToDepthSpaceMapper+idxCurrColorPixel)->Y;
            int idxDepthPixel = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
            if (Single.IsInfinity(xDepthPixel) || Single.IsInfinity(yDepthPixel) || (*(ptrBodyIndexSensorBuffer+idxDepthPixel) == 0xff))
            {
                return;
            }
            else
            {
                int xTranslatedColorSpace = xStart + xTranslationOffset;
                int yTranslatedColorSpace = yStart + yTranslationOffset;
                if ((yTranslatedColorSpace < colorSensorBufferHeight) && (xTranslatedColorSpace < colorSensorBufferWidth) && (yTranslatedColorSpace >= 0) && (xTranslatedColorSpace >= 0))
                {
                    // point to current pixel in image buffer
                    uint* ptrImgBufferPixelInt = ptrBackbuffer + (yTranslatedColorSpace * colorSensorBufferWidth + xTranslatedColorSpace);

                    //pixel already visited
                    if (*(((byte*)ptrImgBufferPixelInt) + 3) == (byte)(this.userTransparency * Constants.HAND_TRANSLATED_ALPHAFACTOR))
                    {
                        return;
                    }
                    
                    // assign color value (4 bytes)
                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency * Constants.HAND_TRANSLATED_ALPHAFACTOR);
                }
                else
                {
                    return;
                }
            }

            //4-way neighbourhood to visit all pixels of hand (can have background pixel btw fingers)
            this.translateHand((xStart + 1), yStart);
            this.translateHand((xStart - 1), yStart);
            this.translateHand(xStart, (yStart + 1));
            this.translateHand(xStart, (yStart - 1));
        }
    }
}
