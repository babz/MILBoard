using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;

namespace BodyExtractionAndHightlighting
{
    public class HandManagerLowRes : IHandManager
    {
        private int bodyIndexSensorBufferWidth, bodyIndexSensorBufferHeight, colorSensorBufferWidth, colorSensorBufferHeight;
        private byte[] bodyIndexSensorBuffer, colorSensorBuffer;

        private CoordinateMapper coordinateMapper;
        private ushort[] depthDataSource;

        private unsafe Point pWrist, pHandTip, pTouch, pHand, pElbow;
        private unsafe int xWrist, yWrist, xHand, yHand, vElbowWristX, vElbowWristY;
        private volatile unsafe int xTranslationOffset, yTranslationOffset;
        private unsafe Vector vElbowToWristOrig;

        unsafe protected ColorSpacePoint[] depthToColorSpaceMapper = null;
        unsafe protected DepthSpacePoint[] colorToDepthSpaceMapper = null;

        private Helper helper;
        private byte userTransparency;

        private volatile unsafe byte* ptrBodyIndexSensorBuffer, ptrColorSensorBuffer;
        private volatile unsafe uint* ptrImageBufferInt, ptrColorSensorBufferInt;
        private volatile unsafe ColorSpacePoint* ptrDepthToColorSpaceMapper;

        public HandManagerLowRes(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, Dictionary<JointType, Point> armJointPoints, Point pTouch, byte userTransparency) 
        {
            this.bodyIndexSensorBufferWidth = Constants.GetBodyIndexSensorBufferWidth();
            this.bodyIndexSensorBufferHeight = Constants.GetBodyIndexSensorBufferHeight();
            this.colorSensorBufferWidth = Constants.GetColorSensorBufferWidth();
            this.colorSensorBufferHeight = Constants.GetColorSensorBufferHeight();

            this.bodyIndexSensorBuffer = bodyIndexSensorBuffer;
            this.colorSensorBuffer = colorSensorBuffer;

            this.coordinateMapper = Constants.GetCoordinateMapper();
            this.depthDataSource = depthDataSource;

            this.pWrist = armJointPoints[JointType.WristRight];
            this.pHandTip = armJointPoints[JointType.HandTipRight];
            this.pTouch = pTouch;
            this.pHand = armJointPoints[JointType.HandRight];
            this.pElbow = armJointPoints[JointType.ElbowRight];

            this.xWrist = (int)(pWrist.X + 0.5);
            this.yWrist = (int)(pWrist.Y + 0.5);
            this.xHand = (int)(pHand.X + 0.5);
            this.yHand = (int)(pHand.Y + 0.5);

            this.depthToColorSpaceMapper = new ColorSpacePoint[depthDataSource.Length];
            this.colorToDepthSpaceMapper = new DepthSpacePoint[colorSensorBufferWidth * colorSensorBufferHeight];

            this.helper = Helper.getInstance();
            this.userTransparency = userTransparency;
        }

        public unsafe void processImage(byte[] imageBufferLowRes)
        {
            coordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferLowRes = imageBufferLowRes)
            fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
            {
                this.ptrBodyIndexSensorBuffer = ptrBodyIndexSensorBuffer;
                this.ptrColorSensorBuffer = ptrColorSensorBuffer;
                this.ptrDepthToColorSpaceMapper = ptrDepthToColorSpaceMapper;

                this.ptrImageBufferInt = (uint*)ptrImageBufferLowRes;
                this.ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                this.drawBody();

                //==draw second, translated hand
                this.vElbowToWristOrig = new Vector((pWrist.X - pElbow.X), (pWrist.Y - pElbow.Y));
                this.vElbowWristX = (int)(pWrist.X - pElbow.X + 0.5);
                this.vElbowWristY = (int)(pWrist.Y - pElbow.Y + 0.5);

                this.xTranslationOffset = (int)(pTouch.X - pHandTip.X + 0.5);
                this.yTranslationOffset = (int)(pTouch.Y - pHandTip.Y + 0.5);

                this.translateHand(xHand, yHand);
                //this.transform_LowRes(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferInt, ptrDepthToColorSpaceMapper, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

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
                if ((xCurrOrigArm < bodyIndexSensorBufferWidth) && (xCurrOrigArm >= 0) && (yCurrOrigArm < bodyIndexSensorBufferHeight) && (yCurrOrigArm >= 0))
                {
                    uint* ptrImgBufferPixelInt = ptrImageBufferInt + (((int)(yCurrOrigArm + 0.5)) * bodyIndexSensorBufferWidth + ((int)(xCurrOrigArm + 0.5)));
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
            //int vWristRightNormX = ((int)(vWristRight.X + 0.5));
            //int vWristRightNormY = ((int)(vWristRight.Y + 0.5));

            // new position where the color is written into
            //uint* ptrImgBufferPixelInt = null; 

            //start at 50 pixel right on wrist vector
            float vWrist_posX = (float)(pWrist.X - vWristRight.X * 50);
            float vWrist_posY = (float)(pWrist.Y - vWristRight.Y * 50);
            stepsOnVector = 100;
            for (int i = 0; i < stepsOnVector; i++)
            {
                if ((vWrist_posX < bodyIndexSensorBufferWidth) && (vWrist_posX >= 0) && (vWrist_posY < bodyIndexSensorBufferHeight) && (vWrist_posY >= 0))
                {
                    uint* ptrImgBufferPixelInt = ptrImageBufferInt + (((int)(vWrist_posY + 0.5)) * bodyIndexSensorBufferWidth + ((int)(vWrist_posX + 0.5)));
                    *ptrImgBufferPixelInt = 0xFFFF0000;
                    *(((byte*)ptrImgBufferPixelInt) + 3) = 255;
                }

                vWrist_posX += (float)vWristRight.X;
                vWrist_posY += (float)vWristRight.Y;
            }
        }

        private unsafe void drawBody()
        {
            //pixel target
            uint* ptrImgBufferPixelInt = null;
            uint* ptrColorSensorBufferPixelInt = null;

            //save computing power by incrementing x, y without division/modulo
            int xDepthSpace = 0;
            int yDepthSpace = 0;

            //==draw whole body without manipulation
            int depthSpaceSize = depthDataSource.Length; //bodyIndexSensorBufferHeight* bodyIndexSensorBufferWidth;
            for (int idxDepthSpace = 0; idxDepthSpace < depthSpaceSize; idxDepthSpace++)
            {
                //ptrColorSensorBufferPixelInt = null;
                ptrImgBufferPixelInt = null;
                bool isColorPixelInValidRange = false;

                if (ptrBodyIndexSensorBuffer[idxDepthSpace] != 0xff)
                {
                    int colorPointX = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].X + 0.5);
                    int colorPointY = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].Y + 0.5);

                    if ((colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) &&
                            (colorPointY >= 0) && (colorPointX >= 0))
                    {
                        isColorPixelInValidRange = true;
                    }

                    if (isColorPixelInValidRange)
                    {
                        // point to current pixel in image buffer
                        ptrImgBufferPixelInt = ptrImageBufferInt + idxDepthSpace;

                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
                        // assign color value (4 bytes)
                        *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                        // overwrite the alpha value (last byte)
                        *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                    }
                } //if body

                //increment counter
                if (++xDepthSpace == bodyIndexSensorBufferWidth)
                {
                    xDepthSpace = 0;
                    yDepthSpace++;
                }
            } //for loop
        }

        private unsafe void translateHand(int xStart, int yStart)
        {
            if ((xStart >= bodyIndexSensorBufferWidth) || (xStart < 0) || (yStart >= bodyIndexSensorBufferHeight) || (yStart < 0))
            {
                return;
            }

            //boundary: normal vector of wrist; no normalization necessary!! (only signum is of value)
            int vPointWristX = xStart - xWrist;
            int vPointWristY = yStart - yWrist;
            int sig = vElbowWristX * vPointWristX + vElbowWristY * vPointWristY;
            //point ON line counts to hand
            if (sig < 0)
            {
                return;
            }

            //pixel already visited
            int depthLookup = yStart * bodyIndexSensorBufferWidth + xStart;
            if (ptrBodyIndexSensorBuffer[depthLookup] == 0xff)
            {
                return;
            }
            else
            {
                ptrBodyIndexSensorBuffer[depthLookup] = 0xff; //do not visit same pixel twice
                int xTranslatedDepthSpace = xStart + xTranslationOffset;
                int yTranslatedDepthSpace = yStart + yTranslationOffset;
                int colorPointX = (int)(ptrDepthToColorSpaceMapper[depthLookup].X + 0.5);
                int colorPointY = (int)(ptrDepthToColorSpaceMapper[depthLookup].Y + 0.5);
                if ((yTranslatedDepthSpace < bodyIndexSensorBufferHeight) && (xTranslatedDepthSpace < bodyIndexSensorBufferWidth) &&
                            (yTranslatedDepthSpace >= 0) && (xTranslatedDepthSpace >= 0) && (colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) &&
                            (colorPointY >= 0) && (colorPointX >= 0))
                {
                    // point to current pixel in image buffer
                    uint* ptrImgBufferPixelInt = ptrImageBufferInt + (yTranslatedDepthSpace * bodyIndexSensorBufferWidth + xTranslatedDepthSpace);

                    uint* ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
                    // assign color value (4 bytes)
                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency * Constants.HAND_TRANSLATED_ALPHAFACTOR);
                }
            }

            //8-way neighbourhood to visit all pixels of hand (can have background pixel btw fingers)
            this.translateHand((xStart + 1), yStart);
            this.translateHand((xStart - 1), yStart);
            this.translateHand(xStart, (yStart + 1));
            this.translateHand(xStart, (yStart - 1));
            this.translateHand((xStart - 1), (yStart - 1));
            this.translateHand((xStart - 1), (yStart + 1));
            this.translateHand((xStart + 1), (yStart - 1));
            this.translateHand((xStart + 1), (yStart + 1));
        }

        private unsafe void transform_LowRes(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferInt, ColorSpacePoint* ptrDepthToColorSpaceMapper, float xWrist, float yWrist, float xHandTip, float yHandTip, float xTouch, float yTouch)
        {
            uint* ptrImgBufferPixelInt = null; // this is where we want to write the pixel
            uint* ptrColorSensorBufferPixelInt = null;

            //save computing power by incrementing x, y without division/modulo
            int xDepthSpace = 0;
            int yDepthSpace = 0;

            float xOffset = xTouch - xHandTip;
            float yOffset = yTouch - yHandTip;

            int depthSpaceSize = bodyIndexSensorBufferHeight * bodyIndexSensorBufferWidth;
            for (int idxDepthSpace = 0; idxDepthSpace < depthSpaceSize; idxDepthSpace++)
            {
                //ptrColorSensorBufferPixelInt = null;
                ptrImgBufferPixelInt = null;
                bool isColorPixelInValidRange = false;

                if (ptrBodyIndexSensorBuffer[idxDepthSpace] != 0xff)
                {
                    int colorPointX = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].X + 0.5);
                    int colorPointY = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].Y + 0.5);

                    if ((colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) &&
                            (colorPointY >= 0) && (colorPointX >= 0))
                    {
                        isColorPixelInValidRange = true;
                    }

                    if (isColorPixelInValidRange)
                    {
                        // point to current pixel in image buffer
                        ptrImgBufferPixelInt = ptrImageBufferInt + idxDepthSpace;

                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
                        // assign color value (4 bytes)
                        *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                        // overwrite the alpha value (last byte)
                        *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;


                        #region --- Hand duplication

                        //TODO determine region of hand with floodfill (start: xHandTip/yHandTip until xWrist)
                        if (xDepthSpace >= xWrist)
                        {
                            float xTranslatedDepthSpace = xDepthSpace + xOffset;
                            float yTranslatedDepthSpace = yDepthSpace + yOffset;

                            if ((yTranslatedDepthSpace < bodyIndexSensorBufferHeight) && (xTranslatedDepthSpace < bodyIndexSensorBufferWidth) &&
                                (yTranslatedDepthSpace >= 0) && (xTranslatedDepthSpace >= 0))
                            {
                                ptrImgBufferPixelInt = ptrImageBufferInt + ((int)(yTranslatedDepthSpace + 0.5) * bodyIndexSensorBufferWidth + (int)(xTranslatedDepthSpace + 0.5));

                                // assign color value (4 bytes)
                                *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                                // overwrite the alpha value (last byte)
                                *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency * Constants.HAND_TRANSLATED_ALPHAFACTOR);
                            }
                        }
                        #endregion // hand
                    }
                } //if body

                //increment counter
                if (++xDepthSpace == bodyIndexSensorBufferWidth)
                {
                    xDepthSpace = 0;
                    yDepthSpace++;
                }
            } //for loop
        }
    }
}
