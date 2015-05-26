using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;
using System.Threading;

namespace BodyExtractionAndHightlighting
{
    public class HandManagerHD : IHandManager
    {
        private int bodyIndexSensorBufferWidth, bodyIndexSensorBufferHeight, colorSensorBufferWidth, colorSensorBufferHeight;
        private byte[] bodyIndexSensorBuffer, colorSensorBuffer;

        private CoordinateMapper coordinateMapper;
        private ushort[] depthDataSource;

        private unsafe Point pWrist, pHandTip, pTouch, pHand, pElbow;
        private volatile unsafe int xElbowColorSpace, yElbowColorSpace, xWristColorSpace, yWristColorSpace, xHandTipColorSpace, yHandTipColorSpace, xHandColorSpace, yHandColorSpace;
        private volatile unsafe int xTranslationOffset, yTranslationOffset;

        volatile unsafe protected ColorSpacePoint[] depthToColorSpaceMapper = null;
        volatile unsafe protected DepthSpacePoint[] colorToDepthSpaceMapper = null;

        private Helper helper;
        private byte userTransparency;

        private unsafe Vector vElbowToWristOrig;
        private volatile unsafe byte* ptrBodyIndexSensorBuffer, ptrColorSensorBuffer, ptrImageBufferHD;
        private volatile unsafe uint* ptrImageBufferHDInt, ptrColorSensorBufferInt;
        private volatile unsafe ColorSpacePoint* ptrDepthToColorSpaceMapper;
        private volatile unsafe DepthSpacePoint* ptrColorToDepthSpaceMapper;

        private unsafe Object thisLock = new Object();

        public HandManagerHD(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, Dictionary<JointType, Point> armJointPoints, Point pTouch, byte userTransparency) 
        {
            this.bodyIndexSensorBufferWidth = Constants.GetBodyIndexSensorBufferWidth();
            this.bodyIndexSensorBufferHeight = Constants.GetBodyIndexSensorBufferHeight();
            this.colorSensorBufferWidth = Constants.GetColorSensorBufferWidth();
            this.colorSensorBufferHeight = Constants.GetColorSensorBufferHeight();

            this.bodyIndexSensorBuffer = bodyIndexSensorBuffer;
            this.colorSensorBuffer = colorSensorBuffer;

            this.coordinateMapper = Constants.GetCoordinateMapper();
            this.depthDataSource = depthDataSource;

            this.pElbow = armJointPoints[JointType.ElbowRight];
            this.pWrist = armJointPoints[JointType.WristRight];
            this.pHandTip = armJointPoints[JointType.HandTipRight];
            this.pTouch = pTouch;
            this.pHand = armJointPoints[JointType.HandRight];

            this.depthToColorSpaceMapper = new ColorSpacePoint[depthDataSource.Length];
            this.colorToDepthSpaceMapper = new DepthSpacePoint[colorSensorBufferWidth * colorSensorBufferHeight];

            this.helper = Helper.getInstance();
            this.userTransparency = userTransparency;
        }

        public unsafe void processImage(byte[] imageBufferHD)
        {
            coordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);
            coordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorToDepthSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferHD = imageBufferHD)
            fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
            fixed (DepthSpacePoint* ptrColorToDepthSpaceMapper = colorToDepthSpaceMapper)
            {
                this.ptrBodyIndexSensorBuffer = ptrBodyIndexSensorBuffer;
                this.ptrColorSensorBuffer = ptrColorSensorBuffer;
                this.ptrImageBufferHD = ptrImageBufferHD;

                this.ptrDepthToColorSpaceMapper = ptrDepthToColorSpaceMapper;
                this.ptrColorToDepthSpaceMapper = ptrColorToDepthSpaceMapper;
                
                this.ptrImageBufferHDInt = (uint*)ptrImageBufferHD;
                this.ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                this.drawBody();


                //==draw second, translated hand

                //need body joints in color space
                int idxElbowDepthSpace = (int)(bodyIndexSensorBufferWidth * (int)(pElbow.Y + 0.5) + pElbow.X + 0.5);
                this.xElbowColorSpace = (int)(ptrDepthToColorSpaceMapper[idxElbowDepthSpace].X + 0.5);
                this.yElbowColorSpace = (int)(ptrDepthToColorSpaceMapper[idxElbowDepthSpace].Y + 0.5);

                int idxWristDepthSpace = (int)(bodyIndexSensorBufferWidth * (int)(pWrist.Y + 0.5) + pWrist.X + 0.5);
                this.xWristColorSpace = (int)(ptrDepthToColorSpaceMapper[idxWristDepthSpace].X + 0.5);
                this.yWristColorSpace = (int)(ptrDepthToColorSpaceMapper[idxWristDepthSpace].Y + 0.5);

                int idxHandTipDepthSpace = (int)(bodyIndexSensorBufferWidth * (int)(pHandTip.Y + 0.5) + pHandTip.X + 0.5);
                this.xHandTipColorSpace = (int)(ptrDepthToColorSpaceMapper[idxHandTipDepthSpace].X + 0.5);
                this.yHandTipColorSpace = (int)(ptrDepthToColorSpaceMapper[idxHandTipDepthSpace].Y + 0.5);

                //start point for floodfill 
                int idxHandDepthSpace = ((int)(pHand.Y + 0.5)) * bodyIndexSensorBufferWidth + ((int)(pHand.X + 0.5));
                this.xHandColorSpace = (int)(ptrDepthToColorSpaceMapper[idxHandDepthSpace].X + 0.5);
                this.yHandColorSpace = (int)(ptrDepthToColorSpaceMapper[idxHandDepthSpace].Y + 0.5);

                //upper boundary: normal vector of wrist
                this.vElbowToWristOrig = new Vector((xWristColorSpace - xElbowColorSpace), (yWristColorSpace - yElbowColorSpace));

                this.xTranslationOffset = (int)(pTouch.X - xHandTipColorSpace + 0.5);
                this.yTranslationOffset = (int)(pTouch.Y - yHandTipColorSpace + 0.5);

                int stackSize = 10000000;
                Thread thread = new Thread(() => translateHand(xHandColorSpace, yHandColorSpace), stackSize);
                thread.Start();
                thread.Join();
                //this.translateHand(xHandColorSpace, yHandColorSpace);
                //this.transform_HD(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferHDInt, ptrDepthToColorSpaceMapper, ptrColorToDepthSpaceMapper, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

            } //end fixed
        }

        private unsafe void drawBody()
        {
            //with the cast to int, step size is 4 bytes
            uint* ptrImgBufferPixelInt = null;
            uint* ptrColorSensorBufferPixelInt = null;

            //save computing power by incrementing x, y without division/modulo
            int xColorSpace = 0;
            int yColorSpace = 0;

            //==draw whole body without manipulation
            int lengthColorBuffer = colorToDepthSpaceMapper.Length; //colorSensorBufferHeight * colorSensorBufferWidth;
            //after the loop, only color pixels with a body index value that can be mapped to a depth value will remain in the buffer
            for (int idxColorSpace = 0; idxColorSpace < lengthColorBuffer; idxColorSpace++)
            {
                //where the color img cannot be mapped to the depth image, there are infinity values
                if (!Single.IsInfinity(ptrColorToDepthSpaceMapper[idxColorSpace].Y) && !Single.IsInfinity(ptrColorToDepthSpaceMapper[idxColorSpace].X))
                {
                    int idxDepthSpace = (int)(bodyIndexSensorBufferWidth * ptrColorToDepthSpaceMapper[idxColorSpace].Y + ptrColorToDepthSpaceMapper[idxColorSpace].X); //2D to 1D

                    // bodyIndex can be 0, 1, 2, 3, 4, or 5
                    if (ptrBodyIndexSensorBuffer[idxDepthSpace] != 0xff)
                    {
                        ptrImgBufferPixelInt = ptrImageBufferHDInt + idxColorSpace;

                        //with the cast to int, 4 bytes are copied
                        *ptrImgBufferPixelInt = ptrColorSensorBufferInt[idxColorSpace];
                        // overwrite the alpha value
                        *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
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

        // important: always stop if an error occurs (out of bounds, pixel already visited)
        // NOTE stack too small for recursive impl
        private unsafe void translateHand(int xStart, int yStart)
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
            int vPointWristX = xStart - xWristColorSpace;
            int vPointWristY = yStart - yWristColorSpace;
            int sig = ((int)(vElbowToWristOrig.X + 0.5)) * vPointWristX + ((int)(vElbowToWristOrig.Y + 0.5)) * vPointWristY;
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

            float xDepthPixel = ptrColorToDepthSpaceMapper[yStart * colorSensorBufferWidth + xStart].X;
            float yDepthPixel = ptrColorToDepthSpaceMapper[yStart * colorSensorBufferWidth + xStart].Y;
            int idxDepthPixel = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
            if (Single.IsInfinity(xDepthPixel) || Single.IsInfinity(yDepthPixel) || (ptrBodyIndexSensorBuffer[idxDepthPixel] == 0xff))
            {
                return;
            }
            else
            {
                int xTranslatedColorSpace = xStart + xTranslationOffset;
                int yTranslatedColorSpace = yStart + yTranslationOffset;
                if ((yTranslatedColorSpace < colorSensorBufferHeight) && (xTranslatedColorSpace < colorSensorBufferWidth) && (yTranslatedColorSpace >= 0) && (xTranslatedColorSpace >= 0))
                {
                    lock (thisLock)
                    {
                        // point to current pixel in image buffer
                        uint* ptrImgBufferPixelInt = ptrImageBufferHDInt + (yTranslatedColorSpace * colorSensorBufferWidth + xTranslatedColorSpace);
                    
                        // assign color value (4 bytes)
                        *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                        // overwrite the alpha value (last byte)
                        *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency * Constants.HAND_TRANSLATED_ALPHAFACTOR);

                        //do not visit same pixel twice
                        *(ptrColorSensorBufferInt + (yStart * colorSensorBufferWidth + xStart)) = 0xFF000000;
                    }
                }
                else
                {
                    return;
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

        private unsafe void transform_HD(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferInt, ColorSpacePoint* ptrDepthToColorSpaceMapper, DepthSpacePoint* ptrColorToDepthSpaceMapper, float xWrist, float yWrist, float xHandTip, float yHandTip, float xTouch, float yTouch)
        {
            //with the cast to int, step size is 4 bytes
            uint* ptrImgBufferPixelInt = null;

            float xHandTipColorSpace = ptrDepthToColorSpaceMapper[(int)(yHandTip + 0.5) * bodyIndexSensorBufferWidth + (int)(xHandTip + 0.5)].X;
            float yHandTipColorSpace = ptrDepthToColorSpaceMapper[(int)(yHandTip + 0.5) * bodyIndexSensorBufferWidth + (int)(xHandTip + 0.5)].Y;
            float xWristColorSpace = ptrDepthToColorSpaceMapper[(int)(yWrist + 0.5) * bodyIndexSensorBufferWidth + (int)(xWrist + 0.5)].X;

            //TODO check boundaries
            float xOffsetColorSpace = xTouch - xHandTipColorSpace;
            float yOffsetColorSpace = yTouch - yHandTipColorSpace;

            //save computing power by incrementing x, y without division/modulo
            int xColorSpace = 0;
            int yColorSpace = 0;

            int lengthColorBuffer = colorSensorBufferHeight * colorSensorBufferWidth;
            //after the loop, only color pixels with a body index value that can be mapped to a depth value will remain in the buffer
            for (int idxColorSpace = 0; idxColorSpace < lengthColorBuffer; idxColorSpace++)
            {
                //where the color img cannot be mapped to the depth image, there are infinity values
                if (!Single.IsInfinity(ptrColorToDepthSpaceMapper[idxColorSpace].Y) && !Single.IsInfinity(ptrColorToDepthSpaceMapper[idxColorSpace].X))
                {
                    int idxDepthSpace = (int)(bodyIndexSensorBufferWidth * ptrColorToDepthSpaceMapper[idxColorSpace].Y + ptrColorToDepthSpaceMapper[idxColorSpace].X); //2D to 1D

                    // bodyIndex can be 0, 1, 2, 3, 4, or 5
                    if (ptrBodyIndexSensorBuffer[idxDepthSpace] != 0xff)
                    {
                        ptrImgBufferPixelInt = ptrImageBufferInt + idxColorSpace;

                        //with the cast to int, 4 bytes are copied
                        *ptrImgBufferPixelInt = ptrColorSensorBufferInt[idxColorSpace];
                        // overwrite the alpha value
                        *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;

                        #region --- Region of hand
                        if (xColorSpace >= xWristColorSpace)
                        {
                            float xTranslatedColorSpace = xColorSpace + xOffsetColorSpace;
                            float yTranslatedColorSpace = yColorSpace + yOffsetColorSpace;

                            if ((yTranslatedColorSpace < colorSensorBufferHeight) && (xTranslatedColorSpace < colorSensorBufferWidth) &&
                                (yTranslatedColorSpace >= 0) && (xTranslatedColorSpace >= 0))
                            {
                                ptrImgBufferPixelInt = ptrImageBufferInt + ((int)(yTranslatedColorSpace + 0.5) * colorSensorBufferWidth + (int)(xTranslatedColorSpace + 0.5));

                                //with the cast to int, 4 bytes are copied
                                *ptrImgBufferPixelInt = ptrColorSensorBufferInt[idxColorSpace];
                                // overwrite the alpha value
                                *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                            }
                        }
                        #endregion
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
    }
}
