using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;

namespace BodyExtractionAndHightlighting
{
    public class HandManagerHD : IHandManager
    {
        private int bodyIndexSensorBufferWidth, bodyIndexSensorBufferHeight, colorSensorBufferWidth, colorSensorBufferHeight;
        private byte[] bodyIndexSensorBuffer, colorSensorBuffer;

        private CoordinateMapper coordinateMapper;
        private ushort[] depthDataSource;

        private unsafe Point pWrist, pHandTip, pTouch, pHand, pElbow;

        unsafe protected ColorSpacePoint[] depthToColorSpaceMapper = null;
        unsafe protected DepthSpacePoint[] colorToDepthSpaceMapper = null;

        private Helper helper;
        private byte userTransparency;

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
                uint* ptrImageBufferHDInt = (uint*)ptrImageBufferHD;
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                this.drawBody(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferHDInt, ptrColorToDepthSpaceMapper);


                //==draw second, translated hand

                float xHandTipColorSpace = ptrDepthToColorSpaceMapper[((int)(pHandTip.Y + 0.5)) * bodyIndexSensorBufferWidth + ((int)(pHandTip.X + 0.5))].X;
                float yHandTipColorSpace = ptrDepthToColorSpaceMapper[((int)(pHandTip.Y + 0.5)) * bodyIndexSensorBufferWidth + ((int)(pHandTip.X + 0.5))].Y;
                int xOffset = (int)(pTouch.X - xHandTipColorSpace + 0.5);
                int yOffset = (int)(pTouch.Y - yHandTipColorSpace + 0.5);

                //start point for floodfill 
                int xHandColorSpace = (int)(ptrDepthToColorSpaceMapper[((int)(pHand.Y + 0.5)) * bodyIndexSensorBufferWidth + ((int)(pHand.X + 0.5))].X + 0.5);
                int yHandColorSpace = (int)(ptrDepthToColorSpaceMapper[((int)(pHand.Y + 0.5)) * bodyIndexSensorBufferWidth + ((int)(pHand.X + 0.5))].Y + 0.5);

                //termination condition: normal vector of elbow-wrist-vector 
                int xElbowColorSpace = (int)(ptrDepthToColorSpaceMapper[(int)(pElbow.Y + 0.5) * bodyIndexSensorBufferWidth + (int)(pElbow.X + 0.5)].X + 0.5);
                int yElbowColorSpace = (int)(ptrDepthToColorSpaceMapper[(int)(pElbow.Y + 0.5) * bodyIndexSensorBufferWidth + (int)(pElbow.X + 0.5)].Y + 0.5);
                int xWristColorSpace = (int)(ptrDepthToColorSpaceMapper[(int)(pWrist.Y + 0.5) * bodyIndexSensorBufferWidth + (int)(pWrist.X + 0.5)].X + 0.5);
                int yWristColorSpace = (int)(ptrDepthToColorSpaceMapper[(int)(pWrist.Y + 0.5) * bodyIndexSensorBufferWidth + (int)(pWrist.X + 0.5)].Y + 0.5);
                int vElbowWristX = xWristColorSpace - xElbowColorSpace;
                int vElbowWristY = yWristColorSpace - yElbowColorSpace;

                this.translateHand(xHandColorSpace, yHandColorSpace, vElbowWristX, vElbowWristY, xWristColorSpace, yWristColorSpace, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferHDInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
                //this.transform_HD(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferHDInt, ptrDepthToColorSpaceMapper, ptrColorToDepthSpaceMapper, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

            } //end fixed
        }

        private unsafe void drawBody(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferInt, DepthSpacePoint* ptrColorToDepthSpaceMapper)
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
                        ptrImgBufferPixelInt = ptrImageBufferInt + idxColorSpace;

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
        private unsafe void translateHand(int xStart, int yStart, int vElbowWristX, int vElbowWristY, int xWristColorSpace, int yWristColorSpace, int xOffset, int yOffset, byte* ptrBodyIndexSensorBuffer, uint* ptrImageBufferInt, uint* ptrColorSensorBufferInt, DepthSpacePoint* ptrColorToDepthSpaceMapper)
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
            int sig = vElbowWristX * vPointWristX + vElbowWristY * vPointWristY;
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

            float xDepthPixel = ptrColorToDepthSpaceMapper[idxCurrColorPixel].X;
            float yDepthPixel = ptrColorToDepthSpaceMapper[idxCurrColorPixel].Y;
            int idxDepthPixel = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
            if (Single.IsInfinity(xDepthPixel) || Single.IsInfinity(yDepthPixel) || (ptrBodyIndexSensorBuffer[idxDepthPixel] == 0xff))
            {
                return;
            }
            else
            {
                int xTranslatedColorSpace = xStart + xOffset;
                int yTranslatedColorSpace = yStart + yOffset;
                if ((yTranslatedColorSpace < colorSensorBufferHeight) && (xTranslatedColorSpace < colorSensorBufferWidth) &&
                            (yTranslatedColorSpace >= 0) && (xTranslatedColorSpace >= 0))
                {
                    // point to current pixel in image buffer
                    uint* ptrImgBufferPixelInt = ptrImageBufferInt + (yTranslatedColorSpace * colorSensorBufferWidth + xTranslatedColorSpace);
                    // assign color value (4 bytes)
                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency * HAND_TRANSLATED_ALPHAFACTOR);

                    //do not visit same pixel twice
                    *ptrColorSensorBufferPixelInt = 0xFF000000;
                }
                else
                {
                    return;
                }
            }

            //8-way neighbourhood to visit all pixels of hand (can have background pixel btw fingers)
            this.translateHand((xStart + 1), yStart, vElbowWristX, vElbowWristY, xWristColorSpace, yWristColorSpace, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
            this.translateHand((xStart - 1), yStart, vElbowWristX, vElbowWristY, xWristColorSpace, yWristColorSpace, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
            this.translateHand(xStart, (yStart + 1), vElbowWristX, vElbowWristY, xWristColorSpace, yWristColorSpace, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
            this.translateHand(xStart, (yStart - 1), vElbowWristX, vElbowWristY, xWristColorSpace, yWristColorSpace, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
            this.translateHand((xStart - 1), (yStart - 1), vElbowWristX, vElbowWristY, xWristColorSpace, yWristColorSpace, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
            this.translateHand((xStart - 1), (yStart + 1), vElbowWristX, vElbowWristY, xWristColorSpace, yWristColorSpace, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
            this.translateHand((xStart + 1), (yStart - 1), vElbowWristX, vElbowWristY, xWristColorSpace, yWristColorSpace, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
            this.translateHand((xStart + 1), (yStart + 1), vElbowWristX, vElbowWristY, xWristColorSpace, yWristColorSpace, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
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
