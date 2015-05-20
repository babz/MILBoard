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

        Point pWrist, pHandTip, pTouch, pHand;
        private const double HAND_TRANSLATED_ALPHAFACTOR = 0.75;

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
                float xWrist = (float)pWrist.X;
                float yWrist = (float)pWrist.Y;
                float xHandTip = (float)pHandTip.X;
                float yHandTip = (float)pHandTip.Y;
                float xTouch = (float)pTouch.X;
                float yTouch = (float)pTouch.Y;
                float xHand = (float)pHand.X;
                float yHand = (float)pHand.Y;

                uint* ptrImageBufferHDInt = (uint*)ptrImageBufferHD;
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                this.transform_floodfill(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferHDInt, ptrDepthToColorSpaceMapper, ptrColorToDepthSpaceMapper, xWrist, yWrist, xHand, yHand, xHandTip, yHandTip, xTouch, yTouch);
                //this.transform_HD(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferHDInt, ptrDepthToColorSpaceMapper, ptrColorToDepthSpaceMapper, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

            } //end fixed
        }

        private unsafe void transform_floodfill(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferInt, ColorSpacePoint* ptrDepthToColorSpaceMapper, DepthSpacePoint* ptrColorToDepthSpaceMapper, float xWristF, float yWristF, float xHandF, float yHandF, float xHandTipF, float yHandTipF, float xTouchF, float yTouchF)
        {
            //with the cast to int, step size is 4 bytes
            uint* ptrImgBufferPixelInt = null;
            uint* ptrColorSensorBufferPixelInt = null;

            float xHandTipColorSpace = ptrDepthToColorSpaceMapper[(int)(yHandTipF + 0.5) * bodyIndexSensorBufferWidth + (int)(xHandTipF + 0.5)].X;
            float yHandTipColorSpace = ptrDepthToColorSpaceMapper[(int)(yHandTipF + 0.5) * bodyIndexSensorBufferWidth + (int)(xHandTipF + 0.5)].Y;
            float xWristColorSpace = ptrDepthToColorSpaceMapper[(int)(yWristF + 0.5) * bodyIndexSensorBufferWidth + (int)(xWristF + 0.5)].X;

            //save computing power by incrementing x, y without division/modulo
            int xColorSpace = 0;
            int yColorSpace = 0;

            //==draw whole body without manipulation
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
                    }
                }

                //increment counter
                if (++xColorSpace == colorSensorBufferWidth)
                {
                    xColorSpace = 0;
                    yColorSpace++;
                }
            } // for loop

            //==draw second, translated hand
            int xOffset = (int)(xTouchF - xHandTipColorSpace + 0.5);
            int yOffset = (int)(yTouchF - yHandTipColorSpace + 0.5);
            int xHandColorSpace = (int)(ptrDepthToColorSpaceMapper[(int)(yHandF + 0.5) * bodyIndexSensorBufferWidth + (int)(xHandTipF + 0.5)].X + 0.5);
            int yHandColorSpace = (int)(ptrDepthToColorSpaceMapper[(int)(yHandF + 0.5) * bodyIndexSensorBufferWidth + (int)(xHandTipF + 0.5)].Y + 0.5);
            int xWrist = (int)(xWristColorSpace + 0.5);
            this.floodfill(xHandColorSpace, yHandColorSpace, xWrist, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
        }

        private unsafe void floodfill(int xStart, int yStart, int xEnd, int xOffset, int yOffset, byte* ptrBodyIndexSensorBuffer, uint* ptrImageBufferInt, uint* ptrColorSensorBufferInt, DepthSpacePoint* ptrColorToDepthSpaceMapper)
        {
            //xEnd is left outer boundary
            if ((xStart < xEnd) || (xStart >= colorSensorBufferWidth) || (xStart < 0) || (yStart >= colorSensorBufferHeight) || (yStart < 0))
            {
                return;
            }

            int idxCurrColorPixel = yStart * colorSensorBufferWidth + xStart;
            if (idxCurrColorPixel >= colorToDepthSpaceMapper.Length)
                return; 

            float xDepthPixel = ptrColorToDepthSpaceMapper[idxCurrColorPixel].X;
            float yDepthPixel = ptrColorToDepthSpaceMapper[idxCurrColorPixel].Y;
            if (Single.IsInfinity(xDepthPixel) || Single.IsInfinity(yDepthPixel) || ptrBodyIndexSensorBuffer[(int)((int)(yDepthPixel + 0.5) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5)] == 0xff)
            {
                return;
            }
            else
            {
                ptrBodyIndexSensorBuffer[(int)((int)(yDepthPixel + 0.5) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5)] = 0xff; //do not visit same pixel twice
                int xTranslatedColorSpace = xStart + xOffset;
                int yTranslatedColorSpace = yStart + yOffset;
                if ((yTranslatedColorSpace < colorSensorBufferHeight) && (xTranslatedColorSpace < colorSensorBufferWidth) &&
                            (yTranslatedColorSpace >= 0) && (xTranslatedColorSpace >= 0) && (yStart < colorSensorBufferHeight) && (xStart < colorSensorBufferWidth) &&
                            (yStart >= 0) && (xStart >= 0))
                {
                    // point to current pixel in image buffer
                    uint* ptrImgBufferPixelInt = ptrImageBufferInt + (yTranslatedColorSpace * colorSensorBufferWidth + xTranslatedColorSpace);
                    
                    uint* ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxCurrColorPixel;
                    //TODO test!!!
                    //*ptrImgBufferPixelInt = 0x00;
                    // assign color value (4 bytes)
                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency * HAND_TRANSLATED_ALPHAFACTOR);
                }
            }

            //TODO sometimes ends up in a stack overflow with 8 neighbours
            //8-way neighbourhood to visit all pixels of hand (can have background pixel btw fingers)
            this.floodfill((xStart + 1), yStart, xEnd, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
            this.floodfill((xStart - 1), yStart, xEnd, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
            this.floodfill(xStart, (yStart + 1), xEnd, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
            this.floodfill(xStart, (yStart - 1), xEnd, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
            this.floodfill((xStart - 1), (yStart - 1), xEnd, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
            this.floodfill((xStart - 1), (yStart + 1), xEnd, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
            this.floodfill((xStart + 1), (yStart - 1), xEnd, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
            this.floodfill((xStart + 1), (yStart + 1), xEnd, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrColorToDepthSpaceMapper);
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
