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

        Point pWrist, pHandTip, pTouch, pHand;
        private const double HAND_TRANSLATED_ALPHAFACTOR = 0.75;

        unsafe protected ColorSpacePoint[] depthToColorSpaceMapper = null;
        unsafe protected DepthSpacePoint[] colorToDepthSpaceMapper = null;

        private Helper helper;
        private byte userTransparency;

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
                //TODO open new thread for that?
                ////NOTE specific for each manager!!
                //int xStart = (int)(pHandTip.X + 0.5);
                //int yStart = (int)(pHandTip.Y + 0.5);
                //int xEnd = (int)(pWrist.X + 0.5);
                //int yEnd = (int)(pWrist.Y + 0.5);
                //helper.floodfill(xStart, yStart, xEnd, yEnd);

                float xWrist = (float)pWrist.X;
                float yWrist = (float)pWrist.Y;
                float xHandTip = (float)pHandTip.X;
                float yHandTip = (float)pHandTip.Y;
                float xTouch = (float)pTouch.X;
                float yTouch = (float)pTouch.Y;
                float xHand = (float)pHand.X;
                float yHand = (float)pHand.Y;

                uint* ptrImageBufferInt = (uint*)ptrImageBufferLowRes;
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                this.transform_floodfill(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferInt, ptrDepthToColorSpaceMapper, xWrist, yWrist, xHand, yHand, xHandTip, yHandTip, xTouch, yTouch);
                //this.transform_LowRes(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferInt, ptrDepthToColorSpaceMapper, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

            } //end fixed
        }

        private unsafe void transform_floodfill(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferInt, ColorSpacePoint* ptrDepthToColorSpaceMapper, float xWristF, float yWristF, float xHandF, float yHandF, float xHandTipF, float yHandTipF, float xTouchF, float yTouchF)
        {
            uint* ptrImgBufferPixelInt = null; // this is where we want to write the pixel
            uint* ptrColorSensorBufferPixelInt = null;

            //save computing power by incrementing x, y without division/modulo
            int xDepthSpace = 0;
            int yDepthSpace = 0;

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
                        //if (xDepthSpace >= xWrist)
                        //{
                        //    float xTranslatedDepthSpace = xDepthSpace + xOffset;
                        //    float yTranslatedDepthSpace = yDepthSpace + yOffset;

                        //    if ((yTranslatedDepthSpace < bodyIndexSensorBufferHeight) && (xTranslatedDepthSpace < bodyIndexSensorBufferWidth) &&
                        //        (yTranslatedDepthSpace >= 0) && (xTranslatedDepthSpace >= 0))
                        //    {
                        //        ptrImgBufferPixelInt = ptrImageBufferInt + ((int)(yTranslatedDepthSpace + 0.5) * bodyIndexSensorBufferWidth + (int)(xTranslatedDepthSpace + 0.5));

                        //        // assign color value (4 bytes)
                        //        *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                        //        // overwrite the alpha value (last byte)
                        //        *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency * HAND_TRANSLATED_ALPHAFACTOR);
                        //    }
                        //}
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

            int xOffset = (int)(xTouchF - xHandTipF + 0.5);
            int yOffset = (int)(yTouchF - yHandTipF + 0.5);
            int xHand = (int)(xHandF + 0.5);
            int yHand = (int)(yHandF + 0.5);
            int xWrist = (int)(xWristF + 0.5);
            this.floodfill(xHand, yHand, xWrist, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
        }

        private unsafe void floodfill(int xStart, int yStart, int xEnd, int xOffset, int yOffset, byte* ptrBodyIndexSensorBuffer, uint* ptrImageBufferInt, uint* ptrColorSensorBufferInt, ColorSpacePoint* ptrDepthToColorSpaceMapper)
        {
            //xEnd is left outer boundary
            if ((xStart < xEnd) || (xStart >= bodyIndexSensorBufferWidth) || (xStart < 0) || (yStart >= bodyIndexSensorBufferHeight) || (yStart < 0))
            {
                return;
            }

            int depthLookup = yStart * bodyIndexSensorBufferWidth + xStart;
            if (ptrBodyIndexSensorBuffer[depthLookup] == 0xff)
            {
                return;
            }
            else
            {
                int xTranslatedDepthSpace = xStart + xOffset;
                int yTranslatedDepthSpace = yStart + yOffset;
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
                    *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency * HAND_TRANSLATED_ALPHAFACTOR);
                }
                else
                {
                    return;
                }
            }
            
            this.floodfill((xStart + 1), yStart, xEnd, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
            this.floodfill((xStart - 1), yStart, xEnd, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
            this.floodfill(xStart, (yStart + 1), xEnd, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
            this.floodfill(xStart, (yStart - 1), xEnd, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
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
                                *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency * HAND_TRANSLATED_ALPHAFACTOR);
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
