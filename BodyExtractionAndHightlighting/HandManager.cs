using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;

namespace BodyExtractionAndHightlighting
{
    public class HandManager : ImageProcessor
    {
        Point pWrist, pHandTip, pTouch;

        public HandManager(int bodyIndexSensorBufferWidth, int bodyIndexSensorBufferHeight, int colorSensorBufferWidth, int colorSensorBufferHeight, byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, KinectSensor sensor, ushort[] depthDataSource, Dictionary<JointType, Point> armJointPoints, Point pTouch)
            : base(bodyIndexSensorBufferWidth, bodyIndexSensorBufferHeight, colorSensorBufferWidth, colorSensorBufferHeight, bodyIndexSensorBuffer, colorSensorBuffer, sensor, depthDataSource)
        {
            this.pWrist = armJointPoints[JointType.WristRight];
            this.pHandTip = armJointPoints[JointType.HandTipRight];
            this.pTouch = pTouch;
        }


        public unsafe void processImageLowRes(byte[] imageBufferLowRes)
        {
            sensor.CoordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferLowRes = imageBufferLowRes)
            fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
            {
                float xWrist = (float)pWrist.X;
                float yWrist = (float)pWrist.Y;
                float xHandTip = (float)pHandTip.X;
                float yHandTip = (float)pHandTip.Y;
                float xTouch = (float)pTouch.X;
                float yTouch = (float)pTouch.Y;

                uint* ptrImageBufferInt = (uint*)ptrImageBufferLowRes;
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                this.transform_LowRes(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferInt, ptrDepthToColorSpaceMapper, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

            } //end fixed
        }

        public unsafe void processImageHD(byte[] imageBufferHD)
        {
            sensor.CoordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);
            sensor.CoordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorToDepthSpaceMapper);

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

                uint* ptrImageBufferHDInt = (uint*)ptrImageBufferHD;
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                this.transform_HD(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferHDInt, ptrDepthToColorSpaceMapper, ptrColorToDepthSpaceMapper, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

            } //end fixed
        }


        #region PRIVATE

        private unsafe void transform_LowRes(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferInt, ColorSpacePoint* ptrDepthToColorSpaceMapper, float xWrist, float yWrist, float xHandTip, float yHandTip, float xTouch, float yTouch)
        {
            uint* ptrImgBufferPixelInt = null; // this is where we want to write the pixel
            uint* ptrColorSensorBufferPixelInt = null;

            //save computing power by incrementing x, y without division/modulo
            int xDepthSpace = 0;
            int yDepthSpace = 0;

            float xOffset = xTouch - xWrist;
            float yOffset = yTouch - yWrist;
            
            int depthSpaceSize = bodyIndexBufferHeight * bodyIndexBufferWidth;
            for (int idxDepthSpace = 0; idxDepthSpace < depthSpaceSize; idxDepthSpace++)
            {
                //ptrColorSensorBufferPixelInt = null;
                ptrImgBufferPixelInt = null;
                bool isColorPixelInValidRange = false;

                if (ptrBodyIndexSensorBuffer[idxDepthSpace] != 0xff)
                {
                    int colorPointX = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].X + 0.5);
                    int colorPointY = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].Y + 0.5);

                    if ((colorPointY < colorBufferHeight) && (colorPointX < colorBufferWidth) &&
                            (colorPointY >= 0) && (colorPointX >= 0))
                    {
                        isColorPixelInValidRange = true;
                    }

                    if (isColorPixelInValidRange)
                    {
                        // point to current pixel in image buffer
                        ptrImgBufferPixelInt = ptrImageBufferInt + idxDepthSpace;

                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorBufferWidth + colorPointX);
                        // assign color value (4 bytes)
                        *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                        // overwrite the alpha value (last byte)
                        *(((byte*)ptrImgBufferPixelInt) + 3) = base.userTransparency;


                        #region --- Region of hand
                        if (xDepthSpace >= xWrist)
                        {
                            float xTranslatedDepthSpace = xDepthSpace + xOffset;
                            float yTranslatedDepthSpace = yDepthSpace + yOffset;

                            if ((yTranslatedDepthSpace < bodyIndexBufferHeight) && (xTranslatedDepthSpace < bodyIndexBufferWidth) &&
                                (yTranslatedDepthSpace >= 0) && (xTranslatedDepthSpace >= 0))
                            {
                                ptrImgBufferPixelInt = ptrImageBufferInt + ((int)(yTranslatedDepthSpace + 0.5) * bodyIndexBufferWidth + (int)(xTranslatedDepthSpace + 0.5));

                                ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorBufferWidth + colorPointX);
                                // assign color value (4 bytes)
                                *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                                // overwrite the alpha value (last byte)
                                *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(base.userTransparency * 0.75);
                            }
                        }
                        #endregion // hand
                    }
                } //if body

                //increment counter
                if (++xDepthSpace == bodyIndexBufferWidth)
                {
                    xDepthSpace = 0;
                    yDepthSpace++;
                }
            } //for loop
        }

        private unsafe void transform_HD(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferHDInt, ColorSpacePoint* ptrDepthToColorSpaceMapper, DepthSpacePoint* ptrColorToDepthSpaceMapper, float xWrist, float yWrist, float xHandTip, float yHandTip, float xTouch, float yTouch)
        {
            float xWristColorSpace = ptrDepthToColorSpaceMapper[(int)(yWrist + 0.5) * bodyIndexBufferWidth + (int)(xWrist + 0.5)].X;
            float yWristColorSpace = ptrDepthToColorSpaceMapper[(int)(yWrist + 0.5) * bodyIndexBufferWidth + (int)(xWrist + 0.5)].Y;

            bool isWristOutOfBound = false;
            //where the color img cannot be mapped to the depth image, there are infinity values
            if (Single.IsInfinity(xWristColorSpace) || Single.IsInfinity(yWristColorSpace))
            {
                isWristOutOfBound = true;
            }

            uint* ptrImgBufferHDPixelInt = null; // this is where we want to write the pixel
            uint* ptrColorSensorBufferPixelInt = null;

            float xTouchColorSpace = ptrDepthToColorSpaceMapper[(int)(yTouch + 0.5) * bodyIndexBufferWidth + (int)(xTouch + 0.5)].X;
            float yTouchColorSpace = ptrDepthToColorSpaceMapper[(int)(yTouch + 0.5) * bodyIndexBufferWidth + (int)(xTouch + 0.5)].Y;

            //Vector areaOffset = new Vector((xWrist - xElbow), (yWrist - yElbow));
            //int handOffset = (int)areaOffset.Length / 3;
            //int handTipBoundaryX = (int)(xHandTip + 0.5);// +handOffset;
            //int handTipBoundaryY = (int)(yHandTip + 0.5);// -handOffset;

            //save computing power by incrementing x, y without division/modulo
            int xColorSpace = 0;
            int yColorSpace = 0;

            int colorSpaceSize = colorBufferHeight * colorBufferWidth;
            for (int idxColorSpace = 0; idxColorSpace < colorSpaceSize; idxColorSpace++)
            {
                ptrColorSensorBufferPixelInt = null;

                float xDepthSpace = ptrColorToDepthSpaceMapper[idxColorSpace].X;
                float yDepthSpace = ptrColorToDepthSpaceMapper[idxColorSpace].Y;
                int idxDepthSpace = (int)(yDepthSpace + 0.5) * bodyIndexBufferWidth + (int)(xDepthSpace + 0.5);

                if (!Single.IsInfinity(xDepthSpace) && !Single.IsInfinity(yDepthSpace))
                {
                    if (ptrBodyIndexSensorBuffer[idxDepthSpace] != 0xff)
                    {
                        // right arm
                        if (!isWristOutOfBound && (xColorSpace >= xWristColorSpace))
                        {
                            //TODO convert touch into color space
                            // compute translation (in target image buffer)
                            //int xTranslatedColorSpace = (int)(xWristColorSpace + xTouchColorSpace);
                            //int yTranslatedColorSpace = (int)(yWristColorSpace + yTouchColorSpace);
                            int xTranslatedColorSpace = (int)(xTouchColorSpace);
                            int yTranslatedColorSpace = (int)(yTouchColorSpace);

                            if ((yTranslatedColorSpace < colorBufferHeight) && (xTranslatedColorSpace < colorBufferWidth) &&
                                (yTranslatedColorSpace >= 0) && (xTranslatedColorSpace >= 0))
                            {
                                ptrImgBufferHDPixelInt = ptrImageBufferHDInt + (yTranslatedColorSpace * colorBufferWidth + xTranslatedColorSpace);
                            }
                        }
                        else //rest of body
                        {
                            //point to current pixel in target imgBuffer
                            ptrImgBufferHDPixelInt = ptrImageBufferHDInt + idxColorSpace;
                        }
                    } //if body
                } //if idxDepth not infinity

                if (ptrImgBufferHDPixelInt != null)
                {
                    //check boundaries
                    if ((yColorSpace < colorBufferHeight) && (xColorSpace < colorBufferWidth) &&
                            (yColorSpace >= 0) && (xColorSpace >= 0))
                    {
                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (yColorSpace * colorBufferWidth + xColorSpace);
                        // assign color value (4 bytes)
                        *ptrImgBufferHDPixelInt = *ptrColorSensorBufferPixelInt;
                        // overwrite the alpha value (last byte)
                        *(((byte*)ptrImgBufferHDPixelInt) + 3) = base.userTransparency;
                    }
                }

                //increment counter
                if (++xColorSpace == colorBufferWidth)
                {
                    xColorSpace = 0;
                    yColorSpace++;
                }
            } //for loop
        }

        #endregion
    }
}
