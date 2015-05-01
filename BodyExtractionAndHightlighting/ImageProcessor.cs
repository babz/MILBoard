using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;

namespace BodyExtractionAndHightlighting
{
    public class ImageProcessor
    {
        protected int bodyIndexBufferWidth, bodyIndexBufferHeight, colorBufferWidth, colorBufferHeight;

        protected byte[] bodyIndexSensorBuffer, colorSensorBuffer;

        protected KinectSensor sensor;
        protected ushort[] depthDataSource;

        protected ColorSpacePoint[] depthToColorSpaceMapper;
        protected DepthSpacePoint[] colorToDepthSpaceMapper;

        protected byte userTransparency;
        protected Helper helper;

        public ImageProcessor(int bodyIndexSensorBufferWidth, int bodyIndexSensorBufferHeight, int colorSensorBufferWidth, int colorSensorBufferHeight, byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, KinectSensor sensor, ushort[] depthDataSource)
        {
            this.bodyIndexBufferWidth = bodyIndexSensorBufferWidth;
            this.bodyIndexBufferHeight = bodyIndexSensorBufferHeight;
            this.colorBufferWidth = colorSensorBufferWidth;
            this.colorBufferHeight = colorSensorBufferHeight;

            this.bodyIndexSensorBuffer = bodyIndexSensorBuffer;
            this.colorSensorBuffer = colorSensorBuffer;
            
            this.sensor = sensor;
            this.depthDataSource = depthDataSource;

            this.depthToColorSpaceMapper = null;
            this.colorToDepthSpaceMapper = null;

            this.helper = new Helper();
        }

        public unsafe void processImageSimple_LowRes(byte[] imageBufferLowRes)
        {
            sensor.CoordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferLowRes = imageBufferLowRes)
            fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
            {
                //points to start of ptrImageBufferLowRes
                uint* ptrImageBufferInt = (uint*)ptrImageBufferLowRes;
                uint* ptrImgBufferPixelInt = null;
                uint* ptrColorSensorBufferPixelInt = null;
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                int length = depthToColorSpaceMapper.Length;
                for (int i = 0; i < length; i++)
                {
                    // bodyIndex can be 0, 1, 2, 3, 4, or 5
                    if (ptrBodyIndexSensorBuffer[i] != 0xff)
                    {
                        int colorPointX = (int)(ptrDepthToColorSpaceMapper[i].X + 0.5);
                        int colorPointY = (int)(ptrDepthToColorSpaceMapper[i].Y + 0.5);

                        //check boundaries
                        if ((colorPointY >= this.colorBufferHeight) || (colorPointX >= this.colorBufferWidth) ||
                        (colorPointY < 0) && (colorPointX < 0))
                        {
                            continue;
                        }

                        //point to current pixel in target imgBuffer
                        ptrImgBufferPixelInt = ptrImageBufferInt + i;

                        // corresponding pixel in the 1080p image
                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * this.colorBufferWidth + colorPointX);
                        // assign color value (4 bytes); dereferencing + assigning writes into imgBuffer
                        *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                        // overwrite the alpha value
                        *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                    }
                } //end for
            }
        }

        public unsafe void processImageSimple_HD(byte[] imageBufferHD)
        {
            sensor.CoordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorToDepthSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferHD = imageBufferHD)
            fixed (DepthSpacePoint* ptrColorToDepthSpaceMapper = colorToDepthSpaceMapper)
            {
                //with the cast to int, step size is 4 bytes
                uint* ptrImageBufferInt = (uint*)ptrImageBufferHD;
                uint* ptrImgBufferPixelInt = null;

                int lengthColorBuffer = colorBufferHeight * colorBufferWidth;
                //after the loop, only color pixels with a body index value that can be mapped to a depth value will remain in the buffer
                for (int i = 0; i < lengthColorBuffer; i++)
                {
                    //where the color img cannot be mapped to the depth image, there are infinity values
                    if (Single.IsInfinity(ptrColorToDepthSpaceMapper[i].Y) || Single.IsInfinity(ptrColorToDepthSpaceMapper[i].X))
                    {
                        continue;
                    }

                    int idx = (int)(bodyIndexBufferWidth * ptrColorToDepthSpaceMapper[i].Y + ptrColorToDepthSpaceMapper[i].X); //2D to 1D

                    // bodyIndex can be 0, 1, 2, 3, 4, or 5
                    if (ptrBodyIndexSensorBuffer[idx] != 0xff)
                    {
                        ptrImgBufferPixelInt = ptrImageBufferInt + i;

                        //with the cast to int, 4 bytes are copied
                        *ptrImgBufferPixelInt = ((uint*)ptrColorSensorBuffer)[i];
                        // overwrite the alpha value
                        *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                    }
                    
                } // for loop
            }
        }


        #region Private Methods

        /*
         * @example  int indexElbow = yElbow * fdDepth.Width + xElbow;
         * */
        private int GetLowResImageIndexOfBodyJoint(int xBodyJoint, int yBodyJoint)
        {
            return yBodyJoint * bodyIndexBufferWidth + xBodyJoint;
        }

        #endregion


        #region Properties

        public byte PropUserTransparency
        {
            get
            {
                return this.userTransparency;
            }
            set
            {
                this.userTransparency = value;
            }
        }

        #endregion
    }
}
