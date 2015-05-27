using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace BodyExtractionAndHightlighting
{
    public class BasicManagerHD : IBasicManager 
    {
        private int bodyIndexSensorBufferWidth, bodyIndexSensorBufferHeight, colorSensorBufferWidth, colorSensorBufferHeight;
        private byte[] bodyIndexSensorBuffer, colorSensorBuffer;

        private CoordinateMapper coordinateMapper;
        private ushort[] depthDataSource;

        unsafe protected DepthSpacePoint[] colorToDepthSpaceMapper = null;

        private byte userTransparency;

        public BasicManagerHD(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, byte userTransparency)
        {
            this.bodyIndexSensorBufferWidth = Constants.GetBodyIndexSensorBufferWidth();
            this.bodyIndexSensorBufferHeight = Constants.GetBodyIndexSensorBufferHeight();
            this.colorSensorBufferWidth = Constants.GetColorSensorBufferWidth();
            this.colorSensorBufferHeight = Constants.GetColorSensorBufferHeight();

            this.bodyIndexSensorBuffer = bodyIndexSensorBuffer;
            this.colorSensorBuffer = colorSensorBuffer;

            this.coordinateMapper = Constants.GetCoordinateMapper();
            this.colorToDepthSpaceMapper = new DepthSpacePoint[colorSensorBufferWidth * colorSensorBufferHeight];
            this.depthDataSource = depthDataSource;

            this.userTransparency = userTransparency;
        }

        public unsafe void processImage(byte[] imageBufferHD)
        {
            coordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorToDepthSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferHD = imageBufferHD)
            fixed (DepthSpacePoint* ptrColorToDepthSpaceMapper = colorToDepthSpaceMapper)
            {
                //with the cast to int, step size is 4 bytes
                uint* ptrImageBufferInt = (uint*)ptrImageBufferHD;
                uint* ptrImgBufferPixelInt = null;

                int lengthColorBuffer = colorToDepthSpaceMapper.Length; //colorSensorBufferHeight * colorSensorBufferWidth;
                //after the loop, only color pixels with a body index value that can be mapped to a depth value will remain in the buffer
                for (int i = 0; i < lengthColorBuffer; i++)
                {
                    //where the color img cannot be mapped to the depth image, there are infinity values
                    if (Single.IsInfinity(ptrColorToDepthSpaceMapper[i].Y) || Single.IsInfinity(ptrColorToDepthSpaceMapper[i].X))
                    {
                        continue;
                    }

                    int idx = (int)(bodyIndexSensorBufferWidth * ptrColorToDepthSpaceMapper[i].Y + ptrColorToDepthSpaceMapper[i].X); //2D to 1D

                    // bodyIndex can be 0, 1, 2, 3, 4, or 5
                    if (ptrBodyIndexSensorBuffer[idx] != 0xff)
                    {
                        ptrImgBufferPixelInt = ptrImageBufferInt + i;

                        //with the cast to int, 4 bytes are copied
                        *ptrImgBufferPixelInt = ((uint*)ptrColorSensorBuffer)[i];
                        // overwrite the alpha value
                        *(((byte*)ptrImgBufferPixelInt) + 3) = userTransparency;
                    }

                } // for loop
            }
        }
    }
}
