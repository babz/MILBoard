using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;


namespace BodyExtractionAndHightlighting
{
    public class BasicManagerLowRes : IBasicManager
    {
        private int bodyIndexSensorBufferWidth, bodyIndexSensorBufferHeight, colorSensorBufferWidth, colorSensorBufferHeight;
        private byte[] bodyIndexSensorBuffer, colorSensorBuffer;

        private CoordinateMapper coordinateMapper;
        private ushort[] depthDataSource;

        unsafe protected ColorSpacePoint[] depthToColorSpaceMapper = null;

        private byte userTransparency;

        public BasicManagerLowRes(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, byte userTransparency)
        {
            this.bodyIndexSensorBufferWidth = Constants.GetBodyIndexSensorBufferWidth();
            this.bodyIndexSensorBufferHeight = Constants.GetBodyIndexSensorBufferHeight();
            this.colorSensorBufferWidth = Constants.GetColorSensorBufferWidth();
            this.colorSensorBufferHeight = Constants.GetColorSensorBufferHeight();

            this.bodyIndexSensorBuffer = bodyIndexSensorBuffer;
            this.colorSensorBuffer = colorSensorBuffer;

            this.coordinateMapper = Constants.GetCoordinateMapper();
            this.depthToColorSpaceMapper = new ColorSpacePoint[depthDataSource.Length];
            this.depthDataSource = depthDataSource;

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
                //points to start of ptrImageBufferLowRes
                uint* ptrImageBufferInt = (uint*)ptrImageBufferLowRes;
                uint* ptrImgBufferPixelInt = null;
                uint* ptrColorSensorBufferPixelInt = null;
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                int length = depthToColorSpaceMapper.Length;
                for (int i = 0; i < length; i++)
                {
                    // bodyIndex can be 0, 1, 2, 3, 4, or 5
                    if (*(ptrBodyIndexSensorBuffer + i) != 0xff)
                    {
                        int colorPointX = (int)((ptrDepthToColorSpaceMapper + i)->X + 0.5);
                        int colorPointY = (int)((ptrDepthToColorSpaceMapper + i)->Y + 0.5);

                        //check boundaries
                        if ((colorPointY >= colorSensorBufferHeight) || (colorPointX >= colorSensorBufferWidth) ||
                        (colorPointY < 0) && (colorPointX < 0))
                        {
                            continue;
                        }

                        //point to current pixel in target imgBuffer
                        ptrImgBufferPixelInt = ptrImageBufferInt + i;

                        // corresponding pixel in the 1080p image
                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
                        // assign color value (4 bytes); dereferencing + assigning writes into imgBuffer
                        *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                        // overwrite the alpha value
                        *(((byte*)ptrImgBufferPixelInt) + 3) = userTransparency;
                    }
                } //end for
            }
        }
    }
}
