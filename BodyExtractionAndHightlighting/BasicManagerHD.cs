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

        private volatile unsafe byte* ptrBodyIndexSensorBuffer;
        private volatile unsafe uint* ptrImageBufferInt, ptrColorSensorBufferInt;
        private volatile unsafe DepthSpacePoint* ptrColorToDepthSpaceMapper;

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

        public unsafe void processImage(IntPtr ptrBackbuffer)
        {
            coordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorToDepthSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            //fixed (byte* ptrImageBufferHD = imageBufferHD)
            fixed (DepthSpacePoint* ptrColorToDepthSpaceMapper = colorToDepthSpaceMapper)
            {
                //with the cast to int, step size is 4 bytes
                this.ptrBodyIndexSensorBuffer = ptrBodyIndexSensorBuffer;
                this.ptrImageBufferInt = (uint*)ptrBackbuffer;
                this.ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;
                this.ptrColorToDepthSpaceMapper = ptrColorToDepthSpaceMapper;

                this.drawBody();
                //this.bodyFloodFill();
            }
        }

        private unsafe void drawBody()
        {
            uint* ptrImgBufferPixelInt = null;

            int lengthColorBuffer = colorToDepthSpaceMapper.Length; //colorSensorBufferHeight * colorSensorBufferWidth;
            //after the loop, only color pixels with a body index value that can be mapped to a depth value will remain in the buffer
            for (int i = 0; i < lengthColorBuffer; i++)
            {
                //where the color img cannot be mapped to the depth image, there are infinity values
                if (Single.IsInfinity((ptrColorToDepthSpaceMapper + i)->Y) || Single.IsInfinity((ptrColorToDepthSpaceMapper + i)->X))
                {
                    continue;
                }

                int idx = (int)(bodyIndexSensorBufferWidth * (ptrColorToDepthSpaceMapper + i)->Y + (ptrColorToDepthSpaceMapper + i)->X); //2D to 1D

                // bodyIndex can be 0, 1, 2, 3, 4, or 5
                if (*(ptrBodyIndexSensorBuffer + idx) != 0xff)
                {
                    ptrImgBufferPixelInt = ptrImageBufferInt + i;

                    //with the cast to int, 4 bytes are copied
                    *ptrImgBufferPixelInt = *(ptrColorSensorBufferInt + i);
                    // overwrite the alpha value
                    *(((byte*)ptrImgBufferPixelInt) + 3) = userTransparency;
                }

            } // for loop
        }

        private unsafe void bodyFloodFill(int xStart, int yStart)
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

            //pixel already visited
            uint* ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxCurrColorPixel;
            if ((*ptrColorSensorBufferPixelInt) == 0xFF000000)
            {
                return;
            }

            float xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->X;
            float yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->Y;
            int idxDepthPixel = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
            if (Single.IsInfinity(xDepthPixel) || Single.IsInfinity(yDepthPixel) || (*(ptrBodyIndexSensorBuffer + idxDepthPixel) == 0xff))
            {
                return;
            }
            else
            {
                // point to current pixel in image buffer
                uint* ptrImgBufferPixelInt = ptrImageBufferInt + (yStart * colorSensorBufferWidth + xStart);

                // assign color value (4 bytes)
                *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                // overwrite the alpha value (last byte)
                *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency);

                //do not visit same pixel twice
                *ptrColorSensorBufferPixelInt = 0xFF000000;
            }

            //4-way neighbourhood to visit all pixels of hand (can have background pixel btw fingers)
            this.bodyFloodFill((xStart + 1), yStart);
            this.bodyFloodFill((xStart - 1), yStart);
            this.bodyFloodFill(xStart, (yStart + 1));
            this.bodyFloodFill(xStart, (yStart - 1));
        }
    }
}
