﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;

namespace BodyExtractionAndHightlighting
{
    public class SymbolManagerHD : ISymbolManager
    {
        private int bodyIndexSensorBufferWidth, colorSensorBufferWidth;
        private byte[] bodyIndexSensorBuffer, colorSensorBuffer;

        private CoordinateMapper coordinateMapper;
        private ushort[] depthDataSource;

        unsafe protected DepthSpacePoint[] colorToDepthSpaceMapper = null;

        private volatile unsafe DepthSpacePoint* ptrColorToDepthSpaceMapper;
        private volatile unsafe byte* ptrBodyIndexSensorBuffer;
        private volatile unsafe uint* ptrImageBufferHDInt, ptrColorSensorBufferInt;

        private byte userTransparency;
        Point pTouch;
        System.Windows.Controls.Image guiSymbol;

        public SymbolManagerHD(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, Point pTouch, byte userTransparency, System.Windows.Controls.Image guiPointerSymbol)
        {
            this.bodyIndexSensorBufferWidth = Constants.GetBodyIndexSensorBufferWidth();
            this.colorSensorBufferWidth = Constants.GetColorSensorBufferWidth();

            this.bodyIndexSensorBuffer = bodyIndexSensorBuffer;
            this.colorSensorBuffer = colorSensorBuffer;

            this.coordinateMapper = Constants.GetCoordinateMapper();
            this.depthDataSource = depthDataSource;

            this.colorToDepthSpaceMapper = new DepthSpacePoint[colorSensorBufferWidth * Constants.GetColorSensorBufferHeight()];

            this.guiSymbol = guiPointerSymbol;
            this.userTransparency = userTransparency;
            this.pTouch = pTouch;
        }
        
        public unsafe void processImage(byte[] imageBufferHD)
        {
            coordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorToDepthSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferHD = imageBufferHD)
            fixed (DepthSpacePoint* ptrColorToDepthSpaceMapper = colorToDepthSpaceMapper)
            {
                this.ptrBodyIndexSensorBuffer = ptrBodyIndexSensorBuffer;
                this.ptrColorToDepthSpaceMapper = ptrColorToDepthSpaceMapper;

                this.ptrImageBufferHDInt = (uint*)ptrImageBufferHD;
                this.ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                this.drawBody();
                this.drawSymbol();
            }
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
            int lengthColorBuffer = colorToDepthSpaceMapper.Length; // = colorSensorBufferHeight * colorSensorBufferWidth;
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

        private unsafe void drawSymbol()
        {
            System.Windows.Controls.Canvas.SetTop(guiSymbol, pTouch.Y);
            System.Windows.Controls.Canvas.SetLeft(guiSymbol, pTouch.X);
            System.Windows.Controls.Canvas.SetZIndex(guiSymbol, 5);
        }
    }
}
