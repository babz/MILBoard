using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;

namespace BodyExtractionAndHightlighting
{
    public class SymbolManager : ImageProcessor
    {
        Point pTouch;

        public SymbolManager(int bodyIndexSensorBufferWidth, int bodyIndexSensorBufferHeight, int colorSensorBufferWidth, int colorSensorBufferHeight, byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, KinectSensor sensor, ushort[] depthDataSource, Point pTouch)
            : base(bodyIndexSensorBufferWidth, bodyIndexSensorBufferHeight, colorSensorBufferWidth, colorSensorBufferHeight, bodyIndexSensorBuffer, colorSensorBuffer, sensor, depthDataSource)
        {
            this.pTouch = pTouch;
        }


        internal void processImageLowRes(byte[] imageBufferLowRes)
        {
            throw new NotImplementedException();
        }

        internal void processImageHD(byte[] imageBufferHD)
        {
            throw new NotImplementedException();
        }
    }
}
