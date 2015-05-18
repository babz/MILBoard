using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;

namespace BodyExtractionAndHightlighting
{
    public class SymbolManagerLowRes : ISymbolManager
    {
        Point pTouch;

        public SymbolManagerLowRes(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, Point pTouch)
        {
            this.pTouch = pTouch;
        }

        public unsafe void processImage(byte[] imageBufferLowRes)
        {
            throw new NotImplementedException();
        }
    }
}
