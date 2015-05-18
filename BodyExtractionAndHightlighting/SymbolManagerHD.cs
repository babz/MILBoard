using System;
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
        Point pTouch;

        public SymbolManagerHD(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, Point pTouch)
        {
            this.pTouch = pTouch;
        }
        
        public unsafe void processImage(byte[] imageBufferHD)
        {
            throw new NotImplementedException();
        }
    }
}
