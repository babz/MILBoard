using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;

namespace BodyExtractionAndHightlighting
{
    public class SymbolManagerLowRes : LowResManager, ISymbolManager
    {
        private byte[] bodyIndexSensorBuffer, colorSensorBuffer;
        private ushort[] depthDataSource;
        private ColorSpacePoint[] depthToColorSpaceMapper = null;

        private Point pTouch;
        private System.Windows.Controls.Image guiSymbol;

        public SymbolManagerLowRes(IntPtr ptrBackbuffer, byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, IReadOnlyDictionary<JointType, Joint> bodyJoints, byte userTransparency, Point pTouch, System.Windows.Controls.Image guiPointerSymbol)
            : base(ptrBackbuffer, bodyJoints, userTransparency)
        {
            this.bodyIndexSensorBuffer = bodyIndexSensorBuffer;
            this.colorSensorBuffer = colorSensorBuffer;

            this.depthToColorSpaceMapper = new ColorSpacePoint[bodyIndexImageLength];
            this.depthDataSource = depthDataSource;

            this.guiSymbol = guiPointerSymbol;
            this.pTouch = pTouch;
        }

        public unsafe void processImage()
        {
            coordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
            {
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;
                base.SetRequiredBuffers(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt);
                base.SetCoordinateMapper(ptrDepthToColorSpaceMapper);

                base.drawFullBody();
                this.drawSymbol();
            }
        }

        private unsafe void drawSymbol()
        {
            System.Windows.Controls.Canvas.SetTop(guiSymbol, pTouch.Y);
            System.Windows.Controls.Canvas.SetLeft(guiSymbol, pTouch.X);
            System.Windows.Controls.Canvas.SetZIndex(guiSymbol, 5);
        }
    }
}
