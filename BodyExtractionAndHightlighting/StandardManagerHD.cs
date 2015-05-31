using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace BodyExtractionAndHightlighting
{
    public class StandardManagerHD : HDManager, IStandardManager 
    {
        private byte[] bodyIndexSensorBuffer, colorSensorBuffer;
        private ushort[] depthDataSource;
        private unsafe DepthSpacePoint[] colorToDepthSpaceMapper = null;

        public StandardManagerHD(IntPtr ptrBackbuffer, byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, IReadOnlyDictionary<JointType, Joint> bodyJoints, byte userTransparency)
            : base(ptrBackbuffer, bodyJoints, userTransparency)
        {
            this.bodyIndexSensorBuffer = bodyIndexSensorBuffer;
            this.colorSensorBuffer = colorSensorBuffer;

            this.colorToDepthSpaceMapper = new DepthSpacePoint[colorImageLength];
            this.depthDataSource = depthDataSource;
        }

        public unsafe void processImage()
        {
            coordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorToDepthSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (DepthSpacePoint* ptrColorToDepthSpaceMapper = colorToDepthSpaceMapper)
            {
                this.ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;
                base.SetRequiredBuffers(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt);
                base.SetCoordinateMapper(ptrColorToDepthSpaceMapper);

                base.drawFullBody();
            }
        }
    }
}
