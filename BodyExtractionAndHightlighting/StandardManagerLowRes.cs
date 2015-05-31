using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;


namespace BodyExtractionAndHightlighting
{
    public class StandardManagerLowRes : LowResManager, IStandardManager
    {
        private byte[] bodyIndexSensorBuffer, colorSensorBuffer;
        private ushort[] depthDataSource;
        private unsafe ColorSpacePoint[] depthToColorSpaceMapper = null;

        public StandardManagerLowRes(IntPtr ptrBackbuffer, byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, IReadOnlyDictionary<JointType, Joint> bodyJoints, byte userTransparency)
            : base(ptrBackbuffer, bodyJoints, userTransparency)
        {
            this.bodyIndexSensorBuffer = bodyIndexSensorBuffer;
            this.colorSensorBuffer = colorSensorBuffer;

            this.depthToColorSpaceMapper = new ColorSpacePoint[bodyIndexImageLength];
            this.depthDataSource = depthDataSource;
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
            }
        }
    }
}
