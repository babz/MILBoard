using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;

namespace BodyExtractionAndHightlighting
{
    public class ImgProcessorFactoryHD : IImgProcessorFactory
    {
        public override IStandardManager createStandardManager(IntPtr ptrBackbuffer, byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, IReadOnlyDictionary<JointType, Joint> bodyJoints, byte userTransparency)
        {
            return (IStandardManager)(new StandardManagerHD(ptrBackbuffer, bodyIndexSensorBuffer, colorSensorBuffer, depthDataSource, bodyJoints, userTransparency));
        }

        public override ISymbolManager createSymbolManager(IntPtr ptrBackbuffer, byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, IReadOnlyDictionary<JointType, Joint> bodyJoints, byte userTransparency, Point pTouch, System.Windows.Controls.Image guiPointerSymbol)
        {
            return (ISymbolManager)(new SymbolManagerHD(ptrBackbuffer, bodyIndexSensorBuffer, colorSensorBuffer, depthDataSource, bodyJoints, userTransparency, pTouch, guiPointerSymbol));
        }

        public override IHandManager createHandManager(IntPtr ptrBackbuffer, byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, IReadOnlyDictionary<JointType, Joint> bodyJoints, byte userTransparency, Point pTouch)
        {
            return (IHandManager)(new HandManagerHD(ptrBackbuffer, bodyIndexSensorBuffer, colorSensorBuffer, depthDataSource, bodyJoints, userTransparency, pTouch));
        }

        public override IArmExtensionManager createArmExtensionManager(IntPtr ptrBackbuffer, byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, IReadOnlyDictionary<JointType, Joint> bodyJoints, byte userTransparency, Point pTouch)
        {
            return (IArmExtensionManager)(new ArmExtensionManagerHD(ptrBackbuffer, bodyIndexSensorBuffer, colorSensorBuffer, depthDataSource, bodyJoints, userTransparency, pTouch));
        }
    }
}
