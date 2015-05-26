using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;

namespace BodyExtractionAndHightlighting
{
    public class ImgProcessorFactoryLowRes : IImgProcessorFactory
    {
        public override IBasicManager createBasicManager(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthSensorBuffer, byte userTransparency)
        {
            return (IBasicManager)(new BasicManagerLowRes(bodyIndexSensorBuffer, colorSensorBuffer, depthSensorBuffer, userTransparency));
        }

        public override ISymbolManager createSymbolManager(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthSensorBuffer, Point pTouch, byte userTransparency, System.Windows.Controls.Image guiPointerSymbol)
        {
            return (ISymbolManager)(new SymbolManagerLowRes(bodyIndexSensorBuffer, colorSensorBuffer, depthSensorBuffer, pTouch, userTransparency, guiPointerSymbol));
        }

        public override IHandManager createHandManager(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthSensorBuffer, Dictionary<JointType, Point> armJointPoints, Point pTouch, byte userTransparency)
        {
            return (IHandManager)(new HandManagerLowRes(bodyIndexSensorBuffer, colorSensorBuffer, depthSensorBuffer, armJointPoints, pTouch, userTransparency));
        }

        public override IArmExtensionManager createArmExtensionManager(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthSensorBuffer, Dictionary<JointType, Point> armJointPoints, Point pTouch, byte userTransparency)
        {
            return (IArmExtensionManager)(new ArmExtensionManagerLowRes(bodyIndexSensorBuffer, colorSensorBuffer, depthSensorBuffer, armJointPoints, pTouch, userTransparency));
        }
    }
}
