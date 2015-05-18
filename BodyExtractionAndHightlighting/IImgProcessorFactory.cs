using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;

namespace BodyExtractionAndHightlighting
{
    public enum ImageProcessorType { LowRes, HD };

    public abstract class IImgProcessorFactory
    {
        public abstract IBasicManager createBasicManager(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthSensorBuffer, byte userTransparency);

        public abstract ISymbolManager createSymbolManager(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthSensorBuffer, Point pTouch);

        public abstract IHandManager createHandManager(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthSensorBuffer, Dictionary<JointType, Point> armJointPoints, Point pTouch, byte userTransparency);

        public abstract IArmExtensionManager createArmExtensionManager(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthSensorBuffer, Dictionary<JointType, Point> armJointPoints, Point pTouch, byte userTransparency);
    }
}
