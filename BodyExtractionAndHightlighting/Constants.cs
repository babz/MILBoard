using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace BodyExtractionAndHightlighting
{
    public class Constants
    {
        private static int bodyIndexSensorBufferWidth, bodyIndexSensorBufferHeight, colorSensorBufferWidth, colorSensorBufferHeight;
        private static CoordinateMapper coordinateMapper;

        private Constants() 
        { 
            //empty on purpose (never instantiate this class)
        }

        public static void initConstants(int bodyIndexSensorBufferWidth, int bodyIndexSensorBufferHeight, int colorSensorBufferWidth, int colorSensorBufferHeight, CoordinateMapper coordinateMapper)
        {
            Constants.bodyIndexSensorBufferWidth = bodyIndexSensorBufferWidth;
            Constants.bodyIndexSensorBufferHeight = bodyIndexSensorBufferHeight;
            Constants.colorSensorBufferWidth = colorSensorBufferWidth;
            Constants.colorSensorBufferHeight = colorSensorBufferHeight;
            Constants.coordinateMapper = coordinateMapper;
        }

        public static int GetBodyIndexSensorBufferWidth()
        {
            return Constants.bodyIndexSensorBufferWidth;
        }

        public static int GetBodyIndexSensorBufferHeight()
        {
            return Constants.bodyIndexSensorBufferHeight;
        }

        public static int GetColorSensorBufferWidth() 
        {
            return Constants.colorSensorBufferWidth;
        }

        public static int GetColorSensorBufferHeight()
        {
            return Constants.colorSensorBufferHeight;
        }

        public static CoordinateMapper GetCoordinateMapper()
        {
            return Constants.coordinateMapper;
        }
    }
}
