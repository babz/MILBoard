using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

namespace BodyExtractionAndHightlighting
{
    public class Helper
    {
        /**
         * @return  Angle in radians
         * */
        public double CalculateRotationAngle(float v1_StartX, float v1_StartY, float v1_EndX, float v1_EndY, float v2_EndX, float v2_EndY)
        {
            double newAngleDeg = this.CalculateDotProduct(v1_StartX, v1_StartY, v1_EndX, v1_EndY, v2_EndX, v2_EndY);

            //TODO consider other cases
            //if (yTouch < yHandTip)
            //{
            //    newAngleDeg = 360 - newAngleDeg;
            //}
            //Console.Out.WriteLine("touchPoint " + this.touchPosition.X + ", " + this.touchPosition.Y + ", newAngle in deg: " + newAngleDeg);

            double newAngleRad = newAngleDeg * Math.PI / 180.0; // conversion into rad

            return newAngleRad;
        }
        
        /**
         * @return  Normalized value that is based on the angle between the arm and the button
         * */
        public double CalculateAbsoluteAngleInDegreeToXaxis(float v1_StartX, float v1_StartY, float v1_EndX, float v1_EndY)
        {
            Vector horizontalLine = new Vector(v1_StartX + 1, v1_StartY);
            double angleInDegrees = this.CalculateDotProduct(v1_StartX, v1_StartY, v1_EndX, v1_EndY, (float)horizontalLine.X, (float)horizontalLine.Y);

            // no differentiation btw pos or neg slope; normalized direction
            double normalizedAngle = Math.Abs(angleInDegrees);
            if (normalizedAngle > 90.0)
            {
                normalizedAngle = 90.0;
            }
            return normalizedAngle;
        }

        /**
         * @see http://whatis.techtarget.com/definition/dot-product-scalar-product
         * @see http://hotmath.com/hotmath_help/topics/magnitude-and-direction-of-vectors.html
         * */
        private double CalculateDotProduct(float v1_StartX, float v1_StartY, float v1_EndX, float v1_EndY, float v2_EndX, float v2_EndY)
        {
            double angleInDegrees = 0.0;

            Vector v1 = new Vector((v1_EndX - v1_StartX), (v1_EndY - v1_StartY));
            Vector v2 = new Vector((v2_EndX - v1_StartX), (v2_EndY - v1_StartY));
            v1.Normalize();
            v2.Normalize();
            angleInDegrees = Vector.AngleBetween(v1, v2);

            return angleInDegrees;
        }

        List<Point> targetNodes = null; //points of region that is supposed to be manipulated 
        /*
         * @param endNode: left most node that is included by the floodfill
         * */
        public void floodfill(int xStart, int yStart, int xEnd, int yEnd)
        {
            int threshold = 10;
            if (xStart < xEnd)
                return;
            if ((yStart < (yEnd + threshold)) || (yStart > (yEnd - threshold)))
                return;

            targetNodes.Add(new Point(xStart, yStart));

            this.floodfill((xStart + 1), yStart, xEnd, yEnd);
            this.floodfill((xStart - 1), yStart, xEnd, yEnd);
            this.floodfill(xStart, (yStart + 1), xEnd, yEnd);
            this.floodfill(xStart, (yStart - 1), xEnd, yEnd);
        }

        public List<Point> GetTargetNodes()
        {
            return targetNodes;
        }
    }
}
