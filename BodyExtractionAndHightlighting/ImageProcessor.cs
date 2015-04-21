using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;

namespace BodyExtractionAndHightlighting
{
    public class ImageProcessor
    {
        private int bodyIndexBufferWidth, bodyIndexBufferHeight, colorBufferWidth, colorBufferHeight;
        private byte userTransparency;

        public ImageProcessor(int bodyIndexSensorBufferWidth, int bodyIndexSensorBufferHeight, int colorSensorBufferWidth, int colorSensorBufferHeight)
        {
            this.bodyIndexBufferWidth = bodyIndexSensorBufferWidth;
            this.bodyIndexBufferHeight = bodyIndexSensorBufferHeight;
            this.colorBufferWidth = colorSensorBufferWidth;
            this.colorBufferHeight = colorSensorBufferHeight;
        }

        public unsafe void ComputeTransformedImage(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, byte[] imageBuffer, ColorSpacePoint[] depthToColorSpaceMapper,
            Point pElbow, Point pWrist, Point pHandTip, Point pTouch)
        {
            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBuffer = imageBuffer)
            fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
            {
                int xElbow = (int)pElbow.X;
                int yElbow = (int)pElbow.Y;
                int xWrist = (int)pWrist.X;
                int yWrist = (int)pWrist.Y;
                int xHandTip = (int)pHandTip.X;
                int yHandTip = (int)pHandTip.Y;
                int xTouch = (int)pTouch.X;
                int yTouch = (int)pTouch.Y;

                this.CalculateTransformation(ptrBodyIndexSensorBuffer, ptrColorSensorBuffer, ptrImageBuffer, ptrDepthToColorSpaceMapper, xElbow, yElbow, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

            } //end fixed
        }

        public unsafe void ComputeSimpleImage(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, byte[] imageBuffer, ColorSpacePoint[] depthToColorSpaceMapper)
        {
            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBuffer = imageBuffer)
            fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
            {
                int length = depthToColorSpaceMapper.Length;
                for (int i = 0; i < length; i++)
                {
                    // bodyIndex can be 0, 1, 2, 3, 4, or 5
                    if (ptrBodyIndexSensorBuffer[i] != 0xff)
                    {
                        int colorPointX = (int)(ptrDepthToColorSpaceMapper[i].X + 0.5);
                        int colorPointY = (int)(ptrDepthToColorSpaceMapper[i].Y + 0.5);
                        uint* intPtr = (uint*)(ptrImageBuffer + i * 4);

                        if ((colorPointY < this.colorBufferHeight) && (colorPointX < this.colorBufferWidth) &&
                        (colorPointY >= 0) && (colorPointX >= 0))
                        {
                            // corresponding pixel in the 1080p image
                            uint* intPtr1080p = (uint*)(ptrColorSensorBuffer + (colorPointY * this.colorBufferWidth + colorPointX) * 4);
                            // assign color value (4 bytes)
                            *intPtr = *intPtr1080p;
                            // overwrite the alpha value
                            *(((byte*)intPtr) + 3) = this.userTransparency;
                        }
                    }
                } //end for
            }
        }

        #region Private Methods

        /**
         * @see http://whatis.techtarget.com/definition/dot-product-scalar-product
         * @see http://hotmath.com/hotmath_help/topics/magnitude-and-direction-of-vectors.html
         * */
        private double CalculateDotProduct(int v1_StartX, int v1_StartY, int v1_EndX, int v1_EndY, int v2_EndX, int v2_EndY) 
        {
            double angleInDegrees = 0.0;

            Vector v1 = new Vector((v1_EndX - v1_StartX), (v1_EndY - v1_StartY));
            Vector v2 = new Vector((v2_EndX - v1_StartX), (v2_EndY - v1_StartY));
            v1.Normalize();
            v2.Normalize();
            angleInDegrees = Vector.AngleBetween(v1, v2);

            return angleInDegrees;
        }

        /**
         * @return  Angle in radians
         * */
        private double CalculateRotationAngle(int v1_StartX, int v1_StartY, int v1_EndX, int v1_EndY, int v2_EndX, int v2_EndY)
        {
            double newAngleDeg = this.CalculateDotProduct(v1_StartX, v1_StartY, v1_EndX, v1_EndY, v2_EndX, v2_EndY);

            //TODO consider other cases
            //if (yTouch < yHandTip)
            //{
            //    newAngleDeg = 360 - newAngleDeg;
            //}
            //Console.Out.WriteLine("touchPoint " + this.touchPosition.X + ", " + this.touchPosition.Y + ", newAngle in deg: " + newAngleDeg);

            double newAngleRad = newAngleDeg * Math.PI / 180; // conversion into rad

            return newAngleRad;
        }

        /**
         * @return  Normalized angle in degrees 
         * */
        private double CalculateNormalizedAngle(int v1_StartX, int v1_StartY, int v1_EndX, int v1_EndY)
        {
            Vector horizontalLine = new Vector(1, 0);
            double angleInDegrees = this.CalculateDotProduct(v1_StartX, v1_StartY, v1_EndX, v1_EndY, (int)horizontalLine.X, (int)horizontalLine.Y);

            // no differentiation btw pos or neg slope; normalized direction
            double normalizedAngle = Math.Abs(angleInDegrees / 180.0f);
            return normalizedAngle;
        }

        /*
         * @example  int indexElbow = yElbow * fdDepth.Width + xElbow;
         * */
        private int GetImageIndexOfBodyJoint(int xBodyJoint, int yBodyJoint)
        {
            return yBodyJoint * bodyIndexBufferWidth + xBodyJoint;
        }

        private unsafe void CalculateTransformation(byte* ptrBodyIndexBuffer, byte* ptrColorBuffer, byte* ptrImageBuffer, ColorSpacePoint* ptrDepthToColorSpaceMapper, int xElbow, int yElbow, int xWrist, int yWrist, int xHandTip, int yHandTip, int xTouch, int yTouch)
        {
            //===== CALC_ROTATION_ANGLE ==========

            double rotationAngleInRad = this.CalculateRotationAngle(xElbow, yElbow, xWrist, yWrist, xTouch, yTouch);
            double cos = Math.Cos(rotationAngleInRad);
            double sin = Math.Sin(rotationAngleInRad);

            //===== CALC_NORMALIZED_ANGLE ==========
            //TODO check if angle need to be calculated from rotated pixel
            double normalizedAngle = this.CalculateNormalizedAngle(xElbow, yElbow, xWrist, yWrist);
            int imgWidth = bodyIndexBufferWidth;

            int lengthOfMapper = bodyIndexBufferHeight * bodyIndexBufferWidth;
            for (int i = 0; i < lengthOfMapper; i++)
            {
                int x = i % bodyIndexBufferWidth;
                int y = i / bodyIndexBufferWidth;
                        
                // color gets assigned to where body index is given (pixel belongs to a body)
                // bodyIndex can be 0, 1, 2, 3, 4, or 5
                //TODO regard only one body
                if (ptrBodyIndexBuffer[i] != 0xff)
                {
                    int colorPointX = (int)(ptrDepthToColorSpaceMapper[i].X + 0.5);
                    int colorPointY = (int)(ptrDepthToColorSpaceMapper[i].Y + 0.5);

                    //check boundaries
                    //if ((colorPointY >= colorBufferHeight) || (colorPointX >= colorBufferWidth) ||
                    //(colorPointY < 0) || (colorPointX < 0))
                    //{
                    //    return;
                    //}


                    uint* ptrTargetPixel = null; // this is where we want to write the pixel

                    #region --- Region of lower arm

                    Vector areaOffset = new Vector((xWrist - xElbow), (yWrist - yElbow));
                    int handOffset = (int)areaOffset.Length / 3;
                    int handTipBoundaryX = xHandTip + handOffset;
                    int handTipBoundaryY = yHandTip + handOffset;
                    // area of the hand
                    if ((x >= xElbow) && (x <= handTipBoundaryX) &&
                        (((handTipBoundaryY <= yElbow) && (y >= handTipBoundaryY) && (y <= yElbow)) ||
                            ((handTipBoundaryY > yElbow) && (y >= yElbow) && (y <= handTipBoundaryY)))
                        )
                    {

                        //=== GET_ROTATED_PIXEL_POS

                        //clockwise rotation:
                        int rotatedX = (int)(cos * (x - xElbow) - sin * (y - yElbow) + xElbow + 0.5);
                        int rotatedY = (int)(sin * (x - xElbow) + cos * (y - yElbow) + yElbow + 0.5);


                        //rotated pixel
                        ptrTargetPixel = (uint*)(ptrImageBuffer + (rotatedY * imgWidth + rotatedX) * 4);

                        // calculates the extension factor
                        #region --- Arm SCALE mode ---

                        //int offsetX = rotatedX - xElbow;
                        ////int lookupX = (int) (xElbow + (offsetX / (2.0 - normalizedAngle)));
                        //int lookupX = (int)(xElbow + (offsetX / (2.0 + pointerOffset - normalizedAngle)));

                        //int offsetY = rotatedY - yElbow;
                        ////int lookupY = (int) (yElbow + (offsetY / (1.0 + normalizedAngle)));
                        //int lookupY = (int)(yElbow + (offsetY / (1.0 + pointerOffset + normalizedAngle)));

                        //// bodyIndex can be 0, 1, 2, 3, 4, or 5
                        //if (biDataSource[imgWidth * lookupY + lookupX] != 0xff)
                        //{
                        //    int colorPointX_stretch = (int)(ptrDepthIntoColorSpace[imgWidth * lookupY + lookupX].X + 0.5);
                        //    int colorPointY_stretch = (int)(ptrDepthIntoColorSpace[imgWidth * lookupY + lookupX].Y + 0.5);
                        //    uint* intPtr_stretch = (uint*)(ptrCombiColorBuffer + i * 4); //stays the same   

                        //    if ((colorPointY_stretch < fdColor.Height) && (colorPointX_stretch < fdColor.Width) &&
                        //    (colorPointY_stretch >= 0) && (colorPointX_stretch >= 0))
                        //    {
                        //        // corresponding pixel in the 1080p color image
                        //        uint* intPtr1080p = (uint*)(ptrCombiColorBuffer1080p + (colorPointY_stretch * fdColor.Width + colorPointX_stretch) * 4);
                        //        // assign color value (4 bytes)
                        //        *intPtr_stretch = *intPtr1080p;
                        //        // overwrite the alpha value
                        //        *(((byte*)intPtr_stretch) + 3) = (byte)userTransparency.Value;
                        //    }
                        //}

                        # endregion

                    }
                    #endregion
                    else
                    {
                        // point to current pixel in image buffer
                        ptrTargetPixel = (uint*)(ptrImageBuffer + i * 4); 
                    }

                    //check boundaries of image
                    if ((colorPointY < colorBufferHeight) && (colorPointX < colorBufferWidth) &&
                    (colorPointY >= 0) && (colorPointX >= 0))
                    {
                        // corresponding pixel in the 1080p image
                        uint* ptrSourcePixel = (uint*)(ptrColorBuffer + (colorPointY * bodyIndexBufferWidth + colorPointX) * 4);

                        // assign color value (4 bytes)
                        *ptrTargetPixel = *ptrSourcePixel;

                        // overwrite the alpha value (last byte)
                        *(((byte*)ptrTargetPixel) + 3) = this.userTransparency; 
                    }

                } // if pixel belongs to body

            } //end for
        }

        #endregion

        #region Properties

        public byte PropUserTransparency
        {
            get
            {
                return this.userTransparency;
            }
            set
            {
                this.userTransparency = value;
            }
        }
        #endregion
    }
}
