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

        public unsafe void ComputeTransformedImage_LowRes(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, byte[] imageBufferLowRes, DepthSpacePoint[] colorToDepthSpaceMapper,
            Point pElbow, Point pWrist, Point pHandTip, Point pTouch)
        {
            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferLowRes = imageBufferLowRes)
            fixed (DepthSpacePoint* ptrColorToDepthSpaceMapper = colorToDepthSpaceMapper)
            {
                int xElbow = (int)pElbow.X;
                int yElbow = (int)pElbow.Y;
                int xWrist = (int)pWrist.X;
                int yWrist = (int)pWrist.Y;
                int xHandTip = (int)pHandTip.X;
                int yHandTip = (int)pHandTip.Y;
                int xTouch = (int)pTouch.X;
                int yTouch = (int)pTouch.Y;

                this.CalculateTransformation_LowRes(ptrBodyIndexSensorBuffer, ptrColorSensorBuffer, ptrImageBufferLowRes, ptrColorToDepthSpaceMapper, xElbow, yElbow, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

            } //end fixed
        }

        public unsafe void ComputeTransformedImage_HD(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, byte[] imageBufferHD, DepthSpacePoint[] colorToDepthSpaceMapper, Point pElbow, Point pWrist, Point pHandTip, Point pTouch)
        {
            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferHD = imageBufferHD)
            fixed (DepthSpacePoint* ptrColorToDepthSpaceMapper = colorToDepthSpaceMapper)
            {
                int xElbow = (int)pElbow.X;
                int yElbow = (int)pElbow.Y;
                int xWrist = (int)pWrist.X;
                int yWrist = (int)pWrist.Y;
                int xHandTip = (int)pHandTip.X;
                int yHandTip = (int)pHandTip.Y;
                int xTouch = (int)pTouch.X;
                int yTouch = (int)pTouch.Y;

                this.CalculateTransformation_HD(ptrBodyIndexSensorBuffer, ptrColorSensorBuffer, ptrImageBufferHD, ptrColorToDepthSpaceMapper, xElbow, yElbow, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

            } //end fixed
        }

        public unsafe void ComputeSimpleImage_LowRes(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, byte[] imageBufferLowRes, ColorSpacePoint[] depthToColorSpaceMapper)
        {
            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferLowRes = imageBufferLowRes)
            fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
            {
                //points to start of ptrImageBufferLowRes
                uint* ptrImageBufferInt = (uint*)ptrImageBufferLowRes;
                uint* ptrImgBufferPixelInt = null;
                uint* ptrColorSensorBufferPixelInt = null;
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                int length = depthToColorSpaceMapper.Length;
                for (int i = 0; i < length; i++)
                {
                    // bodyIndex can be 0, 1, 2, 3, 4, or 5
                    if (ptrBodyIndexSensorBuffer[i] != 0xff)
                    {
                        int colorPointX = (int)(ptrDepthToColorSpaceMapper[i].X + 0.5);
                        int colorPointY = (int)(ptrDepthToColorSpaceMapper[i].Y + 0.5);

                        //check boundaries
                        if ((colorPointY >= this.colorBufferHeight) || (colorPointX >= this.colorBufferWidth) ||
                        (colorPointY < 0) && (colorPointX < 0))
                        {
                            continue;
                        }

                        //point to current pixel in target imgBuffer
                        ptrImgBufferPixelInt = ptrImageBufferInt + i;

                        // corresponding pixel in the 1080p image
                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * this.colorBufferWidth + colorPointX);
                        // assign color value (4 bytes); dereferencing + assigning writes into imgBuffer
                        *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                        // overwrite the alpha value
                        *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                    }
                } //end for
            }
        }

        public unsafe void ComputeSimpleImage_HD(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, byte[] imageBufferHD, DepthSpacePoint[] colorToDepthSpaceMapper)
        {
            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferHD = imageBufferHD)
            fixed (DepthSpacePoint* ptrColorToDepthSpaceMapper = colorToDepthSpaceMapper)
            {
                //with the cast to int, step size is 4 bytes
                uint* ptrImageBufferInt = (uint*)ptrImageBufferHD;
                uint* ptrImgBufferPixelInt = null;

                int lengthColorBuffer = colorBufferHeight * colorBufferWidth;
                //after the loop, only color pixels with a body index value that can be mapped to a depth value will remain in the buffer
                for (int i = 0; i < lengthColorBuffer; i++)
                {
                    //where the color img cannot be mapped to the depth image, there are infinity values
                    if (Single.IsInfinity(ptrColorToDepthSpaceMapper[i].Y) || Single.IsInfinity(ptrColorToDepthSpaceMapper[i].X))
                    {
                        continue;
                    }

                    int idx = (int)(bodyIndexBufferWidth * ptrColorToDepthSpaceMapper[i].Y + ptrColorToDepthSpaceMapper[i].X); //2D to 1D

                    // bodyIndex can be 0, 1, 2, 3, 4, or 5
                    if (ptrBodyIndexSensorBuffer[idx] != 0xff)
                    {
                        ptrImgBufferPixelInt = ptrImageBufferInt + i;

                        //with the cast to int, 4 bytes are copied
                        *ptrImgBufferPixelInt = ((uint*)ptrColorSensorBuffer)[i];
                        // overwrite the alpha value
                        *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                    }
                    
                } // for loop
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

        private unsafe void CalculateTransformation_LowRes(byte* ptrBodyIndexBuffer, byte* ptrColorBuffer, byte* ptrImageBufferLowRes, DepthSpacePoint* ptrColorToDepthSpaceMapper, int xElbow, int yElbow, int xWrist, int yWrist, int xHandTip, int yHandTip, int xTouch, int yTouch)
        {
            //===== CALC_ROTATION_ANGLE ==========

            double rotationAngleInRad = this.CalculateRotationAngle(xElbow, yElbow, xWrist, yWrist, xTouch, yTouch);
            double cos = Math.Cos(rotationAngleInRad);
            double sin = Math.Sin(rotationAngleInRad);

            //===== CALC_NORMALIZED_ANGLE ==========
            //TODO check if angle need to be calculated from rotated pixel
            double normalizedAngle = this.CalculateNormalizedAngle(xElbow, yElbow, xWrist, yWrist);

            uint* ptrImgBufferPixelInt = null; // this is where we want to write the pixel
            uint* ptrImageBufferInt = (uint*)ptrImageBufferLowRes;
            uint* ptrColorSensorBufferInt = (uint*)ptrColorBuffer;
            uint* ptrColorSensorBufferPixelInt = null;

            Vector areaOffset = new Vector((xWrist - xElbow), (yWrist - yElbow));
            int handOffset = (int)areaOffset.Length / 3;
            int handTipBoundaryX = xHandTip + handOffset;
            int handTipBoundaryY = yHandTip;// -handOffset;

            int lengthOfMapper = colorBufferHeight * colorBufferWidth;
            for (int idxColorSpace = 0; idxColorSpace < lengthOfMapper; idxColorSpace++)
            {
                float xDepthSpaceF = ptrColorToDepthSpaceMapper[idxColorSpace].X;
                float yDepthSpaceF = ptrColorToDepthSpaceMapper[idxColorSpace].Y;

                //where the color img cannot be mapped to the depth image, there are infinity values
                if (Single.IsInfinity(yDepthSpaceF) || Single.IsInfinity(xDepthSpaceF))
                {
                    continue;
                }

                int xDepthSpace = (int)(ptrColorToDepthSpaceMapper[idxColorSpace].X + 0.5);
                int yDepthSpace = (int)(ptrColorToDepthSpaceMapper[idxColorSpace].Y + 0.5);

                //corrresponding pixel in the bodyIndexBuffer; mapper returns pixel in depth space
                int idxDepthSpace = (int)(bodyIndexBufferWidth * yDepthSpace + xDepthSpace); //2D to 1D

                // color gets assigned to where body index is given (pixel belongs to a body)
                // bodyIndex can be 0, 1, 2, 3, 4, or 5
                //TODO regard only one body
                if (ptrBodyIndexBuffer[idxDepthSpace] != 0xff)
                {
                    #region --- Region of lower arm

                    // area of the hand
                    if ((xDepthSpace >= xElbow) && (xDepthSpace <= handTipBoundaryX) &&
                        (((handTipBoundaryY <= yElbow) && (yDepthSpace >= handTipBoundaryY) && (yDepthSpace <= yElbow)) ||
                            ((handTipBoundaryY > yElbow) && (yDepthSpace >= yElbow) && (yDepthSpace <= handTipBoundaryY)))
                        )
                    {
                        //=== GET_ROTATED_PIXEL_POS

                        //clockwise rotation:
                        int xDepthSpace_rot = (int)(cos * (xDepthSpace - xElbow) - sin * (yDepthSpace - yElbow) + xElbow + 0.5);
                        int yDepthSpace_rot = (int)(sin * (xDepthSpace - xElbow) + cos * (yDepthSpace - yElbow) + yElbow + 0.5);

                        //rotated pixel
                        ptrImgBufferPixelInt = ptrImageBufferInt + (yDepthSpace_rot * bodyIndexBufferWidth + xDepthSpace_rot);

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
                        ptrImgBufferPixelInt = ptrImageBufferInt + idxDepthSpace;
                    }

                    int xColorSpace = idxColorSpace % colorBufferWidth;
                    int yColorSpace = idxColorSpace / colorBufferWidth;

                    /* Overwrite body-index-pixel with color pixel
                     */
                    // corresponding pixel in the 1080p image
                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (yColorSpace * colorBufferWidth + xColorSpace);

                    // assign color value (4 bytes)
                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;

                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;

                } // if pixel belongs to body

            } //end for
        }

        private unsafe void CalculateTransformation_HD(byte* ptrBodyIndexSensorBuffer, byte* ptrColorSensorBuffer, byte* ptrImageBufferHD, DepthSpacePoint* ptrColorToDepthSpaceMapper, int xElbow, int yElbow, int xWrist, int yWrist, int xHandTip, int yHandTip, int xTouch, int yTouch)
        {
            //===== CALC_ROTATION_ANGLE ==========

            double rotationAngleInRad = this.CalculateRotationAngle(xElbow, yElbow, xWrist, yWrist, xTouch, yTouch);
            double cos = Math.Cos(rotationAngleInRad);
            double sin = Math.Sin(rotationAngleInRad);

            //===== CALC_NORMALIZED_ANGLE ==========
            //TODO check if angle need to be calculated from rotated pixel
            double normalizedAngle = this.CalculateNormalizedAngle(xElbow, yElbow, xWrist, yWrist);

            //with the cast to int, step size is 4 bytes
            uint* ptrImageBufferInt = (uint*)ptrImageBufferHD;
            uint* ptrImgBufferPixelInt = null;

            int handTipBoundaryX = xHandTip;// +handOffset;
            int handTipBoundaryY = yHandTip;// -handOffset;

            int lengthOfMapper = colorBufferHeight * colorBufferWidth;
            for (int i = 0; i < lengthOfMapper; i++)
            {
                //where the color img cannot be mapped to the depth image, there are infinity values
                if (Single.IsInfinity(ptrColorToDepthSpaceMapper[i].Y) || Single.IsInfinity(ptrColorToDepthSpaceMapper[i].X))
                {
                    continue;
                }

                int x = i % colorBufferWidth;
                int y = i / colorBufferWidth;

                //corrresponding pixel in the bodyIndexBuffer
                int idx = (int)(bodyIndexBufferWidth * ptrColorToDepthSpaceMapper[i].Y + ptrColorToDepthSpaceMapper[i].X); //2D to 1D

                // bodyIndex can be 0, 1, 2, 3, 4, or 5
                if (ptrBodyIndexSensorBuffer[idx] != 0xff)
                {
                    #region --- Region of lower arm

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
                        ptrImgBufferPixelInt = ptrImageBufferInt + (rotatedY * bodyIndexBufferWidth + rotatedX);
                    }
                    #endregion
                    else
                    {
                        // point to current pixel in image buffer
                        ptrImgBufferPixelInt = ptrImageBufferInt + i;
                    }

                    //ptrImgBufferPixelInt = ptrImageBufferInt + i;

                    //with the cast to int, 4 bytes are copied
                    *ptrImgBufferPixelInt = ((uint*)ptrColorSensorBuffer)[i];
                    // overwrite the alpha value
                    *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                }

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
