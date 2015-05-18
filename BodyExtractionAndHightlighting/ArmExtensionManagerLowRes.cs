using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;

namespace BodyExtractionAndHightlighting
{
    public class ArmExtensionManagerLowRes : IArmExtensionManager
    {
        private int bodyIndexSensorBufferWidth, bodyIndexSensorBufferHeight, colorSensorBufferWidth, colorSensorBufferHeight;
        private byte[] bodyIndexSensorBuffer, colorSensorBuffer;

        private CoordinateMapper coordinateMapper;
        private ushort[] depthDataSource;

        private Point pElbow, pWrist, pHandTip, pTouch;
        private const double HAND_TRANSLATED_ALPHAFACTOR = 0.75;

        unsafe protected ColorSpacePoint[] depthToColorSpaceMapper = null;
        unsafe protected DepthSpacePoint[] colorToDepthSpaceMapper = null;

        private Helper helper;
        private byte userTransparency;

        public ArmExtensionManagerLowRes(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, Dictionary<JointType, Point> armJointPoints, Point pTouch, byte userTransparency)
        {
            this.bodyIndexSensorBufferWidth = Constants.GetBodyIndexSensorBufferWidth();
            this.bodyIndexSensorBufferHeight = Constants.GetBodyIndexSensorBufferHeight();
            this.colorSensorBufferWidth = Constants.GetColorSensorBufferWidth();
            this.colorSensorBufferHeight = Constants.GetColorSensorBufferHeight();

            this.bodyIndexSensorBuffer = bodyIndexSensorBuffer;
            this.colorSensorBuffer = colorSensorBuffer;

            this.coordinateMapper = Constants.GetCoordinateMapper();
            this.depthDataSource = depthDataSource;

            this.pElbow = armJointPoints[JointType.ElbowRight];
            this.pWrist = armJointPoints[JointType.WristRight];
            this.pHandTip = armJointPoints[JointType.HandTipRight];
            this.pTouch = pTouch;

            this.depthToColorSpaceMapper = new ColorSpacePoint[depthDataSource.Length];
            this.colorToDepthSpaceMapper = new DepthSpacePoint[colorSensorBufferWidth * colorSensorBufferHeight];

            this.helper = Helper.getInstance();
            this.userTransparency = userTransparency;
        }

        public unsafe void processImage(byte[] imageBufferLowRes)
        {
            coordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferLowRes = imageBufferLowRes)
            fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
            {
                float xElbow = (float)pElbow.X;
                float yElbow = (float)pElbow.Y;
                float xWrist = (float)pWrist.X;
                float yWrist = (float)pWrist.Y;
                float xHandTip = (float)pHandTip.X;
                float yHandTip = (float)pHandTip.Y;
                float xTouch = (float)pTouch.X;
                float yTouch = (float)pTouch.Y;

                uint* ptrImageBufferInt = (uint*)ptrImageBufferLowRes;
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                this.transform_LowRes(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferInt, ptrDepthToColorSpaceMapper, xElbow, yElbow, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);
            } //end fixed
        }

        public unsafe void processImage_rotationOnly(byte[] imageBufferLowRes)
        {
            coordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferLowRes = imageBufferLowRes)
            fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
            {
                float xElbow = (float)pElbow.X;
                float yElbow = (float)pElbow.Y;
                float xWrist = (float)pWrist.X;
                float yWrist = (float)pWrist.Y;
                float xHandTip = (float)pHandTip.X;
                float yHandTip = (float)pHandTip.Y;
                float xTouch = (float)pTouch.X;
                float yTouch = (float)pTouch.Y;

                uint* ptrImageBufferInt = (uint*)ptrImageBufferLowRes;
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                this.transform_LowRes_rotationOnly(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferInt, ptrDepthToColorSpaceMapper, xElbow, yElbow, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

            } //end fixed
        }

        public unsafe void processImage_scaleOnly(byte[] imageBufferLowRes)
        {
            coordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferLowRes = imageBufferLowRes)
            fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
            {
                float xElbow = (float)pElbow.X;
                float yElbow = (float)pElbow.Y;
                float xTouch = (float)pTouch.X;
                float yTouch = (float)pTouch.Y;
                float xWrist = (float)pWrist.X;
                float yWrist = (float)pWrist.Y;

                uint* ptrImageBufferInt = (uint*)ptrImageBufferLowRes;
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                this.transform_LowRes_scaleOnly(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferInt, ptrDepthToColorSpaceMapper, xElbow, yElbow, xWrist, yWrist, xTouch, yTouch);

            } //end fixed
        }

        //TODO wrong algorithm!!!!
        private unsafe void transform_LowRes(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferInt, ColorSpacePoint* ptrDepthToColorSpaceMapper, float xElbow, float yElbow, float xWrist, float yWrist, float xHandTip, float yHandTip, float xTouch, float yTouch)
        {
            
            uint* ptrImgBufferPixelInt = null; // this is where we want to write the pixel
            uint* ptrColorSensorBufferPixelInt = null;

            // v = (x, y)
            Vector vOrigArm = new Vector((xHandTip - xElbow), (yHandTip - yElbow));
            float vOrigArmLength = (float)vOrigArm.Length;
            vOrigArm.Normalize();

            //NOTE vector normals different as coordinate system origin (0,0) is upper left corner!
            //v_nleft = (y, -x)
            Vector vNormLeftOrigArm = new Vector(vOrigArm.Y, -(vOrigArm.X));
            //v_nright = (-y, x)
            Vector vNormRightOrigArm = new Vector(-(vOrigArm.Y), vOrigArm.X);

            Vector vNewArm = new Vector((xTouch - xElbow), (yTouch - yElbow));
            float vNewArmLength = (float)vNewArm.Length;
            vNewArm.Normalize();

            Vector vNormLeftNewArm = new Vector(vNewArm.Y, -(vNewArm.X));
            Vector vNormRightNewArm = new Vector(-(vNewArm.Y), vNewArm.X);
            

            float totalSteps = 100; //TODO set totalSteps = vNewArmLength to avoid holes
            float stepSizeOrigArm = vOrigArmLength / totalSteps;
            float stepSizeNewArm = vNewArmLength / totalSteps;

            float xCurrOrigArm = xElbow;
            float yCurrOrigArm = yElbow;
            float xCurrNewArm = xElbow;
            float yCurrNewArm = yElbow;
            for (int i = 0; i < totalSteps; i++)
            {
                int colorLookup = (int)(bodyIndexSensorBufferWidth * yCurrOrigArm + xCurrOrigArm + 0.5);
                int colorPointX = (int)(ptrDepthToColorSpaceMapper[colorLookup].X + 0.5);
                int colorPointY = (int)(ptrDepthToColorSpaceMapper[colorLookup].Y + 0.5);

                if ((xCurrNewArm < bodyIndexSensorBufferWidth) && (yCurrNewArm < bodyIndexSensorBufferHeight) && (xCurrNewArm >= 0) && (yCurrNewArm >= 0) && (colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) &&
                    (colorPointY >= 0) && (colorPointX >= 0))
                {
                    //write color pixel into position of new arm
                    int newPositionX = (int)(xCurrNewArm + 0.5);
                    int newPositionY = (int)(yCurrNewArm + 0.5);
                    ptrImgBufferPixelInt = ptrImageBufferInt + (newPositionY * bodyIndexSensorBufferWidth + newPositionX);

                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
                    // assign color value (4 bytes)
                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                }
                
                //calculate for left normal of vector
                float xNormLeft = xCurrOrigArm + (float)vNormLeftOrigArm.X;
                float yNormLeft = yCurrOrigArm + (float)vNormLeftOrigArm.Y;
                int lookupIdxNormLeft = (int)(yNormLeft * bodyIndexSensorBufferWidth + xNormLeft + 0.5);

                //while (ptrBodyIndexSensorBuffer[lookupIdxNormLeft] != 0xff)
                //{
                //    //normal might point inside body
                //    if (xNormLeft <= xElbow)
                //    {
                //        continue;
                //    }

                //    int normLeftColorPointX = (int)(ptrDepthToColorSpaceMapper[lookupIdxNormLeft].X + 0.5);
                //    int normLeftColorPointY = (int)(ptrDepthToColorSpaceMapper[lookupIdxNormLeft].Y + 0.5);

                //    if ((xNormLeft < bodyIndexSensorBufferWidth) && (yNormLeft < bodyIndexSensorBufferHeight) && (xNormLeft >= 0) && (yNormLeft >= 0) && (normLeftColorPointY < colorSensorBufferHeight) && (normLeftColorPointX < colorSensorBufferWidth) &&
                //   (normLeftColorPointY >= 0) && (normLeftColorPointX >= 0))
                //    {
                //        //write color pixel into position of new arm
                //        int newPosNormLeftX = (int)(xCurrNewArm + (float)vNormLeftNewArm.X + 0.5);
                //        int newPosNormLeftY = (int)(yCurrNewArm + (float)vNormLeftNewArm.Y + 0.5);
                //        ptrImgBufferPixelInt = ptrImageBufferInt + (newPosNormLeftY * bodyIndexSensorBufferWidth + newPosNormLeftX);

                //        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (normLeftColorPointY * colorSensorBufferWidth + normLeftColorPointX);
                //        // assign color value (4 bytes)
                //        *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                //        // overwrite the alpha value (last byte)
                //        *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                //    }

                //    //increment to move along left normal vector
                //    xNormLeft += (float)vNormLeftOrigArm.X;
                //    yNormLeft += (float)vNormLeftOrigArm.Y;
                //    lookupIdxNormLeft = (int)(yNormLeft * bodyIndexSensorBufferWidth + xNormLeft + 0.5);
                //}

                //calculate for right normal of vector
                float xNormRight = xCurrOrigArm + (float)vNormRightOrigArm.X;
                float yNormRight = yCurrOrigArm + (float)vNormRightOrigArm.Y;
                int lookupIdxNormRight = (int)(yNormRight * bodyIndexSensorBufferWidth + xNormRight + 0.5);

                while (ptrBodyIndexSensorBuffer[lookupIdxNormRight] != 0xff)
                {
                    //normal might point inside body
                    if (xNormRight <= xElbow)
                    {
                        continue;
                    }

                    int normRightColorPointX = (int)(ptrDepthToColorSpaceMapper[lookupIdxNormRight].X + 0.5);
                    int normRightColorPointY = (int)(ptrDepthToColorSpaceMapper[lookupIdxNormRight].Y + 0.5);

                    if ((xNormRight < bodyIndexSensorBufferWidth) && (yNormRight < bodyIndexSensorBufferHeight) && (xNormRight >= 0) && (yNormRight >= 0) && (normRightColorPointY < colorSensorBufferHeight) && (normRightColorPointX < colorSensorBufferWidth) &&
                   (normRightColorPointY >= 0) && (normRightColorPointX >= 0))
                    {
                        //write color pixel into position of new arm
                        int newPosNormRightX = (int)(xCurrNewArm + (float)vNormRightNewArm.X + 0.5);
                        int newPosNormRightY = (int)(yCurrNewArm + (float)vNormRightNewArm.Y + 0.5);
                        ptrImgBufferPixelInt = ptrImageBufferInt + (newPosNormRightY * bodyIndexSensorBufferWidth + newPosNormRightX);

                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (normRightColorPointY * colorSensorBufferWidth + normRightColorPointX);
                        // assign color value (4 bytes)
                        *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                        // overwrite the alpha value (last byte)
                        *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                    }

                    //increment to move along right normal vector
                    xNormRight += (float)vNormRightOrigArm.X;
                    yNormRight += (float)vNormRightOrigArm.Y;
                    lookupIdxNormRight = (int)(yNormRight * bodyIndexSensorBufferWidth + xNormRight + 0.5);
                }

                //increment to move along vector
                xCurrOrigArm += (float)(vOrigArm.X) * stepSizeOrigArm;
                yCurrOrigArm += (float)(vOrigArm.Y) * stepSizeOrigArm;
                xCurrNewArm += (float)(vNewArm.X) * stepSizeNewArm;
                yCurrNewArm += (float)(vNewArm.Y) * stepSizeNewArm;
             
            }
            
        }

        private unsafe void transform_LowRes_scaleOnly(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferInt, ColorSpacePoint* ptrDepthToColorSpaceMapper, float xElbow, float yElbow, float xWrist, float yWrist, float xTouch, float yTouch)
        {
            double absAngle = helper.CalculateAbsoluteAngleInDegreeToXaxis(xElbow, yElbow, xWrist, yWrist) / 180.0 * Math.PI;
            double factorX = Math.Cos(absAngle);
            double factorY = Math.Sin(absAngle);
            double testLength = factorX * factorX + factorY * factorY;
            //Console.WriteLine("X: " + factorX + " Y: " + factorY + " Length: " + testLength);
            float factorXsquare = (float)(1.0 + factorX * factorX);
            float factorYsquare = (float)(1.0 + factorY * factorY);
            //factorXsquare = 1.5f;
            //factorYsquare = 1.5f;


            uint transparencyMask = (0xffffff00u | this.userTransparency);

            int unsafecolorSensorBufferWidth = this.colorSensorBufferWidth;
            int unsafecolorSensorBufferHeight = this.colorSensorBufferHeight;

            //save computing power by incrementing x, y without division/modulo
            int xDepthSpace = 0;
            int yDepthSpace = 0;
            uint* ptrColorSensorBufferPixelInt = null;
            uint* ptrImgBufferPixelInt = ptrImageBufferInt;
            int depthSpaceSize = bodyIndexSensorBufferHeight * bodyIndexSensorBufferWidth;

            for (int idxDepthSpace = 0; idxDepthSpace < depthSpaceSize; ++idxDepthSpace)
            {
                ptrColorSensorBufferPixelInt = null;

                if (xDepthSpace > xElbow) // handle right arm
                {
                    // compute stretched point in depth space and transform it in color space for lookup
                    float offsetXdepthSpace = xDepthSpace - xElbow;
                    int lookupXdepthSpace = (int)(xElbow + (offsetXdepthSpace / factorXsquare) + 0.5);

                    float offsetYdepthSpace = yDepthSpace - yElbow;
                    int lookupYdepthSpace = (int)(yElbow + (offsetYdepthSpace / factorYsquare) + 0.5);

                    int idxStretchedDepthSpace = bodyIndexSensorBufferWidth * lookupYdepthSpace + lookupXdepthSpace;

                    if ((idxStretchedDepthSpace < depthSpaceSize) && (ptrBodyIndexSensorBuffer[idxStretchedDepthSpace] != 0xff))
                    {
                        int colorPointXstretched = (int)(ptrDepthToColorSpaceMapper[idxStretchedDepthSpace].X + 0.5);
                        int colorPointYstretched = (int)(ptrDepthToColorSpaceMapper[idxStretchedDepthSpace].Y + 0.5);

                        if ((colorPointYstretched < unsafecolorSensorBufferHeight) && (colorPointXstretched < unsafecolorSensorBufferWidth) &&
                                (colorPointYstretched >= 0) && (colorPointXstretched >= 0))
                        {
                            ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointYstretched * unsafecolorSensorBufferWidth + colorPointXstretched);
                        }
                    }
                }
                else // handle rest of the body
                {
                    if (ptrBodyIndexSensorBuffer[idxDepthSpace] != 0xff)
                    {
                        int colorPointX = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].X + 0.5);
                        int colorPointY = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].Y + 0.5);

                        if ((colorPointY < unsafecolorSensorBufferHeight) && (colorPointX < unsafecolorSensorBufferWidth) &&
                        (colorPointY >= 0) && (colorPointX >= 0))
                        {
                            ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * unsafecolorSensorBufferWidth + colorPointX);
                        }
                    }
                }

                if (ptrColorSensorBufferPixelInt != null)
                {
                    // assign color value (4 bytes); dereferencing + assigning writes into imgBuffer
                    // mask the last byte which contains the alpha value
                    *ptrImgBufferPixelInt = (*ptrColorSensorBufferPixelInt) & transparencyMask;
                }

                // increment counter
                ++ptrImgBufferPixelInt;
                if (++xDepthSpace == bodyIndexSensorBufferWidth)
                {
                    xDepthSpace = 0;
                    ++yDepthSpace;
                }
            } // end for each pixel in depth space
        }

        private unsafe void transform_LowRes_rotationOnly(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferInt, ColorSpacePoint* ptrDepthToColorSpaceMapper, float xElbow, float yElbow, float xWrist, float yWrist, float xHandTip, float yHandTip, float xTouch, float yTouch)
        {
            double rotationAngleInRad = helper.CalculateRotationAngle(xElbow, yElbow, xWrist, yWrist, xTouch, yTouch);
            double cos = Math.Cos(rotationAngleInRad);
            double sin = Math.Sin(rotationAngleInRad);

            uint* ptrImgBufferPixelInt = null; // this is where we want to write the pixel
            uint* ptrColorSensorBufferPixelInt = null;

            Vector areaOffset = new Vector((xWrist - xElbow), (yWrist - yElbow));
            int handOffset = (int)areaOffset.Length / 3;
            int handTipBoundaryX = (int)(xHandTip + 0.5);// +handOffset;
            int handTipBoundaryY = (int)(yHandTip + 0.5);// -handOffset;

            //save computing power by incrementing x, y without division/modulo
            int xDepthSpace = 0;
            int yDepthSpace = 0;

            int depthSpaceSize = bodyIndexSensorBufferHeight * bodyIndexSensorBufferWidth;
            for (int idxDepthSpace = 0; idxDepthSpace < depthSpaceSize; idxDepthSpace++)
            {
                ptrColorSensorBufferPixelInt = null;

                if (ptrBodyIndexSensorBuffer[idxDepthSpace] != 0xff)
                {
                    ptrImgBufferPixelInt = null;

                    #region --- Region of lower arm
                    //TODO determine region of hand with floodfill (start: xWrist/yWrist until xElbow)
                    if (xDepthSpace >= xElbow)
                    {
                        // compute rotation (in target image buffer)
                        int rotatedX = (int)(cos * (xDepthSpace - xElbow) - sin * (yDepthSpace - yElbow) + xElbow + 0.5);
                        int rotatedY = (int)(sin * (xDepthSpace - xElbow) + cos * (yDepthSpace - yElbow) + yElbow + 0.5);

                        if ((rotatedY < bodyIndexSensorBufferHeight) && (rotatedX < bodyIndexSensorBufferWidth) &&
                            (rotatedY >= 0) && (rotatedX >= 0))
                        {
                            ptrImgBufferPixelInt = ptrImageBufferInt + (rotatedY * bodyIndexSensorBufferWidth + rotatedX);
                        }
                    }
                    #endregion // lower arm
                    else
                    {
                        // point to current pixel in image buffer
                        ptrImgBufferPixelInt = ptrImageBufferInt + idxDepthSpace;
                    }

                    if (ptrImgBufferPixelInt != null)
                    {
                        int colorPointX = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].X + 0.5);
                        int colorPointY = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].Y + 0.5);

                        //check boundaries
                        if ((colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) &&
                                (colorPointY >= 0) && (colorPointX >= 0))
                        {
                            ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
                            // assign color value (4 bytes)
                            *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                            // overwrite the alpha value (last byte)
                            *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                        }
                    }
                } //if body

                //increment counter
                if (++xDepthSpace == bodyIndexSensorBufferWidth)
                {
                    xDepthSpace = 0;
                    yDepthSpace++;
                }
            } //for loop
        }
    }
}
