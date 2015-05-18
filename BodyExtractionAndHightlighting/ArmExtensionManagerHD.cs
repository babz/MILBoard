using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;

namespace BodyExtractionAndHightlighting
{
    public class ArmExtensionManagerHD : IArmExtensionManager
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

        public ArmExtensionManagerHD(byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, Dictionary<JointType, Point> armJointPoints, Point pTouch, byte userTransparency)
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

        public unsafe void processImage(byte[] imageBufferHD)
        {
            coordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorToDepthSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferHD = imageBufferHD)
            fixed (DepthSpacePoint* ptrColorToDepthSpaceMapper = colorToDepthSpaceMapper)
            {
                float xElbow = (float)pElbow.X;
                float yElbow = (float)pElbow.Y;
                float xWrist = (float)pWrist.X;
                float yWrist = (float)pWrist.Y;
                float xHandTip = (float)pHandTip.X;
                float yHandTip = (float)pHandTip.Y;
                float xTouch = (float)pTouch.X;
                float yTouch = (float)pTouch.Y;

                uint* ptrImageBufferHDInt = (uint*)ptrImageBufferHD;
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                this.transform_HD(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferHDInt, ptrColorToDepthSpaceMapper, xElbow, yElbow, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

            } //end fixed
        }

        public unsafe void processImage_rotationOnly(byte[] imageBufferHD)
        {
            coordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);
            coordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorToDepthSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferHD = imageBufferHD)
            fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
            fixed (DepthSpacePoint* ptrColorToDepthSpaceMapper = colorToDepthSpaceMapper)
            {
                float xElbow = (float)pElbow.X;
                float yElbow = (float)pElbow.Y;
                float xWrist = (float)pWrist.X;
                float yWrist = (float)pWrist.Y;
                float xHandTip = (float)pHandTip.X;
                float yHandTip = (float)pHandTip.Y;
                float xTouch = (float)pTouch.X;
                float yTouch = (float)pTouch.Y;

                uint* ptrImageBufferHDInt = (uint*)ptrImageBufferHD;
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                this.transform_HD_rotationOnly(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferHDInt, ptrDepthToColorSpaceMapper, ptrColorToDepthSpaceMapper, xElbow, yElbow, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

            } //end fixed
        }

        public unsafe void processImage_scaleOnly(byte[] imageBufferHD)
        {
            coordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);
            coordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorToDepthSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferHD = imageBufferHD)
            fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
            fixed (DepthSpacePoint* ptrColorToDepthSpaceMapper = colorToDepthSpaceMapper)
            {
                float xElbow = (float)pElbow.X;
                float yElbow = (float)pElbow.Y;
                float xWrist = (float)pWrist.X;
                float yWrist = (float)pWrist.Y;
                float xHandTip = (float)pHandTip.X;
                float yHandTip = (float)pHandTip.Y;
                float xTouch = (float)pTouch.X;
                float yTouch = (float)pTouch.Y;

                uint* ptrImageBufferHDInt = (uint*)ptrImageBufferHD;
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                this.transform_HD_scaleOnly(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferHDInt, ptrDepthToColorSpaceMapper, ptrColorToDepthSpaceMapper, xElbow, yElbow, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

            } //end fixed
        }

        private unsafe void transform_HD(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferInt, DepthSpacePoint* ptrColorToDepthSpaceMapper, float xElbow, float yElbow, float xWrist, float yWrist, float xHandTip, float yHandTip, float xTouch, float yTouch)
        {
            //===== CALC_ROTATION_ANGLE ==========

            double rotationAngleInRad = helper.CalculateRotationAngle(xElbow, yElbow, xWrist, yWrist, xTouch, yTouch);
            double cos = Math.Cos(rotationAngleInRad);
            double sin = Math.Sin(rotationAngleInRad);

            //===== CALC_NORMALIZED_ANGLE ==========
            //TODO check if angle need to be calculated from rotated pixel
            //double normalizedAngle = helper.CalculateNormalizedAngle(xElbow, yElbow, xWrist, yWrist);

            uint* ptrImgBufferPixelInt = null; // this is where we want to write the pixel

            uint* ptrColorSensorBufferPixelInt = null;

            Vector areaOffset = new Vector((xWrist - xElbow), (yWrist - yElbow));
            int handOffset = (int)areaOffset.Length / 3;
            int handTipBoundaryX = (int)(xHandTip + handOffset + 0.5);
            int handTipBoundaryY = (int)(yHandTip + 0.5);// -handOffset;

            int lengthOfMapper = colorSensorBufferHeight * colorSensorBufferWidth;
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
                int idxDepthSpace = (int)(bodyIndexSensorBufferWidth * yDepthSpace + xDepthSpace); //2D to 1D

                // color gets assigned to where body index is given (pixel belongs to a body)
                // bodyIndex can be 0, 1, 2, 3, 4, or 5
                //TODO regard only one body
                if (ptrBodyIndexSensorBuffer[idxDepthSpace] != 0xff)
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

                        if ((yDepthSpace_rot >= bodyIndexSensorBufferHeight) || (xDepthSpace_rot >= bodyIndexSensorBufferWidth) ||
                            (yDepthSpace_rot < 0) || (xDepthSpace_rot < 0))
                        {
                            continue;
                        }

                        //rotated pixel
                        ptrImgBufferPixelInt = ptrImageBufferInt + (yDepthSpace_rot * bodyIndexSensorBufferWidth + xDepthSpace_rot);

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

                    int xColorSpace = idxColorSpace % colorSensorBufferWidth;
                    int yColorSpace = (int)(((float)idxColorSpace) / colorSensorBufferWidth + 0.5);

                    /* Overwrite body-index-pixel with color pixel
                     */
                    // corresponding pixel in the 1080p image
                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (yColorSpace * colorSensorBufferWidth + xColorSpace);

                    // assign color value (4 bytes)
                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;

                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;

                } // if pixel belongs to body

            } //end for
        }

        private unsafe void transform_HD_scaleOnly(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferHDInt, ColorSpacePoint* ptrDepthToColorSpaceMapper, DepthSpacePoint* ptrColorToDepthSpaceMapper, float xElbow, float yElbow, float xWrist, float yWrist, float xHandTip, float yHandTip, float xTouch, float yTouch)
        {
            double normalizedAngle = 0.5; // todo helper.CalculateNormalizedAngleToXaxis(xElbow, yElbow, xWrist, yWrist);

            float xElbowColorSpace = ptrDepthToColorSpaceMapper[(int)(yElbow + 0.5) * bodyIndexSensorBufferWidth + (int)(xElbow + 0.5)].X;
            float yElbowColorSpace = ptrDepthToColorSpaceMapper[(int)(yElbow + 0.5) * bodyIndexSensorBufferWidth + (int)(xElbow + 0.5)].Y;

            bool isElbowOutOfBound = false;
            //where the color img cannot be mapped to the depth image, there are infinity values
            if (Single.IsInfinity(xElbowColorSpace) || Single.IsInfinity(yElbowColorSpace))
            {
                isElbowOutOfBound = true;
            }

            uint* ptrImgBufferHDPixelInt = null; // this is where we want to write the pixel
            uint* ptrColorSensorBufferPixelInt = null;

            //save computing power by incrementing x, y without division/modulo
            int xColorSpace = 0;
            int yColorSpace = 0;

            int colorSpaceSize = colorSensorBufferHeight * colorSensorBufferWidth;
            for (int idxColorSpace = 0; idxColorSpace < colorSpaceSize; idxColorSpace++)
            {
                ptrColorSensorBufferPixelInt = null;

                //point to current pixel in target imgBuffer
                ptrImgBufferHDPixelInt = ptrImageBufferHDInt + idxColorSpace;

                if (!isElbowOutOfBound && (xColorSpace >= xElbowColorSpace)) // right arm
                {
                    float offsetX = xColorSpace - xElbowColorSpace;
                    int lookupX = (int)(xElbowColorSpace + (offsetX / (2.0 - normalizedAngle)) + 0.5);

                    float offsetY = yColorSpace - yElbowColorSpace;
                    int lookupY = (int)(yElbowColorSpace + (offsetY / (1.0 + normalizedAngle)) + 0.5);

                    if ((lookupY < colorSensorBufferHeight) && (lookupX < colorSensorBufferWidth) &&
                            (lookupY >= 0) && (lookupX >= 0))
                    {
                        int idxColorSpaceStreched = lookupY * colorSensorBufferWidth + lookupX;
                        float xDepthSpace = ptrColorToDepthSpaceMapper[idxColorSpaceStreched].X;
                        float yDepthSpace = ptrColorToDepthSpaceMapper[idxColorSpaceStreched].Y;

                        //where the color img cannot be mapped to the depth image, there are infinity values
                        if (!Single.IsInfinity(xDepthSpace) && !Single.IsInfinity(yDepthSpace))
                        {
                            int idxDepthSpace = ((int)(yDepthSpace + 0.5)) * bodyIndexSensorBufferWidth + ((int)(xDepthSpace + 0.5));
                            if (ptrBodyIndexSensorBuffer[idxDepthSpace] != 0xff)
                            {
                                // corresponding pixel color sensor buffer (source)
                                ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxColorSpaceStreched; // corresponding pixel in the 1080p image
                            }
                        }
                    }
                } //end: right arm
                else // normal pixel
                {
                    float xDepthSpace = ptrColorToDepthSpaceMapper[idxColorSpace].X;
                    float yDepthSpace = ptrColorToDepthSpaceMapper[idxColorSpace].Y;

                    //where the color img cannot be mapped to the depth image, there are infinity values
                    if (!Single.IsInfinity(xDepthSpace) && !Single.IsInfinity(yDepthSpace))
                    {
                        //corrresponding pixel in the bodyIndexBuffer; mapper returns pixel in depth space
                        int idxDepthSpace = ((int)(yDepthSpace + 0.5)) * bodyIndexSensorBufferWidth + ((int)(xDepthSpace + 0.5)); //2D to 1D
                        if (ptrBodyIndexSensorBuffer[idxDepthSpace] != 0xff)
                        {
                            // corresponding pixel color sensor buffer (source)
                            ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (yColorSpace * this.colorSensorBufferWidth + xColorSpace);
                        }
                    }
                } // end: normal pixel

                if (ptrColorSensorBufferPixelInt != null)
                {
                    // assign color value (4 bytes); dereferencing + assigning writes into imgBuffer
                    *ptrImgBufferHDPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value
                    *(((byte*)ptrImgBufferHDPixelInt) + 3) = this.userTransparency;
                }

                //increment counter
                if (++xColorSpace == colorSensorBufferWidth)
                {
                    xColorSpace = 0;
                    yColorSpace++;
                }
            } //end for
        }

        private unsafe void transform_HD_rotationOnly(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferHDInt, ColorSpacePoint* ptrDepthToColorSpaceMapper, DepthSpacePoint* ptrColorToDepthSpaceMapper, float xElbow, float yElbow, float xWrist, float yWrist, float xHandTip, float yHandTip, float xTouch, float yTouch)
        {
            double rotationAngleInRad = helper.CalculateRotationAngle(xElbow, yElbow, xWrist, yWrist, xTouch, yTouch);
            double cos = Math.Cos(rotationAngleInRad);
            double sin = Math.Sin(rotationAngleInRad);

            float xElbowColorSpace = ptrDepthToColorSpaceMapper[(int)(yElbow + 0.5) * bodyIndexSensorBufferWidth + (int)(xElbow + 0.5)].X;
            float yElbowColorSpace = ptrDepthToColorSpaceMapper[(int)(yElbow + 0.5) * bodyIndexSensorBufferWidth + (int)(xElbow + 0.5)].Y;

            bool isElbowOutOfBound = false;
            //where the color img cannot be mapped to the depth image, there are infinity values
            if (Single.IsInfinity(xElbowColorSpace) || Single.IsInfinity(yElbowColorSpace))
            {
                isElbowOutOfBound = true;
            }

            uint* ptrImgBufferHDPixelInt = null; // this is where we want to write the pixel
            uint* ptrColorSensorBufferPixelInt = null;

            Vector areaOffset = new Vector((xWrist - xElbow), (yWrist - yElbow));
            int handOffset = (int)areaOffset.Length / 3;
            int handTipBoundaryX = (int)(xHandTip + 0.5);// +handOffset;
            int handTipBoundaryY = (int)(yHandTip + 0.5);// -handOffset;

            //save computing power by incrementing x, y without division/modulo
            int xColorSpace = 0;
            int yColorSpace = 0;

            int colorSpaceSize = colorSensorBufferHeight * colorSensorBufferWidth;
            for (int idxColorSpace = 0; idxColorSpace < colorSpaceSize; idxColorSpace++)
            {
                ptrColorSensorBufferPixelInt = null;

                float xDepthSpace = ptrColorToDepthSpaceMapper[idxColorSpace].X;
                float yDepthSpace = ptrColorToDepthSpaceMapper[idxColorSpace].Y;
                int idxDepthSpace = (int)(yDepthSpace + 0.5) * bodyIndexSensorBufferWidth + (int)(xDepthSpace + 0.5);

                if (!Single.IsInfinity(xDepthSpace) && !Single.IsInfinity(yDepthSpace))
                {
                    if (ptrBodyIndexSensorBuffer[idxDepthSpace] != 0xff)
                    {
                        // right arm
                        if (!isElbowOutOfBound && (xColorSpace >= xElbowColorSpace))
                        {

                            // compute rotation (in target image buffer)
                            int xRotatedColorSpace = (int)(cos * (xColorSpace - xElbowColorSpace) - sin * (yColorSpace - yElbowColorSpace) + xElbowColorSpace + 0.5);
                            int yRotatedColorSpace = (int)(sin * (xColorSpace - xElbowColorSpace) + cos * (yColorSpace - yElbowColorSpace) + yElbowColorSpace + 0.5);

                            if ((yRotatedColorSpace < colorSensorBufferHeight) && (xRotatedColorSpace < colorSensorBufferWidth) &&
                                (yRotatedColorSpace >= 0) && (xRotatedColorSpace >= 0))
                            {
                                ptrImgBufferHDPixelInt = ptrImageBufferHDInt + (yRotatedColorSpace * colorSensorBufferWidth + xRotatedColorSpace);
                            }
                        }
                        else //rest of body
                        {
                            //point to current pixel in target imgBuffer
                            ptrImgBufferHDPixelInt = ptrImageBufferHDInt + idxColorSpace;
                        }
                    } //if body
                } //if idxDepth not infinity

                if (ptrImgBufferHDPixelInt != null)
                {
                    //check boundaries
                    if ((yColorSpace < colorSensorBufferHeight) && (xColorSpace < colorSensorBufferWidth) &&
                            (yColorSpace >= 0) && (xColorSpace >= 0))
                    {
                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (yColorSpace * colorSensorBufferWidth + xColorSpace);
                        // assign color value (4 bytes)
                        *ptrImgBufferHDPixelInt = *ptrColorSensorBufferPixelInt;
                        // overwrite the alpha value (last byte)
                        *(((byte*)ptrImgBufferHDPixelInt) + 3) = this.userTransparency;
                    }
                }

                //increment counter
                if (++xColorSpace == colorSensorBufferWidth)
                {
                    xColorSpace = 0;
                    yColorSpace++;
                }
            } //for loop
        }

    }
}
