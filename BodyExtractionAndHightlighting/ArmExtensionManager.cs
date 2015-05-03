﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;

namespace BodyExtractionAndHightlighting
{
    public class ArmExtensionManager : ImageProcessor
    {
        Point pElbow, pWrist, pHandTip, pTouch;

        public ArmExtensionManager(int bodyIndexSensorBufferWidth, int bodyIndexSensorBufferHeight, int colorSensorBufferWidth, int colorSensorBufferHeight, byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, KinectSensor sensor, ushort[] depthDataSource, Dictionary<JointType, Point> armJointPoints, Point pTouch)
            : base(bodyIndexSensorBufferWidth, bodyIndexSensorBufferHeight, colorSensorBufferWidth, colorSensorBufferHeight, bodyIndexSensorBuffer, colorSensorBuffer, sensor, depthDataSource)
        {
            this.pElbow = armJointPoints[JointType.ElbowRight];
            this.pWrist = armJointPoints[JointType.WristRight];
            this.pHandTip = armJointPoints[JointType.HandTipRight];
            this.pTouch = pTouch;
        }

        public unsafe void processImageLowRes(byte[] imageBufferLowRes)
        {
            sensor.CoordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);

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

                this.transform_LowRes(ptrBodyIndexSensorBuffer, ptrColorSensorBuffer, ptrImageBufferLowRes, ptrDepthToColorSpaceMapper, xElbow, yElbow, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);
            } //end fixed
        }

        public unsafe void processImageLowRes_HD_test(byte[] imageBufferLowRes)
        {
            sensor.CoordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorToDepthSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferLowRes = imageBufferLowRes)
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

                this.transform_LowRes_HD_test(ptrBodyIndexSensorBuffer, ptrColorSensorBuffer, ptrImageBufferLowRes, ptrColorToDepthSpaceMapper, xElbow, yElbow, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

            } //end fixed
        }

        public unsafe void processImageHD(byte[] imageBufferHD)
        {
            sensor.CoordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorToDepthSpaceMapper);

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

                this.transform_HD(ptrBodyIndexSensorBuffer, ptrColorSensorBuffer, ptrImageBufferHD, ptrColorToDepthSpaceMapper, xElbow, yElbow, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

            } //end fixed
        }

        //---------- Rotate only, Scale only

        public unsafe void processImage_scaleOnly(byte[] imageBufferLowRes)
        {
            sensor.CoordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (byte* ptrImageBufferLowRes = imageBufferLowRes)
            fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
            {
                float xElbow = (float)pElbow.X;
                float yElbow = (float)pElbow.Y;
                float xTouch = (float)pTouch.X;
                float yTouch = (float)pTouch.Y;

                this.transform_LowRes_scaleOnly(ptrBodyIndexSensorBuffer, ptrColorSensorBuffer, ptrImageBufferLowRes, ptrDepthToColorSpaceMapper, xElbow, yElbow, xTouch, yTouch);

            } //end fixed
        }

        public unsafe void processImage_rotationOnly(byte[] imageBufferLowRes)
        {
            sensor.CoordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);

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

                this.transform_LowRes_rotationOnly(ptrBodyIndexSensorBuffer, ptrColorSensorBuffer, ptrImageBufferLowRes, ptrDepthToColorSpaceMapper, xElbow, yElbow, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch);

            } //end fixed
        }

        #region PRIVATE

        private unsafe void transform_LowRes(byte* ptrBodyIndexSensorBuffer, byte* ptrColorSensorBuffer, byte* ptrImageBuffer, ColorSpacePoint* ptrDepthToColorSpaceMapper, float xElbow, float yElbow, float xWrist, float yWrist, float xHandTip, float yHandTip, float xTouch, float yTouch)
        {

            // ###################################### simple version ###################################
            //===== CALC_ROTATION_ANGLE ==========

            double rotationAngleInRad = helper.CalculateRotationAngle(xElbow, yElbow, xWrist, yWrist, xTouch, yTouch);
            double cos = Math.Cos(rotationAngleInRad);
            double sin = Math.Sin(rotationAngleInRad);

            //===== CALC_NORMALIZED_ANGLE ==========
            //TODO check if angle need to be calculated from rotated pixel
            //double normalizedAngle = this.CalculateNormalizedAngle(xElbow, yElbow, xWrist, yWrist);
            double normalizedAngle = helper.CalculateNormalizedAngle(xElbow, yElbow, xTouch, yTouch);
            normalizedAngle = 0.5;
            //Console.Out.Println("Normalized angle: " + normalizedAngle);

            // compute strech
            float xStretchFactor = Math.Abs((xTouch - xElbow) / (xWrist - xElbow));
            float yStretchFactor = Math.Abs((yTouch - yElbow) / (yWrist - yElbow));

            uint* ptrImgBufferPixelInt = null; // this is where we want to write the pixel
            uint* ptrImageBufferInt = (uint*)ptrImageBuffer;
            uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;
            uint* ptrColorSensorBufferPixelInt = null;

            Vector areaOffset = new Vector((xWrist - xElbow), (yWrist - yElbow));
            int handOffset = (int)areaOffset.Length / 3;
            int handTipBoundaryX = (int)(xHandTip + 0.5);// +handOffset;
            int handTipBoundaryY = (int)(yHandTip + 0.5);// -handOffset;

            int lengthOfMapper = bodyIndexBufferHeight * bodyIndexBufferWidth;
            for (int idxDepthSpace = 0; idxDepthSpace < lengthOfMapper; idxDepthSpace++)
            {
                int xDepthSpace = idxDepthSpace % bodyIndexBufferWidth;
                int yDepthSpace = (int)(((float)idxDepthSpace) / bodyIndexBufferWidth + 0.5);
                ptrColorSensorBufferPixelInt = null;

                #region --- Region of lower arm
                if (xDepthSpace >= xElbow)
                {
                    // calculates the extension factor
                    #region --- Arm SCALE mode ---


                    int offsetX = (int)(xDepthSpace - xElbow); // todo: float ?
                    //int lookupX = (int)(xElbow + (offsetX / (2.0 - normalizedAngle)));
                    int lookupX = (int)(xElbow + (offsetX / xStretchFactor));

                    int offsetY = (int)(yDepthSpace - yElbow); // todo: float ?
                    //int lookupY = (int)(yElbow + (offsetY / (1.0 + normalizedAngle)));
                    int lookupY = (int)(yElbow + (offsetY / yStretchFactor));

                    //lookupX and lookupY must be inside boundaries (bodyIndexWidth/Height)
                    if (lookupX >= bodyIndexBufferWidth || lookupY >= bodyIndexBufferHeight || lookupX < 0 || lookupY < 0)
                    {
                        continue;
                    }

                    int lookupIdx = bodyIndexBufferWidth * lookupY + lookupX;
                    // bodyIndex can be 0, 1, 2, 3, 4, or 5
                    if (ptrBodyIndexSensorBuffer[lookupIdx] != 0xff)
                    {
                        int colorPointX_stretch = (int)(ptrDepthToColorSpaceMapper[lookupIdx].X + 0.5);
                        int colorPointY_stretch = (int)(ptrDepthToColorSpaceMapper[lookupIdx].Y + 0.5);


                        if ((colorPointY_stretch >= colorBufferHeight) || (colorPointX_stretch >= colorBufferWidth) ||
                                (colorPointY_stretch < 0) || (colorPointX_stretch < 0))
                        {
                            continue;
                        }
                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY_stretch * colorBufferWidth + colorPointX_stretch);
                    }
                    else
                    {
                        continue; // todo: required ?
                    }
                    #endregion
                    #region comment
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

                    // compute rotation (in target image buffer)
                    int rotatedX = (int)(cos * (xDepthSpace - xElbow) - sin * (yDepthSpace - yElbow) + xElbow + 0.5);
                    int rotatedY = (int)(sin * (xDepthSpace - xElbow) + cos * (yDepthSpace - yElbow) + yElbow + 0.5);
                    ptrImgBufferPixelInt = ptrImageBufferInt + (rotatedY * bodyIndexBufferWidth + rotatedX);
                }
                #endregion // lower arm
                else
                {
                    if (ptrBodyIndexSensorBuffer[idxDepthSpace] != 0xff)
                    {
                        // point to current pixel in image buffer
                        ptrImgBufferPixelInt = ptrImageBufferInt + idxDepthSpace;
                    }
                    else
                    {
                        continue;
                    }
                }

                // nomal color lookup if no strech is applied
                if (ptrColorSensorBufferPixelInt == null)
                {
                    int colorPointX = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].X + 0.5);
                    int colorPointY = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].Y + 0.5);

                    //check boundaries
                    if ((colorPointY >= colorBufferHeight) || (colorPointX >= colorBufferWidth) ||
                            (colorPointY < 0) || (colorPointX < 0))
                    {
                        continue;
                    }
                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorBufferWidth + colorPointX);
                }

                // assign color value (4 bytes)
                *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;

                // overwrite the alpha value (last byte)
                *(((byte*)ptrImgBufferPixelInt) + 3) = base.userTransparency;
            } //end for   
        }

        private unsafe void transform_LowRes_HD_test(byte* ptrBodyIndexSensorBuffer, byte* ptrColorSensorBuffer, byte* ptrImageBuffer, DepthSpacePoint* ptrColorToDepthSpaceMapper, float xElbow, float yElbow, float xWrist, float yWrist, float xHandTip, float yHandTip, float xTouch, float yTouch)
        {
            // ###################################### HD version ###################################
            //===== CALC_ROTATION_ANGLE ==========

            double rotationAngleInRad = helper.CalculateRotationAngle(xElbow, yElbow, xWrist, yWrist, xTouch, yTouch);
            double cos = Math.Cos(rotationAngleInRad);
            double sin = Math.Sin(rotationAngleInRad);

            //===== CALC_NORMALIZED_ANGLE ==========
            //TODO check if angle need to be calculated from rotated pixel
            double normalizedAngle = helper.CalculateNormalizedAngle(xElbow, yElbow, xWrist, yWrist);

            uint* ptrImgBufferPixelInt = null; // this is where we want to write the pixel
            uint* ptrImageBufferInt = (uint*)ptrImageBuffer;
            uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;
            uint* ptrColorSensorBufferPixelInt = null;

            Vector areaOffset = new Vector((xWrist - xElbow), (yWrist - yElbow));
            int handOffset = (int)areaOffset.Length / 3;
            int handTipBoundaryX = (int)(xHandTip + handOffset);
            int handTipBoundaryY = (int)yHandTip;// -handOffset;

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

                int xDepthSpace = (int)(xDepthSpaceF + 0.5);
                int yDepthSpace = (int)(yDepthSpaceF + 0.5);

                //corrresponding pixel in the bodyIndexBuffer; mapper returns pixel in depth space
                int idxDepthSpace = (int)(bodyIndexBufferWidth * yDepthSpace + xDepthSpace); //2D to 1D

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
                    int yColorSpace = (int)(((float)idxColorSpace) / colorBufferWidth + 0.5);

                    // Overwrite body-index-pixel with color pixel

                    // corresponding pixel in the 1080p image
                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (yColorSpace * colorBufferWidth + xColorSpace);

                    // assign color value (4 bytes)
                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;

                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrImgBufferPixelInt) + 3) = base.userTransparency;

                } // if pixel belongs to body

            } //end for

        }

        private unsafe void transform_HD(byte* ptrBodyIndexSensorBuffer, byte* ptrColorSensorBuffer, byte* ptrImageBufferHD, DepthSpacePoint* ptrColorToDepthSpaceMapper, float xElbow, float yElbow, float xWrist, float yWrist, float xHandTip, float yHandTip, float xTouch, float yTouch)
        {
            //===== CALC_ROTATION_ANGLE ==========

            double rotationAngleInRad = helper.CalculateRotationAngle(xElbow, yElbow, xWrist, yWrist, xTouch, yTouch);
            double cos = Math.Cos(rotationAngleInRad);
            double sin = Math.Sin(rotationAngleInRad);

            //===== CALC_NORMALIZED_ANGLE ==========
            //TODO check if angle need to be calculated from rotated pixel
            double normalizedAngle = helper.CalculateNormalizedAngle(xElbow, yElbow, xWrist, yWrist);

            uint* ptrImgBufferPixelInt = null; // this is where we want to write the pixel
            uint* ptrImageBufferInt = (uint*)ptrImageBufferHD;
            uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;
            uint* ptrColorSensorBufferPixelInt = null;

            Vector areaOffset = new Vector((xWrist - xElbow), (yWrist - yElbow));
            int handOffset = (int)areaOffset.Length / 3;
            int handTipBoundaryX = (int)(xHandTip + handOffset + 0.5);
            int handTipBoundaryY = (int)(yHandTip + 0.5);// -handOffset;

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
                    int yColorSpace = (int)(((float)idxColorSpace) / colorBufferWidth + 0.5);

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

        //---------- Rotate only, Scale only

        private unsafe void transform_LowRes_scaleOnly(byte* ptrBodyIndexSensorBuffer, byte* ptrColorSensorBuffer, byte* ptrImageBuffer, ColorSpacePoint* ptrDepthToColorSpaceMapper, float xElbow, float yElbow, float xTouch, float yTouch)
        {
            double normalizedAngle = helper.CalculateNormalizedAngle(xElbow, yElbow, xTouch, yTouch);

            int lengthOfMapper = bodyIndexBufferHeight * bodyIndexBufferWidth;
            for (int idxDepthSpace = 0; idxDepthSpace < lengthOfMapper; idxDepthSpace++)
            {
                bool extendedHandDrawn = false;

                int xDepthSpace = idxDepthSpace % bodyIndexBufferWidth;
                int yDepthSpace = (int)(((float)idxDepthSpace) / bodyIndexBufferWidth + 0.5);

                if (xDepthSpace > xElbow)
                {
                    extendedHandDrawn = true; // hack

                    int offsetX = (int)(xDepthSpace - xElbow);
                    int lookupX = (int)(xElbow + (offsetX / (2.0 - normalizedAngle)));

                    int offsetY = (int)(yDepthSpace - yElbow);
                    int lookupY = (int)(yElbow + (offsetY / (1.0 + normalizedAngle)));

                    int colorPointX_stretch = (int)(ptrDepthToColorSpaceMapper[bodyIndexBufferWidth * lookupY + lookupX].X + 0.5);
                    int colorPointY_stretch = (int)(ptrDepthToColorSpaceMapper[bodyIndexBufferWidth * lookupY + lookupX].Y + 0.5);
                    uint* intPtr_stretch = (uint*)(ptrImageBuffer + idxDepthSpace * 4); //stays the same

                    if ((ptrBodyIndexSensorBuffer[bodyIndexBufferWidth * lookupY + lookupX] != 0xff) &&
                        (colorPointY_stretch < colorBufferHeight) && (colorPointX_stretch < colorBufferWidth) &&
                        (colorPointY_stretch >= 0) && (colorPointX_stretch >= 0))
                    {
                        uint* intPtr1080p = (uint*)(ptrColorSensorBuffer + (colorPointY_stretch * colorBufferWidth + colorPointX_stretch) * 4); // corresponding pixel in the 1080p image
                        *intPtr_stretch = *intPtr1080p; // assign color value (4 bytes)
                        *(((byte*)intPtr_stretch) + 3) = (byte)userTransparency; // overwrite the alpha value                                            
                    }
                } //end if pixel on right side of xElbow
                if (extendedHandDrawn)
                {
                    continue;
                }

                int colorPointX = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].X + 0.5);
                int colorPointY = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].Y + 0.5);
                uint* intPtr = (uint*)(ptrImageBuffer + idxDepthSpace * 4);

                if ((ptrBodyIndexSensorBuffer[idxDepthSpace] != 0xff) &&
                    (colorPointY < colorBufferHeight) && (colorPointX < colorBufferWidth) &&
                    (colorPointY >= 0) && (colorPointX >= 0))
                {
                    uint* intPtr1080p = (uint*)(ptrColorSensorBuffer + (colorPointY * colorBufferWidth + colorPointX) * 4); // corresponding pixel in the 1080p image
                    *intPtr = *intPtr1080p; // assign color value (4 bytes)
                    *(((byte*)intPtr) + 3) = (byte)userTransparency; // overwrite the alpha value
                }
            } //end for
        }

        private unsafe void transform_LowRes_rotationOnly(byte* ptrBodyIndexSensorBuffer, byte* ptrColorSensorBuffer, byte* ptrImageBuffer, ColorSpacePoint* ptrDepthToColorSpaceMapper, float xElbow, float yElbow, float xWrist, float yWrist, float xHandTip, float yHandTip, float xTouch, float yTouch)
        {
            double rotationAngleInRad = helper.CalculateRotationAngle(xElbow, yElbow, xWrist, yWrist, xTouch, yTouch);
            double cos = Math.Cos(rotationAngleInRad);
            double sin = Math.Sin(rotationAngleInRad);

            uint* ptrImgBufferPixelInt = null; // this is where we want to write the pixel
            uint* ptrImageBufferInt = (uint*)ptrImageBuffer;
            uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;
            uint* ptrColorSensorBufferPixelInt = null;

            Vector areaOffset = new Vector((xWrist - xElbow), (yWrist - yElbow));
            int handOffset = (int)areaOffset.Length / 3;
            int handTipBoundaryX = (int)(xHandTip + 0.5);// +handOffset;
            int handTipBoundaryY = (int)(yHandTip + 0.5);// -handOffset;

            int lengthOfMapper = bodyIndexBufferHeight * bodyIndexBufferWidth;
            for (int idxDepthSpace = 0; idxDepthSpace < lengthOfMapper; idxDepthSpace++)
            {
                int xDepthSpace = idxDepthSpace % bodyIndexBufferWidth;
                int yDepthSpace = (int)(((float)idxDepthSpace) / bodyIndexBufferWidth + 0.5);
                ptrColorSensorBufferPixelInt = null;

                #region --- Region of lower arm
                if ((xDepthSpace >= xElbow) && (xDepthSpace <= handTipBoundaryX) &&
                        (((handTipBoundaryY <= yElbow) && (yDepthSpace >= handTipBoundaryY) && (yDepthSpace <= yElbow)) ||
                            ((handTipBoundaryY > yElbow) && (yDepthSpace >= yElbow) && (yDepthSpace <= handTipBoundaryY)))
                        )
                {
                    // compute rotation (in target image buffer)
                    int rotatedX = (int)(cos * (xDepthSpace - xElbow) - sin * (yDepthSpace - yElbow) + xElbow + 0.5);
                    int rotatedY = (int)(sin * (xDepthSpace - xElbow) + cos * (yDepthSpace - yElbow) + yElbow + 0.5);
                    ptrImgBufferPixelInt = ptrImageBufferInt + (rotatedY * bodyIndexBufferWidth + rotatedX);
                }
                #endregion // lower arm
                else
                {
                    if (ptrBodyIndexSensorBuffer[idxDepthSpace] != 0xff)
                    {
                        // point to current pixel in image buffer
                        ptrImgBufferPixelInt = ptrImageBufferInt + idxDepthSpace;
                    }
                    else
                    {
                        continue;
                    }
                }

                // nomal color lookup if no strech is applied
                if (ptrColorSensorBufferPixelInt == null)
                {
                    int colorPointX = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].X + 0.5);
                    int colorPointY = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].Y + 0.5);

                    //check boundaries
                    if ((colorPointY >= colorBufferHeight) || (colorPointX >= colorBufferWidth) ||
                            (colorPointY < 0) || (colorPointX < 0))
                    {
                        continue;
                    }
                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorBufferWidth + colorPointX);
                }

                // assign color value (4 bytes)
                *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;

                // overwrite the alpha value (last byte)
                *(((byte*)ptrImgBufferPixelInt) + 3) = base.userTransparency;
            } //for loop
        }

        #endregion
    }
}
