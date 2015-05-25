using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;
using System.Windows.Media;
using System.Windows.Shapes;

namespace BodyExtractionAndHightlighting
{
    public class ArmExtensionManagerLowRes : IArmExtensionManager
    {
        private int bodyIndexSensorBufferWidth, bodyIndexSensorBufferHeight, colorSensorBufferWidth, colorSensorBufferHeight;
        private byte[] bodyIndexSensorBuffer, colorSensorBuffer;

        private CoordinateMapper coordinateMapper;
        private ushort[] depthDataSource;

        private unsafe Point pElbow, pWrist, pHandTip, pTouch, pHand, pShoulder;
        private unsafe int xElbow, yElbow, xWrist, yWrist, xHandTip, yHandTip, xTouch, yTouch, xHand, yHand, xShoulder, yShoulder;
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
            this.pHand = armJointPoints[JointType.HandRight];
            this.pShoulder = armJointPoints[JointType.ShoulderRight];
            this.pTouch = pTouch;

            this.xElbow = (int)(pElbow.X + 0.5);
            this.yElbow = (int)(pElbow.Y + 0.5);
            this.xWrist = (int)(pWrist.X + 0.5);
            this.yWrist = (int)(pWrist.Y + 0.5);
            this.xHand = (int)(pHand.X + 0.5);
            this.yHand = (int)(pHand.Y + 0.5);
            this.xHandTip = (int)(pHandTip.X + 0.5);
            this.yHandTip = (int)(pHandTip.Y + 0.5);
            this.xTouch = (int)(pTouch.X + 0.5);
            this.yTouch = (int)(pTouch.Y + 0.5);
            this.xShoulder = (int)(pShoulder.X + 0.5);
            this.yShoulder = (int)(pShoulder.Y + 0.5);

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
                uint* ptrImageBufferInt = (uint*)ptrImageBufferLowRes;
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                this.drawBodyWithoutRightHand(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferInt, ptrDepthToColorSpaceMapper);
                //this.drawBodyfloodfill((xElbow - 1), yElbow, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);

                //start point is (xElbow + 1)
                //this.detectRightLowerArm((xElbow + 1), yElbow, vElbowWristOrig, xElbow, yElbow, xWrist, yWrist, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);


                 //upper boundary: normal vector of wrist
                Vector vElbowWristOrig = new Vector((pWrist.X - pElbow.X), (pWrist.Y - pElbow.Y));

                //lower boundary: half vector of shoulder and wrist
                // H = (S+W) / |(S+W)|
                Vector vElbowShoulder = new Vector((pShoulder.X - pElbow.X), (pShoulder.Y - pElbow.Y));
                Vector vHalfShoulderWrist = new Vector((vElbowShoulder.X + vElbowWristOrig.X), (vElbowShoulder.Y + vElbowWristOrig.Y));
                vHalfShoulderWrist.Normalize();
                Vector vHalfShoulderWrist_NormRight = new Vector(-(vHalfShoulderWrist.Y), vHalfShoulderWrist.X); 

                this.drawStretchedRightLowerArm(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferInt, ptrDepthToColorSpaceMapper, vElbowWristOrig, vHalfShoulderWrist_NormRight);

                int xOffset = (int)(pTouch.X - pHandTip.X + 0.5);
                int yOffset = (int)(pTouch.Y - pHandTip.Y + 0.5);

                this.drawTranslatedRightHand(xHand, yHand, vElbowWristOrig, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
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
                float xElbowF = (float)pElbow.X;
                float yElbowF = (float)pElbow.Y;
                float xWristF = (float)pWrist.X;
                float yWristF = (float)pWrist.Y;
                float xHandTipF = (float)pHandTip.X;
                float yHandTipF = (float)pHandTip.Y;
                float xTouchF = (float)pTouch.X;
                float yTouchF = (float)pTouch.Y;

                uint* ptrImageBufferInt = (uint*)ptrImageBufferLowRes;
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                this.transform_LowRes_rotationOnly(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferInt, ptrDepthToColorSpaceMapper, xElbowF, yElbowF, xWristF, yWristF, xHandTipF, yHandTipF, xTouchF, yTouchF);

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
                float xElbowF = (float)pElbow.X;
                float yElbowF = (float)pElbow.Y;
                float xTouchF = (float)pTouch.X;
                float yTouchF = (float)pTouch.Y;
                float xWristF = (float)pWrist.X;
                float yWristF = (float)pWrist.Y;

                uint* ptrImageBufferInt = (uint*)ptrImageBufferLowRes;
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

                this.transform_LowRes_scaleOnly(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferInt, ptrDepthToColorSpaceMapper, xElbowF, yElbowF, xWristF, yWristF, xTouchF, yTouchF);

            } //end fixed
        }

        private unsafe void drawBodyWithoutRightHand(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferInt, ColorSpacePoint* ptrDepthToColorSpaceMapper)
        {
            //pixel target
            uint* ptrImgBufferPixelInt = null;
            uint* ptrColorSensorBufferPixelInt = null;

            //save computing power by incrementing x, y without division/modulo
            int xDepthSpace = 0;
            int yDepthSpace = 0;

            //==draw whole body without manipulation
            int depthSpaceSize = bodyIndexSensorBufferHeight * bodyIndexSensorBufferWidth;
            for (int idxDepthSpace = 0; idxDepthSpace < depthSpaceSize; idxDepthSpace++)
            {
                //ptrColorSensorBufferPixelInt = null;
                ptrImgBufferPixelInt = null;
                bool isColorPixelInValidRange = false;

                //draw until xElbow
                if ((ptrBodyIndexSensorBuffer[idxDepthSpace] != 0xff) && (xDepthSpace <= xElbow))
                {
                    int colorPointX = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].X + 0.5);
                    int colorPointY = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].Y + 0.5);

                    if ((colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) &&
                            (colorPointY >= 0) && (colorPointX >= 0))
                    {
                        isColorPixelInValidRange = true;
                    }

                    if (isColorPixelInValidRange)
                    {
                        // point to current pixel in image buffer
                        ptrImgBufferPixelInt = ptrImageBufferInt + idxDepthSpace;

                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
                        // assign color value (4 bytes)
                        *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                        // overwrite the alpha value (last byte)
                        *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
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

        private unsafe void drawBodyfloodfill(int xStart, int yStart, byte* ptrBodyIndexSensorBuffer, uint* ptrImageBufferInt, uint* ptrColorSensorBufferInt, ColorSpacePoint* ptrDepthToColorSpaceMapper)
        {
            if ((xStart >= pElbow.X) || (xStart >= bodyIndexSensorBufferWidth) || (xStart < 0) || (yStart >= bodyIndexSensorBufferHeight) || (yStart < 0))
            {
                return;
            }

            //pixel target
            uint* ptrImgBufferPixelInt = null;
            uint* ptrColorSensorBufferPixelInt = null;
            bool isColorPixelInValidRange = false;

            //pixel already visited
            int idxDepthSpace = yStart * bodyIndexSensorBufferWidth + xStart;
            if (ptrBodyIndexSensorBuffer[idxDepthSpace] == 0xff)
            {
                return;
            }
            else
            {
                ptrBodyIndexSensorBuffer[idxDepthSpace] = 0xff; //do not visit same pixel twice
                int colorPointX = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].X + 0.5);
                int colorPointY = (int)(ptrDepthToColorSpaceMapper[idxDepthSpace].Y + 0.5);

                if ((colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) &&
                        (colorPointY >= 0) && (colorPointX >= 0))
                {
                    isColorPixelInValidRange = true;
                }

                if (isColorPixelInValidRange)
                {
                    // point to current pixel in image buffer
                    ptrImgBufferPixelInt = ptrImageBufferInt + idxDepthSpace;

                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
                    // assign color value (4 bytes)
                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                }
            }

            //8-way neighbourhood to visit all pixels of hand (can have background pixel btw fingers)
            this.drawBodyfloodfill((xStart + 1), yStart, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
            this.drawBodyfloodfill((xStart - 1), yStart, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
            this.drawBodyfloodfill(xStart, (yStart + 1), ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
            this.drawBodyfloodfill(xStart, (yStart - 1), ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
            this.drawBodyfloodfill((xStart - 1), (yStart - 1), ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
            this.drawBodyfloodfill((xStart - 1), (yStart + 1), ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
            this.drawBodyfloodfill((xStart + 1), (yStart - 1), ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
            this.drawBodyfloodfill((xStart + 1), (yStart + 1), ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
        }
        //private unsafe void detectRightLowerArm(int xStart, int yStart, Vector vElbowWristOrig, int xElbow, int yElbow, int xWrist, int yWrist, byte* ptrBodyIndexSensorBuffer, uint* ptrImageBufferInt, uint* ptrColorSensorBufferInt, ColorSpacePoint* ptrDepthToColorSpaceMapper)
        //{
        //     //xEnd is left outer boundary
        //    if ((xStart >= bodyIndexSensorBufferWidth) || (xStart < 0) || (yStart >= bodyIndexSensorBufferHeight) || (yStart < 0))
        //    {
        //        return;
        //    }

        //    //boundary: normal vector of wrist/elbow; no normalization necessary!! (only signum is of value)
        //    int vPointElbowX = xStart - xElbow;
        //    int vPointElbowY = yStart - yElbow;
        //    int sigPointElbow = ((int)(vElbowWristOrig.X + 0.5)) * vPointElbowX + ((int)(vElbowWristOrig.Y + 0.5)) * vPointElbowY;

        //    int vPointWristX = xStart - xWrist;
        //    int vPointWristY = yStart - yWrist;
        //    int sigPointWrist = ((int)(vElbowWristOrig.X + 0.5)) * vPointWristX + ((int)(vElbowWristOrig.Y + 0.5)) * vPointWristY;
        //    //boundary lower arm: p >= Elbow && p < Wrist
        //    if ((sigPointElbow < 0) && (sigPointWrist >= 0))
        //    {
        //        return;
        //    }

        //    //pixel already visited
        //    int depthLookup = yStart * bodyIndexSensorBufferWidth + xStart;
        //    if (ptrBodyIndexSensorBuffer[depthLookup] == 0xff)
        //    {
        //        return;
        //    }
        //    else
        //    {
        //        ptrBodyIndexSensorBuffer[depthLookup] = 0xff; //do not visit same pixel twice
        //        //DO call function to draw stretch arm
                   
        //    }

        //    //8-way neighbourhood to visit all pixels of hand (can have background pixel btw fingers)
        //    this.detectRightLowerArm((xStart + 1), yStart, vElbowWristOrig, xElbow, yElbow, xWrist, yWrist, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
        //    this.detectRightLowerArm((xStart - 1), yStart, vElbowWristOrig, xElbow, yElbow, xWrist, yWrist, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
        //    this.detectRightLowerArm(xStart, (yStart + 1), vElbowWristOrig, xElbow, yElbow, xWrist, yWrist, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
        //    this.detectRightLowerArm(xStart, (yStart - 1), vElbowWristOrig, xElbow, yElbow, xWrist, yWrist, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
        //    this.detectRightLowerArm((xStart - 1), (yStart - 1), vElbowWristOrig, xElbow, yElbow, xWrist, yWrist, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
        //    this.detectRightLowerArm((xStart - 1), (yStart + 1), vElbowWristOrig, xElbow, yElbow, xWrist, yWrist, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
        //    this.detectRightLowerArm((xStart + 1), (yStart - 1), vElbowWristOrig, xElbow, yElbow, xWrist, yWrist, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
        //    this.detectRightLowerArm((xStart + 1), (yStart + 1), vElbowWristOrig, xElbow, yElbow, xWrist, yWrist, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
        //}

        private unsafe void drawStretchedRightLowerArm(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferInt, ColorSpacePoint* ptrDepthToColorSpaceMapper, Vector vElbowWristOrig, Vector vHalfShoulderWrist_NormRight)
        {
            #region Draw lower arm STRETCHED

            uint* ptrImgBufferPixelInt = null; // new position where the color is written into
            uint* ptrColorSensorBufferPixelInt = null; // color pixel position in color frame

            //FROM ELBOW TO WRIST
            // v = (x, y)
            float vOrigArmLength = (float)vElbowWristOrig.Length;
            vElbowWristOrig.Normalize();

            //NOTE vector normals different as coordinate system origin (0,0) is upper left corner!
            //v_nleft = (y, -x)
            Vector vNormLeftOrigArm = new Vector(vElbowWristOrig.Y, -(vElbowWristOrig.X));
            //v_nright = (-y, x)
            Vector vNormRightOrigArm = new Vector(-(vElbowWristOrig.Y), vElbowWristOrig.X);

            Vector vNewArm = new Vector((xTouch - xElbow), (yTouch - yElbow));
            float vNewArmLength = (float)vNewArm.Length;
            vNewArm.Normalize();

            Vector vNormLeftNewArm = new Vector(vNewArm.Y, -(vNewArm.X));
            Vector vNormRightNewArm = new Vector(-(vNewArm.Y), vNewArm.X);

            //sampling rate
            float totalSteps;
            //TODO think about factor
            if (vOrigArmLength < vNewArmLength)
                totalSteps = (float)(vNewArmLength * 1.2);
            else
                totalSteps = (float)(vOrigArmLength * 1.2);
            float stepSizeOrigArm = vOrigArmLength / totalSteps;
            float stepSizeNewArm = vNewArmLength / totalSteps;

            float xCurrOrigArm = xElbow;
            float yCurrOrigArm = yElbow;
            float xCurrNewArm = xElbow;
            float yCurrNewArm = yElbow;
            for (int i = 0; i < totalSteps; i++)
            {
                int colorLookup = (int)(bodyIndexSensorBufferWidth * (int)(yCurrOrigArm + 0.5) + xCurrOrigArm + 0.5);
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
                    //TODO draw yellow line where vector is
                    *ptrImgBufferPixelInt = 0xFF00FFFF; //*ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    //*(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;

                    //TODO
                    //draw red line where orig hand vector was
                    ptrImgBufferPixelInt = ptrImageBufferInt + (int)((int)(yCurrOrigArm + 0.5) * bodyIndexSensorBufferWidth + xCurrOrigArm + 0.5);
                    *ptrImgBufferPixelInt = 0xFFFF0000; //ARGB in storage

                    *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                }
                
                //calculate for left normal of vector
                float xNormLeft = xCurrOrigArm + (float)vNormLeftOrigArm.X;
                float yNormLeft = yCurrOrigArm + (float)vNormLeftOrigArm.Y;
                int lookupIdxNormLeft = (int)((int)(yNormLeft + 0.5) * bodyIndexSensorBufferWidth + xNormLeft + 0.5);
                float newPosNormLeftX = xCurrNewArm + (float)vNormLeftNewArm.X;
                float newPosNormLeftY = yCurrNewArm + (float)vNormLeftNewArm.Y;
                while (ptrBodyIndexSensorBuffer[lookupIdxNormLeft] != 0xff)
                {
                    //normal might point inside body
                    //lower boundary: half vector of shoulder and wrist
                    int vPointElbowX = (int)(xNormLeft - xElbow + 0.5);
                    int vPointElbowY = (int)(yNormLeft - yElbow + 0.5);
                    int sigPointElbow = ((int)(vHalfShoulderWrist_NormRight.X)) * vPointElbowX + ((int)(vHalfShoulderWrist_NormRight.Y)) * vPointElbowY;
                    //upper boundary: normal vector of wrist
                    int vPointWristX = (int)(xNormLeft - xWrist + 0.5);
                    int vPointWristY = (int)(yNormLeft - yWrist + 0.5);
                    int sigPointWrist = ((int)(vElbowWristOrig.X)) * vPointWristX + ((int)(vElbowWristOrig.Y)) * vPointWristY;
                    //point is not drawn if p < Elbow AND p >= Wrist
                    if ((sigPointElbow < 0) || (sigPointWrist >= 0))
                    {
                        break;
                    }

                    int normLeftColorPointX = (int)(ptrDepthToColorSpaceMapper[lookupIdxNormLeft].X + 0.5);
                    int normLeftColorPointY = (int)(ptrDepthToColorSpaceMapper[lookupIdxNormLeft].Y + 0.5);

                    if ((xNormLeft < bodyIndexSensorBufferWidth) && (yNormLeft < bodyIndexSensorBufferHeight) && (xNormLeft >= 0) && (yNormLeft >= 0) && (normLeftColorPointY < colorSensorBufferHeight) && (normLeftColorPointX < colorSensorBufferWidth) &&
                   (normLeftColorPointY >= 0) && (normLeftColorPointX >= 0))
                    {
                        //write color pixel into position of new arm
                        ptrImgBufferPixelInt = ptrImageBufferInt + (int)((int)(newPosNormLeftY + 0.5) * bodyIndexSensorBufferWidth + newPosNormLeftX + 0.5);

                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (normLeftColorPointY * colorSensorBufferWidth + normLeftColorPointX);
                        // assign color value (4 bytes)
                        *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                        // overwrite the alpha value (last byte)
                        *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                    }

                    //increment to move along left normal vector
                    xNormLeft += (float)vNormLeftOrigArm.X;
                    yNormLeft += (float)vNormLeftOrigArm.Y;
                    newPosNormLeftX += (float)vNormLeftNewArm.X;
                    newPosNormLeftY += (float)vNormLeftNewArm.Y;
                    lookupIdxNormLeft = (int)((int)(yNormLeft + 0.5) * bodyIndexSensorBufferWidth + xNormLeft + 0.5);
                }

                //calculate for right normal of vector
                float xNormRight = xCurrOrigArm + (float)vNormRightOrigArm.X;
                float yNormRight = yCurrOrigArm + (float)vNormRightOrigArm.Y;
                int lookupIdxNormRight = (int)((int)(yNormRight + 0.5) * bodyIndexSensorBufferWidth + xNormRight + 0.5);
                float newPosNormRightX = xCurrNewArm + (float)vNormRightNewArm.X;
                float newPosNormRightY = yCurrNewArm + (float)vNormRightNewArm.Y;
                while (ptrBodyIndexSensorBuffer[lookupIdxNormRight] != 0xff)
                {
                    //normal might point inside body
                    //lower boundary: half vector of shoulder and wrist
                    int vPointElbowX = (int)(xNormRight - xElbow + 0.5);
                    int vPointElbowY = (int)(yNormRight - yElbow + 0.5);
                    int sigPointElbow = ((int)(vHalfShoulderWrist_NormRight.X)) * vPointElbowX + ((int)(vHalfShoulderWrist_NormRight.Y)) * vPointElbowY;
                    //upper boundary: normal vector of wrist
                    int vPointWristX = (int)(xNormRight - xWrist + 0.5);
                    int vPointWristY = (int)(yNormRight - yWrist + 0.5);
                    int sigPointWrist = ((int)(vElbowWristOrig.X)) * vPointWristX + ((int)(vElbowWristOrig.Y)) * vPointWristY;
                    //point is not drawn if p < Elbow AND p >= Wrist
                    if ((sigPointElbow < 0) || (sigPointWrist >= 0))
                    {
                        break;
                    }

                    int normRightColorPointX = (int)(ptrDepthToColorSpaceMapper[lookupIdxNormRight].X + 0.5);
                    int normRightColorPointY = (int)(ptrDepthToColorSpaceMapper[lookupIdxNormRight].Y + 0.5);

                    if ((xNormRight < bodyIndexSensorBufferWidth) && (yNormRight < bodyIndexSensorBufferHeight) && (xNormRight >= 0) && (yNormRight >= 0) && (normRightColorPointY < colorSensorBufferHeight) && (normRightColorPointX < colorSensorBufferWidth) &&
                   (normRightColorPointY >= 0) && (normRightColorPointX >= 0))
                    {
                        //write color pixel into position of new arm
                        ptrImgBufferPixelInt = ptrImageBufferInt + (int)((int)(newPosNormRightY + 0.5) * bodyIndexSensorBufferWidth + newPosNormRightX + 0.5);

                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (normRightColorPointY * colorSensorBufferWidth + normRightColorPointX);
                        // assign color value (4 bytes)
                        *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                        // overwrite the alpha value (last byte)
                        *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                    }

                    //increment to move along right normal vector
                    xNormRight += (float)vNormRightOrigArm.X;
                    yNormRight += (float)vNormRightOrigArm.Y;
                    newPosNormRightX += (float)vNormRightNewArm.X;
                    newPosNormRightY += (float)vNormRightNewArm.Y;
                    lookupIdxNormRight = (int)((int)(yNormRight + 0.5) * bodyIndexSensorBufferWidth + xNormRight + 0.5);
                }

                //increment to move along vector; use normal vectors for increment
                xCurrOrigArm += (float)(vElbowWristOrig.X) * stepSizeOrigArm;
                yCurrOrigArm += (float)(vElbowWristOrig.Y) * stepSizeOrigArm;
                xCurrNewArm += (float)(vNewArm.X) * stepSizeNewArm;
                yCurrNewArm += (float)(vNewArm.Y) * stepSizeNewArm;

            }

            #endregion
        }

        private unsafe void drawTranslatedRightHand(int xStart, int yStart, Vector vElbowWristOrig, int xOffset, int yOffset, byte* ptrBodyIndexSensorBuffer, uint* ptrImageBufferInt, uint* ptrColorSensorBufferInt, ColorSpacePoint* ptrDepthToColorSpaceMapper)
        {
            //xEnd is left outer boundary
            if ((xStart >= bodyIndexSensorBufferWidth) || (xStart < 0) || (yStart >= bodyIndexSensorBufferHeight) || (yStart < 0))
            {
                return;
            }

            //boundary: normal vector of wrist; no normalization necessary!! (only signum is of value)
            int vPointWristX = xStart - xWrist;
            int vPointWristY = yStart - yWrist;
            int sig = ((int)(vElbowWristOrig.X + 0.5)) * vPointWristX + ((int)(vElbowWristOrig.Y + 0.5)) * vPointWristY;
            //point ON line counts to hand
            if (sig < 0)
            {
                return;
            }

            //pixel already visited
            int depthLookup = yStart * bodyIndexSensorBufferWidth + xStart;
            if (ptrBodyIndexSensorBuffer[depthLookup] == 0xff)
            {
                return;
            }
            else
            {
                ptrBodyIndexSensorBuffer[depthLookup] = 0xff; //do not visit same pixel twice
                int xTranslatedDepthSpace = xStart + xOffset;
                int yTranslatedDepthSpace = yStart + yOffset;
                int colorPointX = (int)(ptrDepthToColorSpaceMapper[depthLookup].X + 0.5);
                int colorPointY = (int)(ptrDepthToColorSpaceMapper[depthLookup].Y + 0.5);
                if ((yTranslatedDepthSpace < bodyIndexSensorBufferHeight) && (xTranslatedDepthSpace < bodyIndexSensorBufferWidth) &&
                            (yTranslatedDepthSpace >= 0) && (xTranslatedDepthSpace >= 0) && (colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) &&
                            (colorPointY >= 0) && (colorPointX >= 0))
                {
                    // point to current pixel in image buffer
                    uint* ptrImgBufferPixelInt = ptrImageBufferInt + (yTranslatedDepthSpace * bodyIndexSensorBufferWidth + xTranslatedDepthSpace);

                    uint* ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
                    // assign color value (4 bytes)
                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency * HAND_TRANSLATED_ALPHAFACTOR);
                }
            }

            //8-way neighbourhood to visit all pixels of hand (can have background pixel btw fingers)
            this.drawTranslatedRightHand((xStart + 1), yStart, vElbowWristOrig, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
            this.drawTranslatedRightHand((xStart - 1), yStart, vElbowWristOrig, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
            this.drawTranslatedRightHand(xStart, (yStart + 1), vElbowWristOrig, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
            this.drawTranslatedRightHand(xStart, (yStart - 1), vElbowWristOrig, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
            this.drawTranslatedRightHand((xStart - 1), (yStart - 1), vElbowWristOrig, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
            this.drawTranslatedRightHand((xStart - 1), (yStart + 1), vElbowWristOrig, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
            this.drawTranslatedRightHand((xStart + 1), (yStart - 1), vElbowWristOrig, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
            this.drawTranslatedRightHand((xStart + 1), (yStart + 1), vElbowWristOrig, xOffset, yOffset, ptrBodyIndexSensorBuffer, ptrImageBufferInt, ptrColorSensorBufferInt, ptrDepthToColorSpaceMapper);
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
