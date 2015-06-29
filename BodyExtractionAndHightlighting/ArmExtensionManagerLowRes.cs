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
    public class ArmExtensionManagerLowRes : LowResManager, IArmExtensionManager
    {
        private byte[] bodyIndexSensorBuffer, colorSensorBuffer;
        private ushort[] depthDataSource;
        private ColorSpacePoint[] depthToColorSpaceMapper = null;

        private Point pTouch;
        private int xElbow, yElbow, xWrist, yWrist, xHand, yHand, xHandTip, yHandTip;
        private int xTranslationOffset, yTranslationOffset;

        private Vector vElbowToWristOrig, vElbowToWristOrigNorm, vHalfShoulderWrist_NormRight;

        private Helper helper;

        public ArmExtensionManagerLowRes(IntPtr ptrBackbuffer, byte[] bodyIndexSensorBuffer, byte[] colorSensorBuffer, ushort[] depthDataSource, IReadOnlyDictionary<JointType, Joint> bodyJoints, byte userTransparency, Point pTouch)
            : base(ptrBackbuffer, bodyJoints, userTransparency, depthDataSource)
        {
            this.bodyIndexSensorBuffer = bodyIndexSensorBuffer;
            this.colorSensorBuffer = colorSensorBuffer;

            this.depthToColorSpaceMapper = new ColorSpacePoint[bodyIndexImageLength];
            this.depthDataSource = depthDataSource;

            this.pTouch = pTouch;
            this.helper = Helper.getInstance();
        }

        public unsafe void processImage()
        {
            coordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);

            fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
            fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
            fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
            {
                uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;
                base.SetRequiredBuffers(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt);
                base.SetCoordinateMapper(ptrDepthToColorSpaceMapper);

                if (isRightArmTracked())
                {
                    Dictionary<JointType, DepthSpacePoint> rightArmJoints = base.GetRightArmJointsDepthSpace();

                    this.xElbow = (int)(rightArmJoints[JointType.ElbowRight].X + 0.5);
                    this.yElbow = (int)(rightArmJoints[JointType.ElbowRight].Y + 0.5);
                    this.xWrist = (int)(rightArmJoints[JointType.WristRight].X + 0.5);
                    this.yWrist = (int)(rightArmJoints[JointType.WristRight].Y + 0.5);
                    //start point for floodfill 
                    this.xHand = (int)(rightArmJoints[JointType.HandRight].X + 0.5);
                    this.yHand = (int)(rightArmJoints[JointType.HandRight].Y + 0.5);
                    int xHandTip = (int)(rightArmJoints[JointType.HandTipRight].X + 0.5);
                    int yHandTip = (int)(rightArmJoints[JointType.HandTipRight].Y + 0.5);
                    int xShoulder = (int)(rightArmJoints[JointType.ShoulderRight].X + 0.5);
                    int yShoulder = (int)(rightArmJoints[JointType.ShoulderRight].Y + 0.5);
                    //offset for hand translation
                    this.xTranslationOffset = (int)(pTouch.X - xHandTip + 0.5);
                    this.yTranslationOffset = (int)(pTouch.Y - yHandTip + 0.5);

                    //vector elbow to wrist, also normal vector of wrist
                    this.vElbowToWristOrig = new Vector((xWrist - xElbow), (yWrist - yElbow));
                    this.vElbowToWristOrigNorm = vElbowToWristOrig;
                    this.vElbowToWristOrigNorm.Normalize();

                    //TEST artefacts: gaps
                    //this.vElbowToHandTip = new Vector((pHandTip.X - pElbow.X), (pHandTip.Y - pElbow.Y));

                    //lower boundary: half vector of shoulder and wrist
                    Vector vElbowToShoulder = new Vector((xShoulder - xElbow), (yShoulder - yElbow));
                    // H = (S+W)
                    Vector vHalfShoulderWrist = new Vector((vElbowToShoulder.X + vElbowToWristOrig.X), (vElbowToShoulder.Y + vElbowToWristOrig.Y));
                    //vHalfShoulderWrist.Normalize();
                    this.vHalfShoulderWrist_NormRight = new Vector(-vHalfShoulderWrist.Y, vHalfShoulderWrist.X);

                    this.drawBodyfloodfill(xShoulder, yShoulder);
                    //sequential approach
                    //this.drawBodyWithoutRightHand();

                    this.drawStretchedRightLowerArm();

                    this.drawTranslatedRightHand(xHand, yHand);
                }
                else
                {
                    //==write-through
                    base.drawFullBody();
                }
                
            } //end fixed
        }

        //public unsafe void processImage_rotationOnly()
        //{
        //    coordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);

        //    fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
        //    fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
        //    //fixed (byte* ptrImageBufferLowRes = imageBufferLowRes)
        //    fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
        //    {
        //        float xElbowF = (float)pElbow.X;
        //        float yElbowF = (float)pElbow.Y;
        //        float xWristF = (float)pWrist.X;
        //        float yWristF = (float)pWrist.Y;
        //        float xHandTipF = (float)pHandTip.X;
        //        float yHandTipF = (float)pHandTip.Y;
        //        float xTouchF = (float)pTouch.X;
        //        float yTouchF = (float)pTouch.Y;

        //        uint* ptrImageBufferInt = (uint*)ptrBackbuffer;
        //        uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

        //        this.transform_LowRes_rotationOnly(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferInt, ptrDepthToColorSpaceMapper, xElbowF, yElbowF, xWristF, yWristF, xHandTipF, yHandTipF, xTouchF, yTouchF);

        //    } //end fixed
        //}

        //public unsafe void processImage_scaleOnly()
        //{
        //    coordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthToColorSpaceMapper);

        //    fixed (byte* ptrBodyIndexSensorBuffer = bodyIndexSensorBuffer)
        //    fixed (byte* ptrColorSensorBuffer = colorSensorBuffer)
        //    //fixed (byte* ptrImageBufferLowRes = imageBufferLowRes)
        //    fixed (ColorSpacePoint* ptrDepthToColorSpaceMapper = depthToColorSpaceMapper)
        //    {
        //        float xElbowF = (float)pElbow.X;
        //        float yElbowF = (float)pElbow.Y;
        //        float xTouchF = (float)pTouch.X;
        //        float yTouchF = (float)pTouch.Y;
        //        float xWristF = (float)pWrist.X;
        //        float yWristF = (float)pWrist.Y;

        //        uint* ptrImageBufferInt = (uint*)ptrBackbuffer;
        //        uint* ptrColorSensorBufferInt = (uint*)ptrColorSensorBuffer;

        //        this.transform_LowRes_scaleOnly(ptrBodyIndexSensorBuffer, ptrColorSensorBufferInt, ptrImageBufferInt, ptrDepthToColorSpaceMapper, xElbowF, yElbowF, xWristF, yWristF, xTouchF, yTouchF);

        //    } //end fixed
        //}

        private unsafe void drawBodyWithoutRightHand()
        {
            //pixel target
            uint* ptrImgBufferPixelInt = null;
            uint* ptrColorSensorBufferPixelInt = null;

            //save computing power by incrementing x, y without division/modulo
            int xDepthSpace = 0;
            int yDepthSpace = 0;

            //==draw whole body without manipulation
            int depthSpaceSize = depthDataSource.Length; //bodyIndexSensorBufferHeight * bodyIndexSensorBufferWidth;
            for (int idxDepthSpace = 0; idxDepthSpace < depthSpaceSize; idxDepthSpace++)
            {
                //ptrColorSensorBufferPixelInt = null;
                ptrImgBufferPixelInt = null;
                bool isColorPixelInValidRange = false;

                //draw until xElbow
                if (*(ptrBodyIndexSensorBuffer + idxDepthSpace) != 0xff)
                {
                    //omit right lower hand pixel; use half vector btw shoulder and wrist as termination criteria
                    int vPointElbowX = (int)(xDepthSpace - xElbow + 0.5);
                    int vPointElbowY = (int)(yDepthSpace - yElbow + 0.5);
                    int sigPointElbow = ((int)(vHalfShoulderWrist_NormRight.X + 0.5)) * vPointElbowX + ((int)(vHalfShoulderWrist_NormRight.Y + 0.5)) * vPointElbowY;
                    //point is not drawn if p > Elbow
                    if (sigPointElbow <= 0)
                    {
                        int colorPointX = (int)((ptrDepthToColorSpaceMapper + idxDepthSpace)->X + 0.5);
                        int colorPointY = (int)((ptrDepthToColorSpaceMapper + idxDepthSpace)->Y + 0.5);

                        if ((colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) &&
                                (colorPointY >= 0) && (colorPointX >= 0))
                        {
                            isColorPixelInValidRange = true;
                        }

                        if (isColorPixelInValidRange)
                        {
                            // point to current pixel in image buffer
                            ptrImgBufferPixelInt = ptrBackbuffer + idxDepthSpace;

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

        private unsafe void drawBodyfloodfill(int xStart, int yStart)
        {
            if ((xStart >= xElbow) || (xStart >= bodyIndexSensorBufferWidth) || (xStart < 0) || (yStart >= bodyIndexSensorBufferHeight) || (yStart < 0))
            {
                return;
            }

            //pixel target
            uint* ptrImgBufferPixelInt = null;
            uint* ptrColorSensorBufferPixelInt = null;
            bool isColorPixelInValidRange = false;

            //pixel already visited
            int idxDepthSpace = yStart * bodyIndexSensorBufferWidth + xStart;
            if (*(ptrBodyIndexSensorBuffer + idxDepthSpace) == 0xff)
            {
                return;
            }
            else
            {
                *(ptrBodyIndexSensorBuffer + idxDepthSpace) = 0xff; //do not visit same pixel twice
                int colorPointX = (int)((ptrDepthToColorSpaceMapper + idxDepthSpace)->X + 0.5);
                int colorPointY = (int)((ptrDepthToColorSpaceMapper + idxDepthSpace)->Y + 0.5);

                if ((colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) &&
                        (colorPointY >= 0) && (colorPointX >= 0))
                {
                    isColorPixelInValidRange = true;
                }

                if (isColorPixelInValidRange)
                {
                    // point to current pixel in image buffer
                    ptrImgBufferPixelInt = ptrBackbuffer + idxDepthSpace;

                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
                    // assign color value (4 bytes)
                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                }
            }

            //8-way neighbourhood to visit all pixels of hand (can have background pixel btw fingers)
            this.drawBodyfloodfill((xStart + 1), yStart);
            this.drawBodyfloodfill((xStart - 1), yStart);
            this.drawBodyfloodfill(xStart, (yStart + 1));
            this.drawBodyfloodfill(xStart, (yStart - 1));
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

        private unsafe void drawStretchedRightLowerArm()
        {
            uint* ptrImgBufferPixelInt = null; // new position where the color is written into
            uint* ptrColorSensorBufferPixelInt = null; // color pixel position in color frame

            //FROM ELBOW TO WRIST
            // v = (x, y)
            float vOrigArmLength = (float)vElbowToWristOrig.Length;
            //TEST
            //float vOrigArmLength = (float)vElbowToHandTip.Length;

            //NOTE vector normals different as coordinate system origin (0,0) is upper left corner!
            //v_nleft = (y, -x)
            Vector vNormLeftOrigArm = new Vector(vElbowToWristOrigNorm.Y, -(vElbowToWristOrigNorm.X));
            //v_nright = (-y, x)
            Vector vNormRightOrigArm = new Vector(-(vElbowToWristOrigNorm.Y), vElbowToWristOrigNorm.X);

            //move arm so that it fits to the shifted hand
            Vector offsetNewArmWrist = new Vector((xHandTip - xWrist), (yHandTip - yWrist));

            Vector vNewArm = new Vector((pTouch.X - ((int)(offsetNewArmWrist.X + 0.5)) - xElbow), (pTouch.Y - ((int)(offsetNewArmWrist.Y + 0.5)) - yElbow));
            float vNewArmLength = (float)vNewArm.Length;
            vNewArm.Normalize();

            Vector vNormLeftNewArm = new Vector(vNewArm.Y, -(vNewArm.X));
            Vector vNormRightNewArm = new Vector(-(vNewArm.Y), vNewArm.X);

            //sampling rate
            float totalSteps;
            //TODO think about factor
            if (vOrigArmLength < vNewArmLength)
                totalSteps = (float)(vNewArmLength * 2.2);
            else
                totalSteps = (float)(vOrigArmLength * 2.2);
            float stepSizeOrigArm = vOrigArmLength / totalSteps;
            float stepSizeNewArm = vNewArmLength / totalSteps;

            float xCurrOrigArm = xElbow;
            float yCurrOrigArm = yElbow;
            float xCurrNewArm = xElbow;
            float yCurrNewArm = yElbow;
            for (int i = 0; i < totalSteps; i++)
            {
                int colorLookup = (int)(bodyIndexSensorBufferWidth * (int)(yCurrOrigArm + 0.5) + xCurrOrigArm + 0.5);
                int colorPointX = (int)((ptrDepthToColorSpaceMapper + colorLookup)->X + 0.5);
                int colorPointY = (int)((ptrDepthToColorSpaceMapper + colorLookup)->Y + 0.5);

                if ((xCurrNewArm < bodyIndexSensorBufferWidth) && (yCurrNewArm < bodyIndexSensorBufferHeight) && (xCurrNewArm >= 0) && (yCurrNewArm >= 0) && (colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) &&
                    (colorPointY >= 0) && (colorPointX >= 0))
                {
                    //write color pixel into position of new arm
                    int newPositionX = (int)(xCurrNewArm + 0.5);
                    int newPositionY = (int)(yCurrNewArm + 0.5);
                    ptrImgBufferPixelInt = ptrBackbuffer + (newPositionY * bodyIndexSensorBufferWidth + newPositionX);

                    // assign color value
                    if (Constants.IsSkeletonShown)
                    {
                        //draw line where new stretched vector is
                        *ptrImgBufferPixelInt = 0xFF00FFFF;

                        //draw red line where orig hand vector is
                        ptrImgBufferPixelInt = ptrBackbuffer + (int)((int)(yCurrOrigArm + 0.5) * bodyIndexSensorBufferWidth + xCurrOrigArm + 0.5);
                        *ptrImgBufferPixelInt = 0xFFFF0000; //ARGB in storage

                        *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                    }
                    else
                    {
                        //color of original arm pixel
                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
                        // assign color value (4 bytes)
                        *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                        // overwrite the alpha value (last byte)
                        *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
                    }
                }

                #region left normal vector of elbow-wrist vector

                //calculate for left normal of vector
                float xNormLeft = xCurrOrigArm + (float)vNormLeftOrigArm.X;
                float yNormLeft = yCurrOrigArm + (float)vNormLeftOrigArm.Y;
                int lookupIdxNormLeft = (int)((int)(yNormLeft + 0.5) * bodyIndexSensorBufferWidth + xNormLeft + 0.5);
                float newPosNormLeftX = xCurrNewArm + (float)vNormLeftNewArm.X;
                float newPosNormLeftY = yCurrNewArm + (float)vNormLeftNewArm.Y;
                while (*(ptrBodyIndexSensorBuffer + lookupIdxNormLeft) != 0xff)
                {
                    if ((xNormLeft >= bodyIndexSensorBufferWidth) || (yNormLeft >= bodyIndexSensorBufferHeight) || (xNormLeft < 0) || (yNormLeft < 0) || (newPosNormLeftX >= bodyIndexSensorBufferWidth) || (newPosNormLeftX < 0) || (newPosNormLeftY >= bodyIndexSensorBufferWidth) || (newPosNormLeftY < 0)) 
                    {
                        break;
                    }

                    //normal might point inside body
                    //lower boundary: half vector of shoulder and wrist (upper boundary implicitly given through touch point)
                    int vPointElbowX = (int)(xNormLeft - xElbow + 0.5);
                    int vPointElbowY = (int)(yNormLeft - yElbow + 0.5);
                    int sigPointElbow = ((int)(vHalfShoulderWrist_NormRight.X + 0.5)) * vPointElbowX + ((int)(vHalfShoulderWrist_NormRight.Y + 0.5)) * vPointElbowY;
                    //point is not drawn if p < Elbow
                    if (sigPointElbow < 0)
                    {
                        break;
                    }

                    int normLeftColorPointX = (int)((ptrDepthToColorSpaceMapper + lookupIdxNormLeft)->X + 0.5);
                    int normLeftColorPointY = (int)((ptrDepthToColorSpaceMapper + lookupIdxNormLeft)->Y + 0.5);

                    if ((normLeftColorPointY < colorSensorBufferHeight) && (normLeftColorPointX < colorSensorBufferWidth) && (normLeftColorPointY >= 0) && (normLeftColorPointX >= 0))
                    {
                        //write color pixel into position of new arm
                        ptrImgBufferPixelInt = ptrBackbuffer + (int)((int)(newPosNormLeftY + 0.5) * bodyIndexSensorBufferWidth + newPosNormLeftX + 0.5);

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
                #endregion

                #region right normal vector of elbow-wrist vector

                //calculate for right normal of vector
                float xNormRight = xCurrOrigArm + (float)vNormRightOrigArm.X;
                float yNormRight = yCurrOrigArm + (float)vNormRightOrigArm.Y;
                int lookupIdxNormRight = (int)((int)(yNormRight + 0.5) * bodyIndexSensorBufferWidth + xNormRight + 0.5);
                float newPosNormRightX = xCurrNewArm + (float)vNormRightNewArm.X;
                float newPosNormRightY = yCurrNewArm + (float)vNormRightNewArm.Y;
                while (*(ptrBodyIndexSensorBuffer + lookupIdxNormRight) != 0xff)
                {
                    if ((xNormRight >= bodyIndexSensorBufferWidth) || (yNormRight >= bodyIndexSensorBufferHeight) || (xNormRight < 0) || (yNormRight < 0) || (newPosNormRightX >= bodyIndexSensorBufferWidth) || (newPosNormRightX < 0) || (newPosNormRightY >= bodyIndexSensorBufferWidth) || (newPosNormRightY < 0))
                    {
                        break;
                    }

                    //normal might point inside body
                    //lower boundary: half vector of shoulder and wrist (upper boundary implicitly given through touch point)
                    int vPointElbowX = (int)(xNormRight - xElbow + 0.5);
                    int vPointElbowY = (int)(yNormRight - yElbow + 0.5);
                    int sigPointElbow = ((int)(vHalfShoulderWrist_NormRight.X + 0.5)) * vPointElbowX + ((int)(vHalfShoulderWrist_NormRight.Y + 0.5)) * vPointElbowY;
                    //point is not drawn if p < Elbow
                    if (sigPointElbow < 0)
                    {
                        break;
                    }

                    int normRightColorPointX = (int)((ptrDepthToColorSpaceMapper + lookupIdxNormRight)->X + 0.5);
                    int normRightColorPointY = (int)((ptrDepthToColorSpaceMapper + lookupIdxNormRight)->Y + 0.5);

                    if ((normRightColorPointY < colorSensorBufferHeight) && (normRightColorPointX < colorSensorBufferWidth) && (normRightColorPointY >= 0) && (normRightColorPointX >= 0))
                    {
                        //write color pixel into position of new arm
                        ptrImgBufferPixelInt = ptrBackbuffer + (int)((int)(newPosNormRightY + 0.5) * bodyIndexSensorBufferWidth + newPosNormRightX + 0.5);

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
                #endregion

                //increment to move along vector; use normal vectors for increment
                xCurrOrigArm += (float)(vElbowToWristOrigNorm.X) * stepSizeOrigArm;
                yCurrOrigArm += (float)(vElbowToWristOrigNorm.Y) * stepSizeOrigArm;
                xCurrNewArm += (float)(vNewArm.X) * stepSizeNewArm;
                yCurrNewArm += (float)(vNewArm.Y) * stepSizeNewArm;

            }
        }

        private unsafe void drawTranslatedRightHand(int xStart, int yStart)
        {
            //xEnd is left outer boundary
            if ((xStart >= bodyIndexSensorBufferWidth) || (xStart < 0) || (yStart >= bodyIndexSensorBufferHeight) || (yStart < 0))
            {
                return;
            }

            //boundary: normal vector of wrist; no normalization necessary!! (only signum is of value)
            int vPointToWristX = xStart - xWrist;
            int vPointToWristY = yStart - yWrist;
            int sig = ((int)(vElbowToWristOrig.X + 0.5)) * vPointToWristX + ((int)(vElbowToWristOrig.Y + 0.5)) * vPointToWristY;
            //point ON line counts to hand
            if (sig < 0)
            {
                return;
            }

            //pixel already visited
            int depthLookup = yStart * bodyIndexSensorBufferWidth + xStart;
            if (*(ptrBodyIndexSensorBuffer + depthLookup) == 0xff)
            {
                return;
            }
            else
            {
                *(ptrBodyIndexSensorBuffer + depthLookup) = 0xff; //do not visit same pixel twice
                int xTranslatedDepthSpace = xStart + xTranslationOffset;
                int yTranslatedDepthSpace = yStart + yTranslationOffset;
                int colorPointX = (int)((ptrDepthToColorSpaceMapper + depthLookup)->X + 0.5);
                int colorPointY = (int)((ptrDepthToColorSpaceMapper + depthLookup)->Y + 0.5);
                if ((yTranslatedDepthSpace < bodyIndexSensorBufferHeight) && (xTranslatedDepthSpace < bodyIndexSensorBufferWidth) &&
                            (yTranslatedDepthSpace >= 0) && (xTranslatedDepthSpace >= 0) && (colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) &&
                            (colorPointY >= 0) && (colorPointX >= 0))
                {
                    // point to current pixel in image buffer
                    uint* ptrImgBufferPixelInt = ptrBackbuffer + (yTranslatedDepthSpace * bodyIndexSensorBufferWidth + xTranslatedDepthSpace);

                    uint* ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
                    // assign color value (4 bytes)
                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency);
                }
            }

            //8-way neighbourhood to visit all pixels of hand (can have background pixel btw fingers)
            this.drawTranslatedRightHand((xStart + 1), yStart);
            this.drawTranslatedRightHand((xStart - 1), yStart);
            this.drawTranslatedRightHand(xStart, (yStart + 1));
            this.drawTranslatedRightHand(xStart, (yStart - 1));
        }

        //private unsafe void transform_LowRes_scaleOnly(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferInt, ColorSpacePoint* ptrDepthToColorSpaceMapper, float xElbow, float yElbow, float xWrist, float yWrist, float xTouch, float yTouch)
        //{
        //    double absAngle = helper.CalculateAbsoluteAngleInDegreeToXaxis(xElbow, yElbow, xWrist, yWrist) / 180.0 * Math.PI;
        //    double factorX = Math.Cos(absAngle);
        //    double factorY = Math.Sin(absAngle);
        //    double testLength = factorX * factorX + factorY * factorY;
        //    //Console.WriteLine("X: " + factorX + " Y: " + factorY + " Length: " + testLength);
        //    float factorXsquare = (float)(1.0 + factorX * factorX);
        //    float factorYsquare = (float)(1.0 + factorY * factorY);
        //    //factorXsquare = 1.5f;
        //    //factorYsquare = 1.5f;


        //    uint transparencyMask = (0xffffff00u | this.userTransparency);

        //    int unsafecolorSensorBufferWidth = this.colorSensorBufferWidth;
        //    int unsafecolorSensorBufferHeight = this.colorSensorBufferHeight;

        //    //save computing power by incrementing x, y without division/modulo
        //    int xDepthSpace = 0;
        //    int yDepthSpace = 0;
        //    uint* ptrColorSensorBufferPixelInt = null;
        //    uint* ptrImgBufferPixelInt = ptrImageBufferInt;
        //    int depthSpaceSize = bodyIndexSensorBufferHeight * bodyIndexSensorBufferWidth;

        //    for (int idxDepthSpace = 0; idxDepthSpace < depthSpaceSize; ++idxDepthSpace)
        //    {
        //        ptrColorSensorBufferPixelInt = null;

        //        if (xDepthSpace > xElbow) // handle right arm
        //        {
        //            // compute stretched point in depth space and transform it in color space for lookup
        //            float offsetXdepthSpace = xDepthSpace - xElbow;
        //            int lookupXdepthSpace = (int)(xElbow + (offsetXdepthSpace / factorXsquare) + 0.5);

        //            float offsetYdepthSpace = yDepthSpace - yElbow;
        //            int lookupYdepthSpace = (int)(yElbow + (offsetYdepthSpace / factorYsquare) + 0.5);

        //            int idxStretchedDepthSpace = bodyIndexSensorBufferWidth * lookupYdepthSpace + lookupXdepthSpace;

        //            if ((idxStretchedDepthSpace < depthSpaceSize) && (*(ptrBodyIndexSensorBuffer + idxStretchedDepthSpace) != 0xff))
        //            {
        //                int colorPointXstretched = (int)((ptrDepthToColorSpaceMapper + idxStretchedDepthSpace)->X + 0.5);
        //                int colorPointYstretched = (int)((ptrDepthToColorSpaceMapper + idxStretchedDepthSpace)->Y + 0.5);

        //                if ((colorPointYstretched < unsafecolorSensorBufferHeight) && (colorPointXstretched < unsafecolorSensorBufferWidth) &&
        //                        (colorPointYstretched >= 0) && (colorPointXstretched >= 0))
        //                {
        //                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointYstretched * unsafecolorSensorBufferWidth + colorPointXstretched);
        //                }
        //            }
        //        }
        //        else // handle rest of the body
        //        {
        //            if (*(ptrBodyIndexSensorBuffer + idxDepthSpace) != 0xff)
        //            {
        //                int colorPointX = (int)((ptrDepthToColorSpaceMapper + idxDepthSpace)->X + 0.5);
        //                int colorPointY = (int)((ptrDepthToColorSpaceMapper + idxDepthSpace)->Y + 0.5);

        //                if ((colorPointY < unsafecolorSensorBufferHeight) && (colorPointX < unsafecolorSensorBufferWidth) &&
        //                (colorPointY >= 0) && (colorPointX >= 0))
        //                {
        //                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * unsafecolorSensorBufferWidth + colorPointX);
        //                }
        //            }
        //        }

        //        if (ptrColorSensorBufferPixelInt != null)
        //        {
        //            // assign color value (4 bytes); dereferencing + assigning writes into imgBuffer
        //            // mask the last byte which contains the alpha value
        //            *ptrImgBufferPixelInt = (*ptrColorSensorBufferPixelInt) & transparencyMask;
        //        }

        //        // increment counter
        //        ++ptrImgBufferPixelInt;
        //        if (++xDepthSpace == bodyIndexSensorBufferWidth)
        //        {
        //            xDepthSpace = 0;
        //            ++yDepthSpace;
        //        }
        //    } // end for each pixel in depth space
        //}

        //private unsafe void transform_LowRes_rotationOnly(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt, uint* ptrImageBufferInt, ColorSpacePoint* ptrDepthToColorSpaceMapper, float xElbow, float yElbow, float xWrist, float yWrist, float xHandTip, float yHandTip, float xTouch, float yTouch)
        //{
        //    double rotationAngleInRad = helper.CalculateRotationAngle(xElbow, yElbow, xWrist, yWrist, xTouch, yTouch);
        //    double cos = Math.Cos(rotationAngleInRad);
        //    double sin = Math.Sin(rotationAngleInRad);

        //    uint* ptrImgBufferPixelInt = null; // this is where we want to write the pixel
        //    uint* ptrColorSensorBufferPixelInt = null;

        //    Vector areaOffset = new Vector((xWrist - xElbow), (yWrist - yElbow));
        //    int handOffset = (int)areaOffset.Length / 3;
        //    int handTipBoundaryX = (int)(xHandTip + 0.5);// +handOffset;
        //    int handTipBoundaryY = (int)(yHandTip + 0.5);// -handOffset;

        //    //save computing power by incrementing x, y without division/modulo
        //    int xDepthSpace = 0;
        //    int yDepthSpace = 0;

        //    int depthSpaceSize = bodyIndexSensorBufferHeight * bodyIndexSensorBufferWidth;
        //    for (int idxDepthSpace = 0; idxDepthSpace < depthSpaceSize; idxDepthSpace++)
        //    {
        //        ptrColorSensorBufferPixelInt = null;

        //        if (*(ptrBodyIndexSensorBuffer + idxDepthSpace) != 0xff)
        //        {
        //            ptrImgBufferPixelInt = null;

        //            #region --- Region of lower arm
        //            //TODO determine region of hand with floodfill (start: xWrist/yWrist until xElbow)
        //            if (xDepthSpace >= xElbow)
        //            {
        //                // compute rotation (in target image buffer)
        //                int rotatedX = (int)(cos * (xDepthSpace - xElbow) - sin * (yDepthSpace - yElbow) + xElbow + 0.5);
        //                int rotatedY = (int)(sin * (xDepthSpace - xElbow) + cos * (yDepthSpace - yElbow) + yElbow + 0.5);

        //                if ((rotatedY < bodyIndexSensorBufferHeight) && (rotatedX < bodyIndexSensorBufferWidth) &&
        //                    (rotatedY >= 0) && (rotatedX >= 0))
        //                {
        //                    ptrImgBufferPixelInt = ptrImageBufferInt + (rotatedY * bodyIndexSensorBufferWidth + rotatedX);
        //                }
        //            }
        //            #endregion // lower arm
        //            else
        //            {
        //                // point to current pixel in image buffer
        //                ptrImgBufferPixelInt = ptrImageBufferInt + idxDepthSpace;
        //            }

        //            if (ptrImgBufferPixelInt != null)
        //            {
        //                int colorPointX = (int)((ptrDepthToColorSpaceMapper + idxDepthSpace)->X + 0.5);
        //                int colorPointY = (int)((ptrDepthToColorSpaceMapper + idxDepthSpace)->Y + 0.5);

        //                //check boundaries
        //                if ((colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) &&
        //                        (colorPointY >= 0) && (colorPointX >= 0))
        //                {
        //                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
        //                    // assign color value (4 bytes)
        //                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
        //                    // overwrite the alpha value (last byte)
        //                    *(((byte*)ptrImgBufferPixelInt) + 3) = this.userTransparency;
        //                }
        //            }
        //        } //if body

        //        //increment counter
        //        if (++xDepthSpace == bodyIndexSensorBufferWidth)
        //        {
        //            xDepthSpace = 0;
        //            yDepthSpace++;
        //        }
        //    } //for loop
        //}
    }
}
