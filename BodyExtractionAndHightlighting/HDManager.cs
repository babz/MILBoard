using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;
using System.Threading;

namespace BodyExtractionAndHightlighting
{
    public abstract class HDManager : BasicManager<ColorSpacePoint>
    {
        protected unsafe volatile DepthSpacePoint* ptrColorToDepthSpaceMapper;

        public HDManager(IntPtr ptrBackbuffer, IReadOnlyDictionary<JointType, Joint> bodyJoints, byte userTransparency)
            : base(ptrBackbuffer, bodyJoints, userTransparency)
        {
            //empty on purpose
        }

        protected Dictionary<JointType, ColorSpacePoint> GetRightArmJointsColorSpace()
        {
            return this.convertBodyJoints(base.GetRightArmCameraSpace());
        }

        /*
         * Example: call convertBodyJoints(base.GetRightArmJoints())
         * */
        protected override Dictionary<JointType, ColorSpacePoint> convertBodyJoints(Dictionary<JointType, CameraSpacePoint> bodyJoints)
        {
            Dictionary<JointType, ColorSpacePoint> bodyJointsColorSpace = new Dictionary<JointType,ColorSpacePoint>();

            foreach (KeyValuePair<JointType, CameraSpacePoint> entry in bodyJoints)
            {
                CameraSpacePoint tmpCamSpacePoint = entry.Value;
                if (tmpCamSpacePoint.Z < 0)
                {
                    tmpCamSpacePoint.Z = base.InferredZPositionClamp;
                }
                bodyJointsColorSpace[entry.Key] = coordinateMapper.MapCameraPointToColorSpace(tmpCamSpacePoint);
            }
            return bodyJointsColorSpace;
        }

        /*
         * Call in fixed block of caller before using drawFullBody()!!!
         * */
        protected unsafe void SetCoordinateMapper(DepthSpacePoint* ptrColorToDepthSpaceMapper)
        {
            this.ptrColorToDepthSpaceMapper = ptrColorToDepthSpaceMapper;
        }

        protected override void drawFullBody()
        {
            Thread thread;
            if (base.IsAnyJointTracked())
            {
                ColorSpacePoint bodyPoint = coordinateMapper.MapCameraPointToColorSpace(base.GetAnyBodyPoint());
                thread = new Thread(() => floodfillBody((int)(bodyPoint.X + 0.5), (int)(bodyPoint.Y + 0.5)), Constants.STACK_SIZE_HD);
            }
            else
            {
                thread = new Thread(() => sequentialFillBody(), Constants.STACK_SIZE_LOWRES);
            }
            thread.Start();
            thread.Join();
        }

        private unsafe void floodfillBody(int xStart, int yStart)
        {
            if ((xStart >= colorSensorBufferWidth) || (xStart < 0) || (yStart >= colorSensorBufferHeight) || (yStart < 0))
            {
                return;
            }

            int idxCurrColorPixel = yStart * colorSensorBufferWidth + xStart;
            if (idxCurrColorPixel >= colorImageLength)
            {
                return;
            }

            //pixel already visited
            uint* ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxCurrColorPixel;
            if ((*ptrColorSensorBufferPixelInt) == 0xFF000000)
            {
                return;
            }

            float xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->X;
            float yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->Y;
            int idxDepthPixel = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
            if (Single.IsInfinity(xDepthPixel) || Single.IsInfinity(yDepthPixel) || (*(ptrBodyIndexSensorBuffer + idxDepthPixel) == 0xff))
            {
                return;
            }
            else
            {
                // point to current pixel in image buffer
                uint* ptrImgBufferPixelInt = ptrBackbuffer + (yStart * colorSensorBufferWidth + xStart);

                // assign color value (4 bytes)
                *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                // overwrite the alpha value (last byte)
                *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency);

                //do not visit same pixel twice
                *ptrColorSensorBufferPixelInt = 0xFF000000;
            }

            //4-way neighbourhood to visit all pixels of hand (can have background pixel btw fingers)
            this.floodfillBody((xStart + 1), yStart);
            this.floodfillBody((xStart - 1), yStart);
            this.floodfillBody(xStart, (yStart + 1));
            this.floodfillBody(xStart, (yStart - 1));
        }

        private unsafe void sequentialFillBody()
        {
            uint* ptrImgBufferPixelInt = null;

            //after the loop, only color pixels with a body index value that can be mapped to a depth value will remain in the buffer
            for (int i = 0; i < colorImageLength; i++)
            {
                //where the color img cannot be mapped to the depth image, there are infinity values
                if (Single.IsInfinity((ptrColorToDepthSpaceMapper + i)->Y) || Single.IsInfinity((ptrColorToDepthSpaceMapper + i)->X))
                {
                    continue;
                }

                int idx = (int)(bodyIndexSensorBufferWidth * (ptrColorToDepthSpaceMapper + i)->Y + (ptrColorToDepthSpaceMapper + i)->X); //2D to 1D

                // bodyIndex can be 0, 1, 2, 3, 4, or 5
                if (*(ptrBodyIndexSensorBuffer + idx) != 0xff)
                {
                    ptrImgBufferPixelInt = ptrBackbuffer + i;

                    //with the cast to int, 4 bytes are copied
                    *ptrImgBufferPixelInt = *(ptrColorSensorBufferInt + i);
                    // overwrite the alpha value
                    *(((byte*)ptrImgBufferPixelInt) + 3) = userTransparency;
                }

            } // for loop
        }
    }
}
