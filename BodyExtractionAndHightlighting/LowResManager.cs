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
    public abstract class LowResManager : BasicManager<DepthSpacePoint>
    {
        protected unsafe volatile ColorSpacePoint* ptrDepthToColorSpaceMapper;

        public LowResManager(IntPtr ptrBackbuffer, IReadOnlyDictionary<JointType, Joint> bodyJoints, byte userTransparency)
            : base(ptrBackbuffer, bodyJoints, userTransparency)
        {
            //empty on purpose
        }

        protected Dictionary<JointType, DepthSpacePoint> GetRightArmJointsDepthSpace()
        {
            return this.convertBodyJoints(base.GetRightArmCameraSpace());
        }

        /*
         * Example usage: call convertBodyJoints(base.GetRightArmJoints())
         * */
        protected override Dictionary<JointType, DepthSpacePoint> convertBodyJoints(Dictionary<JointType, CameraSpacePoint> bodyJoints)
        {
            Dictionary<JointType, DepthSpacePoint> bodyJointsDepthSpace = new Dictionary<JointType,DepthSpacePoint>();

            foreach (KeyValuePair<JointType, CameraSpacePoint> entry in bodyJoints)
            {
                CameraSpacePoint tmpCamSpacePoint = entry.Value;
                if (tmpCamSpacePoint.Z < 0)
                {
                    tmpCamSpacePoint.Z = base.InferredZPositionClamp;
                }
                bodyJointsDepthSpace[entry.Key] = coordinateMapper.MapCameraPointToDepthSpace(tmpCamSpacePoint);
            }
            return bodyJointsDepthSpace;
        }

        /*
         * Call in fixed block of caller before using drawFullBody()!!!
         * */
        protected unsafe void SetCoordinateMapper(ColorSpacePoint* ptrDepthToColorSpaceMapper)
        {
            this.ptrDepthToColorSpaceMapper = ptrDepthToColorSpaceMapper;
        }

        protected override void drawFullBody()
        {
            Thread thread;
            if (base.IsAnyJointTracked())
            {
                DepthSpacePoint bodyPoint = coordinateMapper.MapCameraPointToDepthSpace(base.GetAnyBodyPoint());
                thread = new Thread(() => floodfillBody((int)(bodyPoint.X + 0.5), (int)(bodyPoint.Y + 0.5)), Constants.STACK_SIZE_LOWRES);
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
            if ((xStart >= bodyIndexSensorBufferWidth) || (xStart < 0) || (yStart >= bodyIndexSensorBufferHeight) || (yStart < 0))
            {
                return;
            }

            //pixel target
            uint* ptrBackbufferPixelInt = null;
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
                    ptrBackbufferPixelInt = ptrBackbuffer + idxDepthSpace;

                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
                    // assign color value (4 bytes)
                    *ptrBackbufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrBackbufferPixelInt) + 3) = this.userTransparency;
                }
            }

            //4-way neighbourhood to visit all pixels of hand (can have background pixel btw fingers)
            this.floodfillBody((xStart + 1), yStart);
            this.floodfillBody((xStart - 1), yStart);
            this.floodfillBody(xStart, (yStart + 1));
            this.floodfillBody(xStart, (yStart - 1));
        }

        private unsafe void sequentialFillBody()
        {
            //pixel target
            uint* ptrBackbufferPixelInt = null;
            uint* ptrColorSensorBufferPixelInt = null;

            for (int i = 0; i < bodyIndexImageLength; i++)
            {
                // bodyIndex can be 0, 1, 2, 3, 4, or 5
                if (*(ptrBodyIndexSensorBuffer + i) != 0xff)
                {
                    int colorPointX = (int)((ptrDepthToColorSpaceMapper + i)->X + 0.5);
                    int colorPointY = (int)((ptrDepthToColorSpaceMapper + i)->Y + 0.5);

                    //check boundaries
                    if ((colorPointY >= colorSensorBufferHeight) || (colorPointX >= colorSensorBufferWidth) ||
                    (colorPointY < 0) && (colorPointX < 0))
                    {
                        continue;
                    }

                    //point to current pixel in target imgBuffer
                    ptrBackbufferPixelInt = ptrBackbuffer + i;

                    // corresponding pixel in the 1080p image
                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
                    // assign color value (4 bytes); dereferencing + assigning writes into imgBuffer
                    *ptrBackbufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value
                    *(((byte*)ptrBackbufferPixelInt) + 3) = userTransparency;
                }
            } //end for
        }
    }
}
