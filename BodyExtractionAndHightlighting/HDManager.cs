using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;
using System.Threading;
using System.Drawing;
using FloodFill;

namespace BodyExtractionAndHightlighting
{
    public abstract class HDManager : BasicManager<ColorSpacePoint>
    {
        protected unsafe volatile DepthSpacePoint* ptrColorToDepthSpaceMapper;
        FloodFillRangeQueue ranges;

        public HDManager(IntPtr ptrBackbuffer, IReadOnlyDictionary<JointType, Joint> bodyJoints, byte userTransparency, ushort[] depthDataSource)
            : base(ptrBackbuffer, bodyJoints, userTransparency, depthDataSource)
        {
            // Empty on purpose
        }

        protected Dictionary<JointType, ColorSpacePoint> GetRightArmJointsColorSpace()
        {
            return this.convertBodyJointsToImageSpace(base.GetRightArmCameraSpace());
        }

        /*
         * Example: call convertBodyJoints(base.GetRightArmJoints())
         * */
        protected override Dictionary<JointType, ColorSpacePoint> convertBodyJointsToImageSpace(Dictionary<JointType, CameraSpacePoint> bodyJoints)
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
         * determines if hand and body overlap
         * */
        protected unsafe override bool isDepthDifferent(int idxCurrDepthPoint, int xNext, int yNext)
        {
            int depthCurrent = base.getDepth(idxCurrDepthPoint);

            int idxColorSpace = (int)(yNext * colorSensorBufferWidth + xNext + 0.5);
            float xDepthPoint = (ptrColorToDepthSpaceMapper + idxColorSpace)->X;
            float yDepthPoint = (ptrColorToDepthSpaceMapper + idxColorSpace)->Y;
            int idxDepthPoint = (int)(((int)(yDepthPoint + 0.5)) * bodyIndexSensorBufferWidth + xDepthPoint + 0.5);

            int depthNext = base.getDepth(idxDepthPoint);
            
            if (Math.Abs(depthCurrent - depthNext) > depthThreshold)
            {
                return true;
            }
            else
            {
                return false;
            }
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
                //thread = new Thread(() => linefillBody((int)(bodyPoint.X + 0.5), (int)(bodyPoint.Y + 0.5)), Constants.LINEFILL_HD);
                thread = new Thread(() => floodfillBody((int)(bodyPoint.X + 0.5), (int)(bodyPoint.Y + 0.5)), Constants.STACK_SIZE_HD);
                //thread.Start();
                //thread.Join();
                //linefillBody((int)(bodyPoint.X + 0.5), (int)(bodyPoint.Y + 0.5));
            }
            else
            {
                thread = new Thread(() => sequentialFillBody(), Constants.STACK_SIZE_LOWRES);
                thread.Start();
                thread.Join();
            }
            //thread.Start();
            //thread.Join();
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
        /*
        private unsafe void linefillBody(int xStart, int yStart)
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

            int idxCurrColorPixelLeft = idxCurrColorPixel;
            float xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixelLeft)->X;
            float yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixelLeft)->Y;
            int idxDepthPixel = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
            for (int xLeft = xStart; xLeft >= 0; --xLeft) {
                if (Single.IsInfinity(xDepthPixel) || Single.IsInfinity(yDepthPixel) || (*(ptrBodyIndexSensorBuffer + idxDepthPixel) == 0xff))
                {
                    return;
                }
                else if ((*ptrColorSensorBufferPixelInt) == 0xFF000000)
                {
                    return;
                }
                else
                {
                    // point to current pixel in image buffer
                    uint* ptrImgBufferPixelInt = ptrBackbuffer + (yStart * colorSensorBufferWidth + xLeft);

                    // assign color value (4 bytes)
                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency);

                    //do not visit same pixel twice
                    *ptrColorSensorBufferPixelInt = 0xFF000000;
                }
                idxCurrColorPixelLeft--;
                xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixelLeft)->X;
                yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixelLeft)->Y;
                idxDepthPixel = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
            }

            int idxCurrColorPixelRight = idxCurrColorPixel + 1;
            xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixelRight)->X;
            yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixelRight)->Y;
            idxDepthPixel = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
            for (int xRight = xStart + 1; xRight < colorSensorBufferWidth; ++xRight)
            {
                if (Single.IsInfinity(xDepthPixel) || Single.IsInfinity(yDepthPixel) || (*(ptrBodyIndexSensorBuffer + idxDepthPixel) == 0xff))
                {
                    return;
                }
                else if ((*ptrColorSensorBufferPixelInt) == 0xFF000000)
                {
                    return;
                }
                else
                {
                    // point to current pixel in image buffer
                    uint* ptrImgBufferPixelInt = ptrBackbuffer + (yStart * colorSensorBufferWidth + xRight);

                    // assign color value (4 bytes)
                    *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency);

                    //do not visit same pixel twice
                    *ptrColorSensorBufferPixelInt = 0xFF000000;
                }
                idxCurrColorPixelRight++;
                xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixelRight)->X;
                yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixelRight)->Y;
                idxDepthPixel = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
            }

            this.linefillBody(xStart, (yStart + 1));
            this.linefillBody(xStart, (yStart - 1));
        }
        */

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

                int idxDepthSpace = (int)(bodyIndexSensorBufferWidth * (ptrColorToDepthSpaceMapper + i)->Y + (ptrColorToDepthSpaceMapper + i)->X); //2D to 1D

                // bodyIndex can be 0, 1, 2, 3, 4, or 5
                if (*(ptrBodyIndexSensorBuffer + idxDepthSpace) != 0xff)
                {
                    ptrImgBufferPixelInt = ptrBackbuffer + i;

                    //with the cast to int, 4 bytes are copied
                    *ptrImgBufferPixelInt = *(ptrColorSensorBufferInt + i);
                    // overwrite the alpha value
                    *(((byte*)ptrImgBufferPixelInt) + 3) = userTransparency;
                }

            } // for loop
        }


        //################################################### Scanline Floodfill ###################################################

        unsafe bool CheckPixel(int idxCurrColorPixel)
        {
            float xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->X;
            float yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->Y;
            int idxDepthPixel = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
            if (Single.IsInfinity(xDepthPixel) || Single.IsInfinity(yDepthPixel) || (*(ptrBodyIndexSensorBuffer + idxDepthPixel) == 0xff))
            {
                return false;
            }
            return true;
        }


        /// <summary>
        /// Fills the specified point on the bitmap with the currently selected 
        /// fill color.
        /// </summary>
        /// <param name="pt">The starting point for the fill.</param>
        unsafe public void linefillBody(int x, int y)
        {
            ranges = new FloodFillRangeQueue(((colorSensorBufferWidth+colorSensorBufferHeight)/2)*5);

            //***Do first call to floodfill.
            LinearFill(ref x, ref y);

            // Call floodfill routine while floodfill ranges still exist on the queue
            while (ranges.Count > 0)
            {
                //**Get Next Range Off the Queue
                FloodFillRange range = ranges.Dequeue();

                //**Check Above and Below Each Pixel in the Floodfill Range
                int downPxIdx = (colorSensorBufferWidth * (range.Y + 1)) + range.StartX;
                            //CoordsToPixelIndex(lFillLoc,y+1);
                int upPxIdx = (colorSensorBufferWidth * (range.Y - 1)) + range.StartX;
                            //CoordsToPixelIndex(lFillLoc, y - 1);
                int upY=range.Y - 1;//so we can pass the y coord by ref
                int downY = range.Y + 1;
                int idxCurrColorPixel;
                uint* ptrColorSensorBufferPixelInt;
                for (int i = range.StartX; i <= range.EndX; i++)
                {
                    // Start Fill Upwards
                    idxCurrColorPixel = upY * colorSensorBufferWidth + i;
                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxCurrColorPixel;
                    if (range.Y > 0 && (*ptrColorSensorBufferPixelInt) != 0xFF000000 && CheckPixel(idxCurrColorPixel))
                    {
                        LinearFill(ref i, ref upY);
                    }

                    // Start Fill Downwards
                    idxCurrColorPixel = downY * colorSensorBufferWidth + i;
                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxCurrColorPixel;
                    if (range.Y < (colorSensorBufferHeight - 1) && (*ptrColorSensorBufferPixelInt) != 0xFF000000 && CheckPixel(idxCurrColorPixel))
                    {
                        LinearFill(ref i, ref downY); 
                    }                        
                    downPxIdx++;
                    upPxIdx++;
                }
            }
        }

        /// <summary>
        /// Finds the furthermost left and right boundaries of the fill area
        /// on a given y coordinate, starting from a given x coordinate, 
        /// filling as it goes.
        /// Adds the resulting horizontal range to the queue of floodfill ranges,
        /// to be processed in the main loop.
        /// </summary>
        /// <param name="x">The x coordinate to start from.</param>
        /// <param name="y">The y coordinate to check at.</param>
        unsafe void LinearFill(ref int x, ref int y)
        {
            //todo: cache some bitmap and fill info in local variables for a little extra speed
            
            //***Find Left Edge of Color Area
            int lFillLoc = x; //the location to check/fill on the left
            int pxIdx = (colorSensorBufferWidth * y) + x;
            uint* ptrImgBufferPixelInt;
            uint* ptrColorSensorBufferPixelInt;
            ptrImgBufferPixelInt = ptrBackbuffer + pxIdx;
            ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + pxIdx;
            while (true)
            {
                if (lFillLoc < 0 || (*ptrColorSensorBufferPixelInt == 0xFF000000) || !CheckPixel(pxIdx))
                {
                    break;
                }

                // assign color value (4 bytes)
                *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                // overwrite the alpha value (last byte)
                *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency);

                //do not visit same pixel twice
                *ptrColorSensorBufferPixelInt = 0xFF000000;

                //**de-increment
                lFillLoc--;     //de-increment counter
                pxIdx--;        //de-increment pixel index
                ptrImgBufferPixelInt = ptrBackbuffer + pxIdx;
                ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + pxIdx;

                // todo: not working at beginning of the loop
                //**exit loop if we're at edge of bitmap or color area


            }
            lFillLoc++;

            //***Find Right Edge of Color Area
            int rFillLoc = x + 1; //the location to check/fill on the left
            pxIdx = (colorSensorBufferWidth * y) + x + 1;
            ptrImgBufferPixelInt = ptrBackbuffer + pxIdx;
            ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + pxIdx;
            while (true)
            {
                // todo: not working at beginning of the loop
                //**exit loop if we're at edge of bitmap or color area
                if (rFillLoc >= colorSensorBufferWidth || (*ptrColorSensorBufferPixelInt == 0xFF000000) || !CheckPixel(pxIdx))
                {
                    break;
                }

                // assign color value (4 bytes)
                *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                // overwrite the alpha value (last byte)
                *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency);

                //do not visit same pixel twice
                *ptrColorSensorBufferPixelInt = 0xFF000000;

                //**de-increment
                rFillLoc++;     //de-increment counter
                pxIdx++;        //de-increment pixel index
                ptrImgBufferPixelInt = ptrBackbuffer + pxIdx;
                ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + pxIdx;
            }
            rFillLoc--;

            //add range to queue
            FloodFillRange r = new FloodFillRange(lFillLoc, rFillLoc, y);
            ranges.Enqueue(ref r);
        }

        //################################################### Scanline Floodfill ###################################################

    }
}
