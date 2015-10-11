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
        private FloodFillRangeQueue ranges;

        private LinkedList<int> queue = new LinkedList<int>();
        private Stack<int> stack = new Stack<int>();

        private long recursiveCount = 0;

        enum BodyFillVariant
        {
            sequentialFill,         
            recursiveFloodfill,     
            DFSfloodfill,           
            BFSfloodfill            
        };

        // todo: change variant for performance tests
        //#########################################################################
        BodyFillVariant bodyFillVariant = BodyFillVariant.BFSfloodfill;
        //#########################################################################

        static bool isInitialized = false;




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
            if ((Constants.floodfillType != Constants.FloodfillType.NoFF) && base.IsAnyJointTracked())
            {
                if (!isInitialized)
                {
                    isInitialized = true;
                    Console.WriteLine("Variant: " + bodyFillVariant);
                }

                ColorSpacePoint bodyPoint = coordinateMapper.MapCameraPointToColorSpace(base.GetAnyBodyPoint());

                switch(bodyFillVariant)
                {
                    case BodyFillVariant.BFSfloodfill:
                        thread = new Thread(() => floodfill_BreadthFirst((int)(bodyPoint.X + 0.5), (int)(bodyPoint.Y + 0.5)));
                        thread.Start();
                        thread.Join();
                        break;
                    case BodyFillVariant.DFSfloodfill:
                        thread = new Thread(() => floodfill_DepthFirst((int)(bodyPoint.X + 0.5), (int)(bodyPoint.Y + 0.5)));
                        thread.Start();
                        thread.Join();
                        break;
                    case BodyFillVariant.recursiveFloodfill:
                        recursiveCount = 0;
                        thread = new Thread(() => recursiveFloodfillBody((int)(bodyPoint.X + 0.5), (int)(bodyPoint.Y + 0.5)), Constants.STACK_SIZE_HD);
                        thread.Start();
                        thread.Join();
                        Console.Out.Write("H_Recursive;" + recursiveCount + ";");
                        break;
                    case BodyFillVariant.sequentialFill:
                        thread = new Thread(() => sequentialFillBody(), Constants.STACK_SIZE_LOWRES);
                        thread.Start();
                        thread.Join();
                        break;
                }
            }
            else
            {
                Console.WriteLine("H_No joint tracked!");
                thread = new Thread(() => sequentialFillBody(), Constants.STACK_SIZE_LOWRES);
                thread.Start();
                thread.Join();
            }
        }

        //############################################################
        // Variant 1: SequentialFillBody
        // - No recursion
        // - Run through all pixels
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
            Console.Write("H_Sequential;" + colorImageLength + ";");
        }

        //############################################################
        // Variant 2: recursiveFloodfillBody
        // - Start from a joint
        // - Recursively call function again
        private unsafe void recursiveFloodfillBody(int xStart, int yStart)
        {
            ++recursiveCount;
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

            //lookup body idx if pixel covers body and return if not
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
            this.recursiveFloodfillBody((xStart + 1), yStart);
            this.recursiveFloodfillBody((xStart - 1), yStart);
            this.recursiveFloodfillBody(xStart, (yStart + 1));
            this.recursiveFloodfillBody(xStart, (yStart - 1));
        }

        private unsafe void floodfill_BreadthFirst(int xStart, int yStart)
        {
            //if ((xStart >= colorSensorBufferWidth) || (xStart < 0) || (yStart >= colorSensorBufferHeight) || (yStart < 0))
            //{
            //    return;
            //}

            //int idxCurrColorPixel = yStart * colorSensorBufferWidth + xStart;
            //if (idxCurrColorPixel >= colorImageLength)
            //{
            //    return;
            //}

            ////pixel already visited
            //uint* ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxCurrColorPixel;
            //if ((*ptrColorSensorBufferPixelInt) == 0xFF000000)
            //{
            //    return;
            //}

            //float xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->X;
            //float yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->Y;
            //int idxDepthSpace = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
            //if (Single.IsInfinity(xDepthPixel) || Single.IsInfinity(yDepthPixel) || (*(ptrBodyIndexSensorBuffer + idxDepthSpace) == 0xff))
            //{
            //    return;
            //}

            queue.AddFirst(xStart);
            queue.AddFirst(yStart);

            int maxQueueSize = 0;
            while (queue.Count != 0)
            {
                int lastX = queue.Last();
                queue.RemoveLast();
                int lastY = queue.Last();
                queue.RemoveLast();

                if ((lastX < colorSensorBufferWidth) && (lastX >= 0) && (lastY < colorSensorBufferHeight) && (lastY >= 0))
                {
                    int idxCurrColorPixel = lastY * colorSensorBufferWidth + lastX;
                    if (idxCurrColorPixel < colorImageLength)
                    {
                        //pixel already visited
                        uint* ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxCurrColorPixel;
                        if ((*ptrColorSensorBufferPixelInt) != 0xFF000000)
                        {
                            float xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->X;
                            float yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->Y;
                            int idxDepthPixel = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
                            if (!Single.IsInfinity(xDepthPixel) && !Single.IsInfinity(yDepthPixel) && (*(ptrBodyIndexSensorBuffer + idxDepthPixel) != 0xff))
                            {
                                // point to current pixel in image buffer
                                uint* ptrImgBufferPixelInt = ptrBackbuffer + (lastY * colorSensorBufferWidth + lastX);

                                // assign color value (4 bytes)
                                *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                                // overwrite the alpha value (last byte)
                                *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency);
                                //do not visit same pixel twice
                                *ptrColorSensorBufferPixelInt = 0xFF000000;

                                queue.AddFirst(lastX + 1);
                                queue.AddFirst(lastY);
                                queue.AddFirst(lastX - 1);
                                queue.AddFirst(lastY);
                                queue.AddFirst(lastX);
                                queue.AddFirst(lastY + 1);
                                queue.AddFirst(lastX);
                                queue.AddFirst(lastY - 1);
                            }
                        }
                    }
                }
                if (queue.Count > maxQueueSize)
                {
                    maxQueueSize = queue.Count();
                }
            }
            Console.Out.Write("H_BFSmaxQueueSize;" + maxQueueSize + ";");
        }

        private unsafe void floodfill_DepthFirst(int x, int y)
        {
            //if ((x >= colorSensorBufferWidth) || (x < 0) || (y >= colorSensorBufferHeight) || (y < 0))
            //{
            //    return;
            //}

            //int idxCurrColorPixel = y * colorSensorBufferWidth + x;
            //if (idxCurrColorPixel >= colorImageLength)
            //{
            //    return;
            //}

            ////pixel already visited
            //uint* ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxCurrColorPixel;
            //if ((*ptrColorSensorBufferPixelInt) == 0xFF000000)
            //{
            //    return;
            //}

            //float xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->X;
            //float yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->Y;
            //int idxDepthPixel = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
            //if (Single.IsInfinity(xDepthPixel) || Single.IsInfinity(yDepthPixel) || (*(ptrBodyIndexSensorBuffer + idxDepthPixel) == 0xff))
            //{
            //    return;
            //}

            stack.Push(x);
            stack.Push(y);

            int maxStackSize = 0;
            while (stack.Count != 0)
            {
                int lastY = stack.Pop();
                int lastX = stack.Pop();

                if ((lastX < colorSensorBufferWidth) && (lastX >= 0) && (lastY < colorSensorBufferHeight) && (lastY >= 0))
                {
                    int idxCurrColorPixel = lastY * colorSensorBufferWidth + lastX;
                    if (idxCurrColorPixel < colorImageLength)
                    {
                        //pixel already visited
                        uint* ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxCurrColorPixel;
                        if ((*ptrColorSensorBufferPixelInt) != 0xFF000000)
                        {
                            float xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->X;
                            float yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->Y;
                            int idxDepthPixel = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
                            if (!Single.IsInfinity(xDepthPixel) && !Single.IsInfinity(yDepthPixel) && (*(ptrBodyIndexSensorBuffer + idxDepthPixel) != 0xff))
                            {
                                // point to current pixel in image buffer
                                uint* ptrImgBufferPixelInt = ptrBackbuffer + (lastY * colorSensorBufferWidth + lastX);

                                // assign color value (4 bytes)
                                *ptrImgBufferPixelInt = *ptrColorSensorBufferPixelInt;
                                // overwrite the alpha value (last byte)
                                *(((byte*)ptrImgBufferPixelInt) + 3) = (byte)(this.userTransparency);
                                //do not visit same pixel twice
                                *ptrColorSensorBufferPixelInt = 0xFF000000;

                                stack.Push(lastX + 1);
                                stack.Push(lastY);
                                stack.Push(lastX - 1);
                                stack.Push(lastY);
                                stack.Push(lastX);
                                stack.Push(lastY + 1);
                                stack.Push(lastX);
                                stack.Push(lastY - 1);
                            }
                        }
                    }
                }
                if (stack.Count > maxStackSize)
                {
                    maxStackSize = stack.Count;
                }
            }
            Console.Out.Write("H_DFSmaxStackSize;" + maxStackSize + ";");
        }

    }
}
