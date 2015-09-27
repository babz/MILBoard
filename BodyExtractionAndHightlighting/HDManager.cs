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
<<<<<<< Updated upstream
                if (Constants.floodfillType == Constants.FloodfillType.BFS)
                {
                    floodfill_BreadthFirst((int)(bodyPoint.X + 0.5), (int)(bodyPoint.Y + 0.5));
                }
                else if (Constants.floodfillType == Constants.FloodfillType.DFS)
                {
                    floodfill_DepthFirst((int)(bodyPoint.X + 0.5), (int)(bodyPoint.Y + 0.5));
                }
                else if (Constants.floodfillType == Constants.FloodfillType.LinefillRec)
                {
                    //linefillBody((int)(bodyPoint.X + 0.5), (int)(bodyPoint.Y + 0.5));
                    thread = new Thread(() => linefillBody((int)(bodyPoint.X + 0.5), (int)(bodyPoint.Y + 0.5)), Constants.LINEFILL_HD);
                    thread.Start();
                    thread.Join();
                }
                else if (Constants.floodfillType == Constants.FloodfillType.FloodfillRec)
                {
                    thread = new Thread(() => floodfillBody((int)(bodyPoint.X + 0.5), (int)(bodyPoint.Y + 0.5)), Constants.STACK_SIZE_HD);
                    thread.Start();
                    thread.Join();
=======

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
>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
        /*
         * call:
            1) check boundaries

            2) check pixel already visited

            3) if not visited:
	            a) set visited
	            b) draw color at color index (low = depth, hd = color)
	
            4) proceed (visit other pixel)
         * */
        private unsafe void floodfillBody(int xStart, int yStart)
=======


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
>>>>>>> Stashed changes
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
                this.drawColorPixel(idxCurrColorPixel);

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
<<<<<<< Updated upstream
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
            int idxDepthSpace = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
            if (Single.IsInfinity(xDepthPixel) || Single.IsInfinity(yDepthPixel) || (*(ptrBodyIndexSensorBuffer + idxDepthSpace) == 0xff))
            {
                return;
            }

=======
>>>>>>> Stashed changes
            queue.AddFirst(xStart);
            queue.AddFirst(yStart);

            int maxQueueSize = 0;
<<<<<<< Updated upstream
            int lastX, lastY;
            while (queue.Count != 0)
            {
                lastX = queue.Last.Value;
                queue.RemoveLast();
                lastY = queue.Last.Value;
                queue.RemoveLast();

                if ((lastX >= 0) && (lastX < colorSensorBufferWidth) && (lastY >= 0) && (lastY < colorSensorBufferHeight))
                {
                    idxCurrColorPixel = lastY * colorSensorBufferWidth + lastX;
                    if (idxCurrColorPixel < colorImageLength)
                    {
                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxCurrColorPixel;
                        if ((*ptrColorSensorBufferPixelInt) != 0xFF000000)
                        {
                            xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->X;
                            yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->Y;
                            idxDepthSpace = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
                            if (!Single.IsInfinity(xDepthPixel) && !Single.IsInfinity(yDepthPixel) && (*(ptrBodyIndexSensorBuffer + idxDepthSpace) != 0xff))
                            {
                                this.drawColorPixel(idxCurrColorPixel);
=======

            while (queue.Count != 0)
            {
                int lastX = queue.Last();
                queue.RemoveLast();
                int lastY = queue.Last();
                queue.RemoveLast();

                //############################

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
>>>>>>> Stashed changes

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
<<<<<<< Updated upstream
                            }
                        }
                    }
=======
                            }                            
                        }                        
                    }                   
>>>>>>> Stashed changes
                }
                if (queue.Count > maxQueueSize)
                {
                    maxQueueSize = queue.Count();
                }
            }
<<<<<<< Updated upstream
            Console.Out.Write("Breath first queue size:" + maxQueueSize);
        }

        private unsafe void floodfill_DepthFirst(int x, int y)
        {
            if ((x >= colorSensorBufferWidth) || (x < 0) || (y >= colorSensorBufferHeight) || (y < 0))
            {
                return;
            }

            int idxCurrColorPixel = y * colorSensorBufferWidth + x;
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

=======
            Console.Out.Write("H_BFSmaxQueueSize;" + maxQueueSize + ";");
        }



        private unsafe void floodfill_DepthFirst(int x, int y)
        {
>>>>>>> Stashed changes
            stack.Push(x);
            stack.Push(y);

            int maxStackSize = 0;
<<<<<<< Updated upstream
            int lastX, lastY;
            while (stack.Count != 0)
            {
                lastY = stack.Pop();
                lastX = stack.Pop();
                if ((lastX >= 0) && (lastX < colorSensorBufferWidth) && (lastY >= 0) && (lastY < colorSensorBufferHeight))
                {
                    idxCurrColorPixel = lastY * colorSensorBufferWidth + lastX;
                    if (idxCurrColorPixel < colorImageLength)
                    {
                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxCurrColorPixel;
                        if ((*ptrColorSensorBufferPixelInt) != 0xFF000000)
                        {
                            xDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->X;
                            yDepthPixel = (ptrColorToDepthSpaceMapper + idxCurrColorPixel)->Y;
                            idxDepthPixel = (int)(((int)(yDepthPixel + 0.5)) * bodyIndexSensorBufferWidth + xDepthPixel + 0.5);
                            if (!Single.IsInfinity(xDepthPixel) && !Single.IsInfinity(yDepthPixel) && (*(ptrBodyIndexSensorBuffer + idxDepthPixel) != 0xff))
                            {
                                this.drawColorPixel(idxCurrColorPixel);
=======

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
>>>>>>> Stashed changes

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
<<<<<<< Updated upstream
                }
=======
                }          
>>>>>>> Stashed changes
                if (stack.Count > maxStackSize)
                {
                    maxStackSize = stack.Count;
                }
            }
<<<<<<< Updated upstream
            Console.Out.Write("Depth first stack size:" + maxStackSize);
        }

        /*
         * call:
         * 1) set buffers
         * 2) choose pixel position to write to in colorbuffer
         * 3) draw into backbuffer
         * */
        private unsafe void drawColorPixel(int idxColorSpace)
        {
            //pixel target
            ptrBackbufferPixelInt = null;
            ptrColorSensorBufferPixelInt = null;

            // point to current pixel in image buffer
            ptrBackbufferPixelInt = ptrBackbuffer + idxColorSpace;

            // point to according pixel in color buffer
            ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + idxColorSpace;
            
            // assign color value (4 bytes)
            *ptrBackbufferPixelInt = *ptrColorSensorBufferPixelInt;
            // overwrite the alpha value (last byte)
            *(((byte*)ptrBackbufferPixelInt) + 3) = this.userTransparency;
        }
=======
            Console.Out.Write("H_DFSmaxStackSize;" + maxStackSize + ";");
        }



>>>>>>> Stashed changes

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
