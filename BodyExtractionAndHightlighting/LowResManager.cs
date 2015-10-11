﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;
using System.Threading;
using System.Collections;
using System.Diagnostics;

namespace BodyExtractionAndHightlighting
{
    public abstract class LowResManager : BasicManager<DepthSpacePoint>
    {
        protected unsafe volatile ColorSpacePoint* ptrDepthToColorSpaceMapper;

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


        private static bool isInitialized = false;

        public LowResManager(IntPtr ptrBackbuffer, IReadOnlyDictionary<JointType, Joint> bodyJoints, byte userTransparency, ushort[] depthDataSource)
            : base(ptrBackbuffer, bodyJoints, userTransparency, depthDataSource)
        {
            // Empty on purpose
        }

        protected Dictionary<JointType, DepthSpacePoint> GetRightArmJointsDepthSpace()
        {
            return this.convertBodyJointsToImageSpace(base.GetRightArmCameraSpace());
        }

        /*
         * Example usage: call convertBodyJoints(base.GetRightArmJoints())
         * */
        protected override Dictionary<JointType, DepthSpacePoint> convertBodyJointsToImageSpace(Dictionary<JointType, CameraSpacePoint> bodyJoints)
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

        protected override bool isDepthDifferent(int idxCurrDepthPoint, int xNext, int yNext)
        {
            int depthCurrent = base.getDepth(idxCurrDepthPoint);

            int idxDepthSpace = (int)(yNext * bodyIndexSensorBufferWidth + xNext + 0.5);
            int depthNext = base.getDepth(idxDepthSpace);

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
        protected unsafe void SetCoordinateMapper(ColorSpacePoint* ptrDepthToColorSpaceMapper)
        {
            this.ptrDepthToColorSpaceMapper = ptrDepthToColorSpaceMapper;
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

                DepthSpacePoint bodyPoint = coordinateMapper.MapCameraPointToDepthSpace(base.GetAnyBodyPoint());

                switch (bodyFillVariant)
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
                        thread = new Thread(() => recursiveFloodfillBody((int)(bodyPoint.X + 0.5), (int)(bodyPoint.Y + 0.5)), Constants.STACK_SIZE_LOWRES);
                        thread.Start();
                        thread.Join();
                        Console.Out.Write("Recursive;" + recursiveCount + ";");
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
                Console.Out.WriteLine("No Joint tracked!");
                thread = new Thread(() => sequentialFillBody(), Constants.STACK_SIZE_LOWRES);
                stopwatch = Stopwatch.StartNew();
                thread.Start();
                thread.Join();
                stopwatch.Stop();
            }
            Constants.NUMBEROFCALLS++;
            Constants.TOTALELAPSEDTIME += stopwatch.ElapsedMilliseconds;
            //Console.WriteLine("Elapsed time for {0}: {1}ms", Constants.floodfillType, stopwatch.ElapsedMilliseconds);
        }

        private unsafe void recursiveFloodfillBody(int xStart, int yStart)
        {
            ++recursiveCount;
            if ((xStart >= bodyIndexSensorBufferWidth) || (xStart < 0) || (yStart >= bodyIndexSensorBufferHeight) || (yStart < 0))
            {
                return;
            }

            //pixel already visited
            int idxDepthSpace = yStart * bodyIndexSensorBufferWidth + xStart;
            if (*(ptrBodyIndexSensorBuffer + idxDepthSpace) == 0xff)
            {
                return;
            }
            else
            {
                *(ptrBodyIndexSensorBuffer + idxDepthSpace) = 0xff; //do not visit same pixel twice
                this.drawColorPixel(idxDepthSpace);
            }

            //4-way neighbourhood to visit all pixels of hand (can have background pixel btw fingers)
            this.recursiveFloodfillBody((xStart + 1), yStart);
            this.recursiveFloodfillBody((xStart - 1), yStart);
            this.recursiveFloodfillBody(xStart, (yStart + 1));
            this.recursiveFloodfillBody(xStart, (yStart - 1));
        }

        /*
         * call:
         * 1) set buffers
         * 2) choose pixel position to write to in colorbuffer
         * 3) check color value boundaries (lookups from mapper)
         * 4) draw into backbuffer
         * */
        private unsafe void drawColorPixel(int idxDepthSpace)  {
            //pixel target
            ptrBackbufferPixelInt = null;
            ptrColorSensorBufferPixelInt = null;

            //lookup color pixel from color stream at body index position
            //first, depth index is incremented by param, then mapper gets color value
            //TODO what is depth space? = 512x424 body index
            int colorPointX = (int)((ptrDepthToColorSpaceMapper + idxDepthSpace)->X + 0.5);
            int colorPointY = (int)((ptrDepthToColorSpaceMapper + idxDepthSpace)->Y + 0.5);

            if ((colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) && (colorPointY >= 0) && (colorPointX >= 0))
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

        private unsafe void floodfill_BreadthFirst(int xStart, int yStart)
        {
            LinkedList<int> queue = new LinkedList<int>();

            queue.AddFirst(xStart);
            queue.AddFirst(yStart);

            int maxQueueSize = 0;

            int lastX, lastY;
            int idxDepthSpace;
            while (queue.Count != 0)
            {
                lastX = queue.Last.Value;
                queue.RemoveLast();
                lastY = queue.Last.Value;
                queue.RemoveLast();
                if ((lastX >= 0) && (lastX < bodyIndexSensorBufferWidth) && (lastY >= 0) && (lastY < bodyIndexSensorBufferHeight))
                {
                    idxDepthSpace = (int)(lastY * bodyIndexSensorBufferWidth + lastX);
                    if (*(ptrBodyIndexSensorBuffer + idxDepthSpace) != 0xff)
                    {
                        this.drawColorPixel(idxDepthSpace);
                        *(ptrBodyIndexSensorBuffer + idxDepthSpace) = 0xff; //do not visit same pixel twice
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

                if (queue.Count > maxQueueSize)
                {
                    maxQueueSize = queue.Count();
                }
            }
            Console.Out.Write("BFSmaxQueueSize;" + maxQueueSize + ";");
        }

        // Slower compared to floodfill_BreadthFirst
        /*
        private unsafe void floodfill_BreadthFirst_Point(int xStart, int yStart)
        {
            LinkedList<Point> queue = new LinkedList<Point>();
            queue.AddFirst(new Point(xStart, yStart));
            int maxQueueSize = 0;

            Point p;
            int idxDepthSpace;
            while (queue.Count != 0)
            {
                p = queue.Last.Value;
                queue.RemoveLast();
                if ((p.X >= 0) && (p.X < bodyIndexSensorBufferWidth) && (p.Y >= 0) && (p.Y < bodyIndexSensorBufferHeight))
                {
                    idxDepthSpace = (int)(p.Y * bodyIndexSensorBufferWidth + p.X);
                    if (*(ptrBodyIndexSensorBuffer + idxDepthSpace) != 0xff)
                    {
                        this.drawColorPixel(idxDepthSpace);
                        *(ptrBodyIndexSensorBuffer + idxDepthSpace) = 0xff; //do not visit same pixel twice
                        queue.AddFirst(new Point(p.X + 1, p.Y));
                        queue.AddFirst(new Point(p.X - 1, p.Y));
                        queue.AddFirst(new Point(p.X, p.Y + 1));
                        queue.AddFirst(new Point(p.X, p.Y - 1));
                    }
                }

                if (queue.Count > maxQueueSize)
                {
                    maxQueueSize = queue.Count();
                }
            }
            //Console.Out.Write("Breath first queue size:" + maxQueueSize);
        }
         * */

        private unsafe void floodfill_DepthFirst(int x, int y)
        {
            Stack<int> stack = new Stack<int>();

            stack.Push(x);
            stack.Push(y);

            int maxStackSize = 0;

            int idxDepthSpace;
            int lastX, lastY;
            while (stack.Count != 0)
            {
                lastY = stack.Pop();
                lastX = stack.Pop();
                if ((lastX >= 0) && (lastX < bodyIndexSensorBufferWidth) && (lastY >= 0) && (lastY < bodyIndexSensorBufferHeight))
                {
                    idxDepthSpace = (int)(lastY * bodyIndexSensorBufferWidth + lastX);
                    if (*(ptrBodyIndexSensorBuffer + idxDepthSpace) != 0xff)
                    {
                        this.drawColorPixel(idxDepthSpace);
                        *(ptrBodyIndexSensorBuffer + idxDepthSpace) = 0xff; //do not visit same pixel twice
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
                if (stack.Count > maxStackSize)
                {
                    maxStackSize = stack.Count;
                }
            }
            Console.Out.Write("DFSmaxStackSize;" + maxStackSize + ";");
        }

        // Slower compared to floodfill_DepthFirst
        /*
        private unsafe void floodfill_DepthFirst_Point(int x, int y)
        {
            Stack<Point> stack = new Stack<Point>();

            stack.Push(new Point(x, y));
            int maxStackSize = 0;

            int idxDepthSpace;
            Point p;
            while (stack.Count != 0)
            {
                p = stack.Pop();
                if ((p.X >= 0) && (p.X < bodyIndexSensorBufferWidth) && (p.Y >= 0) && (p.Y < bodyIndexSensorBufferHeight))
                {
                    idxDepthSpace = (int)(p.Y * bodyIndexSensorBufferWidth + p.X);
                    if (*(ptrBodyIndexSensorBuffer + idxDepthSpace) != 0xff)
                    {
                        this.drawColorPixel(idxDepthSpace);
                        *(ptrBodyIndexSensorBuffer + idxDepthSpace) = 0xff; //do not visit same pixel twice
                        stack.Push(new Point(p.X + 1, p.Y));
                        stack.Push(new Point(p.X - 1, p.Y));
                        stack.Push(new Point(p.X, p.Y + 1));
                        stack.Push(new Point(p.X, p.Y - 1));
                    }
                }
                if (stack.Count > maxStackSize)
                {
                    maxStackSize = stack.Count;
                }
            }
            //Console.Out.Write("Depth first stack size:" + maxStackSize);
        }
         * */

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
            Console.Write("Sequential;" + bodyIndexImageLength + ";");
        }
    }
}
