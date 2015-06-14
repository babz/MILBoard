using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;
using System.Threading;
using System.Collections;

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
                thread = new Thread(() => linefillBody((int)(bodyPoint.X + 0.5), (int)(bodyPoint.X + 0.5), (int)(bodyPoint.Y + 0.5)), Constants.LINEFILL_LOWRES);
                //this.linefillBody((int)(bodyPoint.X + 0.5), (int)(bodyPoint.X + 0.5), (int)(bodyPoint.Y + 0.5));
                //thread = new Thread(() => floodfillBody((int)(bodyPoint.X + 0.5), (int)(bodyPoint.Y + 0.5)), Constants.STACK_SIZE_LOWRES);
                thread.Start();
                thread.Join();
            }
            else
            {
                thread = new Thread(() => sequentialFillBody(), Constants.STACK_SIZE_LOWRES);
                thread.Start();
                thread.Join();
            }
            
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

        //python floodfill
        //unsafe floodfill: http://www.codeproject.com/Articles/5133/Flood-Fill-Algorithms-in-C-and-GDI
        private unsafe void linefillBody(int xStartLeft, int xStartRight, int yStart)
        {
            if ((xStartLeft >= bodyIndexSensorBufferWidth) || (xStartLeft < 0) || (yStart >= bodyIndexSensorBufferHeight) || (yStart < 0))
            {
                return;
            }

            //pixel target
            uint* ptrBackbufferPixelInt = null;
            uint* ptrColorSensorBufferPixelInt = null;
            bool isColorPixelInValidRange = false;
            int xLeft, xRight;

            //pixel already visited
            int idxDepthSpace = yStart * bodyIndexSensorBufferWidth + xStartLeft;
            if (*(ptrBodyIndexSensorBuffer + idxDepthSpace) == 0xff)
            {
                return;
            }
            // ===scan left side
            int idxDepthSpaceLeft = idxDepthSpace;
            for (xLeft = xStartLeft; xLeft >= 0; --xLeft)
            {
                if (*(ptrBodyIndexSensorBuffer + idxDepthSpaceLeft) == 0xff)
                {
                    break;
                }
                *(ptrBodyIndexSensorBuffer + idxDepthSpaceLeft) = 0xff; //do not visit same pixel twice
                int colorPointX = (int)((ptrDepthToColorSpaceMapper + idxDepthSpaceLeft)->X + 0.5);
                int colorPointY = (int)((ptrDepthToColorSpaceMapper + idxDepthSpaceLeft)->Y + 0.5);

                if ((colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) &&
                        (colorPointY >= 0) && (colorPointX >= 0))
                {
                    isColorPixelInValidRange = true;
                }

                if (isColorPixelInValidRange)
                {
                    // point to current pixel in image buffer
                    ptrBackbufferPixelInt = ptrBackbuffer + idxDepthSpaceLeft;

                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
                    // assign color value (4 bytes)
                    *ptrBackbufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrBackbufferPixelInt) + 3) = this.userTransparency;
                }
                isColorPixelInValidRange = false;
                idxDepthSpaceLeft--;
            }
            if (xLeft < xStartLeft)
            {
                this.linefillBody(xLeft, xStartLeft, (yStart + 1));
                this.linefillBody(xLeft, xStartLeft, (yStart - 1));
                ++xStartLeft;
            }

            // ===scan right side
            int idxDepthSpaceRight = idxDepthSpace + 1;
            for (xRight = xStartRight; xRight < bodyIndexSensorBufferWidth; ++xRight)
            {
                if (*(ptrBodyIndexSensorBuffer + idxDepthSpaceRight) == 0xff)
                {
                    break;
                }
                *(ptrBodyIndexSensorBuffer + idxDepthSpaceRight) = 0xff; //do not visit same pixel twice
                int colorPointX = (int)((ptrDepthToColorSpaceMapper + idxDepthSpaceRight)->X + 0.5);
                int colorPointY = (int)((ptrDepthToColorSpaceMapper + idxDepthSpaceRight)->Y + 0.5);

                if ((colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) &&
                        (colorPointY >= 0) && (colorPointX >= 0))
                {
                    isColorPixelInValidRange = true;
                }

                if (isColorPixelInValidRange)
                {
                    // point to current pixel in image buffer
                    ptrBackbufferPixelInt = ptrBackbuffer + idxDepthSpaceRight;

                    ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
                    // assign color value (4 bytes)
                    *ptrBackbufferPixelInt = *ptrColorSensorBufferPixelInt;
                    // overwrite the alpha value (last byte)
                    *(((byte*)ptrBackbufferPixelInt) + 3) = this.userTransparency;
                }
                isColorPixelInValidRange = false;
                idxDepthSpaceRight++;
            }
            if (xRight > xStartRight)
            {
                this.linefillBody(xStartRight, xRight, (yStart + 1));
                this.linefillBody(xStartRight, xRight, (yStart - 1));
                --xStartRight;
            }

            //xR ... xRight
            //x2 ... xStartRight
            //xL ... xLeft
            //x1 ... xStartLeft
            int idxDepthSpaceBetween = idxDepthSpace;
            for (xRight = xStartLeft; xRight <= xStartRight && xRight < bodyIndexSensorBufferWidth; ++xRight)
            {
                if (*(ptrBodyIndexSensorBuffer + idxDepthSpaceBetween) != 0xff)
                {
                    *(ptrBodyIndexSensorBuffer + idxDepthSpaceBetween) = 0xff; //do not visit same pixel twice
                    int colorPointX = (int)((ptrDepthToColorSpaceMapper + idxDepthSpaceBetween)->X + 0.5);
                    int colorPointY = (int)((ptrDepthToColorSpaceMapper + idxDepthSpaceBetween)->Y + 0.5);

                    if ((colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) &&
                            (colorPointY >= 0) && (colorPointX >= 0))
                    {
                        isColorPixelInValidRange = true;
                    }

                    if (isColorPixelInValidRange)
                    {
                        // point to current pixel in image buffer
                        ptrBackbufferPixelInt = ptrBackbuffer + idxDepthSpaceBetween;

                        ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
                        // assign color value (4 bytes)
                        *ptrBackbufferPixelInt = *ptrColorSensorBufferPixelInt;
                        // overwrite the alpha value (last byte)
                        *(((byte*)ptrBackbufferPixelInt) + 3) = this.userTransparency;
                    }
                    isColorPixelInValidRange = false;
                    idxDepthSpaceRight++;
                }
                else
                {
                    if (xStartLeft < xRight)
                    {
                        this.linefillBody(xStartLeft, xRight-1, (yStart + 1));
                        this.linefillBody(xStartLeft, xRight-1, (yStart - 1));
                        xStartLeft = xRight;
                    }

                    for (; xRight <= xStartRight && xRight < bodyIndexSensorBufferWidth; ++xRight)
                    {
                        if (*(ptrBodyIndexSensorBuffer + idxDepthSpaceBetween) != 0xff)
                        {
                            xStartLeft = xRight--;
                            break;
                        }
                    }
                }
            }

            //scan betweens
            //for (xRight = xStartLeft; xRight <= xStartRight && xRight <= bodyIndexSensorBufferWidth; ++xRight)
            //{
            //    if (*(ptrBodyIndexSensorBuffer + idxDepthSpaceRight) != 0xff)
            //    {
            //        *(ptrBodyIndexSensorBuffer + idxDepthSpaceRight) = 0xff; //do not visit same pixel twice
            //        int colorPointX = (int)((ptrDepthToColorSpaceMapper + idxDepthSpaceRight)->X + 0.5);
            //        int colorPointY = (int)((ptrDepthToColorSpaceMapper + idxDepthSpaceRight)->Y + 0.5);

            //        if ((colorPointY < colorSensorBufferHeight) && (colorPointX < colorSensorBufferWidth) &&
            //                (colorPointY >= 0) && (colorPointX >= 0))
            //        {
            //            isColorPixelInValidRange = true;
            //        }

            //        if (isColorPixelInValidRange)
            //        {
            //            // point to current pixel in image buffer
            //            ptrBackbufferPixelInt = ptrBackbuffer + idxDepthSpaceRight;

            //            ptrColorSensorBufferPixelInt = ptrColorSensorBufferInt + (colorPointY * colorSensorBufferWidth + colorPointX);
            //            // assign color value (4 bytes)
            //            *ptrBackbufferPixelInt = *ptrColorSensorBufferPixelInt;
            //            // overwrite the alpha value (last byte)
            //            *(((byte*)ptrBackbufferPixelInt) + 3) = this.userTransparency;
            //        }
            //        isColorPixelInValidRange = false;
            //        idxDepthSpaceRight++;
            //    }
            //    else
            //    {
            //        if (xStartLeft < xRight)
            //        {
            //            this.linefillBody(xStartLeft, xRight - 1, yStart - 1);
            //            this.linefillBody(xStartLeft, xRight - 1, yStart + 1);
            //        }
                    
            //    }
            //}
        }

        //the stack
        int stackSize = 16777216;
        int[] stack = new int[16777216];
        int stackPointer;
        //http://lodev.org/cgtutor/floodfill.html#Scanline_Floodfill_Algorithm_With_Stack
        //http://www.cs.tufts.edu/~sarasu/courses/comp175-2009fa/pdf/comp175-04-region-filling.pdf
        //private unsafe void linefillBody_Stack(int xStart, int yStart)
        //{
        //    Stack stack = new Stack();

        //    if ((xStart >= bodyIndexSensorBufferWidth) || (xStart < 0) || (yStart >= bodyIndexSensorBufferHeight) || (yStart < 0))
        //    {
        //        return;
        //    }

        //    //pixel target
        //    uint* ptrBackbufferPixelInt = null;
        //    uint* ptrColorSensorBufferPixelInt = null;
        //    bool isColorPixelInValidRange = false;
        //    int xLeft, xRight;

        //    //pixel already visited
        //    int idxDepthSpace = yStart * bodyIndexSensorBufferWidth + xStart;
        //    if (*(ptrBodyIndexSensorBuffer + idxDepthSpace) == 0xff)
        //    {
        //        return;
        //    }

        //    emptyStack(stack);

        //    int y1;
        //    bool spanLeft, spanRight;

        //    if (!push(x, y)) { 
        //        return; 
        //    }

        //    while (pop(x, y))
        //    {
        //        y1 = y;
        //        while (y1 >= 0 && screenBuffer[x][y1] == oldColor) y1--;
        //        y1++;
        //        spanLeft = spanRight = 0;
        //        while (y1 < h && screenBuffer[x][y1] == oldColor)
        //        {
        //            screenBuffer[x][y1] = newColor;
        //            if (!spanLeft && x > 0 && screenBuffer[x - 1][y1] == oldColor)
        //            {
        //                if (!push(x - 1, y1)) return;
        //                spanLeft = 1;
        //            }
        //            else if (spanLeft && x > 0 && screenBuffer[x - 1][y1] != oldColor)
        //            {
        //                spanLeft = 0;
        //            }
        //            if (!spanRight && x < w - 1 && screenBuffer[x + 1][y1] == oldColor)
        //            {
        //                if (!push(x + 1, y1)) return;
        //                spanRight = 1;
        //            }
        //            else if (spanRight && x < w - 1 && screenBuffer[x + 1][y1] != oldColor)
        //            {
        //                spanRight = 0;
        //            }
        //            y1++;
        //        }
        //    }
        //}

        //bool pop(int x, int y)
        //{
        //    if(stackPointer > 0)
        //    {
        //        int p = stack[stackPointer];
        //        x = p / h;
        //        y = p % h;
        //        stackPointer--;
        //        return true;
        //    }    
        //    else
        //    {
        //        return false;
        //    }   
        //}   
 
        //bool push(int x, int y)
        //{
        //    if(stackPointer < stackSize - 1)
        //    {
        //        stackPointer++;
        //        stack[stackPointer] = h * x + y;
        //        return true;
        //    }    
        //    else
        //    {
        //        return false;
        //    }   
        //}    

        //void emptyStack()
        //{
        //    int x, y;
        //    while(pop(x, y));
        //}

        //private void emptyStack(Stack stack)
        //{
        //    while (stack.Count != 0) {
        //        stack.Pop() ;
        //    }
        //}

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
