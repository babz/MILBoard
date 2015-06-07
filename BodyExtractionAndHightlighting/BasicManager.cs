using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows;

namespace BodyExtractionAndHightlighting
{
    public abstract class BasicManager<T>
    {
        protected float InferredZPositionClamp = 0.1f;

        protected CoordinateMapper coordinateMapper;

        protected int bodyIndexSensorBufferWidth, bodyIndexSensorBufferHeight, colorSensorBufferWidth, colorSensorBufferHeight;
        protected byte userTransparency;
        protected int bodyIndexImageLength, colorImageLength;

        protected volatile unsafe byte* ptrBodyIndexSensorBuffer;
        protected volatile unsafe uint* ptrBackbuffer, ptrColorSensorBufferInt; //unmanaged

        private CameraSpacePoint anyBodyJoint;
        private Dictionary<JointType, CameraSpacePoint> rightArmJoints;
        private IReadOnlyDictionary<JointType, Joint> bodyJoints;

        public BasicManager(IntPtr ptrBackbuffer, IReadOnlyDictionary<JointType, Joint> bodyJoints, byte userTransparency)
        {
            this.SetBackbuffer(ptrBackbuffer);
            this.coordinateMapper = Constants.GetCoordinateMapper();

            this.bodyIndexSensorBufferWidth = Constants.GetBodyIndexSensorBufferWidth();
            this.bodyIndexSensorBufferHeight = Constants.GetBodyIndexSensorBufferHeight();
            this.colorSensorBufferWidth = Constants.GetColorSensorBufferWidth();
            this.colorSensorBufferHeight = Constants.GetColorSensorBufferHeight();
            this.userTransparency = userTransparency;

            this.bodyIndexImageLength = bodyIndexSensorBufferWidth * bodyIndexSensorBufferHeight;
            this.colorImageLength = colorSensorBufferWidth * colorSensorBufferHeight;
            this.bodyJoints = bodyJoints;
            this.rightArmJoints = null;
        }

        protected abstract Dictionary<JointType, T> convertBodyJoints(Dictionary<JointType, CameraSpacePoint> bodyJoints);

        protected abstract void drawFullBody();

        /*
         * Source: http://www.codeproject.com/Articles/6017/QuickFill-An-efficient-flood-fill-algorithm
         * http://www.crbond.com/papers/fldfill_v2.pdf
         * */
        protected void quickFillBody()
        {

        }

        protected bool IsAnyJointTracked()
        {
            if (bodyJoints == null)
            {
                return false;
            }
            foreach (KeyValuePair<JointType, Joint> entry in bodyJoints)
            {
                if (entry.Value.TrackingState == TrackingState.Tracked)
                {
                    anyBodyJoint = entry.Value.Position;
                    return true;
                }
            }
            return false;
        }

        protected bool isRightArmTracked()
        {
            if (bodyJoints == null)
            {
                return false;
            }

            bool isShoulderTracked = false;
            bool isElbowTracked = false;
            bool isWristTracked = false;
            bool isHandTracked = false;
            bool isHandTipTracked = false;

            rightArmJoints = new Dictionary<JointType, CameraSpacePoint>();
            foreach (KeyValuePair<JointType, Joint> entry in bodyJoints)
            {
                if (entry.Value.TrackingState != TrackingState.Tracked)
                {
                    continue;
                }

                if (entry.Key == JointType.ShoulderRight)
                {
                    isShoulderTracked = true;
                    rightArmJoints[entry.Key] = entry.Value.Position;
                }
                else if (entry.Key == JointType.ElbowRight)
                {
                    isElbowTracked = true;
                    rightArmJoints[entry.Key] = entry.Value.Position;
                }
                else if (entry.Key == JointType.WristRight)
                {
                    isWristTracked = true;
                    rightArmJoints[entry.Key] = entry.Value.Position;
                }
                else if (entry.Key == JointType.HandRight)
                {
                    isHandTracked = true;
                    rightArmJoints[entry.Key] = entry.Value.Position;
                }
                else if (entry.Key == JointType.HandTipRight)
                {
                    isHandTipTracked = true;
                    rightArmJoints[entry.Key] = entry.Value.Position;
                }
            }

            if(isShoulderTracked && isElbowTracked && isWristTracked && isHandTracked && isHandTipTracked) 
            {
                return true;
            } 
            else 
            {
                rightArmJoints = null;
                return false;
            }
        }

        /* check isRightArmTracked first!
         * https://msdn.microsoft.com/en-us/library/windowspreview.kinect.cameraspacepoint.aspx
         * @return null if not tracked
         * */
        protected Dictionary<JointType, CameraSpacePoint> GetRightArmCameraSpace()
        {
            return this.rightArmJoints;
        }

        /* check isAnyJointTracked first!
         * @return undefined value in case no joint is tracked
         * */
        protected CameraSpacePoint GetAnyBodyPoint()
        {
            return this.anyBodyJoint;
        }

        protected unsafe void SetRequiredBuffers(byte* ptrBodyIndexSensorBuffer, uint* ptrColorSensorBufferInt)
        {
            this.ptrBodyIndexSensorBuffer = ptrBodyIndexSensorBuffer;
            this.ptrColorSensorBufferInt = ptrColorSensorBufferInt;
        }

        private unsafe void SetBackbuffer(IntPtr ptrBackbufferManaged)
        {
            this.ptrBackbuffer = (uint*)ptrBackbufferManaged;
        }

    }
}
