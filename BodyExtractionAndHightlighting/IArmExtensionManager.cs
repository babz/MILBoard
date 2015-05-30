using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BodyExtractionAndHightlighting
{
    public interface IArmExtensionManager
    {
        unsafe void processImage(IntPtr ptrBackbuffer);

        unsafe void processImage_rotationOnly(IntPtr ptrBackbuffer);

        unsafe void processImage_scaleOnly(IntPtr ptrBackbuffer);
    }
}
