using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BodyExtractionAndHightlighting
{
    public interface IArmExtensionManager
    {
        unsafe void processImage();

        unsafe void processImage_rotationOnly();

        unsafe void processImage_scaleOnly();
    }
}
