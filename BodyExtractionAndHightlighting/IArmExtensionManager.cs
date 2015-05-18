using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BodyExtractionAndHightlighting
{
    public interface IArmExtensionManager
    {
        unsafe void processImage(byte[] imageBuffer);

        unsafe void processImage_rotationOnly(byte[] imageBuffer);

        unsafe void processImage_scaleOnly(byte[] imageBuffer);
    }
}
