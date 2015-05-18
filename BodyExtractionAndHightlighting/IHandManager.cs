using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BodyExtractionAndHightlighting
{
    public interface IHandManager
    {
        unsafe void processImage(byte[] imageBuffer);
    }
}
