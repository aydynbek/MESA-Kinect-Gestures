using Microsoft.Kinect;
using System.Windows.Media;

namespace KinectColor
{
    public class ColorMasterConrol
    {
        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        public byte[] colorPixels = null;

        public int width = 0;
        public int height = 0;

        public ColorMasterConrol(int width, int height)
        {
            this.width = width;
            this.height = height;
        }

        public void initializeColorArray(ColorFrame frame)
        {
            int width = frame.FrameDescription.Width;
            int height = frame.FrameDescription.Height;
            PixelFormat format = PixelFormats.Bgr32;

            this.colorPixels = new byte[width * height * ((format.BitsPerPixel + 7) / 8)];

            // FYI: format.BitsPerPixel = 32, ((format.BitsPerPixel + 7) / 8) = 4

            if (frame.RawColorImageFormat == ColorImageFormat.Bgra)
            {
                frame.CopyRawFrameDataToArray(colorPixels);
            }
            else
            {
                frame.CopyConvertedFrameDataToArray(colorPixels, ColorImageFormat.Bgra);
            }
        }
    }
}
