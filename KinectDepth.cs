using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Controls;
using System.Windows.Shapes;

namespace KinectDepth
{
    public class DepthMasterControl
    {
        /// <summary>
        /// The Kinect For Windows parameters. ONLY USED HERE!!
        /// </summary>
        private const float CameraDepthNominalFocalLengthInPixels = 367.0094f; 

        /// <summary>
        /// The x value of the default normalized focal length.
        /// </summary>
        private const float DepthNormFocalLengthX = CameraDepthNominalFocalLengthInPixels / 512.0f; // FocalLengthX

        /// <summary>
        /// The y value of the default normalized focal length.
        /// </summary>
        private const float DepthNormFocalLengthY = CameraDepthNominalFocalLengthInPixels / 424.0f; // FocalLengthY

        /// <summary>
        /// The x value of the depth normalized principal point.
        /// </summary>
        private const float DepthNormPrincipalPointX = 0.511245f; // PrincipalPointX

        /// <summary>
        /// The y value of the depth normalized principal point.
        /// </summary>
        private const float DepthNormPrincipalPointY = 0.489791f; // PrincipalPointY

        /// <summary>
        /// The delimiting window for the contour map calculations
        /// </summary>
        private int upperDelimiter = 0, bottomDelimiter = 211;  //130, 265

        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;

        /// <summary>
        /// The size of the digitized contour map blocks
        /// </summary>
        public const int contourDigitalDivisor = 2;
        
        /// <summary>
        /// 2D array of colored grayscale representation of depth data for simple viewing.
        /// Color was used instead of grayscale to allow the highlightment of important information.
        /// </summary>
        public byte[] depthPixelsColor = null;

        /// <summary>
        /// 2D array of XZ-plane contour map. This is basically a top down view.
        /// </summary>
        // public byte[] array = null;

        /// <summary>
        /// 2D array of point cloud data containing the XYZ coordinates of each pixel of depth data
        /// </summary>
        public int[] pointCloudArray = null;

        /// <summary>
        /// This variable is used to map the Z distances on the contour map to a certain limit,
        /// to only view up to a certain distance
        /// </summary>
        private double contourMappingZDistance;

        /// <summary>
        /// This variable is used to map the X distances on the contour map to a certain limit,
        /// to only view up to a certain distance
        /// </summary>
        private double contourMappingXDistance;

        /// <summary>
        /// Maximum range of the depth information that will be used to generate contours.
        /// </summary>
        private const double maximumZRange = 7900.0;

        /// <summary>
        /// Number of frames to keep in history, used for smoothing.
        /// </summary>
        public int numberOfFrames = 30/3;

        /// <summary>
        /// Number of frames currently in the history
        /// </summary>
        public int frameCount;

        /// <summary>
        /// Keeps a certain number of frames to be used for noice removal
        /// </summary>
        public byte[] contourFrameHistory = null;

        /// <summary>
        /// 2D array containing every pixels depth distance in millimeters. This is the very first frame
        /// depth information that will be passed into this class' constructor, which possibly will be compressed.
        /// </summary>
        public ushort[] depthInformationInitial = null;

        /// <summary>
        /// 2D array containing every pixels depth distance in millimeters, possibly having
        /// been compressed.
        /// </summary>
        public ushort[] depthInformationCompressed = null;

        /// <summary>
        /// This will set to true if we would like to compress our depth frame information.
        /// </summary>
        private bool compressionDesired = false;

        /// <summary>
        /// Contour Map with removed noise
        /// </summary>
        public byte[] contourMapDeNoised = null;
        
        /// <summary>
        /// This is a 2D matrix which has different regions of the contour map 
        /// divided by different colors.
        /// </summary>
        public byte[] contourRegionMap = null;

        /// <summary>
        /// Dimentions of the depth camera
        /// </summary>
        public int widthActual = 0, heightActual = 0;

        /// <summary>
        /// Dimentions of the array after compression is done. These dimentions are the ones
        /// that are actually used, whether or not compression was done or not. If 
        /// compression was not done then the values are the actual dimentions.
        /// </summary>
        public int widthCompressed = 0;
        public int heightCompressed = 0;

        private int heightDownsized = 0;
        private int widthDownsized = 0;

        /// <summary>
        /// Each pixel has BGR component
        /// </summary>
        public int bytesPerPixel = 3;  

        /// <summary>
        /// These variables are currently used for finding the largest and smallest 
        /// vertical and horizontal coordinate
        /// </summary>
        public int maxV, maxH, minV, minH = 0;

        /// <summary>
        /// This is the Contour Graph Representation Grid. It is used to draw lines that will
        /// represent edges of graphs and possibly other useful information.
        /// </summary>
        private Grid CoGRGrid;

        private bool doRunningOfAnalysis = true;

        /* METHODS */
        public DepthMasterControl(int widthActual, int heightActual,
            int widthDesired, int heightDesired, bool compressionDesired, Grid coGRGrid)
        {
            this.CoGRGrid = coGRGrid;

            this.widthActual = widthActual;
            this.heightActual = heightActual;
            widthCompressed = widthDesired;
            heightCompressed = heightDesired;
            heightDownsized = heightCompressed;
            widthDownsized = widthCompressed;

            this.compressionDesired = compressionDesired;

            PixelFormat format = PixelFormats.Bgr24;

            depthInformationInitial = new ushort[widthActual * heightActual];
            
            if(compressionDesired == true)
            {
                depthInformationCompressed = new ushort[widthCompressed * heightCompressed];
                depthPixelsColor = new byte[widthCompressed * heightCompressed * 3];
                pointCloudArray = new int[widthCompressed * heightCompressed * 3];
                contourFrameHistory = new byte[widthCompressed * heightCompressed * numberOfFrames];
            } else
            {
                depthInformationCompressed = new ushort[widthActual * heightActual];
                depthPixelsColor = new byte[widthActual * heightActual * 3]; 
                pointCloudArray = new int[widthActual * heightActual * 3];
                contourFrameHistory = new byte[widthActual * heightActual * numberOfFrames];
            }
            

            frameCount = 0;
            double heightDouble = (double)heightCompressed;
            contourMappingZDistance = (maximumZRange / heightDouble) / 10;
        }
        
        public void runDepthAnalysis(IntPtr depthFrameData,
            ushort minDepth, ushort maxDepth, int depthFrameCounter, Image depthViewCamera,
            Image contourView, Image extraCameraView)
        {
            //drawOnGrid();

            if (depthFrameCounter % 3 == 0)
            {
                // Begin by extracting depth information from frame data:
                extractDepthInformation(depthFrameData, minDepth, maxDepth);

                // Compress; if compression was not desired, then we simply do a shallow copy of the
                // original array:
                compressDepthInformationSimple();

                removeRedDepthNoiseNonAltering(2);
                createDepthGrayscaleColorRepresentation(minDepth, maxDepth);
                createPointCloud();
                copyContourMapIntoHistory(createContourMap());

                //depthViewCamera.Source = ToBitmapDepth(heightCompressed, widthCompressed, depthPixelsColor);
                //drawContourOnGrid(createContourMap());

                if (depthFrameCounter % 30 == 0)
                {
                    // This holds the list of regions extremeties coordinates:
                    //Dictionary<int, int[]> regionsCoordinates;
                    /*
                    contourView.Source = ToBitmapDepth(heightCompressed, widthCompressed, regionColorCoder(upsizeDigitizedContour(
                        regionFiller((shrinkBinary(expandBinary(downsizeDigitizedContour(digitizeContourMap(createContourMap())), false), false))))));
                    */
                    /*
                    extraCameraView.Source = ToBitmapDepthGrayscale(heightCompressed, widthCompressed,
                        upsizeDigitizedContour(boundaryExtractor(downsizeDigitizedContour(digitizeContourMap(createContourMap())), false)));
                    */
                    /*
                    contourView.Source = ToBitmapDepthGrayscale(heightCompressed, widthCompressed,
                           digitizeContourMap(removeNoiseOriginalContour()));
                    */
                    
                    depthViewCamera.Source = ToBitmapDepth(heightCompressed * 3, widthCompressed * 3,
                        markSpecialVerticees(
                        heightDownsized, widthDownsized, boundaryExtractor(heightDownsized * 3, widthDownsized * 3,
                        regionInnerFiller(heightDownsized * 3, widthDownsized * 3, expandedBoundaryArrayConnector(
                            boundaryArrayExpander((boundarySimplifier(boundaryExtractor
                        (heightDownsized, widthDownsized, regionInnerFiller(heightDownsized, widthDownsized,
                        shrinkBinary(expandBinary(((removeNoiseOriginalContour())), false), false)), 120)))))), 120)));

                    extraCameraView.Source = ToBitmapDepthGrayscale(heightCompressed, widthCompressed,((
                        boundaryExtractor(heightDownsized, widthDownsized, regionInnerFiller(heightDownsized,
                        widthDownsized, shrinkBinary(expandBinary(((removeNoiseOriginalContour())), false), false)), 120))));

                    //extraCameraView.Source = ToBitmapDepth(heightCompressed, widthCompressed, depthPixelsColor);

                    doRunningOfAnalysis = false;
                }
            }
        }

        private static ImageSource ToBitmapDepth(int height, int width, byte[] pixels)
        {
            PixelFormat format = PixelFormats.Bgr24;
            int stride = width * format.BitsPerPixel / 8;

            return BitmapSource.Create(width, height, 96, 96, format, null, pixels, stride);
        }

        private static ImageSource ToBitmapDepthGrayscale(int height, int width, byte[] pixels)
        {
            PixelFormat format = PixelFormats.Gray8;
            int stride = width * format.BitsPerPixel / 8;

            return BitmapSource.Create(width, height, 96, 96, format, null, pixels, stride);
        }

        /// <summary>
        /// This method simply fills up a 2D array with depth information that comes 
        /// every thirtieth of a second.
        /// </summary>
        /// <param name="depthFrameData"></param>
        /// <param name="minDepth"></param>
        /// <param name="maxDepth"></param>
        private unsafe void extractDepthInformation(IntPtr depthFrameData,
            ushort minDepth, ushort maxDepth)
        {
            ushort* frameData = (ushort*)depthFrameData;
            for (int i = 0; i < heightActual; i++)    //424
            {
                for (int j = 0; j < widthActual; j++)     //512
                {
                    depthInformationInitial[(i * widthActual) + j] = frameData[(i * widthActual) + j];
                }
            }
        }

        /// <summary>
        /// This method compresses the depth information if necessary.
        /// </summary>
        private void compressDepthInformationSimple()
        {   
            if(compressionDesired == true)
            {
                int ratio = widthActual / widthCompressed;

                for (int i = 0; i < heightCompressed; i++)    //424
                {
                    for (int j = 0; j < widthCompressed; j++)     //512
                    {
                        depthInformationCompressed[(i * widthCompressed) + j] =
                            depthInformationInitial[((i * ratio) * widthActual) + (j * ratio)];
                    }
                }
            } else
            {
                // Shallow copy:
                depthInformationCompressed = depthInformationInitial;
            }
            
        }
        
        /// <summary>
        /// This method converts the depth data information, which is in millimeters,
        /// into a grayscale image for viewing.
        /// </summary>
        private void createDepthGrayscaleColorRepresentation(ushort minDepth, ushort maxDepth)
        {
            int k = 0;
            int bytesPerPixel = 3;  // = 4
            int index = 0;
            byte tmp = 0;

            for (int i = 0; i < heightCompressed; i++)
            {
                for (int j = 0; j < widthCompressed; j++)
                {
                    index = (i * widthCompressed) + j;
                    tmp = (byte)(depthInformationCompressed[index] >= minDepth && depthInformationCompressed[index]
                    <= maxDepth ? (depthInformationCompressed[index] / MapDepthToByte) : 0);

                    if (tmp >= 30)
                    {
                        depthPixelsColor[index * bytesPerPixel + 0] =
                            depthPixelsColor[index * bytesPerPixel + 1] =
                            depthPixelsColor[index * bytesPerPixel + 2] = tmp;
                    }
                    else if (tmp == 0)
                    {
                        depthPixelsColor[index * bytesPerPixel + k++] = 0;
                        depthPixelsColor[index * bytesPerPixel + k++] = 0;
                        depthPixelsColor[index * bytesPerPixel + k++] = 255;
                    }
                    else
                    {
                        depthPixelsColor[index * bytesPerPixel + k++] = 255;
                        depthPixelsColor[index * bytesPerPixel + k++] = 0;
                        depthPixelsColor[index * bytesPerPixel + k++] = 0;
                    }
                    
                    k = 0;
                }
            }
        }
        
        /// <summary>
        /// This method correctly generates a point cloud, that is, XYZ coordinates of every pixel
        /// </summary>
        private void createPointCloud()
        {
            float px = DepthNormPrincipalPointX * widthActual;
            float py = DepthNormPrincipalPointY * heightActual;

            // This is the DepthNormFocalLengthX and DepthNormFocalLengthY from the CameraParameters class:
            float fx = DepthNormFocalLengthX * widthActual;
            float fy = DepthNormFocalLengthX * heightActual;

            float X, Y, Z;

            float depthInfoAtPoint = 0;

            int k = 0, partialIndex = 0;
            int[] pointCloudArrayTemp = new int[widthActual * heightActual * 3];
            
            int ratio = widthActual / widthCompressed;

            for (int i = 0; i < heightCompressed; i++)    //424
            {
                for (int j = 0; j < widthCompressed; j++)     //512
                {
                    depthInfoAtPoint = (float)(depthInformationCompressed[((i * widthCompressed) + j)]);

                    // Assuming depths are measurements along Z.
                    // All measurements are in centimeters:
                    X = (((j * ratio) - px) * (1.0f / fx) * depthInfoAtPoint) / 10.0f;
                    Y = (((heightActual - (i * ratio)) - py) * (1.0f / fy) * depthInfoAtPoint) / 10.0f;
                    Z = (depthInfoAtPoint) / 10.0f;

                    partialIndex = (((i * widthCompressed) + j) * 3);
                    pointCloudArray[partialIndex + k++] = (int)X;
                    pointCloudArray[partialIndex + k++] = (int)Y;
                    pointCloudArray[partialIndex + k++] = (int)Z;
                    k = 0;
                }
            }
        }
        
        /// <summary>
        /// This is a method that creates the contour map with the use of the correctly
        /// calculated XYZ coordinates via point cloud generation.
        /// </summary>
        private byte[] createContourMap()
        {
            int height = heightCompressed;
            int width = widthCompressed;
            int widthOffset = widthCompressed / 2;
            int heightCeiling = 60;
            int heightFloor = -160;
            int Row = 0;
            int Col = 0;
            int y_coordinate = 0;
            byte[] contourMap = new byte[height * width];


            for (int i = 0; i < heightCompressed; i++)
            {
                for (int j = 0; j < widthCompressed; j++)  
                {
                    y_coordinate = pointCloudArray[(((i * widthCompressed) + j) * 3) + 1];
                    if(y_coordinate < heightCeiling && y_coordinate > heightFloor)
                    {
                        /* Columns will now be represented by X, and rows by Z distances!! */
                        Row = pointCloudArray[(((i * widthCompressed) + j) * 3) + 2];
                        //contourMappingZDistance = 7900 / height;
                        Row = (int)(Row / (contourMappingZDistance));
                        
                        if (Row < height)
                        {
                            Col = pointCloudArray[(((i * widthCompressed) + j) * 3)];   // X could be positive or negative
                            contourMappingXDistance = 790 / width;
                            Col = (int)(Col / (contourMappingZDistance)) + widthOffset;

                            if (Col < width)
                            {
                                contourMap[(((Row) * width) + (Col))] = 255;
                            }
                            
                        }
                    }
                }
            }

            //copyContourMapIntoHistory();
            //drawFOVDelimiter();

            return contourMap;
        }
        
        /// <summary>
        /// A method which uses a particular estimation to digitize a contour map with larger size pixels
        /// </summary>
        private byte[] digitizeContourMap(byte[] array)
        {
            int threshold = (contourDigitalDivisor * contourDigitalDivisor) / 4;
            int count = 0;
            byte[] digitizedArray = new byte[heightCompressed * widthCompressed];

            for (int i = 0; i < heightCompressed; i = i + contourDigitalDivisor)    //424
            {
                for (int j = 0; j < widthCompressed; j = j + contourDigitalDivisor)     //512
                {
                    count = 0;
                    for (int ii = 0; ii < contourDigitalDivisor; ii++)
                    {
                        for (int jj = 0; jj < contourDigitalDivisor; jj++)
                        {
                            if (array[(((i + ii) * widthCompressed) + (j + jj))] == 255)
                            {
                                count++;
                            }
                        }
                    }

                    // Might need to implement a better threshold:
                    if (count > threshold)
                    {
                        for (int ii = 0; ii < contourDigitalDivisor; ii++)
                        {
                            for (int jj = 0; jj < contourDigitalDivisor; jj++)
                            {
                                digitizedArray[(((i + ii) * widthCompressed) + (j + jj))] = 255;
                            }
                        }
                    }
                    else
                    {
                        for (int ii = 0; ii < contourDigitalDivisor; ii++)
                        {
                            for (int jj = 0; jj < contourDigitalDivisor; jj++)
                            {
                                digitizedArray[(((i + ii) * widthCompressed) + (j + jj))] = 0;
                            }
                        }
                    }
                }
            }

            //drawFOVDelimiter();

            return digitizedArray;
        }

        /// <summary>
        /// This method simply gets the maximum and minimum X and Y coordinate.
        /// The usefullness of this method is in question.
        /// </summary>
        private void getPointCloudBoundaries()
        {
            int maxV = 0;
            int maxH = 0;
            int minV = 999999999;
            int minH = 999999999;
            int k = 0;
            int X, Y;

            for (int i = 0; i < heightCompressed; i++)    //424
            {
                for (int j = 0; j < heightCompressed; j++)     //512
                {
                    X = pointCloudArray[(((i * widthCompressed) + j) * 3)];
                    Y = pointCloudArray[(((i * widthCompressed) + j) * 3) + 1];

                    if (Y > maxV)
                    {
                        maxV = Y;
                    }

                    if (Y < minV)
                    {
                        minV = Y;
                    }

                    if (X > maxH)
                    {
                        maxH = X;
                    }

                    if (X < minH)
                    {
                        minH = X;
                    }

                }
            }

            this.maxV = maxV;
            this.maxH = maxH;
            this.minV = minV;
            this.minH = minH;
        }

        /// <summary>
        /// This method gathers the contour map that was created at specific iterations
        /// into one array for averaging in order to remove noise.
        /// </summary>
        /// <param name="contourMap"></param>
        private void copyContourMapIntoHistory(byte[] contourMap)
        {
            int dimention = heightCompressed * widthCompressed;
            
            for (int i = 0; i < heightCompressed; i++)  
            {
                for (int j = 0; j < widthCompressed; j++)   
                {
                    contourFrameHistory[(dimention * frameCount) + ((i * widthCompressed) + j)] =
                        contourMap[((i * widthCompressed) + j)];
                }
            }

            frameCount++;
            if (frameCount == numberOfFrames)
            {
                frameCount = 0;
            } 
        }


        /// <summary>
        /// This method tries to remove some of the Red noice from the depth data that is 
        /// the result of the camera not being able to accurately read data because of certain
        /// factors, such as reflective surfaces.
        /// </summary>
        private void removeRedDepthNoiseNonAltering(int count)
        {
            for(int k = 0; k < count; k++) {
                removeRedDepthNoiseNonAltering();
            }
        }

        private void removeRedDepthNoiseNonAltering()
        {
            ushort depthInfoAtPoint = 0;
            ushort[] newArray = new ushort[heightCompressed * widthCompressed];

            for (int i = 1; i < heightCompressed - 1; i++)
            {
                for (int j = 1; j < widthCompressed - 1; j++)
                {
                    depthInfoAtPoint = depthInformationCompressed[((i * widthCompressed) + j)];

                    if (depthInfoAtPoint == 0)
                    {
                        if (depthInformationCompressed[(((i + 1) * widthCompressed) + (j))] != 0)  //down
                        {
                            depthInfoAtPoint = depthInformationCompressed[(((i + 1) * widthCompressed) + (j))];
                        }
                        else if (depthInformationCompressed[(((i - 1) * widthCompressed) + (j))] != 0)    //up
                        {
                            depthInfoAtPoint = depthInformationCompressed[(((i - 1) * widthCompressed) + (j))];
                        }
                        else if (depthInformationCompressed[(((i) * widthCompressed) + (j + 1))] != 0)    //right
                        {
                            depthInfoAtPoint = depthInformationCompressed[(((i) * widthCompressed) + (j + 1))];
                        }
                        else if (depthInformationCompressed[(((i) * widthCompressed) + (j - 1))] != 0)    //left
                        {
                            depthInfoAtPoint = depthInformationCompressed[(((i) * widthCompressed) + (j - 1))];
                        }
                        else if (depthInformationCompressed[(((i - 1) * widthCompressed) + (j + 1))] != 0)    //up right
                        {
                            depthInfoAtPoint = depthInformationCompressed[(((i - 1) * widthCompressed) + (j + 1))];
                        }
                        else if (depthInformationCompressed[(((i - 1) * widthCompressed) + (j - 1))] != 0)    //up left
                        {
                            depthInfoAtPoint = depthInformationCompressed[(((i - 1) * widthCompressed) + (j - 1))];
                        }
                        else if (depthInformationCompressed[(((i + 1) * widthCompressed) + (j + 1))] != 0)    //down right
                        {
                            depthInfoAtPoint = depthInformationCompressed[(((i + 1) * widthCompressed) + (j + 1))];
                        }
                        else if (depthInformationCompressed[(((i + 1) * widthCompressed) + (j - 1))] != 0)    //down left
                        {
                            depthInfoAtPoint = depthInformationCompressed[(((i + 1) * widthCompressed) + (j - 1))];
                        }
                    }
                    newArray[((i * widthCompressed) + j)] = depthInfoAtPoint;
                }
            }

            depthInformationCompressed = newArray;
        }

        /// <summary>
        /// This method tries to remove some of the Red noice from the depth data that is 
        /// the result of the camera not being able to accurately read data because of certain
        /// factors, such as reflective surfaces.
        /// </summary>
        private void removeRedDepthNoiseAltering()
        {
            ushort depthInfoAtPoint = 0;

            // The boundary of the 2D array was not used for simplification:
            for (int i = 1; i < heightCompressed - 1; i++)
            {
                for (int j = 1; j < widthCompressed - 1; j++)
                {
                    depthInfoAtPoint = depthInformationCompressed[((i * widthCompressed) + j)];

                    if (depthInfoAtPoint == 0)
                    {
                        if(depthInformationCompressed[(((i + 1) * widthCompressed) + (j))] != 0)  //down
                        {
                            depthInfoAtPoint = depthInformationCompressed[(((i + 1) * widthCompressed) + (j))];
                        }
                        else if (depthInformationCompressed[(((i - 1) * widthCompressed) + (j))] != 0)    //up
                        {
                            depthInfoAtPoint = depthInformationCompressed[(((i - 1) * widthCompressed) + (j))];
                        }
                        else if (depthInformationCompressed[(((i) * widthCompressed) + (j + 1))] != 0)    //right
                        {
                            depthInfoAtPoint = depthInformationCompressed[(((i) * widthCompressed) + (j + 1))];
                        }
                        else if (depthInformationCompressed[(((i) * widthCompressed) + (j - 1))] != 0)    //left
                        {
                            depthInfoAtPoint = depthInformationCompressed[(((i) * widthCompressed) + (j - 1))];
                        }
                        else if (depthInformationCompressed[(((i - 1) * widthCompressed) + (j + 1))] != 0)    //up right
                        {
                            depthInfoAtPoint = depthInformationCompressed[(((i - 1) * widthCompressed) + (j + 1))];
                        }
                        else if (depthInformationCompressed[(((i - 1) * widthCompressed) + (j - 1))] != 0)    //up left
                        {
                            depthInfoAtPoint = depthInformationCompressed[(((i - 1) * widthCompressed) + (j - 1))];
                        }
                        else if (depthInformationCompressed[(((i + 1) * widthCompressed) + (j + 1))] != 0)    //down right
                        {
                            depthInfoAtPoint = depthInformationCompressed[(((i + 1) * widthCompressed) + (j + 1))];
                        }
                        else if (depthInformationCompressed[(((i + 1) * widthCompressed) + (j - 1))] != 0)    //down left
                        {
                            depthInfoAtPoint = depthInformationCompressed[(((i + 1) * widthCompressed) + (j - 1))];
                        }

                        depthInformationCompressed[((i * widthCompressed) + j)] = depthInfoAtPoint;
                    }
                }
            }
        }

        /// <summary>
        /// This method does the averaging of several contour frames and returns a 
        /// denoised array.
        /// </summary>
        private byte[] removeNoiseOriginalContour()
        {
            int[] depthPixelContourSum = new int[heightCompressed * widthCompressed];
            int dimention = heightCompressed * widthCompressed;
            int index = 0; 
            int singleElementSum = 0;
            byte[] denoisedArray = new byte[heightCompressed * widthCompressed];

            for (int i = 0; i < heightCompressed; i++)    //424
            {
                for (int j = 0; j < widthCompressed; j++)     //512
                {
                    index = (i * widthCompressed) + j;
                    for (int k = 0; k < numberOfFrames; k++)
                    {
                        // Get the sum of all the frames:
                        depthPixelContourSum[index] += contourFrameHistory[(dimention * k) + (index)];
                    }

                    // Compute the mean value:
                    singleElementSum = depthPixelContourSum[index] / numberOfFrames;
                    if(singleElementSum >= (255 / numberOfFrames) * 5)
                    {
                        singleElementSum = 255;
                    } else
                    {
                        singleElementSum = 0;
                    }

                    denoisedArray[(index)] = (byte)(singleElementSum);
                }
            }

            return denoisedArray;
        }

        /// <summary>
        /// Creates a 2D array of smaller size by a factor of the variable "contourDigitalDivisor".
        /// For every contourDigitalDivisor * contourDigitalDivisor block of pixels we now only have 1.
        /// </summary>
        private byte[] downsizeDigitizedContour(byte[] array)
        {
            heightDownsized = heightCompressed / contourDigitalDivisor;
            widthDownsized = widthCompressed / contourDigitalDivisor;

            byte[] newArray = new byte[heightDownsized * widthDownsized];

            for (int i = 0; i < heightDownsized; i++)
            {
                for (int j = 0; j < widthDownsized; j++)
                {
                    newArray[(i * widthDownsized) + j] =
                        array[(((i * contourDigitalDivisor) * widthCompressed) + (j * contourDigitalDivisor))];
                }
            }

            return newArray;
        }

        /// <summary>
        /// Creates a 2D array of larger size by a factor of the variable "contourDigitalDivisor".
        /// This should be called after downsizeDigitizedContour() method.
        /// </summary>
        private byte[] upsizeDigitizedContour(byte[] array)
        {
            byte[] upsizedArray =  new byte[heightCompressed * widthCompressed];

            for (int i = 0; i < heightDownsized; i++)
            {
                for (int j = 0; j < widthDownsized; j++)
                {
                    for (int ii = 0; ii < contourDigitalDivisor; ii++)
                    {
                        for (int jj = 0; jj < contourDigitalDivisor; jj++)
                        {
                            upsizedArray[((((i * contourDigitalDivisor) + ii) * widthCompressed) + 
                                ((j * contourDigitalDivisor) + jj))] = array[(i * widthDownsized) + j];
                        }
                    }
                }
            }

            heightDownsized = heightCompressed;
            widthDownsized = widthCompressed;

            return upsizedArray;
        }

        /// <summary>
        /// Does expansion on a image which is represented as a binary. 8-connected neigbors implememnted.
        /// </summary>
        /// <param name="array"></param>
        /// <param name="do8Neighbors"></param> Set true to also consider 8-neighbors as the closest pixels.
        /// <returns></returns>
        private byte[] expandBinary(byte[] array, bool do8Neighbors)
        {
            byte singleElement = 0;
            byte[] expandedArray = new byte[heightDownsized * widthDownsized];

            for (int i = 1; i < heightDownsized - 1; i++)
            {
                for (int j = 1; j < widthDownsized - 1; j++)
                {
                    singleElement = array[(i * widthDownsized) + j];

                    if(singleElement == 0)
                    {
                        if (array[((i - 1) * widthDownsized) + j] != 0 ||
                            array[((i + 1) * widthDownsized) + j] != 0 ||
                            array[(i * widthDownsized) + (j - 1)] != 0 ||
                            array[(i * widthDownsized) + (j + 1)] != 0 ||
                            (array[((i - 1) * widthDownsized) + (j + 1)] != 0 && do8Neighbors) ||
                            (array[((i - 1) * widthDownsized) + (j - 1)] != 0 && do8Neighbors) ||
                            (array[((i + 1) * widthDownsized) + (j + 1)] != 0 && do8Neighbors) ||
                            (array[((i + 1) * widthDownsized) + (j - 1)] != 0 && do8Neighbors) )
                        {
                            expandedArray[(i * widthDownsized) + j] = 255;
                        } else
                        {
                            expandedArray[(i * widthDownsized) + j] = 0;
                        }
                    } else
                    {
                        expandedArray[(i * widthDownsized) + j] = 255;
                    }
                }
            }

            return expandedArray;
        }

        /// <summary>
        /// Does shrinking on a image which is represented as a binary. 8-connected neigbors implememnted.
        /// </summary>
        /// <param name="array"></param>
        /// <returns></returns>
        private byte[] shrinkBinary(byte[] array, bool do8Neighbors)
        {
            byte singleElement = 0;
            byte[] shrunkArray = new byte[heightDownsized * widthDownsized];

            for (int i = 1; i < heightDownsized - 1; i++)
            {
                for (int j = 1; j < widthDownsized - 1; j++)
                {
                    singleElement = array[(i * widthDownsized) + j];

                    if (singleElement != 0)
                    {
                        if (array[((i - 1) * widthDownsized) + j] == 0 ||
                            array[((i + 1) * widthDownsized) + j] == 0 ||
                            array[(i * widthDownsized) + (j - 1)] == 0 ||
                            array[(i * widthDownsized) + (j + 1)] == 0 ||
                            (array[((i - 1) * widthDownsized) + (j + 1)] == 0 && do8Neighbors) ||
                            (array[((i - 1) * widthDownsized) + (j + 1)] == 0 && do8Neighbors) ||
                            (array[((i - 1) * widthDownsized) + (j + 1)] == 0 && do8Neighbors) ||
                            (array[((i - 1) * widthDownsized) + (j + 1)] == 0 && do8Neighbors) )
                        {
                            shrunkArray[(i * widthDownsized) + j] = 0;
                        }
                        else
                        {
                            shrunkArray[(i * widthDownsized) + j] = 255;
                        }
                    }
                    else
                    {
                        shrunkArray[(i * widthDownsized) + j] = 0;
                    }
                }
            }

            return shrunkArray;
        }

        private byte[] upperBoundaryExtractor(byte[] array)
        {
            byte singleElement = 0;
            byte[] newArray = new byte[heightDownsized * widthDownsized];

            for (int i = 1; i < heightDownsized - 1; i++)
            {
                for (int j = 1; j < widthDownsized - 1; j++)
                {
                    singleElement = array[(i * widthDownsized) + j];

                    if (singleElement != 0)
                    {
                        // Its better not to do the 8 neighbours:
                        if (array[((i - 1) * widthDownsized) + j] == 0 ||
                            array[(i * widthDownsized) + (j - 1)] == 0 ||
                            array[(i * widthDownsized) + (j + 1)] == 0)
                        {
                            newArray[(i * widthDownsized) + j] = 255;
                        }
                        else
                        {
                            newArray[(i * widthDownsized) + j] = 0;
                        }
                    }
                    else
                    {
                        newArray[(i * widthDownsized) + j] = 0;
                    }
                }
            }

            return newArray;
        }

        private byte[] expandedBoundaryArrayConnector(byte[] expandedArray)
        {
            byte[] connectedArray = new byte[heightDownsized * widthDownsized * 9];
            int widthExpanded = widthDownsized * 3;

            for (int i = 0; i < heightDownsized - 1; i++)
            {
                for (int j = 0; j < widthDownsized - 1; j++)
                {
                    // expandedArray[((((i * 3) + 1) * newWidth) + ((j * 3) + 1))] = boundaryArray[(i * widthDownsized) + j];

                    if (expandedArray[((((i * 3 + 1) * widthExpanded)) + (j * 3 + 1))] == 255)
                    {
                        connectedArray[((((i * 3 + 1) * widthExpanded)) + (j * 3 + 1))] = expandedArray[((((i * 3 + 1) * widthExpanded)) + (j * 3 + 1))];

                        if (expandedArray[((((i * 3 - 2) * widthExpanded)) + (j * 3 + 1))] == 255)
                        {
                            connectedArray[((((i * 3 - 1) * widthExpanded)) + (j * 3 + 1))] = 255;
                            connectedArray[((((i * 3) * widthExpanded)) + (j * 3 + 1))] = 255;
                        }

                        if (expandedArray[((((i * 3 + 4) * widthExpanded)) + (j + 1))] == 255)
                        {
                            connectedArray[((((i * 3 + 2) * widthExpanded)) + (j * 3 + 1))] = 255;
                            connectedArray[((((i * 3 + 3) * widthExpanded)) + (j * 3 + 1))] = 255;
                        }

                        if (expandedArray[((((i * 3 + 1) * widthExpanded)) + (j * 3 - 2))] == 255)
                        {
                            connectedArray[((((i * 3 + 1) * widthExpanded)) + (j * 3 - 1))] = 255;
                            connectedArray[((((i * 3 + 1) * widthExpanded)) + (j * 3))] = 255;
                        }

                        if (expandedArray[((((i * 3 + 1) * widthExpanded)) + (j * 3 + 4))] == 255)
                        {
                            connectedArray[((((i * 3 + 1) * widthExpanded)) + (j * 3 + 2))] = 255;
                            connectedArray[((((i * 3 + 1) * widthExpanded)) + (j * 3 + 3))] = 255;
                        }
                    }
                }
            }
            
            return connectedArray;
        }

        private byte[] boundaryArrayExpander(byte[] boundaryArray)
        {
            byte[] expandedArray = new byte[heightDownsized * widthDownsized * 9];
            int newWidth = widthDownsized * 3;

            for (int i = 0; i < heightDownsized; i++)
            {
                for (int j = 0; j < widthDownsized; j++)
                {
                    if(boundaryArray[(i * widthDownsized) + j] == 255)
                    {
                        expandedArray[((((i * 3) + 1) * newWidth) + ((j * 3) + 1))] = boundaryArray[(i * widthDownsized) + j];
                    }
                }
            }

            return expandedArray;
        }

        /// <summary>
        /// This method simplifies the boundary map by replacing diagonal neighbour 
        /// with equivalent 4 neighbors.
        /// </summary>
        /// <param name="boundaryArray"></param>
        private byte[] boundarySimplifier(byte[] boundaryArray)
        {
            byte singleElement = 0;
            byte[] simplifiedArray = new byte[heightDownsized * widthDownsized];

            for (int i = 1; i < heightDownsized - 1; i++)
            {
                for (int j = 1; j < widthDownsized - 1; j++)
                {
                    singleElement = boundaryArray[(i * widthDownsized) + j];

                    if(singleElement == 255)
                    {
                        if (boundaryArray[((i - 1) * widthDownsized) + (j - 1)] == 255)
                        {
                            if (boundaryArray[((i) * widthDownsized) + (j - 1)] == 0 &&
                                boundaryArray[((i - 1) * widthDownsized) + (j)] == 0)
                            {
                                simplifiedArray[((i) * widthDownsized) + (j - 1)] = 255;
                            }
                        }

                        if (boundaryArray[((i + 1) * widthDownsized) + (j - 1)] == 255)
                        {
                            if (boundaryArray[((i) * widthDownsized) + (j - 1)] == 0 &&
                                boundaryArray[((i + 1) * widthDownsized) + (j)] == 0)
                            {
                                simplifiedArray[((i) * widthDownsized) + (j - 1)] = 255;
                            }
                        }

                        if (boundaryArray[((i - 1) * widthDownsized) + (j + 1)] == 255)
                        {
                            if (boundaryArray[((i - 1) * widthDownsized) + (j)] == 0 &&
                                boundaryArray[((i) * widthDownsized) + (j + 1)] == 0)
                            {
                                simplifiedArray[((i) * widthDownsized) + (j + 1)] = 255;
                            }
                        }

                        if (boundaryArray[((i + 1) * widthDownsized) + (j + 1)] == 255)
                        {
                            if (boundaryArray[((i + 1) * widthDownsized) + (j)] == 0 &&
                                boundaryArray[((i) * widthDownsized) + (j + 1)] == 0)
                            {
                                simplifiedArray[((i + 1) * widthDownsized) + (j)] = 255;
                            }
                        }
                    }

                    simplifiedArray[(i * widthDownsized) + j] = singleElement;
                }
            }

            return simplifiedArray;
        }

        /// <summary>
        /// This method modifies the contour map by only keeping the boundaries of 
        /// regions. This will be used to create the CoGR.
        /// </summary>
        /// <param name="array"></param>
        /// <param name="do8Neighbors"></param>
        /// <returns></returns>
        private byte[] boundaryExtractor(int heightDownsized, int widthDownsized, byte[] array, byte backgroundGrayLevel)
        {
            byte singleElement = 0;
            byte[] newArray = new byte[heightDownsized * widthDownsized];
            
            for (int i = 1; i < heightDownsized - 1; i++)
            {
                for (int j = 1; j < widthDownsized - 1; j++)
                {
                    singleElement = array[(i * widthDownsized) + j];

                    if (singleElement == 255)
                    {
                        // Its better not to do the 8 neighbours:
                        if (array[((i - 1) * widthDownsized) + j] == backgroundGrayLevel ||
                            array[((i + 1) * widthDownsized) + j] == backgroundGrayLevel ||
                            array[(i * widthDownsized) + (j - 1)] == backgroundGrayLevel ||
                            array[(i * widthDownsized) + (j + 1)] == backgroundGrayLevel)
                        {
                            newArray[(i * widthDownsized) + j] = 255;
                        }
                        else
                        {
                            newArray[(i * widthDownsized) + j] = 0;
                        }
                    }
                    else
                    {
                        newArray[(i * widthDownsized) + j] = 0;
                    }
                }
            }

            return newArray;
        }

        private byte[] markSpecialVerticees(int height, int width, byte[] array)
        {
            byte[] markedArray = new byte[height * width * 9 * 3];
            byte singleElement = 0;
            int neighbourCount = 0;
            int newWidth = width * 3;
            int newHeight = height * 3;

            for (int i = 0; i < newHeight; i++)
            {
                for (int j = 0; j < newWidth; j++)
                {
                    markedArray[((i * newWidth) + j) * 3] = array[(i * newWidth) + j];
                    markedArray[((i * newWidth) + j) * 3 + 1] = array[(i * newWidth) + j];
                    markedArray[((i * newWidth) + j) * 3 + 2] = array[(i * newWidth) + j];
                }
            }

            for (int i = 1; i < height - 1; i++)
            {
                for (int j = 1; j < width - 1; j++)
                {
                    singleElement = array[((((i * 3) + 1) * newWidth) + ((j * 3) + 1))];

                    if (singleElement == 255)
                    {
                        if(array[((((i * 3)) * newWidth) + ((j * 3) + 1))] == 255)
                        {
                            neighbourCount++;
                        }

                        if (array[((((i * 3) + 2) * newWidth) + ((j * 3) + 1))] == 255)
                        {
                            neighbourCount++;
                        }

                        if (array[((((i * 3) + 1) * newWidth) + ((j * 3)))] == 255)
                        {
                            neighbourCount++;
                        }

                        if (array[((((i * 3) + 1) * newWidth) + ((j * 3) + 2))] == 255)
                        {
                            neighbourCount++;
                        }

                        if(neighbourCount == 1)
                        {
                            //markedArray[((((i * 3) + 1) * newWidth) + ((j * 3) + 1))] = 100;
                            markedArray[(((i * 3 + 1) * newWidth) + (j * 3 + 1)) * 3] = 0;
                            markedArray[(((i * 3 + 1) * newWidth) + (j * 3 + 1)) * 3 + 1] = 0;
                            markedArray[(((i * 3 + 1) * newWidth) + (j * 3 + 1)) * 3 + 2] = 255;
                        }

                        neighbourCount = 0;
                    }
                }
            }

            for (int i = 1; i < height - 1; i++)
            {
                for (int j = 1; j < width - 1; j++)
                {
                    singleElement = array[((((i * 3) + 1) * newWidth) + ((j * 3) + 1))];

                    if (singleElement == 255)
                    {
                        if (markedArray[(((i * 3) * newWidth) + (j * 3 + 1)) * 3 + 2] == 255)
                        {

                        }

                        if (array[((((i * 3) + 2) * newWidth) + ((j * 3) + 1))] == 255)
                        {
                            neighbourCount++;
                        }

                        if (array[((((i * 3) + 1) * newWidth) + ((j * 3)))] == 255)
                        {
                            neighbourCount++;
                        }

                        if (array[((((i * 3) + 1) * newWidth) + ((j * 3) + 2))] == 255)
                        {
                            neighbourCount++;
                        }

                        if (neighbourCount == 1)
                        {
                            //markedArray[((((i * 3) + 1) * newWidth) + ((j * 3) + 1))] = 100;
                            markedArray[(((i * 3 + 1) * newWidth) + (j * 3 + 1)) * 3] = 0;
                            markedArray[(((i * 3 + 1) * newWidth) + (j * 3 + 1)) * 3 + 1] = 0;
                            markedArray[(((i * 3 + 1) * newWidth) + (j * 3 + 1)) * 3 + 2] = 255;
                        }

                        neighbourCount = 0;
                    }
                }
            }

            return markedArray;
        }

        private byte[] regionInnerFiller(int height, int width, byte[] array)
        {
            byte singleElement = 0;
            byte[] overlayyedArray = new byte[height * width];
            int leftDelimeter = -1, rightDelimeter = -1;

            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    overlayyedArray[(i * width) + j] = 120;
                }
            }

            for (int i = 0; i < height; i++)
            {
                // Find the left delimeter:
                for (int j = 0; j < width; j++)
                {
                    singleElement = array[(i * width) + j];

                    if(singleElement == 255)
                    {
                        leftDelimeter = j;
                        break;
                    }
                }

                // Find the right delimeter:
                for (int j = width - 1; j >= 0; j--)
                {
                    singleElement = array[(i * width) + j];

                    if (singleElement == 255)
                    {
                        rightDelimeter = j;
                        break;
                    }
                }

                if (leftDelimeter >= 0 && rightDelimeter >= 0)
                {
                    for (int j = leftDelimeter; j <= rightDelimeter; j++)
                    {
                        overlayyedArray[(i * width) + j] = array[(i * width) + j];
                    }
                }

                leftDelimeter = -1;
                rightDelimeter = -1;
            }

            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    if (overlayyedArray[(i * width) + j] == 0)
                    {
                        if (overlayyedArray[((i + 1) * width) + (j)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i - 1) * width) + (j)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i) * width) + (j + 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i) * width) + (j - 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i - 1) * width) + (j - 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i - 1) * width) + (j + 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i + 1) * width) + (j - 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i + 1) * width) + (j + 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                    }
                }
            }

            for (int i = height - 1; i >= 0; i--)
            {
                for (int j = width - 1; j >= 0; j--)
                {
                    if (overlayyedArray[(i * width) + j] == 0)
                    {
                        if (overlayyedArray[((i + 1) * width) + (j)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i - 1) * width) + (j)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i) * width) + (j + 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i) * width) + (j - 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i - 1) * width) + (j - 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i - 1) * width) + (j + 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i + 1) * width) + (j - 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i + 1) * width) + (j + 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                    }
                }
            }

            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    if(overlayyedArray[(i * width) + j] == 0)
                    {
                        if (overlayyedArray[((i + 1) * width) + (j)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i - 1) * width) + (j)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i) * width) + (j + 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i) * width) + (j - 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i - 1) * width) + (j - 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i - 1) * width) + (j + 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i + 1) * width) + (j - 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i + 1) * width) + (j + 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                    }
                }
            }

            for (int i = height - 1; i >= 0; i--)
            {
                for (int j = width - 1; j >= 0; j--)
                {
                    if (overlayyedArray[(i * width) + j] == 0)
                    {
                        if (overlayyedArray[((i + 1) * width) + (j)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i - 1) * width) + (j)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i) * width) + (j + 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i) * width) + (j - 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i - 1) * width) + (j - 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i - 1) * width) + (j + 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i + 1) * width) + (j - 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                        else if (overlayyedArray[((i + 1) * width) + (j + 1)] == 120)
                        {
                            overlayyedArray[(i * width) + j] = 120;
                        }
                    }
                }
            }

            return overlayyedArray;
        }

        /// <summary>
        /// This method simply colors regions in specific colors so that it is easy 
        /// to visually identify different regions.
        /// </summary>
        /// <param name="array"></param>
        /// <returns></returns>
        private byte[] regionColorCoder(byte[] array)
        {
            byte[] colorCodedArray = new byte[heightCompressed * widthCompressed * bytesPerPixel];
            byte singleElement = 0;
            int k = 0;
            int tripplePower = 0;
            
            for (int i = 0; i < heightCompressed; i++)
            {
                for (int j = 0; j < widthCompressed; j++)
                {
                    singleElement = array[((i * widthCompressed) + j)];
                    if(singleElement != 0)
                    {
                        tripplePower = singleElement * singleElement * singleElement;
                        colorCodedArray[((i * widthCompressed) + j) * bytesPerPixel + k++] = (byte)(tripplePower % (255 * 255));
                        colorCodedArray[((i * widthCompressed) + j) * bytesPerPixel + k++] = (byte)(tripplePower % (255));
                        colorCodedArray[((i * widthCompressed) + j) * bytesPerPixel + k++] = singleElement;
                        k = 0;
                    }
                        
                }
            }

            return colorCodedArray;
        }

        /// <summary>
        /// This method identifies and groups pixels into regions. Basically, if
        /// a pixel is adjacent to another pixel, then they are from the same region.
        /// </summary>
        /// <param name="array"></param>
        /// <returns></returns>
        private byte[] regionFiller(byte[] array)
        {
            byte singleElement = 0;
            byte[] regionFilledArray = new byte[heightDownsized * widthDownsized];
            int regionCount = 255;
            int top = 0, left = 0;
            
            List<HashSet<int>> equivalanceTable = new List<HashSet<int>>();

            for (int i = 0; i < heightDownsized; i++)
            {
                for (int j = 0; j < widthDownsized; j++)
                {
                    regionFilledArray[(i * widthDownsized) + j] = 0;
                }
            }

            for (int i = 1; i < heightDownsized; i++)
            {
                for (int j = 1; j < widthDownsized; j++)
                {
                    singleElement = array[(i * widthDownsized) + j];
                    top = regionFilledArray[((i - 1) * widthDownsized) + j];
                    left = regionFilledArray[(i * widthDownsized) + (j - 1)];

                    if (singleElement != 0)
                    {
                        if (top == 0 && left == 0)
                        {
                            regionCount -= 1;      // -= 40
                            regionFilledArray[(i * widthDownsized) + j] = (byte)regionCount;
                        } else if (top != 0 && left == 0)
                        {
                            regionFilledArray[(i * widthDownsized) + j] = (byte)top;
                        } else if (top == 0 && left != 0)
                        {
                            regionFilledArray[(i * widthDownsized) + j] = (byte)left;
                        } else if (top != 0 && left != 0)
                        {
                            if (top == left)
                            {
                                regionFilledArray[(i * widthDownsized) + j] = (byte)top;
                            }
                            else
                            {
                                bool containedFound = false;
                                if (top > left)
                                {
                                    regionFilledArray[(i * widthDownsized) + j] = (byte)top;
                                } else
                                {
                                    regionFilledArray[(i * widthDownsized) + j] = (byte)left;
                                }

                                foreach (HashSet<int> e in equivalanceTable)
                                { 
                                    if (e.Contains(top))
                                    {
                                        containedFound = true;
                                        e.Add(left);
                                    }
                                    if (e.Contains(left))
                                    {
                                        containedFound = true;
                                        e.Add(top);
                                    }
                                }

                                if(!containedFound)
                                {
                                    HashSet<int> tmp = new HashSet<int>();
                                    tmp.Add(top);
                                    tmp.Add(left);
                                    equivalanceTable.Add(tmp);
                                }
                            }
                        }
                    }
                }
            }

            for (int i = 0; i < heightDownsized; i++)
            {
                for (int j = 0; j < widthDownsized; j++)
                {
                    singleElement = array[(i * widthDownsized) + j];
                    if (singleElement != 0)
                    {
                        foreach (HashSet<int> e in equivalanceTable)
                        {
                            if (e.Contains(regionFilledArray[(i * widthDownsized) + j]))
                            {
                                regionFilledArray[(i * widthDownsized) + j] = (byte)e.Max();
                            }
                        }
                    }
                }
            }

            return regionFilledArray;
        }

        private byte[] regionFillerTopLeft(byte[] array)
        {
            byte singleElement = 0;
            byte[] regionFilledArray = new byte[heightDownsized * widthDownsized];
            int regionCount = 255;
            int top = 0, left = 0, topLeft = 0;

            List<HashSet<int>> equivalanceTable = new List<HashSet<int>>();

            for (int i = 0; i < heightDownsized; i++)
            {
                for (int j = 0; j < widthDownsized; j++)
                {
                    regionFilledArray[(i * widthDownsized) + j] = 0;
                }
            }

            for (int i = 1; i < heightDownsized; i++)
            {
                for (int j = 1; j < widthDownsized; j++)
                {
                    singleElement = array[(i * widthDownsized) + j];
                    top = regionFilledArray[((i - 1) * widthDownsized) + j];
                    left = regionFilledArray[(i * widthDownsized) + (j - 1)];
                    topLeft = regionFilledArray[((i - 1) * widthDownsized) + (j - 1)];

                    if (singleElement != 0)
                    {
                        if (top == 0 && left == 0 && topLeft == 0)
                        {
                            regionCount -= 1;      // -= 40
                            regionFilledArray[(i * widthDownsized) + j] = (byte)regionCount;
                        }
                        else if (top != 0 && left == 0 && topLeft == 0)
                        {
                            regionFilledArray[(i * widthDownsized) + j] = (byte)top;
                        }
                        else if (top == 0 && left != 0 && topLeft == 0)
                        {
                            regionFilledArray[(i * widthDownsized) + j] = (byte)left;
                        }
                        else if (top == 0 && left == 0 && topLeft != 0)
                        {
                            regionFilledArray[(i * widthDownsized) + j] = (byte)topLeft;
                        }
                        else if (top != 0 && left != 0 && topLeft != 0)
                        {
                            if (top == left)
                            {
                                regionFilledArray[(i * widthDownsized) + j] = (byte)top;
                            }
                            else
                            {
                                bool containedFound = false;
                                if (top > left)
                                {
                                    regionFilledArray[(i * widthDownsized) + j] = (byte)top;
                                }
                                else
                                {
                                    regionFilledArray[(i * widthDownsized) + j] = (byte)left;
                                }

                                foreach (HashSet<int> e in equivalanceTable)
                                {
                                    if (e.Contains(top))
                                    {
                                        containedFound = true;
                                        e.Add(left);
                                    }
                                    if (e.Contains(left))
                                    {
                                        containedFound = true;
                                        e.Add(top);
                                    }
                                }

                                if (!containedFound)
                                {
                                    HashSet<int> tmp = new HashSet<int>();
                                    tmp.Add(top);
                                    tmp.Add(left);
                                    equivalanceTable.Add(tmp);
                                }
                            }
                        }
                    }
                }
            }

            for (int i = 0; i < heightDownsized; i++)
            {
                for (int j = 0; j < widthDownsized; j++)
                {
                    singleElement = array[(i * widthDownsized) + j];
                    if (singleElement != 0)
                    {
                        foreach (HashSet<int> e in equivalanceTable)
                        {
                            if (e.Contains(regionFilledArray[(i * widthDownsized) + j]))
                            {
                                regionFilledArray[(i * widthDownsized) + j] = (byte)e.Max();
                            }
                        }
                    }
                }
            }

            return regionFilledArray;
        }


        /// <summary>
        /// This methods usefullness is in question.
        /// </summary>
        /// <param name="array"></param>
        /// <returns></returns>
        private byte[] minimizeGulfs(byte[] array)
        {
            byte[] gulfMinimizedArray = new byte[heightDownsized * widthDownsized];
            int lastPosition = 0;
            bool leftBorderWhite = false, rightBorderWhite = false;

            // Horizontal filling:
            for (int i = 0; i < heightDownsized; i++)
            { 
                for (int j = 0; j < widthDownsized; j++)
                {
                    if (array[(i * widthDownsized) + j] == 255 && array[(i * widthDownsized) + j - 1] == 0 &&
                        leftBorderWhite == false)
                    {
                        leftBorderWhite = true;
                    }

                    if (array[(i * widthDownsized) + j] == 255 && array[(i * widthDownsized) + j + 1] == 0
                        && leftBorderWhite == true)
                    {
                        rightBorderWhite = true;
                        lastPosition = j;
                    }

                    if (array[(i * widthDownsized) + j] == 255 && array[(i * widthDownsized) + j - 1] == 0
                        && array[(i * widthDownsized) + j + 1] == 0 && leftBorderWhite == false && rightBorderWhite == false)
                    {
                        leftBorderWhite = true;
                        rightBorderWhite = true;
                        lastPosition = j;
                    }

                    if (array[(i * widthDownsized) + j] == 255 && array[(i * widthDownsized) + j - 1] == 0 &&
                        leftBorderWhite == true && rightBorderWhite == true)
                    {
                        if (j - lastPosition < 15)
                        {
                            // We can now fill:
                            for (int k = 1; k <= (j - lastPosition); k++)
                            {
                                gulfMinimizedArray[(i * widthDownsized) + (lastPosition + k)] = 255;
                            }
                        }

                        if (array[(i * widthDownsized) + j + 1] != 0)
                        {
                            rightBorderWhite = false;
                        }
                        
                    }

                    gulfMinimizedArray[(i * widthDownsized) + j] = array[(i * widthDownsized) + j];
                }
            }

            // Vertical filling:
            for (int j = 0; j < widthDownsized; j++)
            {
                for (int i = 0; i < heightDownsized; i++)
                {
                    if (gulfMinimizedArray[(i * widthDownsized) + j] == 255 && gulfMinimizedArray[((i - 1) * widthDownsized) + j] == 0 &&
                        leftBorderWhite == false)
                    {
                        leftBorderWhite = true;
                    }

                    if (gulfMinimizedArray[(i * widthDownsized) + j] == 255 && gulfMinimizedArray[((i+1) * widthDownsized) + j] == 0
                        && leftBorderWhite == true)
                    {
                        rightBorderWhite = true;
                        lastPosition = i;
                    }

                    if (gulfMinimizedArray[(i * widthDownsized) + j] == 255 && gulfMinimizedArray[((i+1) * widthDownsized) + j] == 0
                        && gulfMinimizedArray[((i-1) * widthDownsized) + j] == 0 && leftBorderWhite == false && rightBorderWhite == false)
                    {
                        leftBorderWhite = true;
                        rightBorderWhite = true;
                        lastPosition = i;
                    }

                    if (gulfMinimizedArray[(i * widthDownsized) + j] == 255 && gulfMinimizedArray[((i-1) * widthDownsized) + j] == 0 &&
                        leftBorderWhite == true && rightBorderWhite == true)
                    {
                        if (i - lastPosition < 15)
                        {
                            // We can now fill:
                            for (int k = 1; k <= (i - lastPosition); k++)
                            {
                                gulfMinimizedArray[((lastPosition + k) * widthDownsized) + j] = 255;
                            }
                        }

                        if (gulfMinimizedArray[((i + 1) * widthDownsized) + j] != 0)
                        {
                            rightBorderWhite = false;
                        }
                        
                    }
                    
                }
            }


            return gulfMinimizedArray;
        }

        private void drawContourOnGrid(byte[] contourMap)
        {
            for (int i = 0; i < heightCompressed; i++)
            {
                for (int j = 0; j < widthCompressed; j++)
                {
                    if(contourMap[(i * widthCompressed) + j] == 255)
                    {
                        drawLine(j, i, j + 1, i + 1, Brushes.White, Brushes.White);
                    }
                }
            }
        }

        

        private void drawOnGrid()
        {
            drawLine(0, 0, 400, 300, Brushes.White, Brushes.White);
            drawLine(10, 50, 300, 250, Brushes.Red, Brushes.Red);
        }

        private void drawLine(int x1, int y1, int x2, int y2, Brush outer, Brush inner)
        {
            Line objLine = new Line();

            objLine.Stroke = outer;
            objLine.Fill = inner;
            //objLine.Width = 1;

            //< Start-Point > 
            objLine.X1 = x1;
            objLine.Y1 = y1;
            //</ Start-Point > 

            //< End-Point > 
            objLine.X2 = x2;
            objLine.Y2 = y2;
            //</ End-Point > 

            //< show in maingrid > 
            CoGRGrid.Children.Add(objLine);
            //</ show in maingrid > 
        }

        /// <summary>
        /// Converts the grayscale depth pixels into a Bitmap
        /// </summary>
        /// <returns></returns>
        private ImageSource ToBitmap()
        {
            PixelFormat format = PixelFormats.Bgr32;
            int stride = widthActual * format.BitsPerPixel / 8;

            return BitmapSource.Create(widthActual, heightActual, 96, 96, format, null, depthPixelsColor, stride);
        }
    }
}
