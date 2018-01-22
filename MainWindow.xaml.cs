//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Windows;
    using System.Windows.Media;
    using Microsoft.Kinect;
    using System.Windows.Media.Imaging;
    using KinectStreams;
    using KinectColor;
    using KinectDepth;
    using KinectGestures;
    using System.Windows.Media.Media3D;
    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSourceColorHUD;

        /// <summary>
        /// Drawing group HUD
        /// </summary>
        private DrawingGroup drawingGroupColorHUD;
        
        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap depthBitmap = null;

        /// <summary>
        /// Bitmap to display color depth
        /// </summary>
        private WriteableBitmap depthBitmapColor = null;
        
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        private DrawingGroup drawingGroupDepth;

        /* Body Related Variables */
        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);
        
        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;
        private DrawingGroup drawingGroupXZ;
        private DrawingGroup drawingGroupYX;
        private DrawingGroup drawingGroupYZ;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;
        private DrawingImage imageSourceXZ;
        private DrawingImage imageSourceYX;
        private DrawingImage imageSourceYZ;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /* Extras */
        private readonly Brush headGestureBrush = new SolidColorBrush(Color.FromArgb(100, 0, 255, 0));
        private const double headGestureSize = 17;

        public GesturesMasterControl gesture_master_control;
        public ColorMasterConrol color_master_control;
        public DepthMasterControl depth_master_control;

        public Queue<Body> body_queue = new Queue<Body>();
        public bool gesture_MC_loaded = false;
        
        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();
            
            // create the colorFrameDescription from the ColorFrameSource using Bgra format
            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);

            // Instantiate the Color Processing:
            color_master_control = new ColorMasterConrol(colorFrameDescription.Width, colorFrameDescription.Height);

            // Create the drawing group we'll use for drawing
            this.drawingGroupColorHUD = new DrawingGroup();

            // Create an image source that we can use as the HUD
            this.imageSourceColorHUD = new DrawingImage(this.drawingGroupColorHUD);

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;
            
            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;
            
            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();
            this.drawingGroupXZ = new DrawingGroup();
            this.drawingGroupYX = new DrawingGroup();
            this.drawingGroupYZ = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);
            this.imageSourceXZ = new DrawingImage(this.drawingGroupXZ);
            this.imageSourceYX = new DrawingImage(this.drawingGroupYX);
            this.imageSourceYZ = new DrawingImage(this.drawingGroupYZ);

            // use the window object as the view model in this simple example
            this.DataContext = this;
            
            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        public ImageSource ImageSource2
        {
            get
            {
                return this.imageSourceXZ;
            }
        }

        public ImageSource ImageSource3
        {
            get
            {
                return this.imageSourceYX;
            }
        }

        public ImageSource ImageSource4
        {
            get
            {
                return this.imageSourceYZ;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }
        

        private void drawColorHUD()
        {
            using (DrawingContext drawingContext = this.drawingGroupColorHUD.Open())
            {
                CameraSpacePoint position = new CameraSpacePoint();
                position.X = -950;
                position.Y = 0;
                position.Z = 0;

                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                Point x1 = new Point(depthSpacePoint.X, depthSpacePoint.Y);

                position.X = 950;
                position.Y = 0;
                position.Z = 0;

                depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                Point x2 = new Point(depthSpacePoint.X, depthSpacePoint.Y);

                Color colore = new Color();
                colore.ScA = 150;
                colore.ScR = 150;
                colore.ScG = 150;
                colore.ScB = 150;

                SolidColorBrush brushe = new SolidColorBrush(colore);

                Pen drawPen = new Pen(brushe, 10);
                
                drawingContext.DrawLine(drawPen, x1, x2);
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }
        
        public static ImageSource ToBitmapDepth(int height, int width, byte[] pixels)
        {
            PixelFormat format = PixelFormats.Bgr32;
            int stride = width * format.BitsPerPixel / 8;

            return BitmapSource.Create(width, height, 96, 96, format, null, pixels, stride);
        }
        
        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary> 
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
           
            
            bool dataReceived = false;
            
            Vector3D cur_2D = new Vector3D(-1, -1, 0);
            Vector3D prev_2D = new Vector3D(2, 0, 0);
            //main_text_block_4.Text = "Angle: " + Vector3D.AngleBetween(prev_2D, cur_2D).ToString();

            // Loading everything for the first time:
            if (gesture_MC_loaded == false)
            {
                // DataSignalSender.ListenForClient();

                // Kinect Connected. Begin Operations:
                gesture_master_control = new GesturesMasterControl();
                gesture_master_control.stopWatch.Start();
                
                TextInformation.main_text_block_1 = this.main_text_block_1;
                TextInformation.main_text_block_2 = this.main_text_block_2;
                TextInformation.main_text_block_3 = this.main_text_block_3;
                TextInformation.main_text_block_4 = this.main_text_block_4;

                gesture_MC_loaded = true;
            }

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {

                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0, tracked_count = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            tracked_count++;
                            body_queue.Enqueue(body);

                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {

                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    //TextInformation.insert_main_text_block("Tracked bodies: " + tracked_count, 2);
                    if (tracked_count > 0)
                    {

                        Body running_body = choose_skeleton_gesture_sitting(bodies);

                        if (running_body != null)
                        {
                            CameraSpacePoint position = running_body.Joints[JointType.Head].Position;
                            DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                            Point headPoint = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            this.DrawGestureHead(headPoint, dc);
                        }

                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
            
        }

        private void DrawGestureHead(Point head, DrawingContext drawingContext)
        {
            drawingContext.DrawEllipse(this.headGestureBrush, null, head, headGestureSize, headGestureSize);
        }

        private Body choose_skeleton_gesture_sitting(Body[] multiple_bodies)
        {
            Body body_to_run = null;
            int tracking_joint_count = 0;
            bool is_runnable = false, enough_tracked_joints = false, clipped_3_sides = false,
                clipped_bottom = false, past_Z_minimum = false, within_center = false, all_limbs_tracked = false;

            if (body_queue.Count > 0)
            {
                body_to_run = body_queue.Dequeue();

                // Try to see which skeleton is good for gesture:
                while (is_runnable == false)
                {

                    // OPTIONAL: CHECK THAT ALL THE LIMBS THAT ARE USED IN THE GESTURE ANALYSIS ARE TRACKED AND NOT INFERRED:
                    if (body_to_run.Joints[JointType.ShoulderLeft].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.ShoulderRight].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.ElbowLeft].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.ElbowRight].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.HandLeft].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.HandRight].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.SpineShoulder].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.Head].TrackingState == TrackingState.Tracked)
                    {
                        all_limbs_tracked = true;
                    }


                    if (body_to_run.Joints[JointType.ShoulderLeft].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.ShoulderRight].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.ElbowLeft].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.ElbowRight].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.HandLeft].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.HandRight].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.SpineShoulder].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.SpineBase].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.HipLeft].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.HipRight].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.SpineMid].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.AnkleLeft].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.AnkleRight].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.Head].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;

                    // Total checked joints are 14; lets check for 12 joints:
                    if (tracking_joint_count >= 7)
                    {
                        enough_tracked_joints = true;

                    }

                    //main_text_block_4.Text = "Enough Tracked Joints: " + enough_tracked_joints.ToString() + "\n";
                    //main_text_block_4.Text += "Tracked Joints: " + tracking_joint_count.ToString() + "\n";
                    // Lets check if the left, right, and top edges are clipped:
                    if (body_to_run.ClippedEdges.HasFlag(FrameEdges.Left) ||
                        body_to_run.ClippedEdges.HasFlag(FrameEdges.Right) ||
                        body_to_run.ClippedEdges.HasFlag(FrameEdges.Top))
                    {
                        clipped_3_sides = true;
                    }

                    //main_text_block_4.Text += "Clipped 3 Sides: " + clipped_3_sides.ToString() + "\n";

                    // Lets check if the bottom edge is clipped:
                    if (body_to_run.ClippedEdges.HasFlag(FrameEdges.Bottom))
                    {
                        clipped_bottom = true;
                    }

                    //main_text_block_4.Text += "Clipped Bottom: " + clipped_bottom.ToString() + "\n";

                    // We also need to delimit the center:
                    // I made a change: we are now measuring the position of the spine shoulder...
                    double X_position = body_to_run.Joints[JointType.SpineShoulder].Position.X;
                    double Z_position = body_to_run.Joints[JointType.SpineShoulder].Position.Z;

                    // Set the closest distance towards camera:
                    if (Z_position > .5)    /* OLD: 1.5 meters */
                    {
                        past_Z_minimum = true;
                    }

                    //main_text_block_4.Text += "Z distance: " + Z_position.ToString() + "\n" + "X distance: " + X_position.ToString() + "\n";

                    // Check to see if the skeleton is within the center:
                    if (Math.Abs(X_position) < .4)    /* OLD: 0.2 meters */
                    {
                        within_center = true;
                    }

                    //main_text_block_4.Text += "Within Center: " + within_center.ToString() + "\n";
                    //main_text_block_4.Text += "Past Z Minimum: " + past_Z_minimum.ToString() + "\n";

                    // Looking into all of the boolean variables:
                    if (all_limbs_tracked == true && enough_tracked_joints == true && (clipped_3_sides == false || clipped_bottom == true) &&
                       past_Z_minimum == true && within_center == true)
                    {
                        gesture_master_control.setBody(body_to_run, ref drawingGroupXZ, ref drawingGroupYX, ref drawingGroupYZ);
                        gesture_master_control.runGestureAnalysis();
                            
                        is_runnable = true;
                        //main_text_block_4.Text += "Running" + "\n";
                    }
                    else
                    {
                        //main_text_block_4.Text += "Not Running" + "\n";
                        tracking_joint_count = 0;
                        enough_tracked_joints = false;
                        clipped_3_sides = false;
                        clipped_bottom = false;
                        past_Z_minimum = false;
                        within_center = false;
                        all_limbs_tracked = false;

                        // Making sure that we do have something to run with:
                        if (body_queue.Count > 0)
                        {
                            body_to_run = body_queue.Dequeue();
                        }
                        else
                        {
                            break;
                        }
                    }
                }

                body_queue.Clear();

            }
            return body_to_run;
        }

        private Body choose_skeleton_gesture(Body[] multiple_bodies)
        {
            Body body_to_run = null;
            int tracking_joint_count = 0;
            bool is_runnable = false, enough_tracked_joints = false, clipped_3_sides = false,
                clipped_bottom = false, past_Z_minimum = false, within_center = false, all_limbs_tracked = false;

            if (body_queue.Count > 0)
            {
                body_to_run = body_queue.Dequeue();

                // Try to see which skeleton is good for gesture:
                while (is_runnable == false)
                {

                    // OPTIONAL: CHECK THAT ALL THE LIMBS THAT ARE USED IN THE GESTURE ANALYSIS ARE TRACKED AND NOT INFERRED:
                    if (body_to_run.Joints[JointType.ShoulderLeft].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.ShoulderRight].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.ElbowLeft].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.ElbowRight].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.HandLeft].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.HandRight].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.SpineShoulder].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.Head].TrackingState == TrackingState.Tracked)
                    {
                        all_limbs_tracked = true;
                    }


                    if (body_to_run.Joints[JointType.ShoulderLeft].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.ShoulderRight].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.ElbowLeft].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.ElbowRight].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.HandLeft].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.HandRight].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.SpineShoulder].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.SpineBase].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.HipLeft].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.HipRight].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.SpineMid].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.AnkleLeft].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.AnkleRight].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.Head].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;

                    // Total checked joints are 14; lets check for 12 joints:
                    if (tracking_joint_count >= 7)
                    {
                        enough_tracked_joints = true;

                    }

                    //main_text_block_4.Text = "Enough Tracked Joints: " + enough_tracked_joints.ToString() + "\n";
                    //main_text_block_4.Text += "Tracked Joints: " + tracking_joint_count.ToString() + "\n";
                    // Lets check if the left, right, and top edges are clipped:
                    if (body_to_run.ClippedEdges.HasFlag(FrameEdges.Left) ||
                        body_to_run.ClippedEdges.HasFlag(FrameEdges.Right) ||
                        body_to_run.ClippedEdges.HasFlag(FrameEdges.Top))
                    {
                        clipped_3_sides = true;
                    }

                    //main_text_block_4.Text += "Clipped 3 Sides: " + clipped_3_sides.ToString() + "\n";

                    // Lets check if the bottom edge is clipped:
                    if (body_to_run.ClippedEdges.HasFlag(FrameEdges.Bottom))
                    {
                        clipped_bottom = true;
                    }

                    //main_text_block_4.Text += "Clipped Bottom: " + clipped_bottom.ToString() + "\n";

                    // We also need to delimit the center:
                    // I made a change: we are now measuring the position of the spine shoulder...
                    double X_position = body_to_run.Joints[JointType.SpineShoulder].Position.X;
                    double Z_position = body_to_run.Joints[JointType.SpineShoulder].Position.Z;

                    // Set the closest distance towards camera:
                    if (Z_position > 1.9)    /* OLD: 1.5 meters */
                    {
                        past_Z_minimum = true;
                    }

                    //main_text_block_4.Text += "Z distance: " + Z_position.ToString() + "\n" + "X distance: " + X_position.ToString() + "\n";

                    // Check to see if the skeleton is within the center:
                    if (Math.Abs(X_position) < .4)    /* OLD: 0.2 meters */
                    {
                        within_center = true;
                    }

                    //main_text_block_4.Text += "Within Center: " + within_center.ToString() + "\n";
                    //main_text_block_4.Text += "Past Z Minimum: " + past_Z_minimum.ToString() + "\n";

                    // Looking into all of the boolean variables:
                    if (all_limbs_tracked == true && enough_tracked_joints == true && (clipped_3_sides == false || clipped_bottom == true) &&
                       past_Z_minimum == true && within_center == true)
                    {
                        gesture_master_control.setBody(body_to_run, ref drawingGroupXZ, ref drawingGroupYX, ref drawingGroupYZ);
                        gesture_master_control.runGestureAnalysis();

                        

                        is_runnable = true;
                        //main_text_block_4.Text += "Running" + "\n";
                    }
                    else
                    {
                        //main_text_block_4.Text += "Not Running" + "\n";
                        tracking_joint_count = 0;
                        enough_tracked_joints = false;
                        clipped_3_sides = false;
                        clipped_bottom = false;
                        past_Z_minimum = false;
                        within_center = false;
                        all_limbs_tracked = false;

                        // Making sure that we do have something to run with:
                        if (body_queue.Count > 0)
                        {
                            body_to_run = body_queue.Dequeue();
                        }
                        else
                        {
                            break;
                        }
                    }
                }

                body_queue.Clear();

            }
            return body_to_run;
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints,
            DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }



        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints,
            JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }
            
            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        private void TextBox_TextChanged(object sender, System.Windows.Controls.TextChangedEventArgs e)
        {

        }

    }
}
