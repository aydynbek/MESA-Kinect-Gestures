using System;
using Microsoft.Kinect;
using System.Diagnostics;
using System.Windows.Controls;
using System.Windows.Media.Media3D;
using KinectStreams;
using System.Windows.Media;
using System.Windows;

namespace KinectGestures
{
    public enum GestureRunningState { Unknown, None, PanelRotation, RoboticArmHandling }

    public enum JointGeneralState { Unknown, Moving, Still }
    public enum ArmRotationalState { Unknown, RotatingDown, RotatingUp, RotatingClockWise, RotatingCntClockWise, NotRotating }
    public enum ArmPosition { Unknown, ArmStretchedOutHorizontally, ArmUp, ArmDown }
    public enum ArmRotationTrackingState { Unknown, Tracking, NotTracking, Initialized, Uninitialized }

    internal class Rotations
    {
        public Stopwatch stopwatch_rot;
        public Vector3D previous_frame_vector;
        public ArmRotationalState previous_rot_state = new ArmRotationalState();
        public int frame_counter;
        public ArmRotationTrackingState rot_tracking_state = new ArmRotationTrackingState();
        public double angle;
        public double[] previous_angles;
        public bool enough_time_at_angle;
        public bool readyForNewSignal = true;

        public const int SIGNAL_SAME_ANGLE_TIME = 1000;
        public const int PREV_ARRAY_LENGTH = 5;
        // Changed from one to three degrees..
        public const double angle_diff_sensitivity = 2;

        public Rotations()
        {
            stopwatch_rot = new Stopwatch();
            previous_angles = new double[PREV_ARRAY_LENGTH];
            previous_frame_vector = new Vector3D();
            previous_rot_state = ArmRotationalState.Unknown;
            frame_counter = 0;
            rot_tracking_state = ArmRotationTrackingState.Unknown;
            angle = 0;
            enough_time_at_angle = false;

            stopwatch_rot.Start();
        }

        public void managePrevAngles(double angle)
        {
            int i = PREV_ARRAY_LENGTH - 1;
            while (i > 0)
            {
                previous_angles[i] = previous_angles[i - 1];
                i--;
            }

            previous_angles[i] = angle;
        }

        public void stabializeAngle()
        {
            if (frame_counter > PREV_ARRAY_LENGTH)
            {
                if (Math.Abs(previous_angles[0] - previous_angles[PREV_ARRAY_LENGTH - 1]) > angle_diff_sensitivity)
                {
                    // Changing angle:
                    angle = previous_angles[0];

                    // Not the same angle, so no need to compare the times:
                    stopwatch_rot.Reset();
                    stopwatch_rot.Start();
                }
                else
                {
                    // Keeping the old angle:
                    previous_angles[PREV_ARRAY_LENGTH - 2] = angle;
                }

                if (stopwatch_rot.ElapsedMilliseconds > SIGNAL_SAME_ANGLE_TIME)
                {
                    // We have enough time at the same angle:
                    enough_time_at_angle = true;
                }
                else
                {
                    enough_time_at_angle = false;
                    readyForNewSignal = true;
                }
            }
            else
            {
                angle = previous_angles[0];
            }
        }
    }

    internal class Gesture_RoboticArmHandling
    {
        // These vectors will also be used for figuring out which direction the rotation is supposed 
        // to be in:
        private Vector3D shoulder_to_elbow_AB_current;
        private Vector3D elbow_to_wrist_BC_current;
        private Vector3D shoulder_to_elbow_AB_previous;
        private Vector3D elbow_to_wrist_BC_previous;
        
        // These may be good for storing the previous angles:
        // theta is the rotation about the joint axis:
        private double theta_AB = 0;
        private double theta_BC = 0;

        // phi is the rotation about the limb axis:
        private double phi_AB = 0;
        private double phi_BC = 0;
        
        private Joint right_wrist_current;
        private Joint right_elbow_current;
        private Joint right_shoulder_current;

        private Vector3D normalVectorOfPlanePrev_AB;
        private Vector3D normalVectorOfPlaneCurr_AB;
        private Vector3D normalVectorOfPlanePrev_BC;
        private Vector3D normalVectorOfPlaneCurr_BC;

        private Vector3D phiAxix_AB_curr;
        private Vector3D thetaAxix_AB_curr;
        private Vector3D phiAxix_AB_prev;
        private Vector3D thetaAxix_AB_prev;
        private Vector3D phiAxix_BC_curr;
        private Vector3D thetaAxix_BC_curr;
        private Vector3D phiAxix_BC_prev;
        private Vector3D thetaAxix_BC_prev;
        
        // Length of orientation vector is 10cm
        private int cnt = 0;
        private string stt = "";
        
        private bool canAcceptNewMovement = true;

        public Gesture_RoboticArmHandling()
        {
            shoulder_to_elbow_AB_current = new Vector3D();
            elbow_to_wrist_BC_current = new Vector3D();
            shoulder_to_elbow_AB_previous = new Vector3D();
            elbow_to_wrist_BC_previous = new Vector3D();
        }

        /// <summary>
        /// This method is designed to initiate the first rotation of the limbs into their
        /// positions which they were at during triggering. The algorithm assumes that 
        /// the arm was initially streching out normal to the torso and that it then has to
        /// rotate the limbs into the initial position.
        /// </summary>
        internal void initializeGestureJanuary()
        {
            // ATTENTION: All of this code is written with an assumption that the person is standing right in front of 
            // the Kinect, and his Right arm will be used for the measurement, and that arm has to be pointing 
            // generally in the right direction.

            // ATTENTION: We only have positive Z coordinates from the camera. Camera point is 0 for Z, and then 
            // it increases as you move further away.

            // ATTENTION: If we are only going to use 2 servo motors and 2 pairs of boards to try to replicate the whole ABC 
            // arm, then use theta_AB for whole theta and use phi_BC + phi_AB for whole phi. This is for Avery's board.
            
            // ATTENTION: All theta and phi angles are incremental, not absolute. All angle rotations go in the 
            // direction of lowest incremental angle change.
            
            // ATTENTION: From the calculations, theta never took a negative value whilst phi did. Any backwards movement throught 
            // the axix only changes the phi angle.

            // ATTENTION: We rotate figure 2.4 90 degrees counter clockwise about the x-axis.

            /// /////////////////////////////////////
            // ATTENTION: Perhabs, when it comes to the direction of theta rotation, we simply have to calculate with phi in mind, 
            // which movement is shorter.


            right_wrist_current = GesturesMasterControl.body.Joints[JointType.WristRight];
            right_elbow_current = GesturesMasterControl.body.Joints[JointType.ElbowRight];
            right_shoulder_current = GesturesMasterControl.body.Joints[JointType.ShoulderRight];

            // STEP A
            // Lets create the AB and BC vectors

            // The current vector will simply be the shoulder to elbow limb:
            shoulder_to_elbow_AB_current = GestureAuxilaryMethods.updateLimbVectors(JointType.ShoulderRight,
                JointType.ElbowRight);

            elbow_to_wrist_BC_current = GestureAuxilaryMethods.updateLimbVectors(JointType.ElbowRight,
                JointType.WristRight);

            // Previous vector is set to an initial position, that is a vector with only an X component:
            shoulder_to_elbow_AB_previous.X = shoulder_to_elbow_AB_current.Length;
            shoulder_to_elbow_AB_previous.Y = 0;
            shoulder_to_elbow_AB_previous.Z = 0;

            elbow_to_wrist_BC_previous.X = elbow_to_wrist_BC_current.Length;
            elbow_to_wrist_BC_previous.Y = 0;
            elbow_to_wrist_BC_previous.Z = 0;

            // Variables for the Rodrigues formula: v is the vector to be rotated, k is the axis
            Vector3D v, k;

            // Calculating cartesian coordinates of joints:
            double rEcpx = right_elbow_current.Position.X;
            double rEcpy = right_elbow_current.Position.Y;      // Right elbow current position Y
            double rEcpz = right_elbow_current.Position.Z;
            double rScpx = right_shoulder_current.Position.X;
            double rScpy = right_shoulder_current.Position.Y;   // Right shoulder current position Y
            double rScpz = right_shoulder_current.Position.Z;
            double rWcpx = right_wrist_current.Position.X;
            double rWcpy = right_wrist_current.Position.Y;
            double rWcpz = right_wrist_current.Position.Z;

            // Lets get to the AB angles
            // Cartesian coordinates of the tip of the current vector:
            double realXcurr = rEcpx - rScpx;
            double realYcurr = rEcpy - rScpy;
            double realZcurr = rEcpz - rScpz;

            // Initializing the first instances of the rotation axes:
            phiAxix_AB_prev = new Vector3D(shoulder_to_elbow_AB_current.Length, 0, 0);
            phiAxix_BC_prev = new Vector3D(elbow_to_wrist_BC_current.Length, 0, 0);
            thetaAxix_AB_prev = new Vector3D(0, -10, 0);
            thetaAxix_BC_prev = new Vector3D(0, -10, 0);
            

            // Calculating the theta angle:
            theta_AB = Vector3D.AngleBetween(new Vector3D(10, 0, 0), shoulder_to_elbow_AB_current);
            stt += theta_AB.ToString("n4") + " ";
            // This is the vector which we will use to compare the normal of our current vector plane:
            normalVectorOfPlanePrev_AB = new Vector3D(0,10,0);
            normalVectorOfPlaneCurr_AB = Vector3D.CrossProduct(shoulder_to_elbow_AB_current, new Vector3D(10,0,0));

            // We now have the angle, but its unsigned, that is, we dont know the direction of rotation:
            phi_AB = Vector3D.AngleBetween(normalVectorOfPlaneCurr_AB, normalVectorOfPlanePrev_AB);

            // Calculating the direction of phi rotation. We need a vector that is rotated about the theta axis
            // angle theta without the phi rotation. We then rotate this vector about phi in positive and 
            // negative phi degrees and check against the actual point to check for the direction. Remember
            // the right hand rule of the Rodrigues formula.
            // This variable is used to rotate the theta axis vector about itself by angle phi:
            Vector3D thetaAxisPrevUnitVector_AB = Vector3D.Divide(thetaAxix_AB_prev, thetaAxix_AB_prev.Length);
            
            v = phiAxix_AB_prev;
            k = thetaAxisPrevUnitVector_AB;

            // Rotating the phi axis by angle theta about theta axis without the phi rotation:
            Vector3D rotatedPhiAxisJustThetaRotation_AB = Vector3D.Multiply(v, Math.Cos(Math.PI * (theta_AB / 180)))
                + Vector3D.Multiply((Vector3D.CrossProduct(k, v)), Math.Sin(Math.PI * (theta_AB / 180)))
                + Vector3D.Multiply(k, (Vector3D.DotProduct(k, v) * (1 - Math.Cos(Math.PI * (theta_AB / 180)))));

            // Now we rotate this vector about previous phi axis by angle +/- phi and check against the actual point:
            // This variable is used to rotate the theta axis vector about itself by angle phi:
            Vector3D phiAxisPrevUnitVector_AB = Vector3D.Divide(phiAxix_AB_prev, phiAxix_AB_prev.Length);

            v = rotatedPhiAxisJustThetaRotation_AB;
            k = phiAxisPrevUnitVector_AB;

            Vector3D rotatedPhiAxisPositive_AB = Vector3D.Multiply(v, Math.Cos(Math.PI * (phi_AB / 180)))
               + Vector3D.Multiply((Vector3D.CrossProduct(k, v)), Math.Sin(Math.PI * (phi_AB / 180)))
               + Vector3D.Multiply(k, (Vector3D.DotProduct(k, v) * (1 - Math.Cos(Math.PI * (phi_AB / 180)))));

            Vector3D rotatedPhiAxisNegative_AB = Vector3D.Multiply(v, Math.Cos(-1 * Math.PI * (phi_AB / 180)))
               + Vector3D.Multiply((Vector3D.CrossProduct(k, v)), Math.Sin(-1 * Math.PI * (phi_AB / 180)))
               + Vector3D.Multiply(k, (Vector3D.DotProduct(k, v) * (1 - Math.Cos(-1 * Math.PI * (phi_AB / 180)))));

            /*
            TextInformation.insert_main_text_block("POS: (" + rotatedPhiAxisPositive_AB.X.ToString("n3") + ", " + rotatedPhiAxisPositive_AB.Y.ToString("n3") +
                ", " + rotatedPhiAxisPositive_AB.Z.ToString("n3") + ")", 1);
            TextInformation.insert_main_text_block("NEG: (" + rotatedPhiAxisNegative_AB.X.ToString("n3") + ", " + rotatedPhiAxisNegative_AB.Y.ToString("n3") +
                ", " + rotatedPhiAxisNegative_AB.Z.ToString("n3") + ")", 1);
            TextInformation.insert_main_text_block("ACT: (" + realXcurr.ToString("n3") + ", " + realYcurr.ToString("n3") +
               ", " + realZcurr.ToString("n3") + ")", 1);
            */

            if((Math.Abs(rotatedPhiAxisNegative_AB.X - realXcurr) + Math.Abs(rotatedPhiAxisNegative_AB.Y - realYcurr) 
                + Math.Abs(rotatedPhiAxisNegative_AB.Z - realZcurr)) < .001)
            {
                phi_AB = phi_AB * -1;
            }
            

            // Updating the rotation axes:
            phiAxix_AB_curr = shoulder_to_elbow_AB_current;



            /// //////////////////////////
            // For theta axis, we need to use a specific formula: (x, yc - zs, ys + zc)
            // thetaAxix_AB_curr = new Vector3D(0, (0 * Math.Cos(Math.PI * (phi_AB/180))) - (10 * Math.Sin(Math.PI * (phi_AB / 180))),
            //    (0 * Math.Sin(Math.PI * (phi_AB / 180))) + (10 * Math.Cos(Math.PI * (phi_AB / 180))));

            

            // Variables for the Rodrigues formula: v is the vector to be rotated, k is the axis
            v = thetaAxix_AB_prev;
            k = phiAxisPrevUnitVector_AB;

            // Rotating the theta axis first; we also need to rotate the theta axis of BC:
            thetaAxix_AB_curr = Vector3D.Multiply(v, Math.Cos(Math.PI * (phi_AB / 180)))
                + Vector3D.Multiply((Vector3D.CrossProduct(k, v)), Math.Sin(Math.PI * (phi_AB / 180)))
                + Vector3D.Multiply(k, (Vector3D.DotProduct(k, v) * (1 - Math.Cos(Math.PI * (phi_AB / 180)))));





            /// We are not considering BC angles until we have finished the handling method
            // Lets get to the BC angles
            // Cartesian coordinates of the tip of the current vector:
            realXcurr = rWcpx - rEcpx;
            realYcurr = rWcpy - rEcpy;
            realZcurr = rWcpz - rEcpz;

            // This is the vector which we will use to compare the normal of our current vector plane:
            normalVectorOfPlanePrev_BC = new Vector3D(0, 10, 0);
            normalVectorOfPlaneCurr_BC = Vector3D.CrossProduct(elbow_to_wrist_BC_current, new Vector3D(10, 0, 0));

            // We now have the angle, but its unsigned, that is, we dont know the direction of rotation:
            phi_BC = Vector3D.AngleBetween(normalVectorOfPlaneCurr_BC, normalVectorOfPlanePrev_BC);

            // Calculating the direction of phi rotation:
            if (realYcurr < 0)
            {
                phi_BC = phi_BC * -1;
            }

            // Calculating the theta angle:
            theta_BC = Vector3D.AngleBetween(new Vector3D(10, 0, 0), elbow_to_wrist_BC_current);

            // Updating the rotation axes:
            phiAxix_BC_curr = elbow_to_wrist_BC_current;
            // For theta axis, we need to use a specific formula: (x, yc - zs, ys + zc)
            thetaAxix_BC_curr = new Vector3D(realXcurr, (realYcurr * Math.Cos(Math.PI * (phi_BC / 180))) - (realZcurr * Math.Sin(Math.PI * (phi_BC / 180))),
                (realYcurr * Math.Sin(Math.PI * (phi_BC / 180))) + (realZcurr * Math.Cos(Math.PI * (phi_BC / 180))));
            

            

            drawImitatedArmJanuary();

            // Updating the vectors in preparation for the next incoming frame calculations:
            shoulder_to_elbow_AB_previous = shoulder_to_elbow_AB_current;
            elbow_to_wrist_BC_previous = elbow_to_wrist_BC_current;

            normalVectorOfPlanePrev_AB = normalVectorOfPlaneCurr_AB;
            normalVectorOfPlanePrev_BC = normalVectorOfPlaneCurr_BC;

            phiAxix_AB_prev = phiAxix_AB_curr;
            phiAxix_BC_prev = phiAxix_BC_curr;
            thetaAxix_AB_prev = thetaAxix_AB_curr;
            thetaAxix_BC_prev = thetaAxix_BC_curr;

            // Correcting the theta_BC and phi_BC values by finding the difference with AB angles:
            if (theta_BC >= theta_AB)
            {
                theta_BC -= theta_AB;
            }
            else
            {
                theta_BC = theta_AB - theta_BC;
            }

            if (phi_BC >= phi_AB)
            {
                phi_BC -= phi_AB;
            }
            else
            {
                phi_BC = phi_AB - phi_BC;
            }
            

            /// //////////////////////////////
            // We need a way to send a signal:


            // Making sure that for the next reading, which will actually be the first instance we will run the
            // handling method, we actually have arm movement so that we can calculate newer angles. Therefore,
            // we need to make canAcceptNewMovement false:
            canAcceptNewMovement = false;

            TextInformation.insert_main_text_block("Initial", 1);
            TextInformation.insert_main_text_block("thetaAB: " + theta_AB.ToString("n4"), 1);
            TextInformation.insert_main_text_block("phiAB: " + phi_AB.ToString("n4"), 1);
            TextInformation.insert_main_text_block("thetaBC: " + theta_BC.ToString("n4"), 1);
            TextInformation.insert_main_text_block("phiBC: " + phi_BC.ToString("n4"), 1);

            
        }

        internal void drawImitatedArmJanuary()
        {
            Pen penn = new Pen(Brushes.Red, 3);
            Pen penn2 = new Pen(Brushes.Black, 2);
            Brush drawBrush = new SolidColorBrush(Color.FromArgb(255, 130, 15, 180));

            double theta_AB_rad = (theta_AB / 180.0) * Math.PI;
            double theta_BC_rad = (theta_BC / 180.0) * Math.PI;
            double phi_AB_rad = (phi_AB / 180.0) * Math.PI;
            double phi_BC_rad = (phi_BC / 180.0) * Math.PI;

            // ATTENTION: We should only reconstruct the cartesian coordinates using the two angles and
            // the initial positioning of the limb.

            // ATTENTION: We will use the Rodrigues' rotation formula to rotate the prevviously known
            // theta axis about the previous phi axis vector by angle phi. The phi axis vector must be
            // a unit vector.

            // ATTENTION: theta is THETA and phi is PHI in spherical coordinates. R is the parameter
            // thats passed into this function. 

            /// Check again:
            // ATTENTION: The corrected THETA angle has to be the difference
            // of 180 and THETA that we calculated. -- REALLY? Check right below:

            // ATTENTION: Given the Figure 2.4 from the book, we have to switch the X and Z axis. The 
            // corrected Z axis (the real life Z axis) is the X axis that we get from calculations,
            // and the corrected X axis is the negative of the Z axis.

            /*
            Vector3D currentThetaAxis_AB_ = Vector3D.Multiply(k, Vector3D.DotProduct(k, v)) + Vector3D.Multiply(Math.Sin(phi_AB_rad), Vector3D.CrossProduct(k, v)) -
               Vector3D.CrossProduct(Vector3D.Multiply(Math.Cos(phi_AB_rad), k), Vector3D.CrossProduct(k, v));
            */

            // This vector will be used to calculate the rotation of vector BC because of the rotations of AB. This
            // is the sum of original vectors AB and BC; a difference of this sum vector and the AB vector will yield
            // the BC vector.
            Vector3D sumOfBothLimbs = Vector3D.Add(elbow_to_wrist_BC_previous, shoulder_to_elbow_AB_previous);

            // This variable is used to rotate the theta axis vector about itself by angle phi:
            Vector3D phiAxisPrevUnitVector_AB = Vector3D.Divide(phiAxix_AB_prev, phiAxix_AB_prev.Length);
            
            // Variables for the Rodrigues formula: v is the vector to be rotated, k is the axis
            Vector3D v, k;
            v = thetaAxix_AB_prev;
            k = phiAxisPrevUnitVector_AB;

            // Rotating the theta axis first; we also need to rotate the theta axis of BC:
            Vector3D rotatedThetaAxis_AB = Vector3D.Multiply(v, Math.Cos(phi_AB_rad)) + Vector3D.Multiply((Vector3D.CrossProduct(k, v)), Math.Sin(phi_AB_rad)) 
                + Vector3D.Multiply(k, (Vector3D.DotProduct(k, v) * (1 - Math.Cos(phi_AB_rad))));

            v = thetaAxix_AB_prev;
            thetaAxix_BC_prev = Vector3D.Multiply(v, Math.Cos(phi_AB_rad)) + Vector3D.Multiply((Vector3D.CrossProduct(k, v)), Math.Sin(phi_AB_rad))
                + Vector3D.Multiply(k, (Vector3D.DotProduct(k, v) * (1 - Math.Cos(phi_AB_rad))));

            // Now we need to rotate the previous limb vector by angle theta about the theta axis:
            v = phiAxix_AB_prev;
            k = Vector3D.Divide(rotatedThetaAxis_AB, rotatedThetaAxis_AB.Length);
            Vector3D rotatedPhiAxis_AB = Vector3D.Multiply(v, Math.Cos(theta_AB_rad)) + Vector3D.Multiply((Vector3D.CrossProduct(k, v)), Math.Sin(theta_AB_rad))
                + Vector3D.Multiply(k, (Vector3D.DotProduct(k, v) * (1 - Math.Cos(theta_AB_rad))));
            
            // Getting the cartesian coordinates:
            double X_AB = rotatedPhiAxis_AB.X;
            double Y_AB = rotatedPhiAxis_AB.Y;
            double Z_AB = rotatedPhiAxis_AB.Z;


            /// We are not considering BC angles until we have finished the handling method
            v = sumOfBothLimbs;
            Vector3D sumOfBothLimbsRot = Vector3D.Multiply(v, Math.Cos(theta_AB_rad)) + Vector3D.Multiply((Vector3D.CrossProduct(k, v)), Math.Sin(theta_AB_rad))
                + Vector3D.Multiply(k, (Vector3D.DotProduct(k, v) * (1 - Math.Cos(theta_AB_rad))));
            phiAxix_BC_prev = Vector3D.Subtract(sumOfBothLimbsRot, rotatedPhiAxis_AB);

            Vector3D phiAxisPrevUnitVector_BC = Vector3D.Divide(phiAxix_BC_prev, phiAxix_BC_prev.Length);
            
            v = thetaAxix_BC_prev;
            k = phiAxisPrevUnitVector_BC;

            Vector3D rotatedThetaAxis_BC = Vector3D.Multiply(v, Math.Cos(phi_BC_rad)) + Vector3D.Multiply((Vector3D.CrossProduct(k, v)), Math.Sin(phi_BC_rad))
                + Vector3D.Multiply(k, (Vector3D.DotProduct(k, v) * (1 - Math.Cos(phi_BC_rad))));

            v = phiAxix_BC_prev;
            k = Vector3D.Divide(rotatedThetaAxis_BC, rotatedThetaAxis_BC.Length);

            Vector3D rotatedPhiAxis_BC = Vector3D.Multiply(v, Math.Cos(theta_BC_rad)) + Vector3D.Multiply((Vector3D.CrossProduct(k, v)), Math.Sin(theta_BC_rad))
                + Vector3D.Multiply(k, (Vector3D.DotProduct(k, v) * (1 - Math.Cos(theta_BC_rad))));
            
            double X_BC = rotatedPhiAxis_BC.X;
            double Y_BC = rotatedPhiAxis_BC.Y;
            double Z_BC = rotatedPhiAxis_BC.Z;

            Vector3D isolatedVect_BC = new Vector3D(X_BC, Y_BC, Z_BC);
            v = isolatedVect_BC;
            k = phiAxisPrevUnitVector_AB;

            Vector3D isolatedVect_BC_rot = Vector3D.Multiply(v, Math.Cos(phi_AB_rad)) + Vector3D.Multiply((Vector3D.CrossProduct(k, v)), Math.Sin(phi_AB_rad))
                + Vector3D.Multiply(k, (Vector3D.DotProduct(k, v) * (1 - Math.Cos(phi_AB_rad))));
            

            /*
            TextInformation.insert_main_text_block("AB: (" + thetaAxix_AB_curr.X.ToString("n3") + ", " + thetaAxix_AB_curr.Y.ToString("n3") +
                ", " + thetaAxix_AB_curr.Z.ToString("n3") + ")", 1);
            TextInformation.insert_main_text_block("AB*: (" + rotatedThetaAxis_AB.X.ToString("n3") + ", " + rotatedThetaAxis_AB.Y.ToString("n3") +
                ", " + rotatedThetaAxis_AB.Z.ToString("n3") + ")", 1);
            TextInformation.insert_main_text_block("*: " + Vector3D.Equals(thetaAxix_AB_curr, rotatedThetaAxis_AB), 1);
            
            TextInformation.insert_main_text_block("AB: (" + rotatedPhiAxis_AB.X.ToString("n3") + ", " + rotatedPhiAxis_AB.Y.ToString("n3") +
                ", " + rotatedPhiAxis_AB.Z.ToString("n3") + ")", 1);
            TextInformation.insert_main_text_block("AB*: (" + shoulder_to_elbow_AB_current.X.ToString("n3") + ", " + shoulder_to_elbow_AB_current.Y.ToString("n3") +
                ", " + shoulder_to_elbow_AB_current.Z.ToString("n3") + ")", 1);

            TextInformation.insert_main_text_block("BC: (" + rotatedPhiAxis_BC.X.ToString("n3") + ", " + rotatedPhiAxis_BC.Y.ToString("n3") +
                ", " + rotatedPhiAxis_BC.Z.ToString("n3") + ")", 1);
            TextInformation.insert_main_text_block("BC*: (" + elbow_to_wrist_BC_current.X.ToString("n3") + ", " + elbow_to_wrist_BC_current.Y.ToString("n3") +
                ", " + elbow_to_wrist_BC_current.Z.ToString("n3") + ")", 1);
            */

            using (DrawingContext dcXZ = GesturesMasterControl.drawingGroupXZ.Open())
            {

                dcXZ.DrawRectangle(Brushes.Aquamarine, null, new Rect(0.0, 0.0,
                512, 424));

                // Arguements to Point Constructor: (Columns starting from left, Row starting from top)
                Point x1 = new Point(80, 212);
                Point y1 = new Point(80 + 400 * X_AB, 212 + 400 * Z_AB);

                Point x2 = new Point(80 + 400 * Z_AB, 212 + 400 * X_AB);
                Point y2 = new Point(80 + 400 * Z_AB + 400 * Z_BC, 212 + 400 * X_AB + 400 * X_BC);

                dcXZ.DrawLine(penn, x1, y1);
                //dcXZ.DrawLine(penn, x2, y2);
                dcXZ.DrawEllipse(drawBrush, null, x1, 7, 7);
                dcXZ.DrawEllipse(drawBrush, null, y1, 7, 7);
                //dcXZ.DrawEllipse(drawBrush, null, y2, 7, 7);
            }

            using (DrawingContext dcYX = GesturesMasterControl.drawingGroupYX.Open())
            {
                dcYX.DrawRectangle(Brushes.Aqua, null, new Rect(0.0, 0.0,
                512, 424));

                Point x1 = new Point(80, 212);
                Point y1 = new Point(80 + 400 * X_AB, 424 - (212 + 400 * Y_AB));

                Point x2 = new Point(80 + 400 * Z_AB, 424 - (212 + 400 * Y_AB));
                Point y2 = new Point(80 + 400 * Z_AB + 400 * Z_BC, 424 - (212 + 400 * Y_AB + 400 * Y_BC));

                dcYX.DrawLine(penn, x1, y1);
                //dcYX.DrawLine(penn, x2, y2);
                dcYX.DrawEllipse(drawBrush, null, x1, 7, 7);
                dcYX.DrawEllipse(drawBrush, null, y1, 7, 7);
                //dcYX.DrawEllipse(drawBrush, null, y2, 7, 7);
            }

            using (DrawingContext dcYZ = GesturesMasterControl.drawingGroupYZ.Open())
            {
                dcYZ.DrawRectangle(Brushes.Aqua, null, new Rect(0.0, 0.0,
                512, 424));

                Point x1 = new Point(256, 212);
                Point y1 = new Point(256 + 400 * Z_AB, 424 - (212 + 400 * Y_AB));

                Point x2 = new Point(256 + 400 * X_AB, 424 - (212 + 400 * Y_AB));
                Point y2 = new Point(256 + 400 * X_AB + 400 * X_BC, 424 - (212 + 400 * Y_AB + 400 * Y_BC));

                dcYZ.DrawLine(penn, x1, y1);
                //dcYZ.DrawLine(penn, x2, y2);
                dcYZ.DrawEllipse(drawBrush, null, x1, 7, 7);
                dcYZ.DrawEllipse(drawBrush, null, y1, 7, 7);
                //dcYZ.DrawEllipse(drawBrush, null, y2, 7, 7);
            }
        }


        /// <summary>
        /// This method is simply calling the initialization method for testing.
        /// </summary>
        /// <param name="rightElbowState"></param>
        /// <param name="rightWristState"></param>
        /// <returns></returns>
        internal int beginArmHandlingTestBuild(JointGeneralState rightElbowState, JointGeneralState rightWristState)
        {
            initializeGestureJanuary();

            // Stop the handling if the left hand is open:
            if (GesturesMasterControl.body.HandLeftState == HandState.Closed)
            {
                return 1;
            }
            else
            {
                return -1;
            }
        }

        /// <summary>
        /// This method will handle the robotic arm rotation angle calculations right after initialization in the following
        /// way. First, the arm movement will be checked to see if it is still or moving. If moving, no angle reading can
        /// be given yet. If still, angle reading can be made and they will be sent. So, basically the angle reading are
        /// done on a move-stop basis, which are also path independent.
        /// </summary>
        /// <param name="rightElbowState"></param>
        /// <param name="rightWristState"></param>
        /// <returns></returns>
        internal int beginArmHandlingPathIndependentJanuary(JointGeneralState rightElbowState, JointGeneralState rightWristState)
        {
            // ATTENTION: For figuring out the phi angles, we have to, for any case, know the plane that is generated 
            // between current and previous vectors.

            // ATTENTION: We need to keep an absolute value of the theta and phi angles. We need to do this because we can't
            // use the spherical coordinates methodology all the time.


            // initializeGestureJanuary();



            if (rightElbowState == JointGeneralState.Still && rightWristState == JointGeneralState.Still && canAcceptNewMovement == true)
            {






                // TextInformation.insert_main_text_block("Handling.", 1);
                // Elbow is still, so we have a unmoving state of arm and we can send rotation angles.

                // Position variables:
                double rEcpx = right_elbow_current.Position.X;
                double rEcpy = right_elbow_current.Position.Y;      // Right elbow current position Y
                double rEcpz = right_elbow_current.Position.Z;
                double rScpx = right_shoulder_current.Position.X;
                double rScpy = right_shoulder_current.Position.Y;   // Right shoulder current position Y
                double rScpz = right_shoulder_current.Position.Z;
                double rWcpx = right_wrist_current.Position.X;
                double rWcpy = right_wrist_current.Position.Y;
                double rWcpz = right_wrist_current.Position.Z;

                // Lets create the AB and BC vectors:
                shoulder_to_elbow_AB_current = GestureAuxilaryMethods.updateLimbVectors(JointType.ShoulderRight, JointType.ElbowRight);
                elbow_to_wrist_BC_current = GestureAuxilaryMethods.updateLimbVectors(JointType.ElbowRight, JointType.WristRight);

                // Calculating the phi angle for AB:

                // Cartesian coordinates of the tip of the current vector:
                double realXcurr = rEcpx - rScpx;
                double realYcurr = rEcpy - rScpy;
                double realZcurr = rEcpz - rScpz;

                // Variables for the Rodrigues formula: v is the vector to be rotated, k is the axis
                Vector3D v, k;

                TextInformation.insert_main_text_block("PREV: (" + shoulder_to_elbow_AB_previous.X.ToString("n3") + ", " + shoulder_to_elbow_AB_previous.Y.ToString("n3") +
                     ", " + shoulder_to_elbow_AB_previous.Z.ToString("n3") + ")", 1);
                TextInformation.insert_main_text_block("CURR: (" + shoulder_to_elbow_AB_current.X.ToString("n3") + ", " + shoulder_to_elbow_AB_current.Y.ToString("n3") +
                     ", " + shoulder_to_elbow_AB_current.Z.ToString("n3") + ")", 1);

                // Calculating theta angle. Important note: We need to keep track of previous and current vectors, since in 
                // essence, the theta angles are given by an angle between current stance of the limb and previous stance.
                theta_AB = Vector3D.AngleBetween(shoulder_to_elbow_AB_current, shoulder_to_elbow_AB_previous);
                stt += theta_AB.ToString("n4") + " ";

                // This is the vector which we will use to compare the normal of our current vector plane:
                normalVectorOfPlaneCurr_AB = Vector3D.CrossProduct(shoulder_to_elbow_AB_current, shoulder_to_elbow_AB_previous);

                // We now have the angle, but its unsigned, that is, we dont know the direction of rotation:
                phi_AB = Vector3D.AngleBetween(normalVectorOfPlaneCurr_AB, normalVectorOfPlanePrev_AB);
               
                // Calculating the direction of phi rotation. We need a vector that is rotated about the theta axis
                // angle theta without the phi rotation. We then rotate this vector about phi in positive and 
                // negative phi degrees and check against the actual point to check for the direction. Remember
                // the right hand rule of the Rodrigues formula.
                // This variable is used to rotate the theta axis vector about itself by angle phi:
                Vector3D thetaAxisPrevUnitVector_AB = Vector3D.Divide(thetaAxix_AB_prev, thetaAxix_AB_prev.Length);

                v = phiAxix_AB_prev;
                k = thetaAxisPrevUnitVector_AB;

                // Rotating the phi axis by angle theta about theta axis without the phi rotation:
                Vector3D rotatedPhiAxisJustThetaRotation_AB = Vector3D.Multiply(v, Math.Cos(Math.PI * (theta_AB / 180)))
                    + Vector3D.Multiply((Vector3D.CrossProduct(k, v)), Math.Sin(Math.PI * (theta_AB / 180)))
                    + Vector3D.Multiply(k, (Vector3D.DotProduct(k, v) * (1 - Math.Cos(Math.PI * (theta_AB / 180)))));

                // Now we rotate this vector about previous phi axis by angle +/- phi and check against the actual point:
                // This variable is used to rotate the theta axis vector about itself by angle phi:
                Vector3D phiAxisPrevUnitVector_AB = Vector3D.Divide(phiAxix_AB_prev, phiAxix_AB_prev.Length);

                v = rotatedPhiAxisJustThetaRotation_AB;
                k = phiAxisPrevUnitVector_AB;

                Vector3D rotatedPhiAxisPositive_AB = Vector3D.Multiply(v, Math.Cos(Math.PI * (phi_AB / 180)))
                   + Vector3D.Multiply((Vector3D.CrossProduct(k, v)), Math.Sin(Math.PI * (phi_AB / 180)))
                   + Vector3D.Multiply(k, (Vector3D.DotProduct(k, v) * (1 - Math.Cos(Math.PI * (phi_AB / 180)))));

                Vector3D rotatedPhiAxisNegative_AB = Vector3D.Multiply(v, Math.Cos(-1 * Math.PI * (phi_AB / 180)))
                   + Vector3D.Multiply((Vector3D.CrossProduct(k, v)), Math.Sin(-1 * Math.PI * (phi_AB / 180)))
                   + Vector3D.Multiply(k, (Vector3D.DotProduct(k, v) * (1 - Math.Cos(-1 * Math.PI * (phi_AB / 180)))));
                
                if ((Math.Abs(rotatedPhiAxisNegative_AB.X - realXcurr) + Math.Abs(rotatedPhiAxisNegative_AB.Y - realYcurr)
                    + Math.Abs(rotatedPhiAxisNegative_AB.Z - realZcurr)) < .001)
                {
                    phi_AB = phi_AB * -1;
                }
                


                // Updating the rotation axes:
                phiAxix_AB_curr = shoulder_to_elbow_AB_current;
                
                v = thetaAxix_AB_prev;
                k = phiAxisPrevUnitVector_AB;

                // Rotating the theta axis first; we also need to rotate the theta axis of BC:
                thetaAxix_AB_curr = Vector3D.Multiply(v, Math.Cos(Math.PI * (phi_AB / 180)))
                    + Vector3D.Multiply((Vector3D.CrossProduct(k, v)), Math.Sin(Math.PI * (phi_AB / 180)))
                    + Vector3D.Multiply(k, (Vector3D.DotProduct(k, v) * (1 - Math.Cos(Math.PI * (phi_AB / 180)))));


                drawImitatedArmJanuary();
                
                cnt++;
                TextInformation.insert_main_text_block(stt, 2);
                TextInformation.insert_main_text_block("Handling " + cnt, 1);
                TextInformation.insert_main_text_block("thetaAB: " + theta_AB.ToString("n4"), 1);
                TextInformation.insert_main_text_block("phiAB: " + phi_AB.ToString("n4"), 1);
                TextInformation.insert_main_text_block("thetaBC: " + theta_BC.ToString("n4"), 1);
                TextInformation.insert_main_text_block("phiBC: " + phi_BC.ToString("n4"), 1);

                // We need to have a new movement in order to do new motor calculations:
                canAcceptNewMovement = false;

                // We need a way to send a signal:

                // Storing the current vectors into previous ones:
                shoulder_to_elbow_AB_previous = shoulder_to_elbow_AB_current;
                elbow_to_wrist_BC_previous = elbow_to_wrist_BC_current;

                normalVectorOfPlanePrev_AB = normalVectorOfPlaneCurr_AB;
                normalVectorOfPlanePrev_BC = normalVectorOfPlaneCurr_BC;

                phiAxix_AB_prev = phiAxix_AB_curr;
                phiAxix_BC_prev = phiAxix_BC_curr;
                thetaAxix_AB_prev = thetaAxix_AB_curr;
                thetaAxix_BC_prev = thetaAxix_BC_curr;

                // drawImitatedArm(shoulder_to_elbow_AB_current.Length, elbow_to_wrist_BC_current.Length);





            }

            // CAREFULL: the two boolean variables are ORed, not ANDed like before. The reason for this is that 
            // we would need at least one limb to move in order to have new rotations. This would set canAcceptNewMovement
            // true, but afterwards, we would still need both of the limbs to be still so that proper angle reading can
            // occur.
            if ((rightElbowState == JointGeneralState.Moving || rightWristState == JointGeneralState.Moving) && canAcceptNewMovement == false)
            {
                // Elbow is moving, so we need to wait until its stationary to output information.
                 canAcceptNewMovement = true;
            }

            // Stop the handling if the left hand is open:
            if (GesturesMasterControl.body.HandLeftState == HandState.Closed)
            {
                return 1;
            }
            else
            {
                return -1;
            }
        }

    }

    internal class Gesture_PanelRotation
    {
        internal const int BUFFER_LENGTH = 4;
        internal int buffer_filled_spots;

        internal double horizontal_stretch_sensitivity;

        internal const double angle_difference_sensitivity = .055;
        internal const double both_arms_down_sensitivity = .06;

        // Changed from .06...
        internal const double Z_distance_sensitivity = .10;

        internal const double trigg_2_short_limb_diff_sensitivity = .1;
        internal const double trigg_2_long_limb_diff_sensitivity = .15;
        internal const double trigg_2_Z_short_dist_sensitivity = .7;
        internal const double trigg_2_Z_long_dist_sensitivity = .15;
        internal const double trigg_2_horizontal_sensitivity = .05;

        internal double average_angle;

        internal Rotations left_arm_rotation;
        internal Rotations right_arm_rotation;

        ArmPosition left_arm_position;
        ArmPosition right_arm_position;

        internal Vector3D shoulder_to_elbow_left;
        internal Vector3D shoulder_to_elbow_right;

        internal double[] angle_buffer;

        public Gesture_PanelRotation(ArmPosition left_arm_position, ArmPosition right_arm_position,
                                    double horizontal_stretch_sensitivity)
        {
            angle_buffer = new double[BUFFER_LENGTH];
            buffer_filled_spots = 0;

            this.horizontal_stretch_sensitivity = horizontal_stretch_sensitivity;

            left_arm_rotation = new Rotations();
            right_arm_rotation = new Rotations();

            this.left_arm_position = left_arm_position;
            this.right_arm_position = right_arm_position;

            shoulder_to_elbow_left = new Vector3D();
            shoulder_to_elbow_right = new Vector3D();
            average_angle = 0.0;
        }

        internal static bool checkPanelRotationGesture(double horizontal_stretch_sensitivity,
                                    ref ArmPosition left_arm_position, ref ArmPosition right_arm_position)
        {
            // Calling the static method to update the arm position states:
            return panelRotationInitialCondition(horizontal_stretch_sensitivity,
                                       ref left_arm_position, ref right_arm_position);
        }

        internal void initializeGesture()
        {
            // Initiation successful, we can signal the beginning of rotation angle reading:
            left_arm_rotation.rot_tracking_state = ArmRotationTrackingState.Initialized;
            right_arm_rotation.rot_tracking_state = ArmRotationTrackingState.Initialized;

            // Initialing the rotation state:
            left_arm_rotation.previous_rot_state = ArmRotationalState.NotRotating;
            right_arm_rotation.previous_rot_state = ArmRotationalState.NotRotating;

            // Passing this shoulder to elbow vector so that on the next event we can compare:
            left_arm_rotation.previous_frame_vector = GestureAuxilaryMethods.updateLimbVectors(JointType.ShoulderLeft,
                                                                        JointType.ElbowLeft);
            right_arm_rotation.previous_frame_vector = GestureAuxilaryMethods.updateLimbVectors(JointType.ShoulderRight,
                                                                        JointType.ElbowRight);

        }

        // This method returns 0 if everything went well, -1 if reading of the gesture is stopped:
        internal int readArmRotation(JointGeneralState left_elbow_general_state, JointGeneralState right_elbow_general_state)
        {
            // Moved these two lines out of the if statement:
            shoulder_to_elbow_left = GestureAuxilaryMethods.updateLimbVectors(JointType.ShoulderLeft, JointType.ElbowLeft);
            shoulder_to_elbow_right = GestureAuxilaryMethods.updateLimbVectors(JointType.ShoulderRight, JointType.ElbowRight);

            // Checking the boundary conditions to see if we can actually do the reading:
            if (boundaryConditions(left_elbow_general_state, right_elbow_general_state) == true &&
               (left_arm_rotation.rot_tracking_state == ArmRotationTrackingState.Tracking ||
               left_arm_rotation.rot_tracking_state == ArmRotationTrackingState.Initialized) &&
               (right_arm_rotation.rot_tracking_state == ArmRotationTrackingState.Tracking ||
               right_arm_rotation.rot_tracking_state == ArmRotationTrackingState.Initialized))
            {
                left_arm_rotation.rot_tracking_state = ArmRotationTrackingState.Tracking;
                right_arm_rotation.rot_tracking_state = ArmRotationTrackingState.Tracking;
            }
            else
            {
                left_arm_rotation.rot_tracking_state = ArmRotationTrackingState.NotTracking;
                right_arm_rotation.rot_tracking_state = ArmRotationTrackingState.NotTracking;
            }

            if (left_arm_rotation.rot_tracking_state == ArmRotationTrackingState.Tracking &&
                right_arm_rotation.rot_tracking_state == ArmRotationTrackingState.Tracking)
            {
                updateRotationStatus();
                updateArmStatus();

                //TextInformation.insert_main_text_block("Left Arm: " + left_arm_rotation.previous_angles[0].ToString(), 2);
                //TextInformation.insert_main_text_block("Right Arm: " + right_arm_rotation.previous_angles[0].ToString(), 2);

                // Lets stabialize the angle:
                left_arm_rotation.stabializeAngle();
                right_arm_rotation.stabializeAngle();

                TextInformation.insert_main_text_block("Angle Left Arm: " + left_arm_rotation.angle.ToString("n4"), 1);
                TextInformation.insert_main_text_block("Angle Right Arm: " + right_arm_rotation.angle.ToString("n4"), 1);

                if (left_arm_rotation.enough_time_at_angle == true && right_arm_rotation.enough_time_at_angle == true)
                {
                    if (left_arm_rotation.readyForNewSignal == true && right_arm_rotation.readyForNewSignal == true)
                    {
                        TextInformation.insert_main_text_block("Signal Sent", 1);

                        if (DataSignalSender.isClientConnected)
                        {
                            DataSignalSender.SendSignal("L: " + left_arm_rotation.angle.ToString("n4") + " R: " + right_arm_rotation.angle.ToString("n4") + ".");

                        }

                        left_arm_rotation.readyForNewSignal = false;
                        right_arm_rotation.readyForNewSignal = false;
                    }

                    // Terminating the gesture reading. This might be changed later on:
                    //left_arm_rotation.rot_tracking_state = ArmRotationTrackingState.NotTracking;
                    //right_arm_rotation.rot_tracking_state = ArmRotationTrackingState.NotTracking;
                }
                else
                {
                    TextInformation.insert_main_text_block("NOT Sending Signal", 1);
                }

                left_arm_rotation.frame_counter++;
                right_arm_rotation.frame_counter++;

                // Updating previous vectors:
                left_arm_rotation.previous_frame_vector = shoulder_to_elbow_left;
                right_arm_rotation.previous_frame_vector = shoulder_to_elbow_right;

                return 0;
            }
            else
            {
                left_arm_rotation.previous_rot_state = ArmRotationalState.NotRotating;
                right_arm_rotation.previous_rot_state = ArmRotationalState.NotRotating;
                return -1;
            }
        }

        // This method calculates the current angle and inserts it into the array to be stabialized:
        internal void updateArmStatus()
        {
            double left_Y_difference_signed = GesturesMasterControl.body.Joints[JointType.WristLeft].Position.Y -
                               GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.Y;
            double right_Y_difference_signed = GesturesMasterControl.body.Joints[JointType.WristRight].Position.Y -
                                GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.Y;
            double left_X_difference_signed = GesturesMasterControl.body.Joints[JointType.WristLeft].Position.X -
                               GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.X;
            double right_X_difference_signed = GesturesMasterControl.body.Joints[JointType.WristRight].Position.X -
                                GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.X;

            //left_arm_rotation.angle = 180 + (180 * (Math.Atan(left_Y_difference_signed/left_X_difference_signed) / Math.PI));
            //right_arm_rotation.angle = 180 * (Math.Atan(right_Y_difference_signed / right_X_difference_signed) / Math.PI);

            left_arm_rotation.managePrevAngles(180 + (180 * (Math.Atan(left_Y_difference_signed / left_X_difference_signed) / Math.PI)));
            right_arm_rotation.managePrevAngles(180 * (Math.Atan(right_Y_difference_signed / right_X_difference_signed) / Math.PI));

            //average_angle = (Math.Abs(180 * (Math.Atan(left_Y_difference_signed / left_X_difference_signed) / Math.PI))
            //    + Math.Abs(180 * (Math.Atan(right_Y_difference_signed / right_X_difference_signed) / Math.PI))) / 2;

            left_Y_difference_signed = Math.Abs(left_Y_difference_signed);
            right_Y_difference_signed = Math.Abs(right_Y_difference_signed);

        }

        internal void updateRotationStatus()
        {
            // We check to see the difference between the Y positions of the elbow of previous and current vectors: 
            double left_Y_difference = left_arm_rotation.previous_frame_vector.Y - shoulder_to_elbow_left.Y;
            double right_Y_difference = right_arm_rotation.previous_frame_vector.Y - shoulder_to_elbow_right.Y;

            if (left_Y_difference <= 0 && right_Y_difference >= 0)
            {
                left_arm_rotation.previous_rot_state = ArmRotationalState.RotatingClockWise;
                right_arm_rotation.previous_rot_state = ArmRotationalState.RotatingClockWise;
            }
            else if (left_Y_difference >= 0 && right_Y_difference < 0)
            {
                left_arm_rotation.previous_rot_state = ArmRotationalState.RotatingCntClockWise;
                right_arm_rotation.previous_rot_state = ArmRotationalState.RotatingCntClockWise;
            }
            else
            {
                left_arm_rotation.previous_rot_state = ArmRotationalState.NotRotating;
                right_arm_rotation.previous_rot_state = ArmRotationalState.NotRotating;
            }

            TextInformation.insert_main_text_block("Rotation: " + right_arm_rotation.previous_rot_state.ToString(), 1);
        }

        internal bool boundaryConditions(JointGeneralState left_elbow_general_state, JointGeneralState right_elbow_general_state)
        {
            bool angle_shoulder_elbow_steady = false, both_arms_same_dir = false, speed_normal = false,
            arms_Z_pos_constant = false, both_elbows_same_state = false;
            double left_Y_difference = 0, right_Y_difference = 0;

            // PREVIOUSLY: went from shoulder to elbow:
            // We check to see if the angle between spine shoulder and wrist for both arms are close: 
            double left_Y_difference_signed = GesturesMasterControl.body.Joints[JointType.WristLeft].Position.Y -
                                GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.Y;
            double right_Y_difference_signed = GesturesMasterControl.body.Joints[JointType.WristLeft].Position.Y -
                                GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.Y;

            double left_Y_difference_signed_elbow = GesturesMasterControl.body.Joints[JointType.ElbowLeft].Position.Y -
                                GesturesMasterControl.body.Joints[JointType.ShoulderLeft].Position.Y;
            double right_Y_difference_signed_elbow = GesturesMasterControl.body.Joints[JointType.ElbowRight].Position.Y -
                                GesturesMasterControl.body.Joints[JointType.ShoulderRight].Position.Y;

            left_Y_difference = Math.Abs(left_Y_difference_signed);
            right_Y_difference = Math.Abs(right_Y_difference_signed);

            //TextInformation.insert_main_text_block("Left Y Difference: " + left_Y_difference.ToString("n4"), 1);
            //TextInformation.insert_main_text_block("Right Y Difference: " + right_Y_difference.ToString("n4"), 1);
            //TextInformation.insert_main_text_block("Difference Between: " + (left_Y_difference - right_Y_difference).ToString("n4"), 1);

            if (Math.Abs(left_Y_difference - right_Y_difference) > angle_difference_sensitivity)
            {
                angle_shoulder_elbow_steady = false;
            }
            else
            {
                angle_shoulder_elbow_steady = true;
            }

            // Reusing the same variables for the elbow shoulder difference measurement:
            left_Y_difference = Math.Abs(left_Y_difference_signed_elbow);
            right_Y_difference = Math.Abs(right_Y_difference_signed_elbow);

            // If the arms are not stretched out horizontally, we can check to see if both arms are down or up:
            if (left_Y_difference > both_arms_down_sensitivity && right_Y_difference > both_arms_down_sensitivity)
            {
                // Check to see if both variables have the same sign:
                if (left_Y_difference_signed_elbow > 0 && right_Y_difference_signed_elbow > 0)
                {
                    TextInformation.insert_main_text_block("IIIIIII", 4);
                    both_arms_same_dir = true;
                }
                else if (left_Y_difference_signed_elbow < 0 && right_Y_difference_signed_elbow < 0)
                {
                    TextInformation.insert_main_text_block("OOOOOOO", 4);
                    both_arms_same_dir = true;
                }
                else
                {
                    both_arms_same_dir = false;
                }
            }

            // We need to also check to see if the Z distances between shoulder and elbow are close enough:
            double left_Z_difference = GesturesMasterControl.body.Joints[JointType.ElbowLeft].Position.Z -
                                GesturesMasterControl.body.Joints[JointType.ShoulderLeft].Position.Z;
            double right_Z_difference = GesturesMasterControl.body.Joints[JointType.ElbowRight].Position.Z -
                                GesturesMasterControl.body.Joints[JointType.ShoulderRight].Position.Z;

            //TextInformation.insert_main_text_block("Left Z Difference: " + left_Z_difference.ToString("n4"), 2);
            //TextInformation.insert_main_text_block("Right Z Difference: " + right_Z_difference.ToString("n4"), 2);

            left_Z_difference = Math.Abs(left_Z_difference);
            right_Z_difference = Math.Abs(right_Z_difference);

            if (left_Z_difference > Z_distance_sensitivity || right_Z_difference > Z_distance_sensitivity)
            {
                arms_Z_pos_constant = false;
            }
            else
            {
                arms_Z_pos_constant = true;
            }

            // Lets check if one elbow is moving and other is still: 
            // Very hard to make useful... not implemented
            if (left_elbow_general_state == JointGeneralState.Moving && right_elbow_general_state == JointGeneralState.Still)
            {
                both_elbows_same_state = false;
            }
            else if (left_elbow_general_state == JointGeneralState.Still && right_elbow_general_state == JointGeneralState.Moving)
            {
                both_elbows_same_state = false;
            }
            else if (left_elbow_general_state == JointGeneralState.Still && right_elbow_general_state == JointGeneralState.Still)
            {
                both_elbows_same_state = true;
            }
            else if (left_elbow_general_state == JointGeneralState.Moving && right_elbow_general_state == JointGeneralState.Moving)
            {
                both_elbows_same_state = true;
            }

            // Lets check the speed of rotation from one frame to another:
            // Not used due to poor functionality:
            double Y_single_frame_diff_left = shoulder_to_elbow_left.Y - left_arm_rotation.previous_frame_vector.Y;
            double Y_single_frame_diff_right = shoulder_to_elbow_right.Y - right_arm_rotation.previous_frame_vector.Y;

            TextInformation.insert_main_text_block("Left Speed: " + Y_single_frame_diff_left.ToString("n4"), 1);
            TextInformation.insert_main_text_block("Right Speed: " + Y_single_frame_diff_right.ToString("n4"), 1);

            if (Math.Abs(Y_single_frame_diff_left) < .015 && Math.Abs(Y_single_frame_diff_right) < .015)
            {
                speed_normal = true;
            }

            TextInformation.insert_main_text_block("Arms Z Position Constant: " + arms_Z_pos_constant.ToString(), 2);
            TextInformation.insert_main_text_block("Both Arms Same Direction: " + both_arms_same_dir.ToString(), 2);
            TextInformation.insert_main_text_block("Angle Shoulder Elbow Steady: " + angle_shoulder_elbow_steady.ToString(), 2);
            // Combining all booleans:
            if (angle_shoulder_elbow_steady == true && both_arms_same_dir == false && arms_Z_pos_constant == true)
            {
                TextInformation.insert_main_text_block("Boundary Clear", 2);
                return true;
            }
            else
            {
                TextInformation.insert_main_text_block("Boundary NOT Clear", 2);
                return false;
            }
        }

        internal static bool panelRotationInitialCondition(double horizontal_stretch_sensitivity,
                                        ref ArmPosition left_arm_position, ref ArmPosition right_arm_position)
        {
            bool ret_bool;

            /*ret_bool = Gesture_PanelRotation.triggerPosition_1(horizontal_stretch_sensitivity, ref left_arm_position,
                                                                   ref right_arm_position);*/

            ret_bool = triggerPosition_2(.3, .3, ref left_arm_position,
                                                                    ref right_arm_position);
            return ret_bool;
        }

        /// <summary>
        /// This method is designed to trigger a gesture at any angle, not just horizontally stretched.
        /// </summary>
        /// <param name="vert_arm_diff_sensitivity"></param>
        /// <param name="position_sensitivity"></param>
        /// <param name="left_arm_position"></param>
        /// <param name="right_arm_position"></param>
        /// <returns></returns>
        private static bool triggerPosition_2(double vert_arm_diff_sensitivity, double position_sensitivity,
                                       ref ArmPosition left_arm_position, ref ArmPosition right_arm_position)
        {
            bool short_arm_limb_steady = false, long_arm_limb_steady = false, right_Z_dist_steady = false,
                left_Z_dist_steady = false;

            // We get the Y differences between all arm joints:
            double left_Y_shoul_elbow_diff = GesturesMasterControl.body.Joints[JointType.ShoulderLeft].Position.Y -
                                GesturesMasterControl.body.Joints[JointType.ElbowLeft].Position.Y;
            double right_Y_shoul_elbow_diff = GesturesMasterControl.body.Joints[JointType.ShoulderRight].Position.Y -
                                 GesturesMasterControl.body.Joints[JointType.ElbowRight].Position.Y;

            double left_Y_elbow_wrist_diff = GesturesMasterControl.body.Joints[JointType.ElbowLeft].Position.Y -
                                GesturesMasterControl.body.Joints[JointType.WristLeft].Position.Y;
            double right_Y_elbow_wrist_diff = GesturesMasterControl.body.Joints[JointType.ElbowRight].Position.Y -
                                 GesturesMasterControl.body.Joints[JointType.WristRight].Position.Y;

            double left_Y_shoul_wrist_diff = GesturesMasterControl.body.Joints[JointType.ShoulderLeft].Position.Y -
                                GesturesMasterControl.body.Joints[JointType.WristLeft].Position.Y;
            double right_Y_shoul_wrist_diff = GesturesMasterControl.body.Joints[JointType.ShoulderRight].Position.Y -
                                 GesturesMasterControl.body.Joints[JointType.WristRight].Position.Y;

            double shoulder_spine_Y = GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.Y;
            double left_elbow_Y = GesturesMasterControl.body.Joints[JointType.ElbowLeft].Position.Y;
            double right_elbow_Y = GesturesMasterControl.body.Joints[JointType.ElbowRight].Position.Y;

            // First lets check to see if the elbows are not both above or below the spine shoulder:
            if ((left_elbow_Y - shoulder_spine_Y > 0 && right_elbow_Y - shoulder_spine_Y > 0) ||
                (left_elbow_Y - shoulder_spine_Y < 0 && right_elbow_Y - shoulder_spine_Y < 0))
            {
                // Check if both elbows are not horizontal:
                if (Math.Abs(left_elbow_Y - shoulder_spine_Y) > trigg_2_horizontal_sensitivity &&
                    Math.Abs(right_elbow_Y - shoulder_spine_Y) > trigg_2_horizontal_sensitivity)
                {
                    return false;
                }
            }

            left_Y_shoul_elbow_diff = Math.Abs(left_Y_shoul_elbow_diff);
            right_Y_shoul_elbow_diff = Math.Abs(right_Y_shoul_elbow_diff);
            left_Y_elbow_wrist_diff = Math.Abs(left_Y_elbow_wrist_diff);
            right_Y_elbow_wrist_diff = Math.Abs(right_Y_elbow_wrist_diff);
            left_Y_shoul_wrist_diff = Math.Abs(left_Y_shoul_wrist_diff);
            right_Y_shoul_wrist_diff = Math.Abs(right_Y_shoul_wrist_diff);

            //TextInformation.insert_main_text_block("Y Shoulder Elbow Diff: " + Math.Abs(left_Y_shoul_elbow_diff - right_Y_shoul_elbow_diff).ToString("n5"), 4);
            //TextInformation.insert_main_text_block("Y Elbow Wrist Diff: " + Math.Abs(left_Y_shoul_elbow_diff - right_Y_shoul_elbow_diff).ToString("n5"), 4);

            // We need to see if the differences between the first two pairs (short limbs) are within a limit:
            if ((Math.Abs(left_Y_shoul_elbow_diff - right_Y_shoul_elbow_diff) < trigg_2_short_limb_diff_sensitivity) &&
                (Math.Abs(left_Y_elbow_wrist_diff - right_Y_elbow_wrist_diff) < trigg_2_short_limb_diff_sensitivity))
            {
                short_arm_limb_steady = true;
            }

            //TextInformation.insert_main_text_block("Y Long Diff: " + Math.Abs(left_Y_shoul_wrist_diff - right_Y_shoul_wrist_diff).ToString("n5"), 4);

            // We need to see if the differences between the last pair (short limb) is within a limit:
            if ((Math.Abs(left_Y_shoul_wrist_diff - right_Y_shoul_wrist_diff) < trigg_2_long_limb_diff_sensitivity))
            {
                long_arm_limb_steady = true;
            }

            // We check the Z distances from the camera:
            double left_shoulder_Z = GesturesMasterControl.body.Joints[JointType.ShoulderLeft].Position.Z;
            double right_shoulder_Z = GesturesMasterControl.body.Joints[JointType.ShoulderRight].Position.Z;
            double left_elbow_Z = GesturesMasterControl.body.Joints[JointType.ElbowLeft].Position.Z;
            double right_elbow_Z = GesturesMasterControl.body.Joints[JointType.ElbowRight].Position.Z;
            double left_wrist_Z = GesturesMasterControl.body.Joints[JointType.WristLeft].Position.Z;
            double right_wrist_Z = GesturesMasterControl.body.Joints[JointType.WristRight].Position.Z;

            /*TextInformation.insert_main_text_block("Left Shoulder Z: " + left_shoulder_Z.ToString("n5"), 4);
            TextInformation.insert_main_text_block("Right Shoulder Z: " + right_shoulder_Z.ToString("n5"), 4);
            TextInformation.insert_main_text_block("Left Elbow Z: " + left_elbow_Z.ToString("n5"), 4);
            TextInformation.insert_main_text_block("Right Shoulder Z: " + right_elbow_Z.ToString("n5"), 4);
            TextInformation.insert_main_text_block("Left Wrist Z: " + left_wrist_Z.ToString("n5"), 4);
            TextInformation.insert_main_text_block("Right Wrist Z: " + right_wrist_Z.ToString("n5"), 4);
            */
            if ((Math.Abs(right_shoulder_Z - right_elbow_Z) < trigg_2_Z_short_dist_sensitivity) &&
                (Math.Abs(right_elbow_Z - right_wrist_Z) < trigg_2_Z_short_dist_sensitivity) &&
                (Math.Abs(right_shoulder_Z - right_wrist_Z) < trigg_2_Z_long_dist_sensitivity))
            {
                right_Z_dist_steady = true;
            }

            if ((Math.Abs(left_shoulder_Z - left_elbow_Z) < trigg_2_Z_short_dist_sensitivity) &&
                (Math.Abs(left_elbow_Z - left_wrist_Z) < trigg_2_Z_short_dist_sensitivity) &&
                (Math.Abs(left_shoulder_Z - left_wrist_Z) < trigg_2_Z_long_dist_sensitivity))
            {
                left_Z_dist_steady = true;
            }

            if (short_arm_limb_steady == true && long_arm_limb_steady == true &&
                right_Z_dist_steady == true && left_Z_dist_steady == true)
            {
                return true;
            }
            else
            {
                return false;
            }

        }

        // This trigger position is when the arms are stretched out horizontally: 
        private static bool triggerPosition_1(double horizontal_stretch_sensitivity,
                                        ref ArmPosition left_arm_position, ref ArmPosition right_arm_position)
        {
            bool ret_bool = false;

            // We first check to see if ShoulderCenter is above the two other shoulders:
            if (GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.Y >
                GesturesMasterControl.body.Joints[JointType.ShoulderLeft].Position.Y &&
               GesturesMasterControl.body.Joints[JointType.SpineShoulder].Position.Y >
               GesturesMasterControl.body.Joints[JointType.ShoulderRight].Position.Y)
            {
                double shoulder_left_Y = GesturesMasterControl.body.Joints[JointType.ShoulderLeft].Position.Y;
                double shoulder_right_Y = GesturesMasterControl.body.Joints[JointType.ShoulderRight].Position.Y;
                double elbow_left_Y = GesturesMasterControl.body.Joints[JointType.ElbowLeft].Position.Y;
                double elbow_right_Y = GesturesMasterControl.body.Joints[JointType.ElbowRight].Position.Y;

                // Then we check to see if the height between shoulder and elbow is not too much:
                if (Math.Abs(shoulder_left_Y - elbow_left_Y) < horizontal_stretch_sensitivity &&
                   Math.Abs(shoulder_right_Y - elbow_right_Y) < horizontal_stretch_sensitivity)
                {
                    left_arm_position = ArmPosition.ArmStretchedOutHorizontally;
                    right_arm_position = ArmPosition.ArmStretchedOutHorizontally;
                    ret_bool = true;
                }
                else
                {
                    left_arm_position = ArmPosition.Unknown;
                    right_arm_position = ArmPosition.Unknown;
                }
            }
            return ret_bool;
        }
    }

    public class GesturesMasterControl
    {
        // Static member:
        public static Body body;

        public Stopwatch stopWatch;
        public Stopwatch stopWatch_single_event;
        public Stopwatch stopWatch_trigger;

        public GestureRunningState running_gesture = new GestureRunningState();

        Gesture_PanelRotation panel_rotation;
        Gesture_RoboticArmHandling robotic_arm;

        public static DrawingGroup drawingGroupXZ;
        public static DrawingGroup drawingGroupYX;
        public static DrawingGroup drawingGroupYZ;

        private DrawingImage imageSourceXZ;
        private DrawingImage imageSourceYX;
        private DrawingImage imageSourceYZ;

        JointGeneralState right_elbow_general_state = new JointGeneralState();
        JointGeneralState left_elbow_general_state = new JointGeneralState();
        JointGeneralState right_wrist_general_state = new JointGeneralState();
        JointGeneralState left_wrist_general_state = new JointGeneralState();

        ArmPosition left_arm_position = new ArmPosition();
        ArmPosition right_arm_position = new ArmPosition();

        public Joint[] lf_elbow_prev_pos;
        public Joint[] rt_elbow_prev_pos;
        // Wrist added in for Robotic Arm Handling:
        public Joint[] lf_wrist_prev_pos;
        public Joint[] rt_wrist_prev_pos;

        /* Previous values: 15 and .2*/
        //public const int PREV_FRAMES_ARRAY_LENGTH = 15;
        //public const double stillness_sensitivity = .2;

        /* New methodology: instead of cheching the standard deviation, we will now compare the position of 
           the first array element with the last array element: */
        public const int PREV_FRAMES_ARRAY_LENGTH = 5;
        public const double stillness_sensitivity = .005;
        public const double horizontal_stretch_sensitivity = .08;

        /* 1. Select a person most likely for gesture
            2. Wait until triggering conditions are met
            3. Wait for 0.5 seconds when triggered before starting the reading
            4. Read while checking the boundaries
            5. When a rotation is made, and the hands are still again, send a bluetooth signal
                a. at each correct angle reading, measure the time spent at that angle
                b. if the time exceeds some threshhold (1 second), send a signal
                c. if the person wants to continue rotating, he needs to wait a particular amount of time (based on motor rotation speed)
                d. reading to commence again for a new rotation
            6. Wait until another rotation is made */

        /* Constructor: */
        public GesturesMasterControl()
        {
            stopWatch = new Stopwatch();
            stopWatch_single_event = new Stopwatch();
            stopWatch_trigger = new Stopwatch();

            running_gesture = GestureRunningState.Unknown;

            right_elbow_general_state = JointGeneralState.Unknown;
            left_elbow_general_state = JointGeneralState.Unknown;
            right_arm_position = ArmPosition.Unknown;
            left_arm_position = ArmPosition.Unknown;

            lf_elbow_prev_pos = new Joint[PREV_FRAMES_ARRAY_LENGTH];
            rt_elbow_prev_pos = new Joint[PREV_FRAMES_ARRAY_LENGTH];

            lf_wrist_prev_pos = new Joint[PREV_FRAMES_ARRAY_LENGTH];
            rt_wrist_prev_pos = new Joint[PREV_FRAMES_ARRAY_LENGTH];
        }

        /* Methods: */
        public void setBody(Body _body, ref DrawingGroup XZ, ref DrawingGroup YX, ref DrawingGroup YZ)
        {
            body = _body;
            drawingGroupXZ = XZ;
            drawingGroupYX = YX;
            drawingGroupYZ = YZ;
        }

        public void runGestureAnalysis()
        {
            stopWatch_single_event.Reset();
            stopWatch_single_event.Start();
            TextInformation.insert_main_text_block("Stopwatch: " + stopWatch.Elapsed, 3);
            TextInformation.insert_main_text_block("Running Gesture: " + running_gesture.ToString(), 1);

            // I have moved the next 4 blocks of codes out of the if statement...
            /* Searching State: CHANGED FROM WRIST TO ELBOW */
            Joint left_elbow = body.Joints[JointType.ElbowLeft];
            Joint right_elbow = body.Joints[JointType.ElbowRight];
            Joint left_wrist = body.Joints[JointType.WristLeft];
            Joint right_wrist = body.Joints[JointType.WristRight];


            // We make a call to this method to store previous positions:
            GestureAuxilaryMethods.managePrevArray(left_elbow, ref lf_elbow_prev_pos, PREV_FRAMES_ARRAY_LENGTH);
            GestureAuxilaryMethods.managePrevArray(right_elbow, ref rt_elbow_prev_pos, PREV_FRAMES_ARRAY_LENGTH);
            GestureAuxilaryMethods.managePrevArray(left_wrist, ref lf_wrist_prev_pos, PREV_FRAMES_ARRAY_LENGTH);
            GestureAuxilaryMethods.managePrevArray(right_wrist, ref rt_wrist_prev_pos, PREV_FRAMES_ARRAY_LENGTH);
            
            // We make a call to the following methods in order to update the state of elbows:
            GestureAuxilaryMethods.isJointStable(stillness_sensitivity, JointType.ElbowLeft, lf_elbow_prev_pos, PREV_FRAMES_ARRAY_LENGTH,
                                                    ref left_elbow_general_state);
            GestureAuxilaryMethods.isJointStable(stillness_sensitivity, JointType.ElbowRight, rt_elbow_prev_pos, PREV_FRAMES_ARRAY_LENGTH,
                                                    ref right_elbow_general_state);
            // We make a call to the following method in order to update the state of RIGHT WRIST ONLY, we dont need left for now:
            GestureAuxilaryMethods.isJointStable(stillness_sensitivity, JointType.WristRight, rt_wrist_prev_pos, PREV_FRAMES_ARRAY_LENGTH,
                                                    ref right_wrist_general_state);


            // The if else block below is to see if we are in the searching (for gesture) state or reading (the gesture) state:
            if (running_gesture == GestureRunningState.None || running_gesture == GestureRunningState.Unknown)
            {
                // Code written up for checking PanelRotation:
                if (left_elbow_general_state == JointGeneralState.Still && right_elbow_general_state == JointGeneralState.Still
                    && body.HandLeftState == HandState.Open)
                {
                    // Calling this method to see if a panel rotation gesture is possible:
                    if (Gesture_PanelRotation.checkPanelRotationGesture(horizontal_stretch_sensitivity, ref left_arm_position, ref right_arm_position))
                    {
                        // We can now initialize a PanelRotation class and go on further:
                        panel_rotation = new Gesture_PanelRotation(left_arm_position, right_arm_position, horizontal_stretch_sensitivity);
                        panel_rotation.initializeGesture();

                        // Update the gesture state:
                        running_gesture = GestureRunningState.PanelRotation;

                        // Start the stopwatch, we will wait .5 seconds for the gesture reading to begin:
                        stopWatch_trigger.Start();
                    }
                }

                // Code written up for checking RoboticArmHandling:
                if (left_elbow_general_state == JointGeneralState.Still && right_elbow_general_state == JointGeneralState.Still
                    && body.HandLeftState == HandState.Closed)
                {
                    robotic_arm = new Gesture_RoboticArmHandling();
                    robotic_arm.initializeGestureJanuary();

                    // Update the gesture state:
                    running_gesture = GestureRunningState.RoboticArmHandling;

                    // Start the stopwatch, we will wait .5 seconds for the gesture reading to begin:
                    stopWatch_trigger.Start();
                }
            }
            else      /* Reading State: */
            {
                if (stopWatch_trigger.ElapsedMilliseconds > 500)
                {
                    int reading_result = -1;

                    // There is a gesture that is to be read, we need to check which:
                    if (running_gesture == GestureRunningState.PanelRotation)
                    {
                        reading_result = panel_rotation.readArmRotation(left_elbow_general_state, right_elbow_general_state);
                    }

                    if (running_gesture == GestureRunningState.RoboticArmHandling)
                    {
                        //reading_result = robotic_arm.beginArmHandlingTestBuild(right_elbow_general_state, right_wrist_general_state);
                        reading_result = robotic_arm.beginArmHandlingPathIndependentJanuary(right_elbow_general_state, right_wrist_general_state);
                    }

                    if (reading_result == -1)
                    {
                        // Discontinue the reading of the gesture:
                        running_gesture = GestureRunningState.None;

                        // Clear out the previous hand positions arrays:
                        lf_elbow_prev_pos = new Joint[PREV_FRAMES_ARRAY_LENGTH];
                        rt_elbow_prev_pos = new Joint[PREV_FRAMES_ARRAY_LENGTH];
                        rt_wrist_prev_pos = new Joint[PREV_FRAMES_ARRAY_LENGTH];

                        // Clear out the statuses of Limbs: 
                        left_elbow_general_state = JointGeneralState.Unknown;
                        right_elbow_general_state = JointGeneralState.Unknown;
                        right_wrist_general_state = JointGeneralState.Unknown;

                        // Reset the stopwatch:
                        stopWatch_trigger.Reset();
                    }
                }
            }

            TextInformation.insert_main_text_block("Event: " + stopWatch_single_event.Elapsed, 3);
            TextInformation.update_main_text();
        }

    }

    public static class GestureAuxilaryMethods
    {
        public static Vector3D updateLimbVectors(JointType joint_start_type, JointType joint_end_type)
        {
            Vector3D ret_vector = new Vector3D();
            Joint joint_start = GesturesMasterControl.body.Joints[joint_start_type];
            Joint joint_end = GesturesMasterControl.body.Joints[joint_end_type];

            ret_vector.X = joint_end.Position.X - joint_start.Position.X;
            ret_vector.Y = joint_end.Position.Y - joint_start.Position.Y;
            ret_vector.Z = joint_end.Position.Z - joint_start.Position.Z;

            return ret_vector;
        }

        public static bool isJointStable(double sensitivity, JointType joint, Joint[] prev_array, int prev_length, ref JointGeneralState elbow_state)
        {
            bool return_bool = false;
            //double avg_standard_dev = calcAverageStandardDev(prev_array, prev_length);

            //if (avg_standard_dev < sensitivity)
            if (calcFirstLastDiff(prev_array, prev_length, sensitivity))
            {
                return_bool = true;
                elbow_state = JointGeneralState.Still;
                TextInformation.insert_main_text_block(joint + " Still", 2);
            }
            else
            {
                return_bool = false;
                elbow_state = JointGeneralState.Moving;
                TextInformation.insert_main_text_block(joint + " Moving", 2);
            }

            return return_bool;
        }

        private static bool calcFirstLastDiff(Joint[] joint_array, int prev_length, double sensitivity)
        {
            double[] x_positions = new double[2];
            double[] y_positions = new double[2];
            double[] z_positions = new double[2];

            // Getting the position for first and last element:
            x_positions[0] = joint_array[0].Position.X;
            y_positions[0] = joint_array[0].Position.Y;
            z_positions[0] = joint_array[0].Position.Z;
            x_positions[1] = joint_array[prev_length - 1].Position.X;
            y_positions[1] = joint_array[prev_length - 1].Position.Y;
            z_positions[1] = joint_array[prev_length - 1].Position.Z;

            if (Math.Abs(x_positions[0] - x_positions[1]) < sensitivity &&
               Math.Abs(y_positions[0] - y_positions[1]) < sensitivity &&
               Math.Abs(z_positions[0] - z_positions[1]) < sensitivity)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        private static double calcAverageStandardDev(Joint[] joint_array, int prev_length)
        {
            float[] x_positions = new float[prev_length];
            float[] y_positions = new float[prev_length];
            float[] z_positions = new float[prev_length];
            double x_average = 0;
            double y_average = 0;
            double z_average = 0;
            double x_dev_numerator = 0;
            double y_dev_numerator = 0;
            double z_dev_numerator = 0;
            double x_intermed = 0;
            double y_intermed = 0;
            double z_intermed = 0;
            double avg_standard_dev = 0;
            int i = 0;

            // Getting the position from joints and filling up respective arrays:
            for (i = 0; i < prev_length; i++)
            {
                x_positions[i] = joint_array[i].Position.X;
                y_positions[i] = joint_array[i].Position.Y;
                z_positions[i] = joint_array[i].Position.Z;
            }

            // Calculating averages:
            for (i = 0; i < prev_length; i++)
            {
                x_average += x_positions[i];
                y_average += y_positions[i];
                z_average += z_positions[i];
            }

            x_average /= prev_length;
            y_average /= prev_length;
            z_average /= prev_length;

            // Calculating the numerators:
            for (i = 0; i < prev_length; i++)
            {
                x_intermed = x_positions[i] - x_average;
                x_dev_numerator += x_intermed * x_intermed;
                y_intermed = y_positions[i] - y_average;
                y_dev_numerator += y_intermed * y_intermed;
                z_intermed = z_positions[i] - z_average;
                z_dev_numerator += z_intermed * z_intermed;
            }

            x_dev_numerator /= x_average;
            y_dev_numerator /= y_average;
            z_dev_numerator /= z_average;

            x_dev_numerator = Math.Sqrt(Math.Abs(x_dev_numerator));
            y_dev_numerator = Math.Sqrt(Math.Abs(y_dev_numerator));
            z_dev_numerator = Math.Sqrt(Math.Abs(z_dev_numerator));

            avg_standard_dev = (x_dev_numerator + y_dev_numerator + z_dev_numerator) / 3;
            return avg_standard_dev;
        }

        public static void managePrevArray(Joint joint, ref Joint[] array, int prev_length)
        {
            int i = prev_length - 1;
            while (i > 0)
            {
                array[i] = array[i - 1];
                i--;
            }
            array[i] = joint;
        }
    }

    public static class TextInformation
    {
        public static TextBlock main_text_block_1 = new TextBlock();
        public static TextBlock main_text_block_2 = new TextBlock();
        public static TextBlock main_text_block_3 = new TextBlock();
        public static TextBlock main_text_block_4 = new TextBlock();

        public static String[,] main_text = new String[4, 18];

        private static int[] lines = { 0, 0, 0, 0 };
        private static bool[] has_block_changed = { false, false, false, false };

        public static void insert_main_text_block(String info, int block)
        {
            main_text[block - 1, lines[block - 1]++] = info;
            has_block_changed[block - 1] = true;
        }

        public static void reset_text()
        {
            main_text_block_1.Text = " ";
            main_text_block_2.Text = " ";
            main_text_block_3.Text = " ";
            main_text_block_4.Text = " ";
        }

        public static void update_main_text()
        {
            int i = 0, j = 0;

            for (i = 0; i < 4; i++)
            {
                if (has_block_changed[i] == true)
                {

                    String y = "";
                    // 18 lines can fit:
                    for (j = 0; j < 18; j++)
                    {
                        y += main_text[i, j] + "\n";
                    }

                    if (i == 0)
                    {
                        main_text_block_1.Text = y;
                    }
                    else if (i == 1)
                    {
                        main_text_block_2.Text = y;
                    }
                    else if (i == 2)
                    {
                        main_text_block_3.Text = y;
                    }
                    else
                    {
                        main_text_block_4.Text = y;
                    }

                }

                has_block_changed[i] = false;
                lines[i] = 0;
            }

        }
    }
}
