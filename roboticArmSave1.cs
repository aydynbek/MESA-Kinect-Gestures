
    internal class Gesture_RoboticArmHandling
    {
        private Vector3D shoulder_to_elbow_AB_current;
        private Vector3D elbow_to_wrist_BC_current;
        private Vector3D shoulder_to_elbow_AB_previous;
        private Vector3D elbow_to_wrist_BC_previous;

        private Joint right_hand_previous;
        private Joint right_wrist_current;
        private Joint right_elbow_previous;
        private Joint right_elbow_current;
        private Joint right_shoulder_previous;
        private Joint right_shoulder_current;
        
        private CameraSpacePoint endpoint_C_previous;
        private CameraSpacePoint endpoint_C_current;

        private CameraSpacePoint endpoint_B_previous;
        private CameraSpacePoint endpoint_B_current;

        // These may be good for storing the previous angles:
        // Theta1 is the rotation about the joint axis:
        private double theta1_AB = 0;
        private double theta1_BC = 0;
        // Theta2 is the rotation about the limb axis:
        private double theta2_AB = 90;
        private double theta2_BC = 90;

        // Length of orientation vector is 10cm

        private const double theta_threshold = 5;

        private int state_switch = 0;
        private bool canAcceptNewMovement = true;

        public Gesture_RoboticArmHandling()
        {
            endpoint_C_previous = new CameraSpacePoint();
            endpoint_C_current = new CameraSpacePoint();
            endpoint_B_previous = new CameraSpacePoint();
            endpoint_B_current = new CameraSpacePoint();

            shoulder_to_elbow_AB_current = new Vector3D();
            elbow_to_wrist_BC_current = new Vector3D();
            shoulder_to_elbow_AB_previous = new Vector3D();
            elbow_to_wrist_BC_previous = new Vector3D();
        }

        /// <summary>
        /// This method is designed to initiate the first rotation of the limbs into their
        /// positions which they were at durring triggering. The algorithm assumes that 
        /// the arm was initially streching out normal to the torso and that it then has to
        /// rotate the limbs into the initial position.
        /// </summary>
        internal void initializeGesture()
        {
            bool AB_moved = false;

            right_wrist_current = GesturesMasterControl.body.Joints[JointType.WristRight];
            right_elbow_current = GesturesMasterControl.body.Joints[JointType.ElbowRight];
            right_shoulder_current = GesturesMasterControl.body.Joints[JointType.ShoulderRight];

            // Lets create the AB and BC vectors:
            shoulder_to_elbow_AB_current = GestureAuxilaryMethods.updateLimbVectors(JointType.ShoulderRight, JointType.ElbowRight);
            shoulder_to_elbow_AB_previous.X = 0;
            shoulder_to_elbow_AB_previous.Y = 0;
            shoulder_to_elbow_AB_previous.Z = -1 * shoulder_to_elbow_AB_current.Length;      // Need to check this...

            elbow_to_wrist_BC_current = GestureAuxilaryMethods.updateLimbVectors(JointType.ElbowRight, JointType.WristRight);
            elbow_to_wrist_BC_previous.X = 0;
            elbow_to_wrist_BC_previous.Y = 0;
            elbow_to_wrist_BC_previous.Z = elbow_to_wrist_BC_current.Length;      // Need to check this...

            // Lets get to the AB angles:
            theta1_AB = Vector3D.AngleBetween(shoulder_to_elbow_AB_current, shoulder_to_elbow_AB_previous);

            // Lets calculate the theta2 by assuming that the current vector projects on the initial straight out arm:
            double a = Math.Sqrt((right_elbow_current.Position.X - right_shoulder_current.Position.X) * (right_elbow_current.Position.X - right_shoulder_current.Position.X) +
                      (right_elbow_current.Position.Y - right_shoulder_current.Position.Y) * (right_elbow_current.Position.Y - right_shoulder_current.Position.Y));
            double b = 10;
            double c = Math.Sqrt((right_elbow_current.Position.X - right_shoulder_current.Position.X + 10) * (right_elbow_current.Position.X - right_shoulder_current.Position.X + 10) +
                       (right_elbow_current.Position.Y - right_shoulder_current.Position.Y) * (right_elbow_current.Position.Y - right_shoulder_current.Position.Y));
            theta2_AB = (c * c - a * a - b * b) / (-2 * a * b);
            theta2_AB = Math.Acos(theta2_AB);

            // Another methodology to calculate theta1:
            shoulder_to_elbow_AB_current = GestureAuxilaryMethods.updateLimbVectors(JointType.ShoulderRight, JointType.ElbowRight);
            shoulder_to_elbow_AB_previous.X = shoulder_to_elbow_AB_current.Length;
            shoulder_to_elbow_AB_previous.Y = 0;
            shoulder_to_elbow_AB_previous.Z = 0;

            // Lets get to the AB angles:
            theta1_AB = Vector3D.AngleBetween(shoulder_to_elbow_AB_current, shoulder_to_elbow_AB_previous);

            // Another methodology to calculate theta2:
            double cur_y = shoulder_to_elbow_AB_current.Y;

            double cur_z = shoulder_to_elbow_AB_current.Z;
            Vector3D cur_2D = new Vector3D(0, right_elbow_current.Position.Y, right_shoulder_current.Position.Z - right_elbow_current.Position.Z);
            Vector3D prev_2D = new Vector3D(0, 10, 0);

            // Update: it seems like the code works most of the time for theta2, but we need to fix the orientation,
            // as it gives readings at 180 degrees.
            // Solution: Subtract that angle from 180 and take the absolute value:
            theta2_AB = Math.Abs(180 - Vector3D.AngleBetween(prev_2D, cur_2D));
           
            //TextInformation.insert_main_text_block("a: " + a.ToString(), 1);
            //TextInformation.insert_main_text_block("b: " + b.ToString(), 1);
            //TextInformation.insert_main_text_block("c: " + c.ToString(), 1);
            
            TextInformation.insert_main_text_block("Theta1AB: " + theta1_AB.ToString(), 1);
            TextInformation.insert_main_text_block("Theta2AB: " + theta2_AB.ToString(), 1);

            Vector3D pseudo_unit_vector = new Vector3D();

            // Now we have both thetas for AB, lets see if they surpass the threshold:
            if (theta1_AB > theta_threshold || theta2_AB > theta_threshold)
            {
                // Set this to true for any rotation of AB. This will let us know later on
                // if we need to rotate the previous BC vector by the same angles:
                AB_moved = true;

                // We need to first do the theta2 rotation, which simply is rotating the vector
                // about its own axix, no real vector rotation is needed yet:
                theta2_BC = theta2_AB;

                Vector3D xy_projection = new Vector3D();
                xy_projection.X = 0;
                xy_projection.Y = 0;
                xy_projection.Z = 0;

                Vector3D xz_projection = new Vector3D();
                xz_projection.X = 0;
                xz_projection.Y = shoulder_to_elbow_AB_previous.Z;    // Z is Y in this case
                xz_projection.Z = 0;

                double x1 = (Math.Cos(theta1_BC) * xz_projection.X) - (Math.Sin(theta1_BC) * xz_projection.Y);
                double z1 = (Math.Sin(theta1_BC) * xz_projection.X) + (Math.Cos(theta1_BC) * xz_projection.Y);

                double x2 = (Math.Cos(theta2_BC) * xy_projection.X) - (Math.Sin(theta2_BC) * xy_projection.Y);
                double y1 = (Math.Sin(theta2_BC) * xy_projection.X) + (Math.Cos(theta2_BC) * xy_projection.Y);
                
                pseudo_unit_vector.X = x1;
                pseudo_unit_vector.Y = y1;
                pseudo_unit_vector.Z = z1;

                double length_ratios = shoulder_to_elbow_AB_current.Length / pseudo_unit_vector.Length;
                Vector3D.Multiply(pseudo_unit_vector, length_ratios);
                
            }


            // Lets get the BC angles:

        }

        internal int beginArmHandling(JointGeneralState elbowState)
        {
            int retValue = 0;

            if(elbowState == JointGeneralState.Still && canAcceptNewMovement == true)
            {
                // Elbow is still, so we have a unmoving state of arm and we can output information.

                // Lets create the AB and BC vectors:
                shoulder_to_elbow_AB_current = GestureAuxilaryMethods.updateLimbVectors(JointType.ShoulderLeft, JointType.ElbowLeft);
                elbow_to_wrist_BC_current = GestureAuxilaryMethods.updateLimbVectors(JointType.ShoulderRight, JointType.ElbowRight);

                theta1_AB = Vector3D.AngleBetween(shoulder_to_elbow_AB_current, shoulder_to_elbow_AB_previous);
                theta1_BC = Vector3D.AngleBetween(elbow_to_wrist_BC_current, elbow_to_wrist_BC_previous);

                // Calculating theta2 angles:
                Vector3D orientation_of_previous = new Vector3D();
                
                if (endpoint_C_current.Z >= endpoint_C_previous.Z)      // Current projects onto previous
                {
                    // a is for current:
                    float a = (endpoint_C_current.X - endpoint_B_previous.X) * (endpoint_C_current.X - endpoint_C_previous.X) +
                              (endpoint_C_current.Y - endpoint_C_previous.Y) * (endpoint_C_current.Y - endpoint_C_previous.Y);
                    float b = (endpoint_C_current.X - endpoint_C_previous.X) * (endpoint_C_current.X - endpoint_C_previous.X) +
                              (endpoint_C_current.Y - endpoint_C_previous.Y) * (endpoint_C_current.Y - endpoint_C_previous.Y);

                } else      // Previous projects onto current
                {
                    
                }


                // We need to have a new movement in order to do new motor calculations:
                canAcceptNewMovement = false;

                // Storing the current vectors into previous ones:
                shoulder_to_elbow_AB_previous = shoulder_to_elbow_AB_current;
                elbow_to_wrist_BC_previous = elbow_to_wrist_BC_current;

            }

            if (elbowState == JointGeneralState.Moving)
            {
                // Elbow is moving, so we need to wait until its stationary to output information.
                canAcceptNewMovement = true;
            }
            


            return retValue;
        }
    }
