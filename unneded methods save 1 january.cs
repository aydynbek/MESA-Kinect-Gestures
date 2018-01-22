
        /// <summary>
        /// This method is designed to initiate the first rotation of the limbs into their
        /// positions which they were at during triggering. The algorithm assumes that 
        /// the arm was initially streching out normal to the torso and that it then has to
        /// rotate the limbs into the initial position.
        /// </summary>
        internal void initializeGesture()
        {
            // ATTENTION: All of this code is written with an assumption that the person is standing right in front of 
            // the Kinect, and his Right arm will be used for the measurement, and that arm has to be pointing 
            // generally in the right direction.

            // ATTENTION: We only have positive Z coordinates from the camera. Camera point is 0 for Z, and then 
            // it increases as you move further away.

            // ATTENTION: If we are only going to use 2 servo motors and 2 pairs of boards to try to replicate the whole ABC 
            // arm, then use theta_AB for whole theta and use phi_BC + phi_AB for whole phi. This is for Avery's board.

            // ATTENTION: theta is THETA and phi is PHI in spherical coordinates. R is the parameter
            // thats passed into this function. The corrected THETA angle has to be the difference
            // of 180 and THETA that we calculated.

            // ATTENTION: All theta and phi angles are absolute, not incremental. All angle rotations go in the 
            // direction of lowest incremental angle change.

            // ATTENTION: The initial phi angle is 0.

            // ATTENTION: We need to make sure that theta stays within 0 to pi range. If it goes to the negative range
            // then we have to simply change phi.

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

            // Previous vector is set to an initial position, that is a vector with only an X component:
            shoulder_to_elbow_AB_previous.X = shoulder_to_elbow_AB_current.Length;
            shoulder_to_elbow_AB_previous.Y = 0;
            shoulder_to_elbow_AB_previous.Z = 0;
            
            // Lets get to the AB angles
            double rEcpx = right_elbow_current.Position.X;
            double rEcpy = right_elbow_current.Position.Y;      // Right elbow current position Y
            double rEcpz = right_elbow_current.Position.Z;
            double rScpx = right_shoulder_current.Position.X;
            double rScpy = right_shoulder_current.Position.Y;   // Right shoulder current position Y
            double rScpz = right_shoulder_current.Position.Z;
            double rWcpx = right_wrist_current.Position.X;
            double rWcpy = right_wrist_current.Position.Y;
            double rWcpz = right_wrist_current.Position.Z;
            
            // Cartesian coordinates of the tip of the current vector:
            double realXcurr = rEcpx - rScpx;
            double realYcurr = rEcpy - rScpy;
            double realZcurr = rEcpz - rScpz;
            
            // R, phi, and theta based coordinates for the tip of the previous vector:
            // We need to assume that the x in the formula is the real z axis, whilst z in the 
            // formula is the the real x. Also, we do not negate x values when we try to insert 
            // and input for z in the formula. This is because our spherical coordinates are reflected x-z plane
            // of figure 2.4:
            double sphericalR = Math.Sqrt(realZcurr * realZcurr + realYcurr * realYcurr + realXcurr * realXcurr);
            double sphericalTheta = Math.Atan2(Math.Sqrt(realZcurr * realZcurr + realYcurr * realYcurr), realXcurr);
            double sphericalPhi = Math.Atan2(realYcurr, realZcurr);

            theta_AB = ((sphericalTheta/Math.PI)*180);
            phi_AB = ((sphericalPhi/Math.PI)*180);

            rotation_dir_theta_AB = false;
            rotation_dir_phi_AB = false;

            // Lets create the orientation vector for the phi rotation towards the current vector.
            // Lets have the J distance at 10cm and lets get that ratio:
            double jRatio = .1 / shoulder_to_elbow_AB_previous.Length;
            double mDistance = .1 * Math.Cos(sphericalTheta);
            double mRatio = mDistance / shoulder_to_elbow_AB_current.Length;

            Vector3D mVector = new Vector3D();
            mVector = Vector3D.Multiply(shoulder_to_elbow_AB_current, mRatio);
            Vector3D jVector = new Vector3D();
            jVector = Vector3D.Multiply(shoulder_to_elbow_AB_previous, jRatio);

            // Save some variables into previous group for the next frame:
            this.mVectorAB_prev = mVector;
            this.mDistanceAB_prev = mDistance;

            // A vector that travels from the tip of J vector to the tip of M vector:
            jmVectorAB.X = mVector.X - jVector.X;
            jmVectorAB.Y = mVector.Y - jVector.Y;
            jmVectorAB.Z = mVector.Z - jVector.Z;

            /// This is where there is a problem: its seems like mVector and jmVector are not orthogonal....
            //TextInformation.insert_main_text_block("checkInit: " + (Vector3D.AngleBetween(jmVectorAB, mVector)).ToString("n4"), 1);
            //TextInformation.insert_main_text_block("checkInit: " + (Vector3D.DotProduct(jmVectorAB, mVector)).ToString(), 1);
            //TextInformation.insert_main_text_block("checkInit: " + (Vector3D.AngleBetween(jmVectorAB, mVector)).ToString(), 1);

            // Step B
            // Lets do the BC angles:

            // The current vector will simply be the elbow to wrist limb vector:
            elbow_to_wrist_BC_current = GestureAuxilaryMethods.updateLimbVectors(JointType.ElbowRight, JointType.WristRight);

            // Previous vector is set to an initial position, that is a vector with only an X component:
            elbow_to_wrist_BC_previous.X = elbow_to_wrist_BC_current.Length;
            elbow_to_wrist_BC_previous.Y = 0;
            elbow_to_wrist_BC_previous.Z = 0;
            
            // Cartesian coordinates of the tip of the current vector:
            realXcurr = rWcpx - rEcpx;
            realYcurr = rWcpy - rEcpy;
            realZcurr = rWcpz - rEcpz;
            
            // R, phi, and theta based coordinates for the tip of the previous vector:
            // We need to assume that the x in the formula is the real z axis, whilst z in the 
            // formula is the the real x. Also, we do not negate x values when we try to insert 
            // and input for z in the formula. This is because our spherical coordinates are reflected x-z plane
            // of figure 2.4:
            sphericalR = Math.Sqrt(realZcurr * realZcurr + realYcurr * realYcurr + realXcurr * realXcurr);
            sphericalTheta = Math.Atan2(Math.Sqrt(realZcurr * realZcurr + realYcurr * realYcurr), realXcurr);
            sphericalPhi = Math.Atan2(realYcurr, realZcurr);

            theta_BC = ((sphericalTheta / Math.PI) * 180);
            phi_BC = ((sphericalPhi / Math.PI) * 180);

            rotation_dir_theta_AB = false;
            rotation_dir_phi_AB = false;

            // Lets create the orientation vector for the phi rotation towards the current vector.
            // Lets have the J distance at 10cm and lets get that ratio:
            jRatio = .1 / elbow_to_wrist_BC_previous.Length;
            mDistance = .1 / Math.Cos(sphericalTheta);
            mRatio = mDistance / elbow_to_wrist_BC_current.Length;

            mVector = new Vector3D();
            mVector = Vector3D.Multiply(elbow_to_wrist_BC_current, mRatio);
            jVector = new Vector3D();
            jVector = Vector3D.Multiply(elbow_to_wrist_BC_previous, jRatio);

            // Save some variables into previous group for the next frame:
            this.mVectorBC_prev = mVector;
            this.mDistanceBC_prev = mDistance;

            // A vector that travels from the tip of J vector to the tip of M vector:
            jmVectorBC.X = mVector.X - jVector.X;
            jmVectorBC.Y = mVector.Y - jVector.Y;
            jmVectorBC.Z = mVector.Z - jVector.Z;
            


            // Updating the vectors in preparation for the next incoming frame calculations:
            shoulder_to_elbow_AB_previous = shoulder_to_elbow_AB_current;
            elbow_to_wrist_BC_previous = elbow_to_wrist_BC_current;

            /// /// /// /// /// /// /// /// ///                   
            /// STOPED HERE; CHECK TO MAKE SURE IT WORKS:
            // Correcting the theta_BC and phi_BC values by finding the difference with AB angles:
            if (theta_BC >= theta_AB)
            {
                theta_BC -= theta_AB;
            } else
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

            TextInformation.insert_main_text_block("Initial.", 1);
            TextInformation.insert_main_text_block("thetaAB: " + theta_AB.ToString("n4"), 1);
            TextInformation.insert_main_text_block("phiAB: " + phi_AB.ToString("n4"), 1);
            TextInformation.insert_main_text_block("thetaBC: " + theta_BC.ToString("n4"), 1);
            TextInformation.insert_main_text_block("phiBC: " + phi_BC.ToString("n4"), 1);
            

            drawImitatedArm(shoulder_to_elbow_AB_current.Length, elbow_to_wrist_BC_current.Length);
        }
		
		
		
		
		
		

        
        internal void drawImitatedArm(double shoulder_elbow_len, double elbow_wrist_len )
        {
            double theta_AB = this.theta_AB;
            double theta_BC = this.theta_BC + this.theta_AB;
            double phi_AB = this.phi_AB;
            double phi_BC = this.phi_BC + this.phi_AB;

            Pen penn = new Pen(Brushes.Red, 3);
            Pen penn2 = new Pen(Brushes.Black, 2);
            Brush drawBrush = new SolidColorBrush(Color.FromArgb(255, 130, 15, 180));
            

            // ATTENTION: theta is THETA and phi is PHI in spherical coordinates. R is the parameter
            // thats passed into this function. 

            /// Check again:
            // ATTENTION: The corrected THETA angle has to be the difference
            // of 180 and THETA that we calculated. -- REALLY? Check right below:

            // ATTENTION: Given the Figure 2.4 from the book, we have to switch the X and Z axis. The 
            // corrected Z axis (the real life Z axis) is the X axis that we get from calculations,
            // and the corrected X axis is the negative of the Z axis.

            double theta_AB_rad = (theta_AB / 180) * Math.PI;
            double theta_BC_rad = (theta_BC / 180) * Math.PI;
            double phi_AB_rad = (phi_AB / 180) * Math.PI;
            double phi_BC_rad = (phi_BC / 180) * Math.PI;

            double X_AB = shoulder_elbow_len * Math.Sin(theta_AB_rad) * Math.Cos(phi_AB_rad);
            double Y_AB = shoulder_elbow_len * Math.Sin(theta_AB_rad) * Math.Sin(phi_AB_rad);
            double Z_AB = shoulder_elbow_len * Math.Cos(theta_AB_rad);

            double X_BC = elbow_wrist_len * Math.Sin(theta_BC_rad) * Math.Cos(phi_BC_rad);
            double Y_BC = elbow_wrist_len * Math.Sin(theta_BC_rad) * Math.Sin(phi_BC_rad);
            double Z_BC = elbow_wrist_len * Math.Cos(theta_BC_rad);

            using (DrawingContext dcXZ = GesturesMasterControl.drawingGroupXZ.Open())
            {
                
                dcXZ.DrawRectangle(Brushes.Aquamarine, null, new Rect(0.0, 0.0,
                512, 424));

                dcXZ.DrawLine(penn, new Point(10, 10), new Point(40, 30));
                dcXZ.DrawLine(penn, new Point(10, 40), new Point(40, 10));
                
                // Arguements to Point Constructor: (Columns starting from left, Row starting from top)
                Point x1 = new Point(80, 212);
                Point y1 = new Point(80 + 400 * Z_AB, 212 + 400 * X_AB);

                Point x2 = new Point(80 + 400 * Z_AB, 212 + 400 * X_AB);
                Point y2 = new Point(80 + 400 * Z_AB + 400 * Z_BC, 212 + 400 * X_AB + 400 * X_BC);

                dcXZ.DrawLine(penn, x1, y1);
                dcXZ.DrawLine(penn, x2, y2);
                dcXZ.DrawEllipse(drawBrush, null, x1, 7, 7);
                dcXZ.DrawEllipse(drawBrush, null, x2, 7, 7);
                dcXZ.DrawEllipse(drawBrush, null, y2, 7, 7);
            }

            using (DrawingContext dcYX = GesturesMasterControl.drawingGroupYX.Open())
            {
                dcYX.DrawRectangle(Brushes.Aqua, null, new Rect(0.0, 0.0,
                512, 424));

                Point x1 = new Point(80, 212);
                Point y1 = new Point(80 + 400 * Z_AB, 424 - (212 + 400 * Y_AB));

                Point x2 = new Point(80 + 400 * Z_AB, 424 - (212 + 400 * Y_AB));
                Point y2 = new Point(80 + 400 * Z_AB + 400 * Z_BC, 424 - (212 + 400 * Y_AB + 400 * Y_BC));
                
                dcYX.DrawLine(penn, x1, y1);
                dcYX.DrawLine(penn, x2, y2);
                dcYX.DrawEllipse(drawBrush, null, x1, 7, 7);
                dcYX.DrawEllipse(drawBrush, null, x2, 7, 7);
                dcYX.DrawEllipse(drawBrush, null, y2, 7, 7);
            }

            using (DrawingContext dcYZ = GesturesMasterControl.drawingGroupYZ.Open())
            {
                dcYZ.DrawRectangle(Brushes.Aqua, null, new Rect(0.0, 0.0,
                512, 424));

                Point x1 = new Point(256, 212);
                Point y1 = new Point(256 + 400 * X_AB, 424 - (212 + 400 * Y_AB));

                Point x2 = new Point(256 + 400 * X_AB, 424 - (212 + 400 * Y_AB));
                Point y2 = new Point(256 + 400 * X_AB + 400 * X_BC, 424 - (212 + 400 * Y_AB + 400 * Y_BC));

                dcYZ.DrawLine(penn, x1, y1);
                dcYZ.DrawLine(penn, x2, y2);
                dcYZ.DrawEllipse(drawBrush, null, x1, 7, 7);
                dcYZ.DrawEllipse(drawBrush, null, x2, 7, 7);
                dcYZ.DrawEllipse(drawBrush, null, y2, 7, 7);
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
        internal int beginArmHandlingPathIndependent(JointGeneralState rightElbowState, JointGeneralState rightWristState)
        {
            // ATTENTION: For figuring out the phi angles, we have to, for any case, know the plane that is generated 
            // between current and previous vectors.

            // ATTENTION: We need to keep an absolute value of the theta and phi angles. We need to do this because we can't
            // use the spherical coordinates methodology all the time.




            if (rightElbowState == JointGeneralState.Still && rightWristState == JointGeneralState.Still && canAcceptNewMovement == true)
            {
                TextInformation.insert_main_text_block("Handling.", 1);
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
                shoulder_to_elbow_AB_current = GestureAuxilaryMethods.updateLimbVectors(JointType.ShoulderLeft, JointType.ElbowLeft);
                elbow_to_wrist_BC_current = GestureAuxilaryMethods.updateLimbVectors(JointType.ShoulderRight, JointType.ElbowRight);

                // Calculating theta angles. Important note: We need to keep track of previous and current vectors, since in 
                // essence, the theta angles are given by an angle between current stance of the limb and previous stance.
                theta_AB = Vector3D.AngleBetween(shoulder_to_elbow_AB_current, shoulder_to_elbow_AB_previous);
                theta_BC = Vector3D.AngleBetween(elbow_to_wrist_BC_current, elbow_to_wrist_BC_previous);

                // Calculating phi angles.
                // AB angles.
                
                // Lets create the orientation vector for the phi rotation towards the current vector.
                double kDistance = mDistanceAB_prev / Math.Cos(((theta_AB / 180) * Math.PI));
                double kRatio = kDistance / shoulder_to_elbow_AB_current.Length;
                
                Vector3D kVector = new Vector3D();
                kVector = Vector3D.Multiply(shoulder_to_elbow_AB_current, kRatio);

                //TextInformation.insert_main_text_block("kDistance: " + (kDistance).ToString("n4"), 1);
                //TextInformation.insert_main_text_block("kRatio: " + (kRatio).ToString(), 1);
                //TextInformation.insert_main_text_block("mDistAB_pr: " + (mDistanceAB_prev).ToString(), 1);
                
                // A vector that travels from the tip of M vector to the tip of K vector:
                mkVectorAB.X = kVector.X - mVectorAB_prev.X;
                mkVectorAB.Y = kVector.Y - mVectorAB_prev.Y;
                mkVectorAB.Z = kVector.Z - mVectorAB_prev.Z;

                // TextInformation.insert_main_text_block("Check: " + (Vector3D.AngleBetween(mVectorAB_prev, mkVectorAB)).ToString("n4"), 1);

                // Now we have 2 vectors with which we can get the angles:
                phi_AB = Vector3D.AngleBetween(jmVectorAB, mkVectorAB);

                //TextInformation.insert_main_text_block("Check: " + (Vector3D.AngleBetween(mkVectorAB, mVectorAB_prev)).ToString("n4"), 1);

                // Save some variables into previous group for the next frame:
                this.mVectorAB_prev = kVector;
                this.mDistanceAB_prev = kDistance;
                this.jmVectorAB = mkVectorAB;




                // BC angles.
                
                // Lets create the orientation vector for the phi rotation towards the current vector.
                kDistance = mDistanceBC_prev / Math.Cos(((theta_BC / 180) * Math.PI));
                kRatio = kDistance / elbow_to_wrist_BC_current.Length;
                
                kVector = Vector3D.Multiply(elbow_to_wrist_BC_current, kRatio);

                // A vector that travels from the tip of M vector to the tip of K vector:
                mkVectorBC.X = kVector.X - mVectorBC_prev.X;
                mkVectorBC.Y = kVector.Y - mVectorBC_prev.Y;
                mkVectorBC.Z = kVector.Z - mVectorBC_prev.Z;

                

                // Now we have 2 vectors with which we can get the angles:
                phi_BC = Vector3D.AngleBetween(jmVectorBC, mkVectorBC);

                // Save some variables into previous group for the next frame:
                this.mVectorBC_prev = kVector;
                this.mDistanceBC_prev = kDistance;
                this.jmVectorBC = mkVectorBC;


                

                
               
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

                // drawImitatedArm(shoulder_to_elbow_AB_current.Length, elbow_to_wrist_BC_current.Length);
            }

            // CAREFULL: the two boolean variables are ORed, not ANDed like before. The reason for this is that 
            // we would need at least one limb to move in order to have new rotations. This would set canAcceptNewMovement
            // true, but afterwards, we would still need both of the limbs to be still so that proper angle reading can
            // occur.
            if (rightElbowState == JointGeneralState.Moving || rightWristState == JointGeneralState.Moving)
            {
                // Elbow is moving, so we need to wait until its stationary to output information.
                canAcceptNewMovement = true;
            }

            // Stop the handling if the left hand is open:
            if (GesturesMasterControl.body.HandLeftState == HandState.Closed)
            {
                return 1;
            } else
            {
                return -1;
            }
        }