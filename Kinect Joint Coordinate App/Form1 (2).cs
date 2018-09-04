using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Microsoft.Kinect;
using System.Diagnostics;
using System.IO;
using System.Data.SqlClient;
using MathNet.Numerics;
using MathNet.Numerics.Statistics;
using MathNet.Numerics.Distributions;
using System.Windows.Forms.DataVisualization;
using System.Windows.Forms.DataVisualization.Charting;
using System.Speech;
using System.Speech.Synthesis;
using System.Speech.Recognition;
using System.Threading;





namespace Kinect_Joint_Coordinate_App
{
    public partial class Form1 : Form
    {
        //DATA CONNECTIONS --------  1  ----------------------------------------------------------  1  ---------------------------------------------------------  1  ------------------------------------------------------------  1  ------

        SqlConnection dbConnection = new SqlConnection(@"Data Source=(LocalDB)\MSSQLLocalDB;AttachDbFilename=C:\Users\eadadzie\Desktop\GaitDatabase_original\GaitDatabase.mdf;Integrated Security=True;Connect Timeout=30");




        //FUNCTIONS --------  2  ------------------------------------------------------------  2  ---------------------------------------------------------  2  -------------------------------------------------------  2  --------

        //Activate Sensor functions ------  2.1  ----------------------  2.1  ----------------------  2.1  -----------------------  2.1  -------------------------  2.1  -------------------------  2.1  -----------------------  2.1  --------------------
        KinectSensor kinectSensor = null;
        BodyFrameReader bodyFrameReader = null;
        Body[] bodies = null;

        
        public void initialiseKinect()  //Initialze Kinect, define frame-reader type and data source
        {
            kinectSensor = KinectSensor.GetDefault();

            if (kinectSensor != null)
            {
                // turn on kinect
                kinectSensor.Open();

            }


            bodyFrameReader = kinectSensor.BodyFrameSource.OpenReader();

            if (bodyFrameReader != null)
            {
                bodyFrameReader.FrameArrived += Reader_FrameArrived;
            }
        }


        public double x1_ms, x2_ms, y1_ms, y2_ms, z1_ms, z2_ms, x3_ms, y3_ms, z3_ms, x4_ms, y4_ms, z4_ms, distance_ms, distance_1_ms, distance_2_ms, distance_3_ms, gaitSpeed_ms, vel_1_ms, vel_2_ms, vel_3_ms, aceleration_ms;
        public double x1_ra, x2_ra, y1_ra, y2_ra, z1_ra, z2_ra, x3_ra, y3_ra, z3_ra, x4_ra, y4_ra, z4_ra, distance_ra, distance_1_ra, distance_2_ra, distance_3_ra, gaitSpeed_ra, vel_1_ra, vel_2_ra, vel_3_ra, aceleration_ra;
        public double x5_ra, x6_ra, y5_ra, y6_ra, z5_ra, z6_ra, x7_ra, y7_ra, z7_ra, x8_ra, y8_ra, z8_ra, distance_4_ra, distance_5_ra, distance_6_ra, distance_7_ra, vel_4_ra, vel_5_ra, vel_6_ra, vel_7_ra;
        float A, B, C, D;
        double strideVelocity = 0;
        double x1, x2, x3, y1, y2, y3, z1, z2, z3;


        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)    //Define frame reference, receive data, track joints, and compute distance, velocity and acceleration
        {
            bool dataReceived = false;
            

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (bodies == null)
                    {
                        bodies = new Body[bodyFrame.BodyCount];
                     
                    }
                }

                //try
                //{
                //    bodyFrame.GetAndRefreshBodyData(this.bodies);
                //}
                //catch (Exception ex)
                //{
                //    dayofweek.Text = ex.Message;
                //    MessageBox.Show(ex.Message);
                //}
                //finally
                //{
                //    bodyFrame.GetAndRefreshBodyData(this.bodies);
                //}

                // *******FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF********************************
                //Floor plane coordinate**********************************
                 A = bodyFrame.FloorClipPlane.X;
                 B = bodyFrame.FloorClipPlane.Y;
                 C = bodyFrame.FloorClipPlane.Z;
                 D = bodyFrame.FloorClipPlane.W;
                 
            
                bodyFrame.GetAndRefreshBodyData(this.bodies);
                dataReceived = true;
            }

            if (dataReceived)
            {
                foreach (Body body in bodies)
                {

                    if (body.IsTracked)
                    {

                        IReadOnlyDictionary<JointType, Joint> joints = body.Joints;
                        Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                        //Defining the skeleton joints
                        Joint midSpine = joints[JointType.SpineMid];
                        Joint leftAnkle = joints[JointType.AnkleLeft];
                        Joint rightAnkle = joints[JointType.AnkleRight];
                        Joint head = joints[JointType.Head];
                        Joint neck = joints[JointType.Neck];
                        Joint spineShoulder = joints[JointType.SpineShoulder];
                        Joint leftShoulder = joints[JointType.ShoulderLeft];
                        Joint leftElbow = joints[JointType.ElbowLeft];
                        Joint leftWrist = joints[JointType.WristLeft];
                        Joint leftHandTip = joints[JointType.HandTipLeft];
                        Joint spineBase = joints[JointType.SpineBase];
                        Joint leftHip = joints[JointType.HipLeft];
                        Joint leftKnee = joints[JointType.KneeLeft];

                        // Collect coordinate of mid spine joint
                        float midSpine_X = midSpine.Position.X;
                        float midSpine_Y = midSpine.Position.Y;
                        float midSpine_Z = midSpine.Position.Z;

                        // Collect coordinate of left ankle joint
                        float leftAnkle_X = leftAnkle.Position.X;
                        float leftAnkle_Y = leftAnkle.Position.Y;
                        float leftAnkle_Z = leftAnkle.Position.Z;

                        // Collect coordinate of right ankle joint
                        float rightAnkle_X = rightAnkle.Position.X;
                        float rightAnkle_Y = rightAnkle.Position.Y;
                        float rightAnkle_Z = rightAnkle.Position.Z;

                        // Collect coordinate of head joint
                        float head_X = head.Position.X;
                        float head_Y = head.Position.Y;
                        float head_Z = head.Position.Z;

                        // Collect coordinate of neck joint
                        float neck_X = neck.Position.X;
                        float neck_Y = neck.Position.Y;
                        float neck_Z = neck.Position.Z;

                        // Collect coordinate of spine Shoulder joint
                        float spineShoulder_X = spineShoulder.Position.X;
                        float spineShoulder_Y = spineShoulder.Position.Y;
                        float spineShoulder_Z = spineShoulder.Position.Z;

                        // Collect coordinate of left Shoulder joint
                        float leftShoulder_X = leftShoulder.Position.X;
                        float leftShoulder_Y = leftShoulder.Position.Y;
                        float leftShoulder_Z = leftShoulder.Position.Z;

                        // Collect coordinate of left Elbow joint
                        float leftElbow_X = leftElbow.Position.X;
                        float leftElbow_Y = leftElbow.Position.Y;
                        float leftElbow_Z = leftElbow.Position.Z;

                        // Collect coordinate of left Wrist joint
                        float leftWrist_X = leftWrist.Position.X;
                        float leftWrist_Y = leftWrist.Position.Y;
                        float leftWrist_Z = leftWrist.Position.Z;

                        // Collect coordinate of left HandTip joint
                        float leftHandTip_X = leftHandTip.Position.X;
                        float leftHandTip_Y = leftHandTip.Position.Y;
                        float leftHandTip_Z = leftHandTip.Position.Z;

                        // Collect coordinate of spine Base joint
                        float spineBase_X = spineBase.Position.X;
                        float spineBase_Y = spineBase.Position.Y;
                        float spineBase_Z = spineBase.Position.Z;

                        // Collect coordinate of left Hip joint
                        float leftHip_X = leftHip.Position.X;
                        float leftHip_Y = leftHip.Position.Y;
                        float leftHip_Z = leftHip.Position.Z;

                        // Collect coordinate of left Knee joint
                        float leftKnee_X = leftKnee.Position.X;
                        float leftKnee_Y = leftKnee.Position.Y;
                        float leftKnee_Z = leftKnee.Position.Z;

                                                
                        //Computing length of skeleton segments
                        var length_HeadToNeck = CalculateDistance(head_X, head_Y, head_Z, neck_X, neck_Y, neck_Z);
                        var length_NeckToSpineShoulder = CalculateDistance(neck_X, neck_Y, neck_Z, spineShoulder_X, spineShoulder_Y, spineShoulder_Z);
                        var length_SpineShoulderToLeftShoulder = CalculateDistance(spineShoulder_X, spineShoulder_Y, spineShoulder_Z, leftShoulder_X, leftShoulder_Y, leftShoulder_Z);
                        var length_LeftShoulderToLeftElbow = CalculateDistance(leftShoulder_X, leftShoulder_Y, leftShoulder_Z, leftElbow_X, leftElbow_Y, leftElbow_Z);
                        var length_LeftElbowToLeftWrist = CalculateDistance(leftElbow_X, leftElbow_Y, leftElbow_Z, leftWrist_X, leftWrist_Y, leftWrist_Z);
                        var length_LeftWristToHandTip = CalculateDistance(leftWrist_X, leftWrist_Y, leftWrist_Z, leftHandTip_X, leftHandTip_Y, leftHandTip_Z);
                        var length_SpineShoulderToMidSpine = CalculateDistance(spineShoulder_X, spineShoulder_Y, spineShoulder_Z, midSpine_X, midSpine_Y, midSpine_Z);
                        var length_MidSpineToSpineBase = CalculateDistance(midSpine_X, midSpine_Y, midSpine_Z, spineBase_X, spineBase_Y, spineBase_Z);
                        var length_SpineBaseToLeftHip = CalculateDistance(spineBase_X, spineBase_Y, spineBase_Z, leftHip_X, leftHip_Y, leftHip_Z);
                        var length_LeftHipToleftKnee = CalculateDistance(leftHip_X, leftHip_Y, leftHip_Z, leftKnee_X, leftKnee_Y, leftKnee_Z);
                        var length_leftKneeToleftAnkle = CalculateDistance(leftKnee_X, leftKnee_Y, leftKnee_Z, leftAnkle_X, leftAnkle_Y, leftAnkle_Z);


                        //Display segments
                        lblDisplaySegment.Text = String.Format("{0:0.00}", length_HeadToNeck) + " , " + String.Format("{0:0.00}", length_NeckToSpineShoulder) + " , " + String.Format("{0:0.00}", length_SpineShoulderToLeftShoulder)
                           + " , " + String.Format("{0:0.00}", length_SpineShoulderToMidSpine) + " , " + String.Format("{0:0.00}", length_MidSpineToSpineBase);

                        //***************************new CODING PART*****************************

                        double up = ((rightAnkle_X * A) + (rightAnkle_Y * B) + (rightAnkle_Z * C) + (D));
                        double upL = ((leftAnkle_X * A) + (leftAnkle_Y * B) + (leftAnkle_Z * C) + (D));
                        double down = Math.Sqrt(A * A + B * B + C * C);
                        double downL = Math.Sqrt(A * A + B * B + C * C);

                        double rightAnkleDistance = up / down;
                        double leftAnkleDistance = upL / downL;

                        // lblToeClearance.Text += rightAnkleDistance.ToString("#.##") + Environment.NewLine;
                        // lblToeClearance.Text = rightAnkleDistance.ToString("#.##") ;



                        if (rightAnkleDistance > 0.11)
                        {

                            isActive3 = true;
                            x1 = rightAnkle_X;
                            y1 = rightAnkle_Y;
                            z1 = rightAnkle_Z;

                            x2 = 0;
                            y2 = 0;
                            z2 = 0;
                            //lblToeClearance.Text = timeCsec3.ToString();

                        
                        }
                        else

                        {

                            isActive3 = false;
                            var timeElapse = timeCsec3;

                            x3 = x1;
                            y3 = y1;
                            z3 = z1;

                            x2 = rightAnkle_X;
                            y2 = rightAnkle_Y;
                            z2 = rightAnkle_Z;

                            if (timeCsec3 > 0)
                            {
                                var distanceRankle = CalculateDistance(x3, y3, z3, x2, y2, z2);

                                txtToeDistance.Text = distanceRankle.ToString();

                                var velocityRankle = Math.Round((distanceRankle * 100) / timeElapse, 6);
                                //lblToeClearance.Text = timeCsec3.ToString();
                                                            
                                lblToeClearance.Text = velocityRankle.ToString();//+ Environment.NewLine;
                            }
                        
                            timeCsec3 = 0;
                        }












                        // lblToeClearance.Text += rightAnkleDistance.ToString("#.##")+ Environment.NewLine;
                        //lblToeClearance.Text += rightAnkle_X + " " + rightAnkle_Y + " " + rightAnkle_Z + Environment.NewLine;
                        ////lblToeClearance.Text += rightAnkleDistance + "      " + leftAnkleDistance + Environment.NewLine;
                        //   lblToeClearance.Text += rightAnkleDistance.ToString() /*rightAnkleDistance.ToString("#.##")*/ + Environment.NewLine;

                        //var initial = rightAnkleDistance;
                        //var final = rightAnkleDistance;

                        //if ((final - initial) == 0)
                        //{
                        //    lblToeClearance.Text += "YESSSSS" + Environment.NewLine;
                        //}
                        //else
                        //{
                        //    lblToeClearance.Text += "NO" + Environment.NewLine;
                        //}





                        // *********************************************************************************************

                        //Personal Identification using body segments for Emmanuel

                        if ((Math.Round(length_HeadToNeck, 2) >= 0.13 && Math.Round(length_HeadToNeck, 2) <= 0.17) && (Math.Round(length_NeckToSpineShoulder, 2) >= 0.06 && Math.Round(length_NeckToSpineShoulder, 2) <= 0.10) && (Math.Round(length_SpineShoulderToLeftShoulder, 2) >= 0.15 && Math.Round(length_SpineShoulderToLeftShoulder, 2) <= 0.19) && (Math.Round(length_SpineShoulderToMidSpine, 2) >= 0.21 && Math.Round(length_SpineShoulderToMidSpine, 2) <= 0.25))
                        {
                            lblUserName.Text = "Emmanuel";
                        }
                        else
                        {
                            lblUserName.Text = "Unknown";
                        }

                        //if ((Math.Round(length_HeadToNeck, 2) >= 0.10 && Math.Round(length_HeadToNeck, 2) <= 0.18) && (Math.Round(length_NeckToSpineShoulder, 2) >= 0.03 && Math.Round(length_NeckToSpineShoulder, 2) <= 0.11) && (Math.Round(length_SpineShoulderToLeftShoulder, 2) >= 0.13 && Math.Round(length_SpineShoulderToLeftShoulder, 2) <= 0.23) && (Math.Round(length_LeftShoulderToLeftElbow, 2) >= 0.25 && Math.Round(length_LeftShoulderToLeftElbow, 2) <= 0.33) && (Math.Round(length_LeftElbowToLeftWrist, 2) >= 0.21 && Math.Round(length_LeftElbowToLeftWrist, 2) <= 0.29) && (Math.Round(length_LeftWristToHandTip, 2) >= 0.08 && Math.Round(length_LeftWristToHandTip, 2) <= 0.18))
                        //{
                        //    lblUserName.Text = "Emmanuel";
                        //}
                        //else
                        //{
                        //    lblUserName.Text = "Unknown";
                        //}


                        //if (Math.Round(length_HeadToNeck, 2) == 0.14 && Math.Round(length_NeckToSpineShoulder, 2) == 0.07 && Math.Round(length_SpineShoulderToLeftShoulder, 2) == 0.16 && Math.Round(length_LeftShoulderToLeftElbow, 2) == 0.26 && Math.Round(length_LeftElbowToLeftWrist, 2) == 0.26 && Math.Round(length_LeftWristToHandTip, 2) == 0.16)
                        //{
                        //    lblUserName.Text = "Hi Mariah";
                        //}



                        // Display spine coordinate
                        txtSpineX.Text = midSpine_X.ToString("#.##") + ", " + midSpine_Y.ToString("#.##") + ", " + midSpine_Z.ToString("#.##");
                        //txtSpineY.Text = midSpine_Y.ToString("#.##");
                        //txtSpineZ.Text = midSpine_Z.ToString("#.##");
                        //txtMSCord.Text = ms_XYZ_Cord.ToString() + Environment.NewLine;

                        // Display left ankle coordinate
                        txtLAnkle_X.Text = leftAnkle_X.ToString("#.##") + ", " + leftAnkle_Y.ToString("#.##") + ", " + leftAnkle_Z.ToString("#.##");
                        //txtLAnkle_Y.Text = leftAnkle_Y.ToString("#.##");
                        //txtLAnkle_Z.Text = leftAnkle_Z.ToString("#.##");

                        // Display Right ankle coordinate
                        //txtSpineX.Text = RAnk_distance_x.ToString("#.##");
                        //txtSpineY.Text = RAnk_distance_y.ToString("#.##");
                        //txtSpineZ.Text = RAnk_distance_z.ToString("#.##");


                        //Velocity calculation for upper body (mid-spine/ COM) ************************************************************************************

                        if (timeCsec == 0)
                        {
                            //COM variables & calculations
                            x1_ms = midSpine_X;
                            y1_ms = midSpine_Y;
                            z1_ms = midSpine_Z;
                            x2_ms = x4_ms;
                            y2_ms = y4_ms;
                            z2_ms = z4_ms;
                                                    
                            distance_1_ms = CalculateDistance(x1_ms, y1_ms, z1_ms, x2_ms, y2_ms, z2_ms);

                            lblDistance.Text = distance_1_ms.ToString("#.###") + " m";

                            vel_1_ms = distance_1_ms / 0.25;

                            lblVelocity.Text = vel_1_ms.ToString("#.###") + " m/s";


                            //Right ankle variables & calculations
                            x1_ra = rightAnkle_X;
                            y1_ra = rightAnkle_Y;
                            z1_ra = rightAnkle_Z;
                            x2_ra = x4_ra;
                            y2_ra = y4_ra;
                            z2_ra = z4_ra;

                            distance_3_ra = CalculateDistance(x1_ra, y1_ra, z1_ra, x4_ra, y4_ra, z4_ra);

                            lblDistance_RA.Text = distance_3_ra.ToString("#.###") + " m";

                            vel_3_ra = distance_3_ra / 0.25;

                            lblVelocity_RA.Text = vel_3_ra.ToString("#.###") + " m/s";
                            

                            // display();
                            secondDisplay();
                            secondDisplayAnkles();
                            SaveFile();

                        }

                        if (timeCsec == 2)
                        {
                            var x1_1 = x1_ms;
                            var y1_1 = y1_ms;
                            var z1_1 = z1_ms;

                            x2_ms = midSpine_X;
                            y2_ms = midSpine_Y;
                            z2_ms = midSpine_Z;

                            distance_1_ms = CalculateDistance(x1_1, y1_1, z1_1, x2_ms, y2_ms, z2_ms);

                            lblDistance.Text = distance_1_ms.ToString("#.###") + " m";

                            vel_1_ms = distance_1_ms / 0.25;

                            lblVelocity.Text = vel_1_ms.ToString("#.###") + " m/s";


                            var x1_1ra = x1_ra;
                            var y1_1ra = y1_ra;
                            var z1_1ra = z1_ra;

                            x2_ra = rightAnkle_X;
                            y2_ra = rightAnkle_Y;
                            z2_ra = rightAnkle_Z;

                            distance_1_ra = CalculateDistance(x1_1ra, y1_1ra, z1_1ra, x2_ra, y2_ra, z2_ra);

                            lblDistance_RA.Text = distance_1_ra.ToString("#.###") + " m";

                            vel_1_ra = distance_1_ra / 0.25;

                            lblVelocity_RA.Text = vel_1_ra.ToString("#.###") + " m/s";
                            //SaveData();

                            //display();
                            //SaveFile();
                            // WriteToFile();

                        }
                        if (timeCsec == 4)
                        {
                            var x2_2 = x2_ms;
                            var y2_2 = y2_ms;
                            var z2_2 = z2_ms;

                            x3_ms = midSpine_X;
                            y3_ms = midSpine_Y;
                            z3_ms = midSpine_Z;

                            distance_2_ms = CalculateDistance(x2_2, y2_2, z2_2, x3_ms, y3_ms, z3_ms);

                            lblDistance.Text = distance_2_ms.ToString("#.###") + " m";

                            vel_2_ms = distance_2_ms / 0.25;

                            lblVelocity.Text = vel_2_ms.ToString("#.###") + " m/s";



                            var x2_2ra = x2_ra;
                            var y2_2ra = y2_ra;
                            var z2_2ra = z2_ra;

                            x3_ra = rightAnkle_X;
                            y3_ra = rightAnkle_Y;
                            z3_ra = rightAnkle_Z;

                            distance_2_ra = CalculateDistance(x2_2ra, y2_2ra, z2_2ra, x3_ra, y3_ra, z3_ra);

                            lblDistance_RA.Text = distance_2_ra.ToString("#.###") + " m";

                            vel_2_ra = distance_2_ra / 0.25;

                            lblVelocity_RA.Text = vel_2_ra.ToString("#.###") + " m/s";

                            //SaveData();

                            //display();
                            //SaveFile();
                            // WriteToFile();

                        }
                        if (timeCsec == 6)
                        {
                            var x3_3 = x3_ms;
                            var y3_3 = y3_ms;
                            var z3_3 = z3_ms;

                            x4_ms = midSpine_X;
                            y4_ms = midSpine_Y;
                            z4_ms = midSpine_Z;

                            distance_3_ms = CalculateDistance(x3_3, y3_3, z3_3, x4_ms, y4_ms, z4_ms);

                            lblDistance.Text = distance_3_ms.ToString("#.###") + " m";

                            vel_3_ms = distance_3_ms / 0.25;

                            lblVelocity.Text = vel_3_ms.ToString("#.###") + " m/s";



                            var x3_3ra = x3_ra;
                            var y3_3ra = y3_ra;
                            var z3_3ra = z3_ra;

                            x4_ra = rightAnkle_X;
                            y4_ra = rightAnkle_Y;
                            z4_ra = rightAnkle_Z;

                            distance_3_ra = CalculateDistance(x3_3ra, y3_3ra, z3_3ra, x4_ra, y4_ra, z4_ra);

                            lblDistance_RA.Text = distance_3_ra.ToString("#.###") + " m";

                            vel_3_ra = distance_3_ra / 0.25;

                            lblVelocity_RA.Text = vel_3_ra.ToString("#.###") + " m/s";

                            //SaveData();

                            //display();
                            //SaveFile();
                            // WriteToFile();

                        }
                        if (timeCsec > 6)
                        {
                            distance_ms = distance_1_ms + distance_2_ms + distance_3_ms;
                            gaitSpeed_ms = (vel_1_ms + vel_2_ms + vel_3_ms) / 3;
                            aceleration_ms = (vel_3_ms - vel_1_ms) / 0.5;


                            lblDistance.Text = distance_ms.ToString("#.###") + " m";
                            lblVelocity.Text = gaitSpeed_ms.ToString("#.###") + " m/s";
                            lblAcceleration.Text = aceleration_ms.ToString("#.###") + " m/s2";
                            // SaveInTable_N_displayInTxtbox();



                            distance_ra = distance_1_ra + distance_2_ra + distance_3_ra /*+ distance_4_ra + distance_5_ra + distance_6_ra + distance_7_ra*/;
                            gaitSpeed_ra = (vel_1_ra + vel_2_ra + vel_3_ra /*+ vel_4_ra + vel_5_ra + vel_6_ra + vel_7_ra*/) / 3;
                            aceleration_ra = (vel_3_ra - vel_1_ra) / 0.5;


                            lblDistance_RA.Text = distance_ra.ToString("#.###") + " m";
                            lblVelocity_RA.Text = gaitSpeed_ra.ToString("#.###") + " m/s";
                            lblAcceleration_RA.Text = aceleration_ra.ToString("#.###") + " m/s2";
                            //txtTemp_Ankles.Text += String.Format("{0:0.000}", distance_ra) + "               " + String.Format("{0:0.000}", gaitSpeed_ra) + "            " + String.Format("{0:0.000}", aceleration_ra) + "            " + currentTime1.ToString("MM/dd/yyyy") + "            " + lblCurrentTime.Text + Environment.NewLine;

                            SaveInTable_N_displayInTxtbox();
                            SaveInTable_N_displayInTxtbox_Ankles();
                        }


                        //Velocity calculation for lower body (right ankle) ************************************************************************************

                        //if (timeCsec == 0)
                        //{
                        //    x1_ra = rightAnkle_X;
                        //    y1_ra = rightAnkle_Y;
                        //    z1_ra = rightAnkle_Z;
                        //    x2_ra = 0;
                        //    y2_ra = 0;
                        //    z2_ra = 0;

                        //    // display();
                        //    //secondDisplay();
                        //    secondDisplayAnkles();
                        //    SaveFile();

                        //}

                        //if (timeCsec == 2)
                        //{
                        //    var x1_1 = x1_ra;
                        //    var y1_1 = y1_ra;
                        //    var z1_1 = z1_ra;

                        //    x2_ra = rightAnkle_X;
                        //    y2_ra = rightAnkle_Y;
                        //    z2_ra = rightAnkle_Z;

                        //    distance_1_ra = CalculateDistance(x1_1, y1_1, z1_1, x2_ra, y2_ra, z2_ra);

                        //    lblDistance_RA.Text = distance_1_ra.ToString("#.###") + " m";

                        //    vel_1_ra = distance_1_ra / 0.25;

                        //    lblVelocity_RA.Text = vel_1_ra.ToString("#.###") + " m/s";

                        //    //if (vel_1_ra >= 0.075)
                        //    //{

                        //    //    isActive3 = true;

                        //    //}
                        //    //else
                        //    //{
                        //    //    isActive3 = false;
                        //    //    strideVelocity = distance_1_ra/ (timeCsec3 * 0.01);
                        //    //    lblStrideVelocity.Text = strideVelocity.ToString();
                        //    //    ResetStopwatch_3();

                        //    //}

                        //    //SaveData();

                        //    //display();
                        //    //SaveFile();
                        //    // WriteToFile();

                        //}
                        //if (timeCsec == 4)
                        //{
                        //    var x2_2 = x2_ra;
                        //    var y2_2 = y2_ra;
                        //    var z2_2 = z2_ra;

                        //    x3_ra = rightAnkle_X;
                        //    y3_ra = rightAnkle_Y;
                        //    z3_ra = rightAnkle_Z;

                        //    distance_2_ra = CalculateDistance(x2_2, y2_2, z2_2, x3_ra, y3_ra, z3_ra);

                        //    lblDistance_RA.Text = distance_2_ra.ToString("#.###") + " m";

                        //    vel_2_ra = distance_2_ra / 0.25;

                        //    lblVelocity_RA.Text = vel_2_ra.ToString("#.###") + " m/s";

                        //    //if (vel_2_ra >= 0.075)
                        //    //{

                        //    //    isActive3 = true;

                        //    //}
                        //    //else
                        //    //{
                        //    //    isActive3 = false;
                        //    //    strideVelocity = (distance_1_ra + distance_2_ra) / (timeCsec3 * 0.01);
                        //    //    lblStrideVelocity.Text = strideVelocity.ToString();
                        //    //    ResetStopwatch_3();

                        //    //}
                        //    //SaveData();

                        //    //display();
                        //    //SaveFile();
                        //    // WriteToFile();

                        //}
                        //if (timeCsec == 6)
                        //{
                        //    var x3_3 = x3_ra;
                        //    var y3_3 = y3_ra;
                        //    var z3_3 = z3_ra;

                        //    x4_ra = rightAnkle_X;
                        //    y4_ra = rightAnkle_Y;
                        //    z4_ra = rightAnkle_Z;

                        //    distance_3_ra = CalculateDistance(x3_3, y3_3, z3_3, x4_ra, y4_ra, z4_ra);

                        //    lblDistance_RA.Text = distance_3_ra.ToString("#.###") + " m";

                        //    vel_3_ra = distance_3_ra / 0.25;

                        //    lblVelocity_RA.Text = vel_3_ra.ToString("#.###") + " m/s";

                        //    //if (vel_3_ra >= 0.075)
                        //    //{

                        //    //    isActive3 = true;

                        //    //}
                        //    //else
                        //    //{
                        //    //    isActive3 = false;
                        //    //    strideVelocity = (distance_1_ra + distance_2_ra + distance_3_ra) / (timeCsec3 * 0.01);
                        //    //    lblStrideVelocity.Text = strideVelocity.ToString();
                        //    //    ResetStopwatch_3();

                        //    //}
                        //    //SaveData();

                        //    //display();
                        //    //SaveFile();
                        //    // WriteToFile();

                        //}

                        ////if (timeCsec == 4)
                        ////{
                        ////    var x4_4 = x4_ra;
                        ////    var y4_4 = y4_ra;
                        ////    var z4_4 = z4_ra;

                        ////    x5_ra = rightAnkle_X;
                        ////    y5_ra = rightAnkle_Y;
                        ////    z5_ra = rightAnkle_Z;

                        ////    distance_4_ra = CalculateDistance(x4_4, y4_4, z4_4, x5_ra, y5_ra, z5_ra);

                        ////    lblDistance_RA.Text = distance_4_ra.ToString("#.###") + " m";

                        ////    vel_4_ra = distance_4_ra / 0.125;

                        ////    lblVelocity_RA.Text = vel_4_ra.ToString("#.###") + " m/s";

                        //    //if (vel_4_ra >= 0.075)
                        //    //{

                        //    //    isActive3 = true;

                        //    //}
                        //    //else
                        //    //{
                        //    //    isActive3 = false;
                        //    //    strideVelocity = (distance_1_ra + distance_2_ra + distance_3_ra + distance_4_ra) / (timeCsec3 * 0.01);
                        //    //    lblStrideVelocity.Text = strideVelocity.ToString();
                        //    //    ResetStopwatch_3();

                        //    //}
                        //    //SaveData();

                        //    //display();
                        //    //SaveFile();
                        //    // WriteToFile();

                        ////}


                        ////if (timeCsec == 5)
                        ////{
                        ////    var x5_5 = x5_ra;
                        ////    var y5_5 = y5_ra;
                        ////    var z5_5 = z5_ra;

                        ////    x6_ra = rightAnkle_X;
                        ////    y6_ra = rightAnkle_Y;
                        ////    z6_ra = rightAnkle_Z;

                        ////    distance_5_ra = CalculateDistance(x5_5, y5_5, z5_5, x6_ra, y6_ra, z6_ra);

                        ////    lblDistance_RA.Text = distance_5_ra.ToString("#.###") + " m";

                        ////    vel_5_ra = distance_5_ra / 0.125;

                        ////    lblVelocity_RA.Text = vel_5_ra.ToString("#.###") + " m/s";

                        //    //if (vel_5_ra >= 0.075)
                        //    //{

                        //    //    isActive3 = true;

                        //    //}
                        //    //else
                        //    //{
                        //    //    isActive3 = false;
                        //    //    strideVelocity = (distance_1_ra + distance_2_ra + distance_3_ra + distance_4_ra + distance_5_ra) / (timeCsec3 * 0.01);
                        //    //    lblStrideVelocity.Text = strideVelocity.ToString();
                        //    //    ResetStopwatch_3();

                        //    //}
                        //    //SaveData();

                        //    //display();
                        //    //SaveFile();
                        //    // WriteToFile();

                        ////}

                        ////if (timeCsec == 6)
                        ////{
                        ////    var x6_6 = x6_ra;
                        ////    var y6_6 = y6_ra;
                        ////    var z6_6 = z6_ra;

                        ////    x7_ra = rightAnkle_X;
                        ////    y7_ra = rightAnkle_Y;
                        ////    z7_ra = rightAnkle_Z;

                        ////    distance_6_ra = CalculateDistance(x6_6, y6_6, z6_6, x7_ra, y7_ra, z7_ra);

                        ////    lblDistance_RA.Text = distance_6_ra.ToString("#.###") + " m";

                        ////    vel_6_ra = distance_6_ra / 0.125;

                        ////    lblVelocity_RA.Text = vel_6_ra.ToString("#.###") + " m/s";

                        //    //if (vel_6_ra >= 0.075)
                        //    //{

                        //    //    isActive3 = true;

                        //    //}
                        //    //else
                        //    //{
                        //    //    isActive3 = false;
                        //    //    strideVelocity = (distance_1_ra + distance_2_ra + distance_3_ra + distance_4_ra + distance_5_ra + distance_6_ra) / (timeCsec3 * 0.01);
                        //    //    lblStrideVelocity.Text = strideVelocity.ToString();
                        //    //    ResetStopwatch_3();

                        //    //}
                        //    //SaveData();

                        //    //display();
                        //    //SaveFile();
                        //    // WriteToFile();

                        ////}


                        ////if (timeCsec == 7)
                        ////{
                        ////    var x7_7 = x7_ra;
                        ////    var y7_7 = y7_ra;
                        ////    var z7_7 = z7_ra;

                        ////    x8_ra = rightAnkle_X;
                        ////    y8_ra = rightAnkle_Y;
                        ////    z8_ra = rightAnkle_Z;

                        ////    distance_7_ra = CalculateDistance(x7_7, y7_7, z7_7, x8_ra, y8_ra, z8_ra);

                        ////    lblDistance_RA.Text = distance_7_ra.ToString("#.###") + " m";

                        ////    vel_7_ra = distance_7_ra / 0.125;

                        ////    lblVelocity_RA.Text = vel_7_ra.ToString("#.###") + " m/s";

                        //    //if (vel_7_ra >= 0.075)
                        //    //{

                        //    //    isActive3 = true;

                        //    //}
                        //    //else
                        //    //{
                        //    //    isActive3 = false;
                        //    //    strideVelocity = (distance_1_ra + distance_2_ra + distance_3_ra + distance_4_ra + distance_5_ra + distance_6_ra + distance_7_ra) / (timeCsec3 * 0.01);
                        //    //    lblStrideVelocity.Text = strideVelocity.ToString();
                        //    //    ResetStopwatch_3();

                        //    //}
                        //    //SaveData();

                        //    //display();
                        //    //SaveFile();
                        //    // WriteToFile();

                        ////}


                        //if (timeCsec > 6)
                        //{
                        //    distance_ra = distance_1_ra + distance_2_ra + distance_3_ra /*+ distance_4_ra + distance_5_ra + distance_6_ra + distance_7_ra*/;
                        //    gaitSpeed_ra = (vel_1_ra + vel_2_ra + vel_3_ra /*+ vel_4_ra + vel_5_ra + vel_6_ra + vel_7_ra*/) / 3;
                        //    aceleration_ra = (vel_3_ra - vel_1_ra) / 0.5;


                        //    lblDistance_RA.Text = distance_ra.ToString("#.###") + " m";
                        //    lblVelocity_RA.Text = gaitSpeed_ra.ToString("#.###") + " m/s";
                        //    lblAcceleration_RA.Text = aceleration_ra.ToString("#.###") + " m/s2";
                        //    SaveInTable_N_displayInTxtbox();

                        //    //if (vel_7_ra >= 0.075)
                        //    //{

                        //    //    isActive3 = true;

                        //    //}
                        //    //else
                        //    //{
                        //    //    isActive3 = false;
                        //    //    strideVelocity = (distance_1_ra + distance_2_ra + distance_3_ra + distance_4_ra + distance_5_ra + distance_6_ra + distance_7_ra) / (timeCsec3 * 0.01);
                        //    //    lblStrideVelocity.Text = strideVelocity.ToString();
                        //    //    ResetStopwatch_3();

                        //    //}

                        //}


                    }


                }

            }

        }


                

        //Data storage functions ------  2.2  ----------------------  2.2  ----------------------  2.2  -----------------------  2.2  -------------------------  2.2  -------------------------  2.2  -----------------------  2.2  --------------------
        string sums, Averages, Differnce2s, Standard_Devs, CVs = "";
        public void SaveInTable_N_displayInTxtbox() //Save data into database and display data in main textbox
        {
            
            if ((distance_ms >= 0.1 && gaitSpeed_ms >= 0.27 && aceleration_ms < 0.35) && (distance_ms >= 0.1 && gaitSpeed_ms >= 0.27 && aceleration_ms > -0.35) && (distance_ms >= 0.1 && gaitSpeed_ms < 2.0 && aceleration_ms < 0.35) && (distance_ms >= 0.1 && gaitSpeed_ms < 2.0 && aceleration_ms > -0.35) && lblUserName.Text == "Emmanuel")
            {                             
                //Display data in main textbox
                txtTemp.Text += String.Format("{0:0.000}", distance_ms) + "               " + String.Format("{0:0.000}", gaitSpeed_ms) + "            " + String.Format("{0:0.000}", aceleration_ms) + "            " + lblCurrentDate.Text + "            " + lblCurrentTime.Text + Environment.NewLine; //Original code

                //Display data for ankles
                txtTemp_Ankles.Text += String.Format("{0:0.000}", distance_ra) + "               " + String.Format("{0:0.000}", gaitSpeed_ra) + "            " + String.Format("{0:0.000}", aceleration_ra) + "            " + lblCurrentDate.Text + "            " + lblCurrentTime.Text + Environment.NewLine;


                //Save and display data into database
                dbConnection.Open();
                SqlCommand command1 = dbConnection.CreateCommand();
                command1.CommandType = CommandType.Text;
                command1.CommandText = "insert COM_Data values('" + String.Format("{0:0.00000000000000000000}", distance_ms) + "','" + String.Format("{0:0.000}", gaitSpeed_ms) + "','" + String.Format("{0:0.000}", aceleration_ms) + "','" + lblCurrentDate.Text + "','" + lblCurrentTime.Text + "','" + counter2 + "','" + sums + "','" + Averages + "','" + Differnce2s + "','" + Standard_Devs + "','" + DBNull.Value + "')";
                command1.ExecuteNonQuery();
                dbConnection.Close();

            }
            
        }

        public void SaveInTable_N_displayInTxtbox_Ankles() //Save data into database and display data in main textbox
        {

            //if ((distance_ra >= 0.1 && gaitSpeed_ra >= 0.27 && aceleration_ra < 0.35) && (distance_ra >= 0.1 && gaitSpeed_ra >= 0.27 && aceleration_ra > -0.35))
            //{
            //   //Display data for ankles
            //   txtTemp_Ankles.Text += String.Format("{0:0.000}", distance_ra) + "               " + String.Format("{0:0.000}", gaitSpeed_ra) + "            " + String.Format("{0:0.000}", aceleration_ra) + "            " + currentTime1.ToString("MM/dd/yyyy") + "            " + lblCurrentTime.Text + Environment.NewLine;

               
            //    //Save and display data into database
            //    //dbConnection.Open();
            //    //SqlCommand command1 = dbConnection.CreateCommand();
            //    //command1.CommandType = CommandType.Text;
            //    //command1.CommandText = "insert COM_Data values('" + String.Format("{0:0.00000000000000000000}", distance_ms) + "','" + String.Format("{0:0.000}", gaitSpeed_ms) + "','" + String.Format("{0:0.000}", aceleration_ms) + "','" + currentTime1.ToString("MM/dd/yyyy") + "','" + lblCurrentTime.Text + "','" + counter2 + "','" + sums + "','" + Averages + "','" + Differnce2s + "','" + Standard_Devs + "','" + DBNull.Value + "')";
            //    //command1.ExecuteNonQuery();
            //    //dbConnection.Close();

            //}


        }

        public void SaveFile()  //Save data from main textbox into text file
        {

            using (StreamWriter sw = new StreamWriter(path1))
            {
                //textBoxStatus.AppendText(text + Environment.NewLine);
                //txtStatus.Text += "Line 1" + Environment.NewLine;

                //if ((distance >= 0.1 && gaitSpeed >= 0.27 && aceleration < 0.2) && (distance >= 0.1 && gaitSpeed >= 0.27 && aceleration > -0.2))
                //{
                sw.WriteLine(txtDisplayTable.Text, path1);
                //sw.WriteLine(distance.ToString("#.###") + "                       " + gaitSpeed.ToString("#.###"));
                //}
            }

        }

        

      //Calculation functions ------  2.3  ----------------------  2.3  ----------------------  2.3  -----------------------  2.3  -------------------------  2.3  -------------------------  2.3  -----------------------  2.3  --------------------
        public static double CalculateDistance(double X1, double Y1, double Z1, double X2, double Y2, double Z2)    //Compute distance between two ponits in 3D space
        {
            double delta_xSq = Math.Pow((X2 - X1), 2);

            double delta_ySq = Math.Pow((Y2 - Y1), 2);

            double delta_zSq = Math.Pow((Z2 - Z1), 2);

            var distanceResults = Math.Sqrt(delta_xSq + delta_ySq + delta_zSq);
            return distanceResults;
        }


        //Local variables for summing the CVs
        int rowNum4 = 0;
        int cellNum4 = 0;
        double sumCV = 0;
        int cvCounter = 0;
        private void CalculateAllCVs()  //Calculate Average Velocities and CVs, display in data-grid-view 3
        {
            //Display SQL velocities and CVs in data-grid-view 2
            dbConnection.Open();
            SqlCommand command4 = dbConnection.CreateCommand();
            command4.CommandType = CommandType.Text;
            command4.CommandText = "SELECT Avg(COM_Data.Velocity) AS AvgOfVelocity, StDev(COM_Data.Velocity) AS StDevOfVelocity, COM_Data.Distincter, (StDev(COM_Data.Velocity)/Avg(COM_Data.Velocity))*100 AS CV FROM COM_Data GROUP BY COM_Data.Distincter; ";
            command4.ExecuteNonQuery();
            DataTable dTable5 = new DataTable();
            SqlDataAdapter dbAdapter5 = new SqlDataAdapter(command4);
            dbAdapter5.Fill(dTable5);
            dataGridView2.DataSource = dTable5;
            dbConnection.Close();

            //Display Empty column(i.e., Sum) in DataGridView 3 to be able to display derived table
            dbConnection.Open();
            SqlCommand command5 = dbConnection.CreateCommand();
            command5.CommandType = CommandType.Text;
            command5.CommandText = "Select COM_Data.Sum AS Remarks from COM_Data";
            command5.ExecuteNonQuery();
            DataTable dTable6 = new DataTable();
            SqlDataAdapter dbAdapter6 = new SqlDataAdapter(command5);
            dbAdapter6.Fill(dTable6);
            dataGridView3.DataSource = dTable6;
            dbConnection.Close();


            //Looping to number the rows
            int cellNum1 = 0;
            int rowNum1 = 0;

            foreach (DataGridViewRow row in dataGridView1.Rows)
            {
                cellNum1 = cellNum1 + 1;
                dataGridView1.Rows[rowNum1].Cells[6].Value = cellNum1;
                rowNum1 = rowNum1 + 1;
            }

            //Local variables for Group numbering
            int cellNum2 = 1;
            int rowNum2 = 0;
            int counter2 = 0;

            //Local variables for calculating Sum and Average for the Groups
            int cellNum3 = 0;
            int rowNum3 = 0;
            int counter3 = 0;
            double groupSum = 0;
            double average = 0;
            int divisor = 0;
            double diff = 0;
            double diff2Sum = 0;
            double groupStd = 0;
            double groupCV = 0;

            ////Local variables for summing the CVs
            //int cellNum4 = 0;
            ////int rowNum4 = 0;
            //double sumCV = 0;
            //int cvCounter = 0;

            ////Local variables calculating Average and Standard Deviation of CVs.
            //int cellNum5 = 0;
            //int rowNum5 = 0;
            //double cvSum = 0;
            //double AverageCV = 0;
            //double diffCV = 0;

            //For Group numbering
            foreach (DataGridViewRow row in dataGridView1.Rows)
            {
                counter2++;
                if (counter2 >= 7)
                {
                    cellNum2 = cellNum2 + 1;
                    counter2 = 1;
                }
                dataGridView1.Rows[rowNum2].Cells[6].Value = cellNum2;
                rowNum2 = rowNum2 + 1;
            }


            //Looping for Calculating the Sum, Average, Standard Deviation, and CV for the Groups
            foreach (DataGridViewRow row in dataGridView1.Rows)
            {
                counter3 = counter3 + 6;

                for (int k = cellNum3; k < counter3; k++)
                {


                    divisor++;
                    groupSum += Convert.ToDouble(dataGridView1.Rows[rowNum3].Cells[2].Value);

                    if (divisor == 6)
                    {

                        cvCounter++;
                        dataGridView1.Rows[rowNum3].Cells[7].Value = groupSum;
                        average = Math.Round(groupSum / divisor, 3);
                        dataGridView3.Rows[rowNum4].Cells[0].Value = rowNum4 + 1;                                 //Group Numbering for Derived Table

                        //For last row in the Group
                        dataGridView1.Rows[rowNum3].Cells[8].Value = average;                                     //Average stored into this column
                        dataGridView3.Rows[rowNum4].Cells[3].Value = average;
                        dataGridView3.Rows[rowNum4].Cells[1].Value = dataGridView1.Rows[rowNum3].Cells[4].Value;  //Date for Derived table
                        dataGridView3.Rows[rowNum4].Cells[2].Value = dataGridView1.Rows[rowNum3].Cells[5].Value;  //Time for Derived table
                        diff = Convert.ToDouble(dataGridView1.Rows[rowNum3].Cells[2].Value) - average;
                        dataGridView1.Rows[rowNum3].Cells[9].Value = Math.Round(Math.Pow(diff, 2), 6);
                        diff2Sum += Convert.ToDouble(dataGridView1.Rows[rowNum3].Cells[9].Value);

                        //For last 5th row in the Group
                        // dataGridView1.Rows[rowNum3 - 1].Cells[9].Value = average;
                        diff = Convert.ToDouble(dataGridView1.Rows[rowNum3 - 1].Cells[2].Value) - average;
                        dataGridView1.Rows[rowNum3 - 1].Cells[9].Value = Math.Round(Math.Pow(diff, 2), 6);
                        diff2Sum += Convert.ToDouble(dataGridView1.Rows[rowNum3 - 1].Cells[9].Value);

                        //For last 4th row in the Group
                        //dataGridView1.Rows[rowNum3 - 2].Cells[9].Value = average;
                        diff = Convert.ToDouble(dataGridView1.Rows[rowNum3 - 2].Cells[2].Value) - average;
                        dataGridView1.Rows[rowNum3 - 2].Cells[9].Value = Math.Round(Math.Pow(diff, 2), 6);
                        diff2Sum += Convert.ToDouble(dataGridView1.Rows[rowNum3 - 2].Cells[9].Value);

                        //For last 3rd row in the Group
                        //dataGridView1.Rows[rowNum3 - 3].Cells[9].Value = average;
                        diff = Convert.ToDouble(dataGridView1.Rows[rowNum3 - 3].Cells[2].Value) - average;
                        dataGridView1.Rows[rowNum3 - 3].Cells[9].Value = Math.Round(Math.Pow(diff, 2), 6);
                        diff2Sum += Convert.ToDouble(dataGridView1.Rows[rowNum3 - 3].Cells[9].Value);

                        //For last 2nd row in the Group
                        //dataGridView1.Rows[rowNum3 - 4].Cells[9].Value = average;
                        diff = Convert.ToDouble(dataGridView1.Rows[rowNum3 - 4].Cells[2].Value) - average;
                        dataGridView1.Rows[rowNum3 - 4].Cells[9].Value = Math.Round(Math.Pow(diff, 2), 6);
                        diff2Sum += Convert.ToDouble(dataGridView1.Rows[rowNum3 - 4].Cells[9].Value);

                        //For last 1st row in the Group
                        //dataGridView1.Rows[rowNum3 - 5].Cells[9].Value = average;
                        diff = Convert.ToDouble(dataGridView1.Rows[rowNum3 - 5].Cells[2].Value) - average;
                        dataGridView1.Rows[rowNum3 - 5].Cells[9].Value = Math.Round(Math.Pow(diff, 2), 6);

                        //These are computed Sum of the difference-squared, the Standard Deviation and CV for every six consecutive velocities
                        diff2Sum += Convert.ToDouble(dataGridView1.Rows[rowNum3 - 5].Cells[9].Value);
                        groupStd = Math.Round(Math.Sqrt(diff2Sum / 5), 6);
                        groupCV = Math.Round((groupStd / average) * 100, 3);

                        //Values of Standard Deviation and CV stored in the respective columns respectively
                        dataGridView1.Rows[rowNum3].Cells[10].Value = groupStd;                                 //Standard Deviation stored into this column
                        dataGridView3.Rows[rowNum4].Cells[4].Value = groupStd;                                  //Standard Deviation for Derived Table

                        dataGridView1.Rows[rowNum3].Cells[11].Value = groupCV;                                  //CV stored into this column
                        dataGridView3.Rows[rowNum4].Cells[5].Value = groupCV;                                   //CV for Derived Table

                        //Computing the Average of the CV
                        sumCV += Convert.ToDouble(dataGridView1.Rows[rowNum3].Cells[11].Value);
                        txtStatus.Text = (sumCV / cvCounter).ToString();

                        rowNum4++;

                        //refreshTable = true;

                        if (rowNum4 >= dataGridView1.Rows.Count / 6)
                        {

                            cellNum5 = rowNum4;
                            rowNum4 = 0;

                            //Calculate and display pecentiles of CVs
                            //CalcNDisp5thN95thPercentile();


                            return;
                        }


                        //It is DONE!
                    }

                    diff2Sum = 0;
                    rowNum3 = rowNum3 + 1;

                }

                cellNum3 = counter3;
                groupSum = 0;
                divisor = 0;

                if (cellNum3 + 6 > dataGridView1.Rows.Count)
                {
                    break;
                }


            }
        }


        int cellNum5 = 0;
        private void CalcNDisp5thN95thPercentile()  //Compute the mean and Standard deviation for all CVs for evaluating the 5th and 95th percentile
        {
            int rowNum5 = 0;
            double cvSum = 0;
            double AverageCV = 0;
            double sumDiffCV2 = 0;
            double stdCV = 0;
            int cvDivisor = 0;
            double Percentile_5thCV = 0;
            double Percentile_95thCV = 0;

            foreach (DataGridViewRow row in dataGridView3.Rows)
            {
                cvDivisor++;

                cvSum += Convert.ToDouble(dataGridView3.Rows[rowNum5].Cells[5].Value);
                // lbl5thPercentile.Text = cvSum.ToString();

                AverageCV = Math.Round(cvSum / (cvDivisor), 4);
                // lbl95thPercentile.Text = AverageCV.ToString();

                rowNum5++;


                if (rowNum5 >= cellNum5)
                {
                    for (int i = 0; i < rowNum5; i++)
                    {
                        sumDiffCV2 += Math.Pow((Convert.ToDouble(dataGridView3.Rows[i].Cells[5].Value) - AverageCV), 2);
                        stdCV = Math.Round(Math.Sqrt(sumDiffCV2 / (cvDivisor - 1)), 4);
                        lblDisplayCVAveNStd.Text = "Ave CV: " + AverageCV.ToString() + ",  Std CV: " + stdCV.ToString();

                        Percentile_5thCV = Math.Round(AverageCV - (1.64485 * stdCV), 2);
                        Percentile_95thCV = Math.Round(AverageCV + (1.64485 * stdCV), 2);

                        lbl5thPercentile.Text = Percentile_5thCV.ToString() + " %";
                        lbl95thPercentile.Text = Percentile_95thCV.ToString() + " %";
                    }

                    break;
                }
            }
        }


        double aveOverallGaitSpeed;
        double aveOverallCV;
        public void Current_Session_Of_Day()  //Filter out the Gait-speeds and CVs for morning, afternoon and evening of the current day
        {
            int rowNum5 = 0;

            DateTime TD = DateTime.Now;

            var InitialMorning = Convert.ToDateTime("12:00:00 AM");
            var FinalMorning = Convert.ToDateTime("11:59:59 AM");

            var InitialAfternoon = Convert.ToDateTime("12:00:00 PM");
            var FinalAfternoon = Convert.ToDateTime("05:29:59 PM");

            var InitialEvening = Convert.ToDateTime("5:30:00 PM");
            var FinalEvening = Convert.ToDateTime("11:59:59 PM");

            var TodayDate = Convert.ToDateTime(TD.ToShortDateString());

            int morningDivisor = 0;
            int afternoonDivisor = 0;
            int eveningDivisor = 0;
            int overallDivisor = 0;


            double sumMorningCV = 0;
            double sumAfternoonCV = 0;
            double sumEveningCV = 0;
            double sumOverallCV = 0;

            double sumMorningGaitSpeed = 0;
            double sumAfternoonGaitSpeed = 0;
            double sumEveningGaitSpeed = 0;
            double sumOverallGaitSpeed = 0;

            double aveMorningCV;
            double aveAfternoonCV;
            double aveEveningCV;
            // double aveOverallCV;

            double aveMorningGaitSpeed;
            double aveAfternoonGaitSpeed;
            double aveEveningGaitSpeed;
            // double aveOverallGaitSpeed;

            foreach (DataGridViewRow row in dataGridView3.Rows)
            {

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum5].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum5].Cells[2].Value) <= FinalMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum5].Cells[1].Value) == TodayDate)
                {
                    morningDivisor++;
                    //Morning Average CV Calculation
                    sumMorningCV += Convert.ToDouble(dataGridView3.Rows[rowNum5].Cells[5].Value);
                    aveMorningCV = Math.Round(sumMorningCV / morningDivisor, 2);
                    lblMorningAveCV.Text = aveMorningCV.ToString() + " %";

                    //Morning Average Gait Speed Calculation
                    sumMorningGaitSpeed += Convert.ToDouble(dataGridView3.Rows[rowNum5].Cells[3].Value);
                    aveMorningGaitSpeed = Math.Round(sumMorningGaitSpeed / morningDivisor, 2);
                    lblMorningAveGaitSpeed.Text = aveMorningGaitSpeed.ToString() + " m/s";
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum5].Cells[2].Value) >= InitialAfternoon && Convert.ToDateTime(dataGridView3.Rows[rowNum5].Cells[2].Value) <= FinalAfternoon && Convert.ToDateTime(dataGridView3.Rows[rowNum5].Cells[1].Value) == TodayDate)
                {
                    afternoonDivisor++;
                    //Afternoon Average CV Calculation
                    sumAfternoonCV += Convert.ToDouble(dataGridView3.Rows[rowNum5].Cells[5].Value);
                    aveAfternoonCV = Math.Round(sumAfternoonCV / afternoonDivisor, 2);
                    lblAfternoonAveCV.Text = aveAfternoonCV.ToString() + " %";

                    //Afternoon Average Gait Speed Calculation
                    sumAfternoonGaitSpeed += Convert.ToDouble(dataGridView3.Rows[rowNum5].Cells[3].Value);
                    aveAfternoonGaitSpeed = Math.Round(sumAfternoonGaitSpeed / afternoonDivisor, 2);
                    lblAfternoonAveGaitSpeed.Text = aveAfternoonGaitSpeed.ToString() + " m/s";
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum5].Cells[2].Value) >= InitialEvening && Convert.ToDateTime(dataGridView3.Rows[rowNum5].Cells[2].Value) <= FinalEvening && Convert.ToDateTime(dataGridView3.Rows[rowNum5].Cells[1].Value) == TodayDate)
                {
                    eveningDivisor++;
                    //Evening Average CV Calculation
                    sumEveningCV += Convert.ToDouble(dataGridView3.Rows[rowNum5].Cells[5].Value);
                    aveEveningCV = Math.Round(sumEveningCV / eveningDivisor, 2);
                    lblEveningAveCV.Text = aveEveningCV.ToString() + " %";

                    //Evening Average Gait Speed Calculation
                    sumEveningGaitSpeed += Convert.ToDouble(dataGridView3.Rows[rowNum5].Cells[3].Value);
                    aveEveningGaitSpeed = Math.Round(sumEveningGaitSpeed / eveningDivisor, 2);
                    lblEveningAveGaitSpeed.Text = aveEveningGaitSpeed.ToString() + " m/s";
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum5].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum5].Cells[2].Value) <= FinalEvening && Convert.ToDateTime(dataGridView3.Rows[rowNum5].Cells[1].Value) == TodayDate)
                {
                    overallDivisor++;
                    //Overall Average CV Calculation
                    sumOverallCV += Convert.ToDouble(dataGridView3.Rows[rowNum5].Cells[5].Value);
                    aveOverallCV = Math.Round(sumOverallCV / overallDivisor, 2);
                    lblDailyAveCV.Text = aveOverallCV.ToString() + " %";

                    //Overall Average Gait Speed Calculation
                    sumOverallGaitSpeed += Convert.ToDouble(dataGridView3.Rows[rowNum5].Cells[3].Value);
                    aveOverallGaitSpeed = Math.Round(sumOverallGaitSpeed / overallDivisor, 2);
                    lblOverallAveGaitSpeed.Text = aveOverallGaitSpeed.ToString() + " m/s";
                }

                rowNum5++;

                if (rowNum5 > dataGridView3.Rows.Count)
                {
                    rowNum5 = 0;
                    break;
                }
            }

        }


        int arrayCounter = 0;
        public void Weekly_Session_Of_Day()  //Filter out the Gait-speeds and CVs for morning, afternoon and evening of the various days of the current week
        {
            //Local variables for sessions of day
            int rowNum8 = 0;

            DateTime TD = DateTime.Now;
            DateTime toda_y = DateTime.Now;

            var InitialMorning = Convert.ToDateTime("12:00:00 AM");
            var FinalMorning = Convert.ToDateTime("11:59:59 AM");

            var InitialAfternoon = Convert.ToDateTime("12:00:00 PM");
            var FinalAfternoon = Convert.ToDateTime("05:29:59 PM");

            var InitialEvening = Convert.ToDateTime("5:30:00 PM");
            var FinalEvening = Convert.ToDateTime("11:59:59 PM");

            var TodayDate = Convert.ToDateTime(TD.ToShortDateString());

            int morningDivisor = 0;
            int afternoonDivisor = 0;
            int eveningDivisor = 0;
            int overallDivisor = 0;

            double sumMorningCV = 0;
            double sumAfternoonCV = 0;
            double sumEveningCV = 0;
            double sumOverallCV = 0;

            double aveMorningCV = 0;
            double aveAfternoonCV = 0;
            double aveEveningCV = 0;
            double aveOverallCV = 0;

            //Retreiving CVs for the various sessions of day fot Sunday *********************************************************************************************

            foreach (DataGridViewRow row in dataGridView3.Rows)
            {

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalMorning &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Sunday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    morningDivisor++;
                    sumMorningCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveMorningCV = Math.Round(sumMorningCV / morningDivisor, 2);
                    dataGridView5.Rows[0].Cells[1].Value = aveMorningCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialAfternoon && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalAfternoon &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Sunday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    afternoonDivisor++;
                    sumAfternoonCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveAfternoonCV = Math.Round(sumAfternoonCV / afternoonDivisor, 2);
                    dataGridView5.Rows[0].Cells[2].Value = aveAfternoonCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialEvening && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalEvening &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Sunday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    eveningDivisor++;
                    sumEveningCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveEveningCV = Math.Round(sumEveningCV / eveningDivisor, 2);
                    dataGridView5.Rows[0].Cells[3].Value = aveEveningCV;

                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalEvening &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Sunday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    //dataGridView6.Rows[overallDivisor].Cells[0].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in SundayTime column
                    dataGridView7.Rows[overallDivisor].Cells[0].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in SundayTime column

                    //dataGridView6.Rows[overallDivisor].Cells[1].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);   //Display Gait Speed in SundayGS column
                    dataGridView7.Rows[overallDivisor].Cells[1].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);   //Display Gait Speed in SundayGS column

                    //dataGridView6.Rows[overallDivisor].Cells[2].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);   //Display CVs in SundayCVs column
                    dataGridView7.Rows[overallDivisor].Cells[2].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);   //Display CVs in SundayCVs column
                    overallDivisor++;

                }

                rowNum8++;

                if (rowNum8 >= dataGridView3.Rows.Count)
                {
                    morningDivisor = 0;
                    afternoonDivisor = 0;
                    eveningDivisor = 0;
                    overallDivisor = 0;

                    sumMorningCV = 0;
                    sumAfternoonCV = 0;
                    sumEveningCV = 0;

                    aveMorningCV = 0;
                    aveAfternoonCV = 0;
                    aveEveningCV = 0;

                    rowNum8 = 0;
                    break;
                }

            }


            //Retreiving CVs for the various sessions of day fot Monday *********************************************************************************************

            foreach (DataGridViewRow row in dataGridView3.Rows)
            {

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalMorning &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Monday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    morningDivisor++;
                    sumMorningCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveMorningCV = Math.Round(sumMorningCV / morningDivisor, 2);
                    dataGridView5.Rows[1].Cells[1].Value = aveMorningCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialAfternoon && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalAfternoon &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Monday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    afternoonDivisor++;
                    sumAfternoonCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveAfternoonCV = Math.Round(sumAfternoonCV / afternoonDivisor, 2);
                    dataGridView5.Rows[1].Cells[2].Value = aveAfternoonCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialEvening && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalEvening &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Monday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    eveningDivisor++;
                    sumEveningCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveEveningCV = Math.Round(sumEveningCV / eveningDivisor, 2);
                    dataGridView5.Rows[1].Cells[3].Value = aveEveningCV;
                   
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalEvening &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Monday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    //dataGridView6.Rows[overallDivisor].Cells[3].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in MondayTime column
                    dataGridView7.Rows[overallDivisor].Cells[3].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in MondayTime column

                    //dataGridView6.Rows[overallDivisor].Cells[4].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);               //Display Gait Speed in MondayGS column
                    dataGridView7.Rows[overallDivisor].Cells[4].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);               //Display Gait Speed in MondayGS column

                    //dataGridView6.Rows[overallDivisor].Cells[5].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);               //Display CVs in MondayCVs column
                    dataGridView7.Rows[overallDivisor].Cells[5].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);               //Display CVs in MondayCVs column

                    overallDivisor++;
                }

                rowNum8++;

                if (rowNum8 >= dataGridView3.Rows.Count)
                {
                    morningDivisor = 0;
                    afternoonDivisor = 0;
                    eveningDivisor = 0;
                    overallDivisor = 0;

                    sumMorningCV = 0;
                    sumAfternoonCV = 0;
                    sumEveningCV = 0;

                    aveMorningCV = 0;
                    aveAfternoonCV = 0;
                    aveEveningCV = 0;

                    rowNum8 = 0;
                    break;
                }

            }


            //Retreiving CVs for the various sessions of day fot Tuesday *********************************************************************************************

            foreach (DataGridViewRow row in dataGridView3.Rows)
            {

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalMorning &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Tuesday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    morningDivisor++;
                    sumMorningCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveMorningCV = Math.Round(sumMorningCV / morningDivisor, 2);
                    dataGridView5.Rows[2].Cells[1].Value = aveMorningCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialAfternoon && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalAfternoon &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Tuesday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    afternoonDivisor++;
                    sumAfternoonCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveAfternoonCV = Math.Round(sumAfternoonCV / afternoonDivisor, 2);
                    dataGridView5.Rows[2].Cells[2].Value = aveAfternoonCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialEvening && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalEvening &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Tuesday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    eveningDivisor++;
                    sumEveningCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveEveningCV = Math.Round(sumEveningCV / eveningDivisor, 2);
                    dataGridView5.Rows[2].Cells[3].Value = aveEveningCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalEvening &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Tuesday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    //dataGridView6.Rows[overallDivisor].Cells[6].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in TuesdayTime column
                    dataGridView7.Rows[overallDivisor].Cells[6].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in TuesdayTime column

                    //dataGridView6.Rows[overallDivisor].Cells[7].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);               //Display Gait Speed in TuesdayGS column
                    dataGridView7.Rows[overallDivisor].Cells[7].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);               //Display Gait Speed in TuesdayGS column

                    //dataGridView6.Rows[overallDivisor].Cells[8].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);               //Display CVs in TuesdayCVs column
                    dataGridView7.Rows[overallDivisor].Cells[8].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);               //Display CVs in TuesdayCVs column

                    overallDivisor++;
                }

                rowNum8++;

                if (rowNum8 >= dataGridView3.Rows.Count)
                {
                    morningDivisor = 0;
                    afternoonDivisor = 0;
                    eveningDivisor = 0;
                    overallDivisor = 0;

                    sumMorningCV = 0;
                    sumAfternoonCV = 0;
                    sumEveningCV = 0;

                    aveMorningCV = 0;
                    aveAfternoonCV = 0;
                    aveEveningCV = 0;

                    rowNum8 = 0;
                    break;
                }

            }


            //Retreiving CVs for the various sessions of day fot Wednesday *********************************************************************************************

            foreach (DataGridViewRow row in dataGridView3.Rows)
            {

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalMorning &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Wednesday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    morningDivisor++;
                    sumMorningCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveMorningCV = Math.Round(sumMorningCV / morningDivisor, 2);
                    dataGridView5.Rows[3].Cells[1].Value = aveMorningCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialAfternoon && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalAfternoon &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Wednesday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    afternoonDivisor++;
                    sumAfternoonCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveAfternoonCV = Math.Round(sumAfternoonCV / afternoonDivisor, 2);
                    dataGridView5.Rows[3].Cells[2].Value = aveAfternoonCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialEvening && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalEvening &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Wednesday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    eveningDivisor++;
                    sumEveningCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveEveningCV = Math.Round(sumEveningCV / eveningDivisor, 2);
                    dataGridView5.Rows[3].Cells[3].Value = aveEveningCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalEvening &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Wednesday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    //dataGridView6.Rows[overallDivisor].Cells[9].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in WednesdayTime column
                    dataGridView7.Rows[overallDivisor].Cells[9].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in WednesdayTime column

                    //dataGridView6.Rows[overallDivisor].Cells[10].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);               //Display Gait Speed in WednesdayGS column
                    dataGridView7.Rows[overallDivisor].Cells[10].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);               //Display Gait Speed in WednesdayGS column

                    //dataGridView6.Rows[overallDivisor].Cells[11].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);               //Display CVs in WednesdayCVs column
                    dataGridView7.Rows[overallDivisor].Cells[11].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);               //Display CVs in WednesdayCVs column

                    overallDivisor++;
                }

                rowNum8++;

                if (rowNum8 >= dataGridView3.Rows.Count)
                {
                    morningDivisor = 0;
                    afternoonDivisor = 0;
                    eveningDivisor = 0;
                    overallDivisor = 0;


                    sumMorningCV = 0;
                    sumAfternoonCV = 0;
                    sumEveningCV = 0;

                    aveMorningCV = 0;
                    aveAfternoonCV = 0;
                    aveEveningCV = 0;

                    rowNum8 = 0;
                    break;
                }

            }


            //Retreiving CVs for the various sessions of day for Thursday *********************************************************************************************

            foreach (DataGridViewRow row in dataGridView3.Rows)
            {

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalMorning &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Thursday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    morningDivisor++;
                    sumMorningCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveMorningCV = Math.Round(sumMorningCV / morningDivisor, 2);
                    dataGridView5.Rows[4].Cells[1].Value = aveMorningCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialAfternoon && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalAfternoon &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Thursday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    afternoonDivisor++;
                    sumAfternoonCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveAfternoonCV = Math.Round(sumAfternoonCV / afternoonDivisor, 2);
                    dataGridView5.Rows[4].Cells[2].Value = aveAfternoonCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialEvening && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalEvening &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Thursday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    eveningDivisor++;
                    sumEveningCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveEveningCV = Math.Round(sumEveningCV / eveningDivisor, 2);
                    dataGridView5.Rows[4].Cells[3].Value = aveEveningCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalEvening &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Thursday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    //dataGridView6.Rows[overallDivisor].Cells[12].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in ThursdayTime column
                    dataGridView7.Rows[overallDivisor].Cells[12].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in ThursdayTime column

                    //dataGridView6.Rows[overallDivisor].Cells[13].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);               //Display Gait Speed in ThursdayGS column
                    dataGridView7.Rows[overallDivisor].Cells[13].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);               //Display Gait Speed in ThursdayGS column

                    //dataGridView6.Rows[overallDivisor].Cells[14].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);               //Display CVs in ThursdayCVs column
                    dataGridView7.Rows[overallDivisor].Cells[14].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);               //Display CVs in ThursdayCVs column

                    overallDivisor++;
                }

                rowNum8++;

                if (rowNum8 >= dataGridView3.Rows.Count)
                {
                    morningDivisor = 0;
                    afternoonDivisor = 0;
                    eveningDivisor = 0;
                    overallDivisor = 0;

                    sumMorningCV = 0;
                    sumAfternoonCV = 0;
                    sumEveningCV = 0;

                    aveMorningCV = 0;
                    aveAfternoonCV = 0;
                    aveEveningCV = 0;

                    rowNum8 = 0;
                    break;
                }

            }


            //Retreiving CVs for the various sessions of day for Friday *********************************************************************************************

            foreach (DataGridViewRow row in dataGridView3.Rows)
            {

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalMorning &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Friday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    morningDivisor++;
                    sumMorningCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveMorningCV = Math.Round(sumMorningCV / morningDivisor, 2);
                    dataGridView5.Rows[5].Cells[1].Value = aveMorningCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialAfternoon && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalAfternoon &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Friday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    afternoonDivisor++;
                    sumAfternoonCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveAfternoonCV = Math.Round(sumAfternoonCV / afternoonDivisor, 2);
                    dataGridView5.Rows[5].Cells[2].Value = aveAfternoonCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialEvening && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalEvening &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Friday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    eveningDivisor++;
                    sumEveningCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveEveningCV = Math.Round(sumEveningCV / eveningDivisor, 2);
                    dataGridView5.Rows[5].Cells[3].Value = aveEveningCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalEvening &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Friday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    //dataGridView6.Rows[overallDivisor].Cells[15].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in FridayTime column
                    dataGridView7.Rows[overallDivisor].Cells[15].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in FridayTime column

                    //dataGridView6.Rows[overallDivisor].Cells[16].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);               //Display Gait Speed in FridayGS column
                    dataGridView7.Rows[overallDivisor].Cells[16].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);               //Display Gait Speed in FridayGS column

                    //dataGridView6.Rows[overallDivisor].Cells[17].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);               //Display CVs in FridayCVs column
                    dataGridView7.Rows[overallDivisor].Cells[17].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);               //Display CVs in FridayCVs column

                    overallDivisor++;
                }

                rowNum8++;

                if (rowNum8 >= dataGridView3.Rows.Count)
                {
                    morningDivisor = 0;
                    afternoonDivisor = 0;
                    eveningDivisor = 0;
                    overallDivisor = 0;

                    sumMorningCV = 0;
                    sumAfternoonCV = 0;
                    sumEveningCV = 0;

                    aveMorningCV = 0;
                    aveAfternoonCV = 0;
                    aveEveningCV = 0;

                    rowNum8 = 0;
                    break;
                }
            }


            //Retreiving CVs for the various sessions of day fot Saturday *********************************************************************************************

            foreach (DataGridViewRow row in dataGridView3.Rows)
            {

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalMorning &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Saturday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    morningDivisor++;
                    sumMorningCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveMorningCV = Math.Round(sumMorningCV / morningDivisor, 2);
                    dataGridView5.Rows[6].Cells[1].Value = aveMorningCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialAfternoon && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalAfternoon &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Saturday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    afternoonDivisor++;
                    sumAfternoonCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveAfternoonCV = Math.Round(sumAfternoonCV / afternoonDivisor, 2);
                    dataGridView5.Rows[6].Cells[2].Value = aveAfternoonCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialEvening && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalEvening &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Saturday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    eveningDivisor++;
                    sumEveningCV += Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);
                    aveEveningCV = Math.Round(sumEveningCV / eveningDivisor, 2);
                    dataGridView5.Rows[6].Cells[3].Value = aveEveningCV;
                }

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalEvening &&
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Saturday" && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    //dataGridView6.Rows[overallDivisor].Cells[18].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in SaturdayTime column
                    dataGridView7.Rows[overallDivisor].Cells[18].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in SaturdayTime column

                    //dataGridView6.Rows[overallDivisor].Cells[19].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);               //Display Gait Speed in SaturdayGS column
                    dataGridView7.Rows[overallDivisor].Cells[19].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);               //Display Gait Speed in SaturdayGS column

                    //dataGridView6.Rows[overallDivisor].Cells[20].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);               //Display CVs in SaturdayCVs column
                    dataGridView7.Rows[overallDivisor].Cells[20].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);               //Display CVs in SaturdayCVs column

                    overallDivisor++;
                }

                rowNum8++;

                if (rowNum8 >= dataGridView3.Rows.Count)
                {
                    morningDivisor = 0;
                    afternoonDivisor = 0;
                    eveningDivisor = 0;
                    overallDivisor = 0;

                    sumMorningCV = 0;
                    sumAfternoonCV = 0;
                    sumEveningCV = 0;

                    aveMorningCV = 0;
                    aveAfternoonCV = 0;
                    aveEveningCV = 0;

                    rowNum8 = 0;
                    break;
                }
            }


            //Retreiving CVs for the Current day *********************************************************************************************

            foreach (DataGridViewRow row in dataGridView3.Rows)
            {
                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalEvening && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) == TodayDate)
                {
                    //dataGridView6.Rows[overallDivisor].Cells[21].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in TodayTime column
                    dataGridView7.Rows[overallDivisor].Cells[21].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in TodayTime column

                    //dataGridView6.Rows[overallDivisor].Cells[22].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);               //Display Gait Speed in TodayGS column
                    dataGridView7.Rows[overallDivisor].Cells[22].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);               //Display Gait Speed in TodayGS column

                    //dataGridView6.Rows[overallDivisor].Cells[23].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);               //Display CVs in TodayCVs column
                    dataGridView7.Rows[overallDivisor].Cells[23].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);               //Display CVs in TodayCVs column

                    todaySeries.Add(Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value));


                    overallDivisor++;
                }

                rowNum8++;

                if (rowNum8 >= dataGridView3.Rows.Count)
                {

                    overallDivisor = 0;

                    rowNum8 = 0;
                    break;
                }
            }



            //Retreiving CVs for entire Week *********************************************************************************************

            foreach (DataGridViewRow row in dataGridView3.Rows)
            {

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) >= InitialMorning && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value) <= FinalEvening &&
                    (Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Sunday" || Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Monday" ||
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Tuesday" || Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Wednesday" ||
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Thursday" || Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Friday" ||
                    Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek.ToString() == "Saturday")
                    && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    //dataGridView6.Rows[overallDivisor].Cells[24].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in This WeekTime column
                    dataGridView7.Rows[overallDivisor].Cells[24].Value = Convert.ToDateTime(dataGridView3.Rows[rowNum8].Cells[2].Value).TimeOfDay;   //Display time in This WeekTime column

                    //dataGridView6.Rows[overallDivisor].Cells[25].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);               //Display Gait Speed in This WeekGS column
                    dataGridView7.Rows[overallDivisor].Cells[25].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[3].Value);               //Display Gait Speed in This WeekGS column

                    //dataGridView6.Rows[overallDivisor].Cells[26].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);               //Display CVs in WeekCVs column
                    dataGridView7.Rows[overallDivisor].Cells[26].Value = Convert.ToDouble(dataGridView3.Rows[rowNum8].Cells[5].Value);               //Display CVs in WeekCVs column

                    overallDivisor++;
                }

                rowNum8++;

                if (rowNum8 >= dataGridView3.Rows.Count)
                {

                    overallDivisor = 0;

                    rowNum8 = 0;
                    break;
                }
            }
        }


        double aveSundayCV = 0;         double aveSundayGS = 0;
        double aveMondayCV = 0;

        private void lblMorningAveGaitSpeed_Click(object sender, EventArgs e)
        {

        }

        double aveMondayGS = 0;
        double aveTuesdayCV = 0;        double aveTuesdayGS = 0;
        double aveWednesdayCV = 0;      double aveWednesdayGS = 0;
        double aveThursdayCV = 0;       double aveThursdayGS = 0;
        double aveFridayCV = 0;         double aveFridayGS = 0;
        double aveSaturdayCV = 0;       double aveSaturdayGS = 0;
        private void Day_Of_Week()  //Filter out the CVs for the various days of the current week
        {

            DateTime toda_y = DateTime.Now;

            int rowNum6 = 0;

            int sundayDivisor = 0;
            double sumSundayCV = 0;
            //double aveSundayCV = 0;
            double sumSundayGS = 0;
           //double aveSundayGS = 0;

            int mondayDivisor = 0;
            double sumMondayGS = 0;
            //double aveMondayGS = 0;
            double sumMondayCV = 0;
            //double aveMondayCV = 0;

            int tuesdayDivisor = 0;
            double sumTuesdayCV = 0;
            //double aveTuesdayCV = 0;
            double sumTuesdayGS = 0;
            //double aveTuesdayGS = 0;

            int wednesdayDivisor = 0;
            double sumWednesdayCV = 0;
            //double aveWednesdayCV = 0;
            double sumWednesdayGS = 0;
            //double aveWednesdayGS = 0;

            int thursdayDivisor = 0;
            double sumThursdayCV = 0;
            //double aveThursdayCV = 0;
            double sumThursdayGS = 0;
            //double aveThursdayGS = 0;

            int fridayDivisor = 0;
            double sumFridayCV = 0;
            //double aveFridayCV = 0;
            double sumFridayGS = 0;
            //double aveFridayGS = 0;

            int saturdayDivisor = 0;
            double sumSaturdayCV = 0;
            //double aveSaturdayCV = 0;
            double sumSaturdayGS = 0;
            //double aveSaturdayGS = 0;

            foreach (DataGridViewRow row in dataGridView3.Rows)
            {
                //Extract CVs for Sunday of the current Week  ************************************************************  

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value).DayOfWeek.ToString() == "Sunday" && Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    sundayDivisor++;

                    //Calculating average Sunday CV
                    sumSundayCV += Convert.ToDouble(dataGridView3.Rows[rowNum6].Cells[5].Value);
                    aveSundayCV = Math.Round(sumSundayCV / sundayDivisor, 2);

                    //Calculating average Sunday Gait Speed
                    sumSundayGS += Convert.ToDouble(dataGridView3.Rows[rowNum6].Cells[3].Value);
                    aveSundayGS = Math.Round(sumSundayGS / sundayDivisor, 2);

                    //dayofweek.Text = aveSundayCV.ToString() + " %";
                    dataGridView5.Rows[0].Cells[0].Value = "Sunday";
                    dataGridView5.Rows[0].Cells[4].Value = aveSundayCV;
                }
                
                
                //Extract CVs for Monday of the current Week  ************************************************************

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value).DayOfWeek.ToString() == "Monday" && Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    mondayDivisor++;

                    //Calculating average Monday CV
                    sumMondayCV += Convert.ToDouble(dataGridView3.Rows[rowNum6].Cells[5].Value);
                    aveMondayCV = Math.Round(sumMondayCV / mondayDivisor, 2);

                    //Calculating average Monday Gait Speed
                    sumMondayGS += Convert.ToDouble(dataGridView3.Rows[rowNum6].Cells[3].Value);
                    aveMondayGS = Math.Round(sumMondayGS / mondayDivisor, 2);

                    //dayofweek.Text = aveMondayCV.ToString() + " %";
                    dataGridView5.Rows[1].Cells[0].Value = "Monday";
                    dataGridView5.Rows[1].Cells[4].Value = aveMondayCV;
                }



                //Extract CVs for Tuesday of the current Week  ************************************************************

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value).DayOfWeek.ToString() == "Tuesday" && Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    tuesdayDivisor++;

                    //Calculating average Tuesday CV
                    sumTuesdayCV += Convert.ToDouble(dataGridView3.Rows[rowNum6].Cells[5].Value);
                    aveTuesdayCV = Math.Round(sumTuesdayCV / tuesdayDivisor, 2);

                    //Calculating average Tuesday Gait Speed
                    sumTuesdayGS += Convert.ToDouble(dataGridView3.Rows[rowNum6].Cells[3].Value);
                    aveTuesdayGS = Math.Round(sumTuesdayGS / tuesdayDivisor, 2);

                    //dayofweek.Text = aveTuesdayCV.ToString() + " %";
                    dataGridView5.Rows[2].Cells[0].Value = "Tuesday";
                    dataGridView5.Rows[2].Cells[4].Value = aveTuesdayCV;
                }
                

                //Extract CVs for Wednesday of the current Week  ************************************************************

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value).DayOfWeek.ToString() == "Wednesday" && Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    wednesdayDivisor++;

                    //Calculating average Wednesday CV
                    sumWednesdayCV += Convert.ToDouble(dataGridView3.Rows[rowNum6].Cells[5].Value);
                    aveWednesdayCV = Math.Round(sumWednesdayCV / wednesdayDivisor, 2);

                    //Calculating average Wednesday Gait Speed
                    sumWednesdayGS += Convert.ToDouble(dataGridView3.Rows[rowNum6].Cells[3].Value);
                    aveWednesdayGS = Math.Round(sumWednesdayGS / wednesdayDivisor, 2);

                    //dayofweek.Text = aveWednesdayCV.ToString() + " %";
                    dataGridView5.Rows[3].Cells[0].Value = "Wednesday";
                    dataGridView5.Rows[3].Cells[4].Value = aveWednesdayCV;
                }
               

                //Extract CVs for Thursday of the current Week  ************************************************************

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value).DayOfWeek.ToString() == "Thursday" && Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    thursdayDivisor++;

                    //Calculating average Thursday CV
                    sumThursdayCV += Convert.ToDouble(dataGridView3.Rows[rowNum6].Cells[5].Value);
                    aveThursdayCV = Math.Round(sumThursdayCV / thursdayDivisor, 2);

                    //Calculating average Thursday Gait Speed
                    sumThursdayGS += Convert.ToDouble(dataGridView3.Rows[rowNum6].Cells[3].Value);
                    aveThursdayGS = Math.Round(sumThursdayGS / thursdayDivisor, 2);

                    // dayofweek.Text = aveThursdayCV.ToString() + " %";
                    dataGridView5.Rows[4].Cells[0].Value = "Thursday";
                    dataGridView5.Rows[4].Cells[4].Value = aveThursdayCV;
                }
                //if (dataGridView5.Rows[4].Cells[1].Value == null)
                //{
                //    dataGridView5.Rows[4].Cells[0].Value = "Thursday";
                //    dataGridView5.Rows[4].Cells[1].Value = "Unavailable";
                //}


                //Extract CVs for Friday of the current Week  ************************************************************

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value).DayOfWeek.ToString() == "Friday" && Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    fridayDivisor++;

                    //Calculating average Friday CV
                    sumFridayCV += Convert.ToDouble(dataGridView3.Rows[rowNum6].Cells[5].Value);
                    aveFridayCV = Math.Round(sumFridayCV / fridayDivisor, 2);

                    //Calculating average Friday Gait Speed
                    sumFridayGS += Convert.ToDouble(dataGridView3.Rows[rowNum6].Cells[3].Value);
                    aveFridayGS = Math.Round(sumFridayGS / fridayDivisor, 2);

                    //dayofweek.Text = aveFridayCV.ToString() + " %";
                    dataGridView5.Rows[5].Cells[0].Value = "Friday";
                    dataGridView5.Rows[5].Cells[4].Value = aveFridayCV;
                }
                //if (dataGridView5.Rows[5].Cells[1].Value == null)
                //{
                //    dataGridView5.Rows[5].Cells[0].Value = "Friday";
                //    dataGridView5.Rows[5].Cells[1].Value = "Unavailable";
                //}


                //Extract CVs for Saturday of the current Week  ************************************************************

                if (Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value).DayOfWeek.ToString() == "Saturday" && Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value) > toda_y.AddDays(-7) && Convert.ToDateTime(dataGridView3.Rows[rowNum6].Cells[1].Value).DayOfWeek <= toda_y.DayOfWeek)
                {
                    saturdayDivisor++;

                    //Calculating average Saturday CV
                    sumSaturdayCV += Convert.ToDouble(dataGridView3.Rows[rowNum6].Cells[5].Value);
                    aveSaturdayCV = Math.Round(sumSaturdayCV / saturdayDivisor, 2);

                    //Calculating average Saturday Gait Speed
                    sumSaturdayGS += Convert.ToDouble(dataGridView3.Rows[rowNum6].Cells[3].Value);
                    aveSaturdayGS = Math.Round(sumSaturdayGS / saturdayDivisor, 2);

                    //dayofweek.Text = aveSaturdayCV.ToString() + " %";
                    dataGridView5.Rows[6].Cells[0].Value = "Saturday";
                    dataGridView5.Rows[6].Cells[4].Value = aveSaturdayCV;
                }
                
                rowNum6++;

                if (rowNum6 > dataGridView3.Rows.Count)
                {

                    rowNum6 = 0;
                    break;
                }


            }
        }


        private void Last_Week()  //Filter out all CVs for last week
        {

            DateTime toda_y = DateTime.Now;
            //var bee = toda_y.DayOfWeek;
            //if (toda_y.DayOfWeek.ToString() == "monday")
            int rowNum15 = 0;

            int LsundayDivisor = 0;
            int endOfLastWeek = 0;
            int beginningOfLastWeek = 0;

            switch (toda_y.DayOfWeek.ToString())
            {
                case "Sunday":
                    beginningOfLastWeek = 7;
                    endOfLastWeek = 1;
                    break;

                case "Monday":
                    beginningOfLastWeek = 8;
                    endOfLastWeek = 2;
                    break;

                case "Tuesday":
                    beginningOfLastWeek = 9;
                    endOfLastWeek = 3;
                    break;

                case "Wednesday":
                    beginningOfLastWeek = 10;
                    endOfLastWeek = 4;
                    break;

                case "Thursday":
                    beginningOfLastWeek = 11;
                    endOfLastWeek = 5;
                    break;

                case "Friday":
                    beginningOfLastWeek = 12;
                    endOfLastWeek = 6;
                    break;

                case "Saturday":
                    beginningOfLastWeek = 13;
                    endOfLastWeek = 7;
                    break;

                default:
                    beginningOfLastWeek = 1;
                    endOfLastWeek = 1;
                    break;
            }

            foreach (DataGridViewRow row in dataGridView3.Rows)
            {
                if (Convert.ToDateTime(dataGridView3.Rows[rowNum15].Cells[1].Value) <= toda_y.AddDays(-endOfLastWeek) && Convert.ToDateTime(dataGridView3.Rows[rowNum15].Cells[1].Value) >= toda_y.AddDays(-beginningOfLastWeek))
                {
                    dataGridView3.Rows[rowNum15].Cells[6].Value = Convert.ToDouble(dataGridView3.Rows[rowNum15].Cells[5].Value);

                    LsundayDivisor++;
                }
                rowNum15++;

                if (rowNum15 > LsundayDivisor)
                {

                    LsundayDivisor = 0;

                    rowNum15 = 0;
                    break;
                }

            }
        }


        private void CVs_Before_This_Week() //Filter out all CVs before the current week
        {
            int rowNum9 = 0;

            DateTime TD = DateTime.Now;
            DateTime toda_y = DateTime.Now;
            DateTime firstDate = new DateTime(2017, 11, 20);

            var TodayDate = Convert.ToDateTime(TD.ToShortDateString());

            int overallDivisor = 0;

            int endOfLastWeek = 0;


            switch (toda_y.DayOfWeek.ToString())
            {
                case "Sunday":
                    endOfLastWeek = 1;
                    break;

                case "Monday":
                    endOfLastWeek = 2;
                    break;

                case "Tuesday":
                    endOfLastWeek = 3;
                    break;

                case "Wednesday":
                    endOfLastWeek = 4;
                    break;

                case "Thursday":
                    endOfLastWeek = 5;
                    break;

                case "Friday":
                    endOfLastWeek = 6;
                    break;

                case "Saturday":
                    endOfLastWeek = 7;
                    break;

                default:
                    endOfLastWeek = 1;
                    break;
            }

            foreach (DataGridViewRow row in dataGridView3.Rows)
            {
                if ((Convert.ToDateTime(dataGridView3.Rows[rowNum9].Cells[1].Value) > toda_y.AddDays(-(endOfLastWeek + 90))) && (Convert.ToDateTime(dataGridView3.Rows[rowNum9].Cells[1].Value) <= toda_y.AddDays(-endOfLastWeek)))
                {

                    dataGridView7.Rows[overallDivisor].Cells[27].Value = Convert.ToDouble(dataGridView3.Rows[rowNum9].Cells[5].Value);

                    overallDivisor++;
                }

                rowNum9++;

                if (rowNum9 > dataGridView7.Rows.Count)
                {

                    overallDivisor = 0;

                    rowNum9 = 0;
                    break;
                }

            }
        }


        List<double> todaySeries = new List<double>();
        //double[] todaySeries = new double[20];
        public double TTest(List<double> seriesName)
        {

            double seriesMean = seriesName.Mean();

            return seriesMean;

        }



      //Display functions ------  2.4  ----------------------  2.4  ----------------------  2.4  -----------------------  2.4  -------------------------  2.4  -------------------------  2.4  -----------------------  2.4  --------------------
        private void DrawTime() //Display stopwatch in lables, display initial and final joint position in labels
        {
     
            //Display initial and final positions of mid-spine joint and the stopwatch for the upper body
            lblDisplayStopWatch.Text = timeDays.ToString("00") + " days" + " : " + timeHours.ToString("00") + " hrs" + " : " + timeMin.ToString("00") + " mins" + " : " + timeSec.ToString("00") + " sec" + " . " + timeCsec.ToString("00");
            lblipos.Text = x1_ms.ToString("#.##") + ", " + y1_ms.ToString("#.##") + ", " + z1_ms.ToString("#.##");
            lblfpos.Text = x2_ms.ToString("#.##") + ", " + y2_ms.ToString("#.##") + ", " + z2_ms.ToString("#.##");


            //Display initial and final positions of Right Ankle joint and the stopwatch for the lower body
            lblRADisplayStopwatch.Text = timeMin.ToString("00") + " : " + timeSec.ToString("00") + " . " + timeCsec.ToString("00");
            lbl_RAInitialPosition.Text = x1_ra.ToString("#.##") + ", " + y1_ra.ToString("#.##") + ", " + z1_ra.ToString("#.##");
            lbl_RAFinalPosition.Text = x2_ra.ToString("#.##") + ", " + y2_ra.ToString("#.##") + ", " + z2_ra.ToString("#.##");
            //lbltimeCsec.Text = timeCsec.ToString("#.##");

            lblStrideTime.Text = timeCsec3.ToString("00");
        }


        private void ResetStopwatch_1()  //Reset stopwatch to zeros
        {
            timeCsec = 0;
            timeSec = 0;
            timeMin = 0;
        }

        private void ResetStopwatch_3()  //Reset stopwatch to zeros
        {
            timeCsec3 = 0;
            
        }


        private void DisplayTableData()  //Display database table in data-grid-view 1
        {

            dbConnection.Open();
            SqlCommand command1 = dbConnection.CreateCommand();
            command1.CommandType = CommandType.Text;
            command1.CommandText = "Select * from COM_Data";
            command1.ExecuteNonQuery();
            DataTable dTable2 = new DataTable();
            SqlDataAdapter dbAdapter2 = new SqlDataAdapter(command1);
            dbAdapter2.Fill(dTable2);
            dataGridView1.DataSource = dTable2;

            dbConnection.Close();
        }


        double counterInitial = 0;
        double counter1 = 0;
        int counter2 = 1;
        public void secondDisplay()   //Distinct data for main display textbox, counter for refreshing data
        {
            var names = txtTemp.Lines.Distinct();
                        
            txtTemp.Clear();

            foreach (string name in names)
            {
                txtDisplayTable.AppendText(name + "\n");

                lblCounter2.Text = counter2.ToString();

                counterInitial += 1;
                if (counterInitial == 2)
                {
                    counter1++;
                    lblCounter1.Text = counter1.ToString();

                    counterInitial = 0;
                                       
                    if (counter1 > 6)
                    {
                        counter2++;
                        lblCounter2.Text = counter2.ToString();

                        counter1 = 1;
                        lblCounter1.Text = counter1.ToString();

                        Refresh_Data();
                    }
                }
               
            }
           

        }
        public void secondDisplayAnkles()
        {
            var names_Ankles = txtTemp_Ankles.Lines.Distinct();

            txtTemp_Ankles.Clear();

            foreach (string name2 in names_Ankles)
            {
                txtDisplayTable_Ankles.AppendText(name2 + "\n");
            }
        }

        private void DistinctNDisplayRawData()  //Remove duplicate data, display raw data into data-grid-view 4
        {

            // this code distinct the values
            dbConnection.Open();
            SqlCommand command3 = dbConnection.CreateCommand();
            command3.CommandType = CommandType.Text;
            command3.CommandText = "With COM_DataCTE AS(Select * , ROW_NUMBER() oVER(Partition BY Distance order By Time) as RowNumber from COM_Data)Delete From COM_DataCTE where RowNumber > 1 select* from COM_Data";
            command3.ExecuteNonQuery();
            DataTable dTable4 = new DataTable();
            SqlDataAdapter dbAdapter4 = new SqlDataAdapter(command3);
            dbAdapter4.Fill(dTable4);
            dataGridView1.DataSource = dTable4;
            dbConnection.Close();

            //Display Raw data collected and display in DataGridView 4
            dbConnection.Open();
            SqlCommand command6 = dbConnection.CreateCommand();
            command6.CommandType = CommandType.Text;
            command6.CommandText = "Select COM_Data.Id, COM_Data.Distance, COM_Data.Velocity,COM_Data.Acceletation AS Acceleration, COM_Data.Date, COM_Data.Time from COM_Data";
            command6.ExecuteNonQuery();
            DataTable dTable7 = new DataTable();
            SqlDataAdapter dbAdapter7 = new SqlDataAdapter(command6);
            dbAdapter7.Fill(dTable7);
            dataGridView4.DataSource = dTable7;
            dbConnection.Close();

        }

        private void DisplayDailyCV()  //Display the CVs for the days in the current week in data-grid-view 7
        {
            //Display Empty column(i.e., Sum) in DataGridView 5 to be able to display derived table
            dbConnection.Open();
            SqlCommand command7 = dbConnection.CreateCommand();
            command7.CommandType = CommandType.Text;
            command7.CommandText = "Select COM_DailyCVs.DaysOfWeek, COM_DailyCVs.MorningCVs, COM_DailyCVs.AfternoonCVs, COM_DailyCVs.EveningCVs, COM_DailyCVs.DailyCVs, COM_DailyCVs.TTest_PValue, COM_DailyCVs.TTest_Conclusion, COM_DailyCVs.Performance_Rating from COM_DailyCVs";
            command7.ExecuteNonQuery();
            DataTable dTable8 = new DataTable();
            SqlDataAdapter dbAdapter8 = new SqlDataAdapter(command7);
            dbAdapter8.Fill(dTable8);
            dataGridView5.DataSource = dTable8;
            dbConnection.Close();

            //Display Empty column(i.e., CV) in DataGridView 6 to be able to display Current Week's CVs and GaitSpeeds for T-Test
            //dbConnection.Open();
            //SqlCommand command8 = dbConnection.CreateCommand();
            //command8.CommandType = CommandType.Text;
            //command8.CommandText = "Select " +
            //                        "COM_Data.CV AS This_SundayTime, COM_Data.CV AS This_SundayAveGS, COM_Data.CV AS This_SundayCVs, " +              //Display Sunday in Gridview_6
            //                        "COM_Data.CV AS This_MondayTime, COM_Data.CV AS This_MondayAveGS, COM_Data.CV AS This_MondayCVs, " +               //Display Monday in Gridview_6
            //                        "COM_Data.CV AS This_TuesdayTime, COM_Data.CV AS This_TuesdayAveGS, COM_Data.CV AS This_TuesdayCVs, " +            //Display Tuesday in Gridview_6
            //                        "COM_Data.CV AS This_WednesdayTime, COM_Data.CV AS This_WednesdayAveGS, COM_Data.CV AS This_WednesdayCVs, " +      //Display Wednesday in Gridview_6
            //                        "COM_Data.CV AS This_ThursdayTime, COM_Data.CV AS This_ThursdayAveGS, COM_Data.CV AS This_ThursdayCVs, " +         //Display Thursday in Gridview_6
            //                        "COM_Data.CV AS This_FridayTime, COM_Data.CV AS This_FridayAveGS, COM_Data.CV AS This_FridayCVs, " +               //Display Friday in Gridview_6
            //                        "COM_Data.CV AS This_SaturdayTime, COM_Data.CV AS This_SaturdayAveGS, COM_Data.CV AS This_SaturdayCVs," +          //Display Saturday in Gridview_6
            //                        "COM_Data.CV AS TodayTime, COM_Data.CV AS TodayAveGS, COM_Data.CV AS TodayCVs,  " +                                //Display Today in Gridview_6
            //                        "COM_Data.CV AS This_WeekTime, COM_Data.CV AS This_WeekAveGS, COM_Data.CV AS This_WeekCVs, " +
            //                        "COM_Data.CV AS All_CVsToLastWeek " +
            //                        "from COM_Data";
            //command8.ExecuteNonQuery();
            //DataTable dTable9 = new DataTable();
            //SqlDataAdapter dbAdapter9 = new SqlDataAdapter(command8);
            //dbAdapter9.Fill(dTable9);
            //dataGridView6.DataSource = dTable9;
            //dbConnection.Close();


            //Display Empty column(i.e., CV) in DataGridView 6 to be able to display Current Week's CVs and GaitSpeeds for Graphs
            dbConnection.Open();
            SqlCommand command9 = dbConnection.CreateCommand();
            command9.CommandType = CommandType.Text;
            command9.CommandText = "Select " +
                                    "COM_WeeklyCVs_Detailed.This_SundayTime, COM_WeeklyCVs_Detailed.This_SundayAveGS, COM_WeeklyCVs_Detailed.This_SundayCVs, " +              //Display Sunday in Gridview_6
                                    "COM_WeeklyCVs_Detailed.This_MondayTime, COM_WeeklyCVs_Detailed.This_MondayAveGS, COM_WeeklyCVs_Detailed.This_MondayCVs, " +               //Display Monday in Gridview_6
                                    "COM_WeeklyCVs_Detailed.This_TuesdayTime, COM_WeeklyCVs_Detailed.This_TuesdayAveGS, COM_WeeklyCVs_Detailed.This_TuesdayCVs, " +            //Display Tuesday in Gridview_6
                                    "COM_WeeklyCVs_Detailed.This_WednesdayTime, COM_WeeklyCVs_Detailed.This_WednesdayAveGS, COM_WeeklyCVs_Detailed.This_WednesdayCVs, " +      //Display Wednesday in Gridview_6
                                    "COM_WeeklyCVs_Detailed.This_ThursdayTime, COM_WeeklyCVs_Detailed.This_ThursdayAveGS, COM_WeeklyCVs_Detailed.This_ThursdayCVs, " +         //Display Thursday in Gridview_6
                                    "COM_WeeklyCVs_Detailed.This_FridayTime, COM_WeeklyCVs_Detailed.This_FridayAveGS, COM_WeeklyCVs_Detailed.This_FridayCVs, " +               //Display Friday in Gridview_6
                                    "COM_WeeklyCVs_Detailed.This_SaturdayTime, COM_WeeklyCVs_Detailed.This_SaturdayAveGS, COM_WeeklyCVs_Detailed.This_SaturdayCVs," +          //Display Saturday in Gridview_6
                                    "COM_WeeklyCVs_Detailed.TodayTime, COM_WeeklyCVs_Detailed.TodayAveGS, COM_WeeklyCVs_Detailed.TodayCVs,  " +                                //Display Today in Gridview_6
                                    "COM_WeeklyCVs_Detailed.This_WeekTime, COM_WeeklyCVs_Detailed.This_WeekAveGS, COM_WeeklyCVs_Detailed.This_WeekCVs," +
                                    "COM_WeeklyCVs_Detailed.All_CVsToLastWeek " +
                                    "from COM_WeeklyCVs_Detailed";
            command9.ExecuteNonQuery();
            DataTable dTable10 = new DataTable();
            SqlDataAdapter dbAdapter10 = new SqlDataAdapter(command9);
            dbAdapter10.Fill(dTable10);
            dataGridView7.DataSource = dTable10;
            dbConnection.Close();
        }
               
              
        private void Refresh_Data() //Refresh data-grid-view tables, graphs and all calculations
        {
            DistinctNDisplayRawData();
            CalculateAllCVs();
            CalcNDisp5thN95thPercentile();
            Current_Session_Of_Day();
            DisplayDailyCV();
            Day_Of_Week();
            Weekly_Session_Of_Day();
            CVs_Before_This_Week();
            DisplayCharts();
            Last_Week();
           
        }
        

        string todaysTTest_conclusion;          double todayDeviation;                
        string mondayTTest_conclusion;          double mondayDeviation;
        string tuesdayTTest_conclusion;         double tuesdayDeviation;
        string wednesdayTTest_conclusion;       double wednesdayDeviation;
        string thursdayTTest_conclusion;        double thursdayDeviation;
        string fridayTTest_conclusion;          double fridayDeviation;
        string saturdayTTest_conclusion;        double saturdayDeviation;
        string sundayTTest_conclusion;          double sundayDeviation;
        private void DisplayCharts()    //Display charts, conduct T-Test and Performance check 
        {
            double todaysMean = TTest(todaySeries);
            dayofweek.Text = todaysMean.ToString();


            //Display Chart of all Velocities and CVs
            chart1.Series["CV_Series"].XValueMember = "Time";
            chart1.Series["CV_Series"].YValueMembers = "CV";
            chart1.Series["Velocity_Series"].XValueMember = "Time";
            chart1.Series["Velocity_Series"].YValueMembers = "Velocity";
            chart1.DataSource = dataGridView1.DataSource;
            chart1.DataBind();


            //Display Chart of Daily CV
            chart2.Series["MorningCV"].XValueMember = "DaysOfWeek";
            chart2.Series["MorningCV"].YValueMembers = "MorningCVs";
            chart2.Series["AfternoonCV"].XValueMember = "DaysOfWeek";
            chart2.Series["AfternoonCV"].YValueMembers = "AfternoonCVs";
            chart2.Series["EveningCV"].XValueMember = "DaysOfWeek";
            chart2.Series["EveningCV"].YValueMembers = "EveningCVs";
            chart2.Series["DailyCV"].XValueMember = "DaysOfWeek";
            chart2.Series["DailyCV"].YValueMembers = "DailyCVs";
            chart2.DataSource = dataGridView5.DataSource;
            chart2.DataBind();

            //Chart that displays Average Gait Speed and CVs for current day (Today)
            chart4.Series["Today_AveGS"].XValueMember = "TodayTime";
            chart4.Series["Today_AveGS"].YValueMembers = "TodayAveGS";
            chart4.Series["Today_AveCV"].XValueMember = "TodayTime";
            chart4.Series["Today_AveCV"].YValueMembers = "TodayCVs";
            chart4.DataSource = dataGridView7.DataSource;
            chart4.DataBind();

            //Chart that displays Average GaitSpeed (GS) for the various days of the week
            chart5.Series["ThisSunday_AveGS"].XValueMember = "This_SundayTime";
            chart5.Series["ThisSunday_AveGS"].YValueMembers = "This_SundayAveGS";
            chart5.Series["ThisMonday_AveGS"].XValueMember = "This_MondayTime";
            chart5.Series["ThisMonday_AveGS"].YValueMembers = "This_MondayAveGS";
            chart5.Series["ThisTuesday_AveGS"].XValueMember = "This_TuesdayTime";
            chart5.Series["ThisTuesday_AveGS"].YValueMembers = "This_TuesdayAveGS";
            chart5.Series["ThisWednesday_AveGS"].XValueMember = "This_WednesdayTime";
            chart5.Series["ThisWednesday_AveGS"].YValueMembers = "This_WednesdayAveGS";
            chart5.Series["ThisThursday_AveGS"].XValueMember = "This_ThursdayTime";
            chart5.Series["ThisThursday_AveGS"].YValueMembers = "This_ThursdayAveGS";
            chart5.Series["ThisFriday_AveGS"].XValueMember = "This_FridayTime";
            chart5.Series["ThisFriday_AveGS"].YValueMembers = "This_FridayAveGS";
            chart5.Series["ThisSaturday_AveGS"].XValueMember = "This_SaturdayTime";
            chart5.Series["ThisSaturday_AveGS"].YValueMembers = "This_SaturdayAveGS";
            chart5.DataSource = dataGridView7.DataSource;
            chart5.DataBind();

            //Chart that displays Average CVs for the various days of the week
            chart6.Series["ThisSunday_CV"].XValueMember = "This_SundayTime";
            chart6.Series["ThisSunday_CV"].YValueMembers = "This_SundayCVs";
            chart6.Series["ThisMonday_CV"].XValueMember = "This_MondayTime";
            chart6.Series["ThisMonday_CV"].YValueMembers = "This_MondayCVs";
            chart6.Series["ThisTuesday_CV"].XValueMember = "This_TuesdayTime";
            chart6.Series["ThisTuesday_CV"].YValueMembers = "This_TuesdayCVs";
            chart6.Series["ThisWednesday_CV"].XValueMember = "This_WednesdayTime";
            chart6.Series["ThisWednesday_CV"].YValueMembers = "This_WednesdayCVs";
            chart6.Series["ThisThursday_CV"].XValueMember = "This_ThursdayTime";
            chart6.Series["ThisThursday_CV"].YValueMembers = "This_ThursdayCVs";
            chart6.Series["ThisFriday_CV"].XValueMember = "This_FridayTime";
            chart6.Series["ThisFriday_CV"].YValueMembers = "This_FridayCVs";
            chart6.Series["ThisSaturday_CV"].XValueMember = "This_SaturdayTime";
            chart6.Series["ThisSaturday_CV"].YValueMembers = "This_SaturdayCVs";
            chart6.DataSource = dataGridView7.DataSource;
            chart6.DataBind();

            ////Chart that displays Performance Rating for the various days of the week
            //chart7.Series["DailyRatings"].XValueMember = "DaysOfWeek";
            //chart7.Series["DailyRatings"].YValueMembers = "Performance_Rating";
            //chart7.DataSource = dataGridView5.DataSource;
            //chart7.DataBind();

            //Charts for Anova Test
            chart3.Series["ThisSundayCV"].YValueMembers = "This_SundayCVs";
            chart3.Series["ThisMondayCV"].YValueMembers = "This_MondayCVs";
            chart3.Series["ThisTuesdayCV"].YValueMembers = "This_TuesdayCVs";
            chart3.Series["ThisWednesdayCV"].YValueMembers = "This_WednesdayCVs";
            chart3.Series["ThisThursdayCV"].YValueMembers = "This_ThursdayCVs";
            chart3.Series["ThisFridayCV"].YValueMembers = "This_FridayCVs";
            chart3.Series["ThisSaturdayCV"].YValueMembers = "This_SaturdayCVs";
            chart3.Series["TodayCV"].YValueMembers = "TodayCVs";
            chart3.Series["ThisWeekCV"].YValueMembers = "This_WeekCVs";
            chart3.Series["All_CVsBeforeThisWeek"].YValueMembers = "All_CVsToLastWeek";
            chart3.DataSource = dataGridView7.DataSource;
            chart3.DataBind();



            //ANOVA Analysis for the current week *****************************************************************************

            //AnovaResult currentWeekAnova = chart3.DataManipulator.Statistics.Anova(0.05, " ThisSundayCV, ThisMondayCV, ThisTuesdayCV");
            //double FValue = Math.Round(currentWeekAnova.FRatio, 4);
            //double rejectionRegion = Math.Round(currentWeekAnova.FCriticalValue, 4);
            ////double DOF_Monday = currentWeekAnova.DegreeOfFreedomWithinGroups;
            ////textBox1.Text = DOF_Monday.ToString();

            //if (FValue > rejectionRegion)
            //{
            //    dayofweek.Text = "";
            //    dayofweek.Text += "ANOVA Result" + Environment.NewLine;
            //    dayofweek.Text += "" + Environment.NewLine;
            //    dayofweek.Text += "F value : " + FValue.ToString() + Environment.NewLine;
            //    dayofweek.Text += "Rejectio Region : " + rejectionRegion.ToString() + Environment.NewLine;
            //    dayofweek.Text += "Decision : Reject Ho" + Environment.NewLine;
            //    dayofweek.Text += "Conclusion : Significantly different" + Environment.NewLine;
            //}
            //else
            //{
            //    dayofweek.Text = "";
            //    dayofweek.Text += "ANOVA Result" + Environment.NewLine;
            //    dayofweek.Text += "" + Environment.NewLine;
            //    dayofweek.Text += "F value : " + FValue.ToString() + Environment.NewLine;
            //    dayofweek.Text += "Rejectio Region : " + rejectionRegion.ToString() + Environment.NewLine;
            //    dayofweek.Text += "Decision : Fail to Reject Ho" + Environment.NewLine;
            //    dayofweek.Text += "Conclusion : Not significantly different" + Environment.NewLine;
            //}


            //T-Test for Today *****************************************************************************

            if (Convert.ToDouble(dataGridView7.Rows[0].Cells[23].Value) == 0 || Convert.ToDouble(dataGridView7.Rows[0].Cells[27].Value) == 0)
            {
                textBox1.Text = "";
                textBox1.Text += "No data available for today" + Environment.NewLine;
                todaysTTest_conclusion = "I'm sorry there is no data available to check your walking status for today.";
            }
            else
            {
                TTestResult todaysdayTTest = chart3.DataManipulator.Statistics.TTestUnequalVariances(0.00, 0.05, "TodayCV", "All_CVsBeforeThisWeek");
                double todaysTValue = Math.Round(todaysdayTTest.TValue, 4);
                double todayRR = Math.Round(todaysdayTTest.TCriticalValueTwoTail, 4);
                double todayPValue = Math.Round(todaysdayTTest.ProbabilityTTwoTail, 4);
                double todayDOF = todaysdayTTest.DegreeOfFreedom;
                double todayMean1 = todaysdayTTest.FirstSeriesMean;
                double pastMean = todaysdayTTest.SecondSeriesMean;
                todayDeviation = Math.Round(((todayMean1 - pastMean) / pastMean) * 100, 1);

                string deviationQualifier;

                if ((todayMean1 - pastMean) > 0)
                {
                    deviationQualifier = "higher than usual";
                }
                else
                {
                    deviationQualifier = "lower than usual";
                }

                textBox1.Text += "Speed variability is " + todayDeviation.ToString() + "% " + deviationQualifier + Environment.NewLine;


                if (todayPValue < 0.05)
                {
                    textBox1.Text = "";
                    textBox1.Text += "Today's T-Test Result" + Environment.NewLine;
                    textBox1.Text += "" + Environment.NewLine;
                    textBox1.Text += "T value : " + todaysTValue.ToString() + Environment.NewLine;
                    textBox1.Text += "Rejectio Region : " + todayRR.ToString() + "  and  " + "-" + todayRR.ToString() + Environment.NewLine;
                    textBox1.Text += "P-value = " + todayPValue.ToString() + Environment.NewLine;
                    textBox1.Text += "Decision : Reject Ho" + Environment.NewLine;
                    textBox1.Text += "Conclusion : Significantly different" + Environment.NewLine;
                    todaysTTest_conclusion = "Your walking changed significantly from the usual pattern";
                    textBox1.Text += "" + Environment.NewLine;
                    textBox1.Text += "Speed variability is " + todayDeviation.ToString() + "% " + deviationQualifier + Environment.NewLine;
                    textBox1.Text += todayDOF.ToString() + Environment.NewLine;
                    textBox1.Text += todayMean1.ToString() + Environment.NewLine;
                    textBox1.Text += pastMean.ToString() + Environment.NewLine;

                }
                else
                {
                    textBox1.Text = "";
                    textBox1.Text += "Today's T-Test Result" + Environment.NewLine;
                    textBox1.Text += "" + Environment.NewLine;
                    textBox1.Text += "T value : " + todaysTValue.ToString() + Environment.NewLine;
                    textBox1.Text += "Rejectio Region : " + todayRR.ToString() + "  and  " + "-" + todayRR.ToString() + Environment.NewLine;
                    textBox1.Text += "P-value = " + todayPValue.ToString() + Environment.NewLine;
                    textBox1.Text += "Decision : Fail to Reject Ho" + Environment.NewLine;
                    textBox1.Text += "Conclusion : Not significantly different" + Environment.NewLine;
                    todaysTTest_conclusion = "Your walking did not changed from the usual pattern";
                    textBox1.Text += "" + Environment.NewLine;
                    textBox1.Text += "Speed variability is " + todayDeviation.ToString() + "% " + deviationQualifier + Environment.NewLine;
                    textBox1.Text += todayDOF.ToString() + Environment.NewLine;
                    textBox1.Text += todayMean1.ToString() + Environment.NewLine;
                    textBox1.Text += pastMean.ToString() + Environment.NewLine;
                }
            }


            //T-Test for Monday *****************************************************************************

            if (Convert.ToDouble(dataGridView7.Rows[0].Cells[5].Value) == 0 || Convert.ToDouble(dataGridView7.Rows[0].Cells[27].Value) == 0)
            {
                txtMondayTTest.Text = "";
                txtMondayTTest.Text += "No data available for Monday" + Environment.NewLine;
                mondayTTest_conclusion = "On Monday, I'm sorry there was no data available to check your walking status.";
            }
            else
            {
                TTestResult mondayTTest = chart3.DataManipulator.Statistics.TTestUnequalVariances(0.00, 0.05, "ThisMondayCV", "All_CVsBeforeThisWeek");
                double mondayTValue = Math.Round(mondayTTest.TValue, 4);
                double mondayRR = Math.Round(mondayTTest.TCriticalValueTwoTail, 4);
                double mondayPValue = Math.Round(mondayTTest.ProbabilityTTwoTail, 4);
                double mondayMean = mondayTTest.FirstSeriesMean;
                double pastMean = mondayTTest.SecondSeriesMean;
                mondayDeviation = Math.Round(((mondayMean - pastMean) / pastMean) * 100, 1);

                dataGridView5.Rows[1].Cells[7].Value = mondayDeviation;    //Display performance rating in data-grid-view 5

                string deviationQualifier;

                if ((mondayMean - pastMean) > 0)
                {
                    deviationQualifier = "higher than usual";
                }
                else
                {
                    deviationQualifier = "lower than usual";
                }

                if (mondayPValue < 0.05)
                {
                    txtMondayTTest.Text = "";
                    txtMondayTTest.Text += "Monday's T-Test Result" + Environment.NewLine;
                    txtMondayTTest.Text += "" + Environment.NewLine;
                    txtMondayTTest.Text += "T value : " + mondayTValue.ToString() + Environment.NewLine;
                    txtMondayTTest.Text += "Rejectio Region : " + mondayRR.ToString() + "  and  " + "-" + mondayRR.ToString() + Environment.NewLine;
                    txtMondayTTest.Text += "P-value = " + mondayPValue.ToString() + Environment.NewLine;
                    txtMondayTTest.Text += "Decision : Reject Ho" + Environment.NewLine;
                    txtMondayTTest.Text += "Conclusion : Significantly different" + Environment.NewLine;
                    mondayTTest_conclusion = ", Your walking changed significantly from the usual pattern";
                    txtMondayTTest.Text += "" + Environment.NewLine;
                    txtMondayTTest.Text += "Speed variability is " + mondayDeviation.ToString() + "% " + deviationQualifier + Environment.NewLine;

                }
                else
                {
                    txtMondayTTest.Text = "";
                    txtMondayTTest.Text += "Monday's T-Test Result" + Environment.NewLine;
                    txtMondayTTest.Text += "" + Environment.NewLine;
                    txtMondayTTest.Text += "T value : " + mondayTValue.ToString() + Environment.NewLine;
                    txtMondayTTest.Text += "Rejectio Region : " + mondayRR.ToString() + "  and  " + "-" + mondayRR.ToString() + Environment.NewLine;
                    txtMondayTTest.Text += "P-value = " + mondayPValue.ToString() + Environment.NewLine;
                    txtMondayTTest.Text += "Decision : Fail to Reject Ho" + Environment.NewLine;
                    txtMondayTTest.Text += "Conclusion : Not significantly different" + Environment.NewLine;
                    mondayTTest_conclusion = ", Your walking did not changed from the usual pattern";
                    txtMondayTTest.Text += "" + Environment.NewLine;
                    txtMondayTTest.Text += "Speed variability is " + mondayDeviation.ToString() + "% " + deviationQualifier + Environment.NewLine;
                }
            }


            //T-Test for Tuesday *****************************************************************************

            if (Convert.ToDouble(dataGridView7.Rows[0].Cells[8].Value) == 0 || Convert.ToDouble(dataGridView7.Rows[0].Cells[27].Value) == 0)
            {
                txtTuesdayTTest.Text = "";
                txtTuesdayTTest.Text += "No data available for Tuesday" + Environment.NewLine;
                tuesdayTTest_conclusion = "On Tuesday, I'm sorry there was no data available to check your walking status.";
            }
            else
            {
                TTestResult tuesdayTTest = chart3.DataManipulator.Statistics.TTestUnequalVariances(0.00, 0.05, "ThisTuesdayCV", "All_CVsBeforeThisWeek");
                double tuesdayTValue = Math.Round(tuesdayTTest.TValue, 4);
                double tuesdayRR = Math.Round(tuesdayTTest.TCriticalValueTwoTail, 4);
                double tuesdayPValue = Math.Round(tuesdayTTest.ProbabilityTTwoTail, 4);
                double tuesdayMean = tuesdayTTest.FirstSeriesMean;
                double pastMean = tuesdayTTest.SecondSeriesMean;
                tuesdayDeviation = Math.Round(((tuesdayMean - pastMean) / pastMean) * 100, 1);

                dataGridView5.Rows[2].Cells[7].Value = tuesdayDeviation;    //Display performance rating in data-grid-view 5

                string deviationQualifier;

                if ((tuesdayMean - pastMean) > 0)
                {
                    deviationQualifier = "higher than usual";
                }
                else
                {
                    deviationQualifier = "lower than usual";
                }

                if (tuesdayPValue < 0.05)
                {
                    txtTuesdayTTest.Text = "";
                    txtTuesdayTTest.Text += "Tuesday's T-Test Result" + Environment.NewLine;
                    txtTuesdayTTest.Text += "" + Environment.NewLine;
                    txtTuesdayTTest.Text += "T value : " + tuesdayTValue.ToString() + Environment.NewLine;
                    txtTuesdayTTest.Text += "Rejectio Region : " + tuesdayRR.ToString() + "  and  " + "-" + tuesdayRR.ToString() + Environment.NewLine;
                    txtTuesdayTTest.Text += "P-value = " + tuesdayPValue.ToString() + Environment.NewLine;
                    txtTuesdayTTest.Text += "Decision : Reject Ho" + Environment.NewLine;
                    txtTuesdayTTest.Text += "Conclusion : Significantly different" + Environment.NewLine;
                    tuesdayTTest_conclusion = ", Your walking changed significantly from the usual pattern";
                    txtTuesdayTTest.Text += "" + Environment.NewLine;
                    txtTuesdayTTest.Text += "Speed variability is " + tuesdayDeviation.ToString() + "% " + deviationQualifier + Environment.NewLine;
                }
                else
                {
                    txtTuesdayTTest.Text = "";
                    txtTuesdayTTest.Text += "Tuesday's T-Test Result" + Environment.NewLine;
                    txtTuesdayTTest.Text += "" + Environment.NewLine;
                    txtTuesdayTTest.Text += "T value : " + tuesdayTValue.ToString() + Environment.NewLine;
                    txtTuesdayTTest.Text += "Rejectio Region : " + tuesdayRR.ToString() + "  and  " + "-" + tuesdayRR.ToString() + Environment.NewLine;
                    txtTuesdayTTest.Text += "P-value = " + tuesdayPValue.ToString() + Environment.NewLine;
                    txtTuesdayTTest.Text += "Decision : Fail to Reject Ho" + Environment.NewLine;
                    txtTuesdayTTest.Text += "Conclusion : Not significantly different" + Environment.NewLine;
                    tuesdayTTest_conclusion = ", Your walking did not changed from the usual pattern";
                    txtTuesdayTTest.Text += "" + Environment.NewLine;
                    txtTuesdayTTest.Text += "Speed variability is " + tuesdayDeviation.ToString() + "% " + deviationQualifier + Environment.NewLine;
                }
            }


            //T-Test for Wednesday *****************************************************************************

            if (Convert.ToDouble(dataGridView7.Rows[0].Cells[11].Value) == 0 || Convert.ToDouble(dataGridView7.Rows[0].Cells[27].Value) == 0)
            {
                txtWednesdayTTest.Text = "";
                txtWednesdayTTest.Text += "No data available for Wednesday" + Environment.NewLine;
                wednesdayTTest_conclusion = "On Wednesday, I'm sorry there was no data available to check your walking status.";
            }
            else
            {
                TTestResult wednesdayTTest = chart3.DataManipulator.Statistics.TTestUnequalVariances(0.00, 0.05, "ThisWednesdayCV", "All_CVsBeforeThisWeek");
                double wednesdayTValue = Math.Round(wednesdayTTest.TValue, 4);
                double wednesdayRR = Math.Round(wednesdayTTest.TCriticalValueTwoTail, 4);
                double wednesdayPValue = Math.Round(wednesdayTTest.ProbabilityTTwoTail, 4);
                double wednesdayMean = wednesdayTTest.FirstSeriesMean;
                double pastMean = wednesdayTTest.SecondSeriesMean;
                wednesdayDeviation = Math.Round(((wednesdayMean - pastMean) / pastMean) * 100, 1);

                dataGridView5.Rows[3].Cells[7].Value = wednesdayDeviation;    //Display performance rating in data-grid-view 5

                string deviationQualifier;

                if ((wednesdayMean - pastMean) > 0)
                {
                    deviationQualifier = "higher than usual";
                }
                else
                {
                    deviationQualifier = "lower than usual";
                }

                if (wednesdayPValue < 0.05)
                {
                    txtWednesdayTTest.Text = "";
                    txtWednesdayTTest.Text += "Wednesday's T-Test Result" + Environment.NewLine;
                    txtWednesdayTTest.Text += "" + Environment.NewLine;
                    txtWednesdayTTest.Text += "T value : " + wednesdayTValue.ToString() + Environment.NewLine;
                    txtWednesdayTTest.Text += "Rejectio Region : " + wednesdayRR.ToString() + "  and  " + "-" + wednesdayRR.ToString() + Environment.NewLine;
                    txtWednesdayTTest.Text += "P-value = " + wednesdayPValue.ToString() + Environment.NewLine;
                    txtWednesdayTTest.Text += "Decision : Reject Ho" + Environment.NewLine;
                    txtWednesdayTTest.Text += "Conclusion : Significantly different" + Environment.NewLine;
                    wednesdayTTest_conclusion = ", Your walking changed significantly from the usual pattern";
                    txtWednesdayTTest.Text += "" + Environment.NewLine;
                    txtWednesdayTTest.Text += "Speed variability is " + wednesdayDeviation.ToString() + "% " + deviationQualifier + Environment.NewLine;
                }
                else
                {
                    txtWednesdayTTest.Text = "";
                    txtWednesdayTTest.Text += "Wednesday's T-Test Result" + Environment.NewLine;
                    txtWednesdayTTest.Text += "" + Environment.NewLine;
                    txtWednesdayTTest.Text += "T value : " + wednesdayTValue.ToString() + Environment.NewLine;
                    txtWednesdayTTest.Text += "Rejectio Region : " + wednesdayRR.ToString() + "  and  " + "-" + wednesdayRR.ToString() + Environment.NewLine;
                    txtWednesdayTTest.Text += "P-value = " + wednesdayPValue.ToString() + Environment.NewLine;
                    txtWednesdayTTest.Text += "Decision : Fail to Reject Ho" + Environment.NewLine;
                    txtWednesdayTTest.Text += "Conclusion : Not significantly different" + Environment.NewLine;
                    wednesdayTTest_conclusion = ", Your walking did not changed from the usual pattern";
                    txtWednesdayTTest.Text += "" + Environment.NewLine;
                    txtWednesdayTTest.Text += "Speed variability is " + wednesdayDeviation.ToString() + "% " + deviationQualifier + Environment.NewLine;
                }
            }


            //T-Test for Thursday *****************************************************************************

            if (Convert.ToDouble(dataGridView7.Rows[0].Cells[14].Value) == 0 || Convert.ToDouble(dataGridView7.Rows[0].Cells[27].Value) == 0)
            {
                txtThursdayTTest.Text = "";
                txtThursdayTTest.Text += "No data available for Thursday" + Environment.NewLine;
                thursdayTTest_conclusion = "On Thursday, I'm sorry there was no data available to check your walking status.";
            }
            else
            {
                TTestResult thursdayTTest = chart3.DataManipulator.Statistics.TTestUnequalVariances(0.00, 0.05, "ThisThursdayCV", "All_CVsBeforeThisWeek");
                double thursdayTValue = Math.Round(thursdayTTest.TValue, 4);
                double thursdayRR = Math.Round(thursdayTTest.TCriticalValueTwoTail, 4);
                double thursdayPValue = Math.Round(thursdayTTest.ProbabilityTTwoTail, 4);
                double thursdayMean = thursdayTTest.FirstSeriesMean;
                double pastMean = thursdayTTest.SecondSeriesMean;
                thursdayDeviation = Math.Round(((thursdayMean - pastMean) / pastMean) * 100, 1);

                dataGridView5.Rows[4].Cells[7].Value = thursdayDeviation;    //Display performance rating in data-grid-view 5

                string deviationQualifier;

                if ((thursdayMean - pastMean) > 0)
                {
                    deviationQualifier = "higher than usual";
                }
                else
                {
                    deviationQualifier = "lower than usual";
                }

                if (thursdayPValue < 0.05)
                {
                    txtThursdayTTest.Text = "";
                    txtThursdayTTest.Text += "Thursday's T-Test Result" + Environment.NewLine;
                    txtThursdayTTest.Text += "" + Environment.NewLine;
                    txtThursdayTTest.Text += "T value : " + thursdayTValue.ToString() + Environment.NewLine;
                    txtThursdayTTest.Text += "Rejectio Region : " + thursdayRR.ToString() + "  and  " + "-" + thursdayRR.ToString() + Environment.NewLine;
                    txtThursdayTTest.Text += "P-value = " + thursdayPValue.ToString() + Environment.NewLine;
                    txtThursdayTTest.Text += "Decision : Reject Ho" + Environment.NewLine;
                    txtThursdayTTest.Text += "Conclusion : Significantly different" + Environment.NewLine;
                    thursdayTTest_conclusion = ", Your walking changed significantly from the usual pattern";
                    txtThursdayTTest.Text += "" + Environment.NewLine;
                    txtThursdayTTest.Text += "Speed variability is " + thursdayDeviation.ToString() + "% " + deviationQualifier + Environment.NewLine;
                }
                else
                {
                    txtThursdayTTest.Text = "";
                    txtThursdayTTest.Text += "Thursday's T-Test Result" + Environment.NewLine;
                    txtThursdayTTest.Text += "" + Environment.NewLine;
                    txtThursdayTTest.Text += "T value : " + thursdayTValue.ToString() + Environment.NewLine;
                    txtThursdayTTest.Text += "Rejectio Region : " + thursdayRR.ToString() + "  and  " + "-" + thursdayRR.ToString() + Environment.NewLine;
                    txtThursdayTTest.Text += "P-value = " + thursdayPValue.ToString() + Environment.NewLine;
                    txtThursdayTTest.Text += "Decision : Fail to Reject Ho" + Environment.NewLine;
                    txtThursdayTTest.Text += "Conclusion : Not significantly different" + Environment.NewLine;
                    thursdayTTest_conclusion = ", Your walking did not changed from the usual pattern";
                    txtThursdayTTest.Text += "" + Environment.NewLine;
                    txtThursdayTTest.Text += "Speed variability is " + thursdayDeviation.ToString() + "% " + deviationQualifier + Environment.NewLine;
                }
            }

            //T-Test for Friday *****************************************************************************

            if (Convert.ToDouble(dataGridView7.Rows[0].Cells[17].Value) == 0 || Convert.ToDouble(dataGridView7.Rows[0].Cells[27].Value) == 0)
            {
                txtFridayTTest.Text = "";
                txtFridayTTest.Text += "No data available for Friday" + Environment.NewLine;
                fridayTTest_conclusion = "On Friday, I'm sorry there was no data available to check your walking status.";
            }
            else
            {
                TTestResult fridayTTest = chart3.DataManipulator.Statistics.TTestUnequalVariances(0.00, 0.05, "ThisFridayCV", "All_CVsBeforeThisWeek");
                double fridayTValue = Math.Round(fridayTTest.TValue, 4);
                double fridayRR = Math.Round(fridayTTest.TCriticalValueTwoTail, 4);
                double fridayPValue = Math.Round(fridayTTest.ProbabilityTTwoTail, 4);
                double fridayMean = fridayTTest.FirstSeriesMean;
                double pastMean = fridayTTest.SecondSeriesMean;
                fridayDeviation = Math.Round(((fridayMean - pastMean) / pastMean) * 100, 1);

                dataGridView5.Rows[5].Cells[7].Value = fridayDeviation;    //Display performance rating in data-grid-view 5

                string deviationQualifier;

                if ((fridayMean - pastMean) > 0)
                {
                    deviationQualifier = "higher than usual";
                }
                else
                {
                    deviationQualifier = "lower than usual";
                }

                if (fridayPValue < 0.05)
                {
                    txtFridayTTest.Text = "";
                    txtFridayTTest.Text += "Friday's T-Test Result" + Environment.NewLine;
                    txtFridayTTest.Text += "" + Environment.NewLine;
                    txtFridayTTest.Text += "T value : " + fridayTValue.ToString() + Environment.NewLine;
                    txtFridayTTest.Text += "Rejectio Region : " + fridayRR.ToString() + "  and  " + "-" + fridayRR.ToString() + Environment.NewLine;
                    txtFridayTTest.Text += "P-value = " + fridayPValue.ToString() + Environment.NewLine;
                    txtFridayTTest.Text += "Decision : Reject Ho" + Environment.NewLine;
                    txtFridayTTest.Text += "Conclusion : Significantly different" + Environment.NewLine;
                    fridayTTest_conclusion = ", Your walking changed significantly from the usual pattern";
                    txtFridayTTest.Text += "" + Environment.NewLine;
                    txtFridayTTest.Text += "Speed variability is " + fridayDeviation.ToString() + "% " + deviationQualifier + Environment.NewLine;
                }
                else
                {
                    txtFridayTTest.Text = "";
                    txtFridayTTest.Text += "Friday's T-Test Result" + Environment.NewLine;
                    txtFridayTTest.Text += "" + Environment.NewLine;
                    txtFridayTTest.Text += "T value : " + fridayTValue.ToString() + Environment.NewLine;
                    txtFridayTTest.Text += "Rejectio Region : " + fridayRR.ToString() + "  and  " + "-" + fridayRR.ToString() + Environment.NewLine;
                    txtFridayTTest.Text += "P-value = " + fridayPValue.ToString() + Environment.NewLine;
                    txtFridayTTest.Text += "Decision : Fail to Reject Ho" + Environment.NewLine;
                    txtFridayTTest.Text += "Conclusion : Not significantly different" + Environment.NewLine;
                    fridayTTest_conclusion = ", Your walking did not changed from the usual pattern";
                    txtFridayTTest.Text += "" + Environment.NewLine;
                    txtFridayTTest.Text += "Speed variability is " + fridayDeviation.ToString() + "% " + deviationQualifier + Environment.NewLine;
                }
            }

            //T-Test for Saturday *****************************************************************************

            if (Convert.ToDouble(dataGridView7.Rows[0].Cells[20].Value) == 0 || Convert.ToDouble(dataGridView7.Rows[0].Cells[27].Value) == 0)
            {
                txtSaturdayTTest.Text = "";
                txtSaturdayTTest.Text += "No data available for Saturday" + Environment.NewLine;
                saturdayTTest_conclusion = "On Saturday, I'm sorry there was no data available to check your walking status.";
            }
            else
            {
                TTestResult saturdayTTest = chart3.DataManipulator.Statistics.TTestUnequalVariances(0.00, 0.05, "ThisSaturdayCV", "All_CVsBeforeThisWeek");
                double saturdayTValue = Math.Round(saturdayTTest.TValue, 4);
                double saturdayRR = Math.Round(saturdayTTest.TCriticalValueTwoTail, 4);
                double saturdayPValue = Math.Round(saturdayTTest.ProbabilityTTwoTail, 4);
                double saturdayMean = saturdayTTest.FirstSeriesMean;
                double pastMean = saturdayTTest.SecondSeriesMean;
                saturdayDeviation = Math.Round(((saturdayMean - pastMean) / pastMean) * 100, 1);

                dataGridView5.Rows[6].Cells[7].Value = saturdayDeviation;    //Display performance rating in data-grid-view 5

                string deviationQualifier;

                if ((saturdayMean - pastMean) > 0)
                {
                    deviationQualifier = "higher than usual";
                }
                else
                {
                    deviationQualifier = "lower than usual";
                }

                if (saturdayPValue < 0.05)
                {
                    txtSaturdayTTest.Text = "";
                    txtSaturdayTTest.Text += "Saturday's T-Test Result" + Environment.NewLine;
                    txtSaturdayTTest.Text += "" + Environment.NewLine;
                    txtSaturdayTTest.Text += "T value : " + saturdayTValue.ToString() + Environment.NewLine;
                    txtSaturdayTTest.Text += "Rejectio Region : " + saturdayRR.ToString() + "  and  " + "-" + saturdayRR.ToString() + Environment.NewLine;
                    txtSaturdayTTest.Text += "P-value = " + saturdayPValue.ToString() + Environment.NewLine;
                    txtSaturdayTTest.Text += "Decision : Reject Ho" + Environment.NewLine;
                    txtSaturdayTTest.Text += "Conclusion : Significantly different" + Environment.NewLine;
                    saturdayTTest_conclusion = ", Your walking changed significantly from the usual pattern";
                    txtSaturdayTTest.Text += "" + Environment.NewLine;
                    txtSaturdayTTest.Text += "Speed variability is " + saturdayDeviation.ToString() + "% " + deviationQualifier + Environment.NewLine;
                }
                else
                {
                    txtSaturdayTTest.Text = "";
                    txtSaturdayTTest.Text += "Saturday's T-Test Result" + Environment.NewLine;
                    txtSaturdayTTest.Text += "" + Environment.NewLine;
                    txtSaturdayTTest.Text += "T value : " + saturdayTValue.ToString() + Environment.NewLine;
                    txtSaturdayTTest.Text += "Rejectio Region : " + saturdayRR.ToString() + "  and  " + "-" + saturdayRR.ToString() + Environment.NewLine;
                    txtSaturdayTTest.Text += "P-value = " + saturdayPValue.ToString() + Environment.NewLine;
                    txtSaturdayTTest.Text += "Decision : Fail to Reject Ho" + Environment.NewLine;
                    txtSaturdayTTest.Text += "Conclusion : Not significantly different" + Environment.NewLine;
                    saturdayTTest_conclusion = ", Your walking did not changed from the usual pattern";
                    txtSaturdayTTest.Text += "" + Environment.NewLine;
                    txtSaturdayTTest.Text += "Speed variability is " + saturdayDeviation.ToString() + "% " + deviationQualifier + Environment.NewLine;
                }
            }

            //T-Test for Sunday *****************************************************************************

            if (Convert.ToDouble(dataGridView7.Rows[0].Cells[2].Value) == 0 || Convert.ToDouble(dataGridView7.Rows[0].Cells[27].Value) == 0)
            {
                txtSundayTTest.Text = "";
                txtSundayTTest.Text += "No data available for Sunday" + Environment.NewLine;
                sundayTTest_conclusion = "On Sunday, I'm sorry there was no data available to check your walking status.";
            }
            else
            {
                TTestResult sundayTTest = chart3.DataManipulator.Statistics.TTestUnequalVariances(0.00, 0.05, "ThisSundayCV", "All_CVsBeforeThisWeek");
                double sundayTValue = Math.Round(sundayTTest.TValue, 4);
                double sundayRR = Math.Round(sundayTTest.TCriticalValueTwoTail, 4);
                double sundayPValue = Math.Round(sundayTTest.ProbabilityTTwoTail, 4);
                double sundayMean = sundayTTest.FirstSeriesMean;
                double pastMean = sundayTTest.SecondSeriesMean;
                sundayDeviation = Math.Round(((sundayMean - pastMean) / pastMean) * 100, 1);

                dataGridView5.Rows[0].Cells[7].Value = sundayDeviation;    //Display performance rating in data-grid-view 5

                string deviationQualifier;

                if ((sundayMean - pastMean) > 0)
                {
                    deviationQualifier = "higher than usual";
                }
                else
                {
                    deviationQualifier = "lower than usual";
                }

                if (sundayPValue < 0.05)
                {
                    txtSundayTTest.Text = "";
                    txtSundayTTest.Text += "Sunday's T-Test Result" + Environment.NewLine;
                    txtSundayTTest.Text += "" + Environment.NewLine;
                    txtSundayTTest.Text += "T value : " + sundayTValue.ToString() + Environment.NewLine;
                    txtSundayTTest.Text += "Rejectio Region : " + sundayRR.ToString() + "  and  " + "-" + sundayRR.ToString() + Environment.NewLine;
                    txtSundayTTest.Text += "P-value = " + sundayPValue.ToString() + Environment.NewLine;
                    txtSundayTTest.Text += "Decision : Reject Ho" + Environment.NewLine;
                    txtSundayTTest.Text += "Conclusion : Significantly different" + Environment.NewLine;
                    sundayTTest_conclusion = ", Your walking changed significantly from the usual pattern";
                    txtSundayTTest.Text += "" + Environment.NewLine;
                    txtSundayTTest.Text += "Speed variability is " + sundayDeviation.ToString() + " % " + deviationQualifier + Environment.NewLine;
                }
                else
                {
                    txtSundayTTest.Text = "";
                    txtSundayTTest.Text += "Sunday's T-Test Result" + Environment.NewLine;
                    txtSundayTTest.Text += "" + Environment.NewLine;
                    txtSundayTTest.Text += "T value : " + sundayTValue.ToString() + Environment.NewLine;
                    txtSundayTTest.Text += "Rejectio Region : " + sundayRR.ToString() + "  and  " + "-" + sundayRR.ToString() + Environment.NewLine;
                    txtSundayTTest.Text += "P-value = " + sundayPValue.ToString() + Environment.NewLine;
                    txtSundayTTest.Text += "Decision : Fail to Reject Ho" + Environment.NewLine;
                    txtSundayTTest.Text += "Conclusion : Not significantly different" + Environment.NewLine;
                    sundayTTest_conclusion = ", Your walking did not changed from the usual pattern";
                    txtSundayTTest.Text += "" + Environment.NewLine;
                    txtSundayTTest.Text += "Speed variability is " + sundayDeviation.ToString() + "% " + deviationQualifier + Environment.NewLine;
                }
            }

            //Chart that displays Performance Rating for the various days of the week
            chart7.Series["DailyRatings"].XValueMember = "DaysOfWeek";
            chart7.Series["DailyRatings"].YValueMembers = "Performance_Rating";
            chart7.DataSource = dataGridView5.DataSource;
            chart7.DataBind();
        }



        //Report functions ------  2.5  ----------------------  2.5  ----------------------  2.5  -----------------------  2.5  -------------------------  2.5  -------------------------  2.5  -----------------------  2.5  --------------------

        string path2 = Environment.CurrentDirectory + "/" + "todays_report.txt";

        private void lblDisplaySegment_Click(object sender, EventArgs e)
        {

        }

        //string file2 = @"C:\Users\HHCI\Documents\Kinect Joint Coordinate App\Kinect Joint Coordinate App\bin\Debug\todays_report.txt";
        private void Text_Report_CurrentWeek()
        {

            if (!File.Exists(path2))
            {
                File.CreateText(path2);
            }

            string reportHeader = "DAY" + "         "+ "G.S. (m/s)" + "            " + "C.V. (%)" + "             " + "P.R. (%)" ;

            string sundayDetails = "SUN" + "\t    " + aveSundayGS.ToString("#.##") + "\t           " + aveSundayCV.ToString("#.##") + "\t" + sundayDeviation.ToString("#.#");
            string mondayDetails = "MON" + "\t    " + aveMondayGS.ToString("#.##") + "\t           " + aveMondayCV.ToString("#.##") + "\t                    " + mondayDeviation.ToString("#.#");
            string tuedayDetails = "TUE" + "\t    " + aveTuesdayGS.ToString("#.##") + "\t           " + aveTuesdayCV.ToString("#.##") + "\t                    " + tuesdayDeviation.ToString("#.##");
            string wednesdayDetails = "WED" + "\t    " + aveWednesdayGS.ToString("#.##") + "\t           " + aveWednesdayCV.ToString("#.##") + "\t" + wednesdayDeviation.ToString("#.#");
            string thursdayDetails = "THU" + "\t    " + aveThursdayGS.ToString("#.##") + "\t           " + aveThursdayCV.ToString("#.##") + "\t" + thursdayDeviation.ToString("#.#");
            string fridayDetails = "FRI" + "\t    " + aveFridayGS.ToString("#.##") + "\t           " + aveFridayCV.ToString("#.##") + "\t                    " + fridayDeviation.ToString("#.#");
            string saturdayDetails = "SAT" + "\t    " + aveSaturdayGS.ToString("#.##") + "\t           " + aveSaturdayCV.ToString("#.##") + "\t" + saturdayDeviation.ToString("#.#");


            using (StreamWriter sw = new StreamWriter(path2))
            {
                sw.WriteLine("WEEKLY GAIT REPORT");
                sw.WriteLine(" ");
                sw.WriteLine(reportHeader);
                sw.WriteLine(sundayDetails);
                sw.WriteLine(mondayDetails);
                sw.WriteLine(tuedayDetails);
                sw.WriteLine(wednesdayDetails);
                sw.WriteLine(thursdayDetails);
                sw.WriteLine(fridayDetails);
                sw.WriteLine(saturdayDetails);
            }

            Process.Start(@"notepad.exe", path2);

        }

        
        private void Text_Report_Daily(string day, string testConclusions, double aveGaitSpeed, double aveCV, double dayDeviation, bool dataExist)
        {
            if (!File.Exists(path2))
            {
                File.CreateText(path2);
            }

            string dailyTestConclusions = testConclusions.TrimStart(',',' ') + ". Further details are shown below.";
            string dailyGaitSpeedStatement = "=> Gait speed was " + aveGaitSpeed.ToString() + "m/s" ;
            string dailyCVStatement = "=> Speed variability was " + aveCV.ToString() + "%";

            string deviationQualifier;

            if (dayDeviation > 0)
            {
                deviationQualifier = "higher";
            }
            else
            {
                deviationQualifier = "lower";
            }

            string dailyDeviationStatement = "=> Variability deviation was " + Math.Abs(dayDeviation).ToString() + "% " + deviationQualifier + " than the usual";


            if (dataExist == true)
            {
                using (StreamWriter sw = new StreamWriter(path2))
                {
                    sw.WriteLine(day.ToUpper() + " GAIT REPORT");
                    sw.WriteLine(" ");
                    sw.WriteLine(dailyTestConclusions);
                    sw.WriteLine(" ");
                    sw.WriteLine(dailyGaitSpeedStatement);
                    sw.WriteLine(" ");
                    sw.WriteLine(dailyCVStatement);
                    sw.WriteLine(" ");
                    sw.WriteLine(dailyDeviationStatement);
                }

                //Process.Start(@"notepad.exe", path2);
                Process dailyReport = Process.Start("todays_report.txt");
            }
            else
            {
                using (StreamWriter sw = new StreamWriter(path2))
                {
                    sw.WriteLine(testConclusions); 
                }

                //Process.Start(@"notepad.exe", path2);
                Process dailyReport = Process.Start("todays_report.txt");
                                
            }

           
          
        }

        
      //Speech Recognition functions ------  2.6  ----------------------  2.6  ----------------------  2.6  -----------------------  2.6  -------------------------  2.6  -------------------------  2.6  -----------------------  2.6  --------------------
        private void sre_SpeechRecognized(object sender, SpeechRecognizedEventArgs e)   //Speech commands for voice and text reports
        {
            switch (e.Result.Text.ToString())
            {
                case "Hello kinect":
                    ss.SpeakAsync("I'm doing great, what about you");
                    break;

                case "Kinect, How are you":
                    ss.SpeakAsync("I'm doing good, how about you");
                    break;

                case "Kinect What is the current time":
                    ss.SpeakAsync("The current time is" + DateTime.Now.ToLongTimeString());
                    break;

                case "Kinect, Thank you":
                    ss.SpeakAsync("pleasure is mine kinect user");
                    break;

                case "Kinect":
                    ss.SpeakAsync("i'm listening");
                    break;

                case "Kinect shutup":
                    ss.SpeakAsync("I'm sorry if I made you angry, I am just a computer simulation coded by Emmanuel and Ebrahim");
                    break;

                case "Kinect are you out of your mind":
                    ss.SpeakAsync("No please, I do not have a brain, so I cannot go mad.");
                    break;

                case "Kinect who am I":
                    ss.SpeakAsync("Please stop asking silly questions, you know I can't answer that.");
                    break;


             //Text Reports for the various days ***************************  TR  *********************************  TR  *******************************  TR  **************************

                case "Kinect show today's report":
                    if (Convert.ToDouble(dataGridView7.Rows[0].Cells[23].Value) == 0)
                    {
                        Text_Report_Daily("Today's", todaysTTest_conclusion, aveOverallGaitSpeed, aveOverallCV, todayDeviation, false);
                    }
                    else
                    {
                        Text_Report_Daily("Today's", todaysTTest_conclusion, aveOverallGaitSpeed, aveOverallCV, todayDeviation, true);
                        string todaysVoiceReport = Voice_Report("Today", todaysTTest_conclusion, aveOverallGaitSpeed, aveOverallCV, todayDeviation);
                        ss.SpeakAsync(todaysVoiceReport);
                    }                  
                    
                    break;

                case "Kinect show Sunday's report":
                    if (Convert.ToDouble(dataGridView7.Rows[0].Cells[2].Value) == 0)
                    {
                        Text_Report_Daily("Sunday", sundayTTest_conclusion, aveSundayGS, aveSundayCV, sundayDeviation, false);
                    }
                    else
                    {
                        Text_Report_Daily("Sunday", sundayTTest_conclusion, aveSundayGS, aveSundayCV, sundayDeviation, true);
                    }
                    
                    break;

                case "Kinect show Monday's report":
                    if (Convert.ToDouble(dataGridView7.Rows[0].Cells[5].Value) == 0)
                    {
                        Text_Report_Daily("Monday", mondayTTest_conclusion, aveMondayGS, aveMondayCV, mondayDeviation, false);
                    }
                    else
                    {
                        Text_Report_Daily("Monday", mondayTTest_conclusion, aveMondayGS, aveMondayCV, mondayDeviation, true);
                    }
                    
                    break;

                case "Kinect show Tuesday's report":
                    if (Convert.ToDouble(dataGridView7.Rows[0].Cells[8].Value) == 0)
                    {
                        Text_Report_Daily("Tuesday", tuesdayTTest_conclusion, aveTuesdayGS, aveTuesdayCV, tuesdayDeviation, false);
                    }
                    else
                    {
                        Text_Report_Daily("Tuesday", tuesdayTTest_conclusion, aveTuesdayGS, aveTuesdayCV, tuesdayDeviation, true);
                    }

                    break;

                case "Kinect show Wednesday's report":
                    if (Convert.ToDouble(dataGridView7.Rows[0].Cells[11].Value) == 0)
                    {
                        Text_Report_Daily("Wednesday", wednesdayTTest_conclusion, aveWednesdayGS, aveWednesdayCV, wednesdayDeviation, false);
                    }
                    else
                    {
                        Text_Report_Daily("Wednesday", wednesdayTTest_conclusion, aveWednesdayGS, aveWednesdayCV, wednesdayDeviation, true);
                    }

                    break;

                case "Kinect show Thursday's report":
                    if (Convert.ToDouble(dataGridView7.Rows[0].Cells[14].Value) == 0)
                    {
                        Text_Report_Daily("Thursday", thursdayTTest_conclusion, aveThursdayGS, aveThursdayCV, thursdayDeviation, false);
                    }
                    else
                    {
                        Text_Report_Daily("Thursday", thursdayTTest_conclusion, aveThursdayGS, aveThursdayCV, thursdayDeviation, true);
                    }

                    break;

                case "Kinect show Friday's report":
                    if (Convert.ToDouble(dataGridView7.Rows[0].Cells[17].Value) == 0)
                    {
                        Text_Report_Daily("Friday", fridayTTest_conclusion, aveFridayGS, aveFridayCV, fridayDeviation, false);
                    }
                    else
                    {
                        Text_Report_Daily("Friday", fridayTTest_conclusion, aveFridayGS, aveFridayCV, fridayDeviation, true);
                    }

                    break;

                case "Kinect show Saturday's report":
                    if (Convert.ToDouble(dataGridView7.Rows[0].Cells[20].Value) == 0)
                    {
                        Text_Report_Daily("Saturday", saturdayTTest_conclusion, aveSaturdayGS, aveSaturdayCV, saturdayDeviation, false);
                    }
                    else
                    {
                        Text_Report_Daily("Saturday", saturdayTTest_conclusion, aveSaturdayGS, aveSaturdayCV, saturdayDeviation, true);
                    }

                    break;

                case "Kinect show weekly report":
                    ss.SpeakAsync("Here you go!");
                    Text_Report_CurrentWeek();
                    
                    break;

                case "Kinect close report":
                    //Process dailyReport = Process.Start("todays_report.txt");
                    //dailyReport.Kill();

                    break;

                //Voice Reports for the various days *************************  VR  *****************************  VR  ******************************  VR  *********************************                     

                case "Kinect today's report":
                    if (Convert.ToDouble(dataGridView7.Rows[0].Cells[23].Value) == 0)
                    {
                        ss.SpeakAsync(todaysTTest_conclusion);
                    }
                    else
                    {
                        string todaysVoiceReport = Voice_Report("Today", todaysTTest_conclusion, aveOverallGaitSpeed, aveOverallCV, todayDeviation);
                        ss.SpeakAsync(todaysVoiceReport);
                                                
                    }
                    
                    break;

                case "Kinect Sunday's report":
                    if (Convert.ToDouble(dataGridView7.Rows[0].Cells[2].Value) == 0)
                    {
                        ss.SpeakAsync(sundayTTest_conclusion);
                    }
                    else
                    {
                        string sundaysVoiceReport = Voice_Report("Sunday", sundayTTest_conclusion, aveSundayGS, aveSundayCV, sundayDeviation);
                        ss.SpeakAsync(sundaysVoiceReport);
                    }
                    
                    break;

                case "Kinect Monday's report":
                    if (Convert.ToDouble(dataGridView7.Rows[0].Cells[5].Value) == 0)
                    {
                        ss.SpeakAsync(mondayTTest_conclusion);
                    }
                    else
                    {
                        string mondaysVoiceReport = Voice_Report("Monday", mondayTTest_conclusion, aveMondayGS, aveMondayCV, mondayDeviation);
                        ss.SpeakAsync(mondaysVoiceReport);
                    }
                    
                    break;

                case "Kinect Tuesday's report":
                    if (Convert.ToDouble(dataGridView7.Rows[0].Cells[8].Value) == 0)
                    {
                        ss.SpeakAsync(tuesdayTTest_conclusion);
                    }
                    else
                    {
                        string tuesdaysVoiceReport = Voice_Report("Thursday", tuesdayTTest_conclusion, aveTuesdayGS, aveTuesdayCV, tuesdayDeviation);
                        ss.SpeakAsync(tuesdaysVoiceReport);
                    }
                    
                    break;

                case "Kinect Wednesday's report":
                    if (Convert.ToDouble(dataGridView7.Rows[0].Cells[11].Value) == 0)
                    {
                        ss.SpeakAsync(wednesdayTTest_conclusion);
                    }
                    else
                    {
                        string wednesdaysVoiceReport = Voice_Report("Wednesday", wednesdayTTest_conclusion, aveWednesdayGS, aveWednesdayCV, wednesdayDeviation);
                        ss.SpeakAsync(wednesdaysVoiceReport);
                    }
                    
                    break;

                case "Kinect Thursday's report":
                    if (Convert.ToDouble(dataGridView7.Rows[0].Cells[14].Value) == 0)
                    {
                        ss.SpeakAsync(thursdayTTest_conclusion);
                    }
                    else
                    {
                        string thursdaysVoiceReport = Voice_Report("Thursday", thursdayTTest_conclusion, aveThursdayGS, aveThursdayCV, thursdayDeviation);
                        ss.SpeakAsync(thursdaysVoiceReport);
                    }
                    
                    break;

                case "Kinect Friday's report":
                    if (Convert.ToDouble(dataGridView7.Rows[0].Cells[17].Value) == 0)
                    {
                        ss.SpeakAsync(fridayTTest_conclusion);
                    }
                    else
                    {
                        string fridaysVoiceReport = Voice_Report("Friday", fridayTTest_conclusion, aveFridayGS, aveFridayCV, fridayDeviation);
                        ss.SpeakAsync(fridaysVoiceReport);
                    }

                    break;
                  
                case "Kinect Saturday's report":
                    if (Convert.ToDouble(dataGridView7.Rows[0].Cells[20].Value) == 0)
                    {
                        ss.SpeakAsync(saturdayTTest_conclusion);
                    }
                    else
                    {
                        string saturdaysVoiceReport = Voice_Report("Saturday", saturdayTTest_conclusion, aveSaturdayGS, aveSaturdayCV, saturdayDeviation);
                        ss.SpeakAsync(saturdaysVoiceReport);
                    }
                    
                    break;

                case "Kinect Close":
                    Application.Exit();
                    break;
                //Process dailyReport = Process.Start("todays_report.txt");
                //dailyReport.Kill();

                default:
                    ss.SpeakAsync("Sorry i do not get you");
                    //ss.SpeakAsync("These are the options, Hello kinect, Kinect How are you, What is the current time, Kinect current report, Thank you, Kinect today's report");
                    break;

            }
            txtSpeech.Text += e.Result.Text.ToString() + Environment.NewLine;
        }

        private string Voice_Report(string day, string testConclusions, double aveGaitSpeed, double aveCV, double dayDeviation)
        {

            string deviationQualifier;

            if (dayDeviation > 0)
            {
                deviationQualifier = "higher";
            }
            else
            {
                deviationQualifier = "lower";
            }

            if (day == "Today")
            {
                string voiceReport = testConclusions + " for today" + ". Here are more details. Your gait speed is " + aveGaitSpeed.ToString() + "meters per second, Your speed variability is " + aveCV.ToString() + " percent, and your variability deviation is " + Math.Abs(dayDeviation).ToString() + " percent " + deviationQualifier + " than the usual";

                return voiceReport;
            }
            else
            {
                string voiceReport = "On " + day + testConclusions + ". Here are more details. Your gait speed was " + aveGaitSpeed.ToString() + "meters per second, Your speed variability was " + aveCV.ToString() + " percent, and your variability deviation was " + Math.Abs(dayDeviation).ToString() + " percent " + deviationQualifier + " than the usual";

                return voiceReport;
            }
                        
        }



    //EVENTS --------  3  ----------------------------------------------------------  3  ----------------------------------------------------------  3  ------------------------------------------------------  3  -------------

      //Form-load events ------  3.1  ----------------------  3.1  ----------------------  3.1  -----------------------  3.1  -------------------------  3.1  -------------------------  3.1  -----------------------  3.1  --------------------
        public Form1()  //Load main windows form
        {
            InitializeComponent();
            //initialiseKinect();

            timer2.Start();
            isActive2 = true;
            //isActive3 = true;
            Refresh_Data();
           
        }



      //Time events ------  3.2  ----------------------  3.2  ----------------------  3.2  -----------------------  3.2  -------------------------  3.2  -------------------------  3.2  -----------------------  3.2  --------------------
        DateTime currentTime1 = DateTime.Now;

        public int timeDays, timeHours, timeMin, timeSec, timeCsec;
        public int timeSec2;
        public int timeCsec3;
        public bool isActive;
        public bool isActive2;
        public bool isActive3;
        public void timer1_Tick(object sender, EventArgs e)     //Time event for computing velocity and acceleration, and showing duration of run for the sensor
        {
            if (isActive)
            {
                timeCsec++;

                if (timeCsec >= 8)
                {
                    timeSec++;
                    timeCsec = 0;

                    if (timeSec >= 60)
                    {
                        timeMin++;
                        timeSec = 0;

                        if (timeMin >= 60)
                        {
                            timeHours++;
                            timeMin = 0;

                            if (timeHours >= 24)
                            {
                                timeDays++;
                                timeHours = 0;
                                                                
                            }

                        }
                    }
                }
            }

            DrawTime();

            //DateTime currentTime1 = DateTime.Now;
            //this.lblCurrentTime.Text = currentTime1.ToString("hh:mm:ss:ff");
            //this.lblCurrentDate.Text = currentTime1.ToString("MM/dd/yyyy");
        }

        private void timer2_Tick(object sender, EventArgs e)    //Time event for refreshing datatables and graphs
        {
            DateTime currentTime1 = DateTime.Now;
            lblCurrentTime.Text = currentTime1.ToString("hh:mm:ss tt");
            lblCurrentDate.Text = currentTime1.ToString("MM/dd/yyyy");

            //Reset the displays to N/A when a new day starts
            if (lblCurrentTime.Text == "12:00:00 AM")
            {
                lblMorningAveCV.Text = "N/A";
                lblAfternoonAveCV.Text = "N/A";
                lblEveningAveCV.Text = "N/A";
                lblDailyAveCV.Text = "N/A";

                lblMorningAveGaitSpeed.Text = "N/A";
                lblAfternoonAveGaitSpeed.Text = "N/A";
                lblEveningAveGaitSpeed.Text = "N/A";
                lblOverallAveGaitSpeed.Text = "N/A";
            }         

            if (isActive2)
            {
                timeSec2++;

                if (timeSec2 >= 30)
                {
                    //DistinctNDisplayRawData();
                    //CalculateAllCVs();
                    //CalcNDisp5thN95thPercentile();
                    //Current_Session_Of_Day();
                    //DisplayDailyCV();
                    //Day_Of_Week();
                    //Weekly_Session_Of_Day();
                    //CVs_Before_This_Week();
                    //DisplayCharts();

                    timeSec2 = 0;

                }
            }

        }

        private void timer3_Tick(object sender, EventArgs e)
        {
            if (isActive3)
            {
                timeCsec3++;

                //if (timeSec2 >= 30)
                //{
                //    timeCsec3 = 0;
                //}
            }
        }

        //Button-click events ------  3.3  ----------------------  3.3  ----------------------  3.3  -----------------------  3.3  -------------------------  3.3  -------------------------  3.3  -----------------------  3.3  --------------------
        private void btnStartSensor_Click_1(object sender, EventArgs e) //Click to start and stop Sensor
        {
            if (btnStartSensor.Text == "Start Sensor")
            {
                ResetStopwatch_1();
                isActive = true;
                initialiseKinect();
                txtStatus.Text = "Running";
                btnStartSensor.Text = "Stop Sensor";

            }
            else if (kinectSensor.IsOpen)
            {
                isActive = false;
                kinectSensor.Close();
                txtStatus.Text = "Inactive";
                btnStartSensor.Text = "Start Sensor";

                //DisplayTableData();
                // RemoveDup();

            }
        }
        

        string path1 = Environment.CurrentDirectory + "/" + "ab.txt";
        //string file = @"C:\Users\HHCI\Documents\Kinect Joint Coordinate App\Kinect Joint Coordinate App\bin\Debug\ab2.txt";
        private void btnCreateFile_Click_1(object sender, EventArgs e)  //Click to create file for storing data to text file
        {
            if (!File.Exists(path1))
            {
                File.CreateText(path1);
                MessageBox.Show("File Created Successfully");

            }
            else
            {
                MessageBox.Show("File already created");
            }
        }
        

        private void btnClear_Click_1(object sender, EventArgs e)   //Click to clear main display textbox
        {
            txtDisplayTable.Text = "";
        }
        

        private void btnStopSpeech_Click(object sender, EventArgs e)    //Click to stop speech recognition
        {
            //stop button click
            sre.RecognizeAsyncStop();

            btnStartSpeech.Enabled = true;
            btnStopSpeech.Enabled = false;
        }


        SpeechSynthesizer ss = new SpeechSynthesizer();
        PromptBuilder pb = new PromptBuilder();
        SpeechRecognitionEngine sre = new SpeechRecognitionEngine();
        Choices Clist = new Choices();
        private void btnStartSpeech_Click(object sender, EventArgs e)   //Click to activate speech recognition
        {
            // start the command but the other button will be invisible
            btnStartSpeech.Enabled = false;
            btnStopSpeech.Enabled = true;
            Clist.Add(new string[] { "Kinect show today's report", "Kinect show Sunday's report", "Kinect show Monday's report", "Kinect show Tuesday's report", "Kinect show Wednesday's report", "Kinect show Thursday's report", "Kinect show Friday's report", "Kinect show Saturday's report", "Kinect show weekly report", "Kinect close report",
                                     "Kinect today's report", "Kinect Monday's report", "Kinect Sunday's report", "Kinect Tuesday's report", "Kinect Wednesday's report", "Kinect Thursday's report", "Kinect Friday's report", "Kinect Saturday's report",
                                     "Kinect", "Hello kinect", "Kinect, How are you", "Kinect What is the current time", "Kinect, Thank you", "Kinect shutup", "Kinect are you out of your mind", "Kinect who am I", "Kinect A", "Kinect E", "Kinect I", "KinectO", "Kinect U" , "Kinect Close"});
            
            Grammar gr = new Grammar(new GrammarBuilder(Clist));

            try
            {

                sre.RequestRecognizerUpdate();
                sre.LoadGrammar(gr);
                sre.SpeechRecognized += sre_SpeechRecognized;
                sre.SetInputToDefaultAudioDevice();
                sre.RecognizeAsync(RecognizeMode.Multiple);

            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message, "Error");

            }
        }


        private void button3_Click(object sender, EventArgs e)  //Click to refresh the datatables, graphes and all calculations
        {
            Refresh_Data();
                      
        }


        private void button1_Click(object sender, EventArgs e)  //Empty for now
        {
            Day_Of_Week();
        }
               
    }

   //CLASSES --------  4  ----------------------------------------------------------  4  ------------------------------------------------------------  4  ------------------------------------------------------  4  -------------
}

