    /* Copyright (c) 2019 FIRST. All rights reserved.
    *
    * Redistribution and use in source and binary forms, with or without modification,
    * are permitted (subject to the limitations in the disclaimer below) provided that
    * the following conditions are met:
    *
    * Redistributions of source code must retain the above copyright notice, this list
    * of conditions and the following disclaimer.
    *
    * Redistributions in binary form must reproduce the above copyright notice, this
    * list of conditions and the following disclaimer in the documentation and/or
    * other materials provided with the distribution.
    *
    * Neither the name of FIRST nor the names of its contributors may be used to endorse or
    * promote products derived from this software without specific prior written permission.
    *
    * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
    * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
    * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
    * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    */

    package org.firstinspires.ftc.teamcode;

    import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
    import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
    import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
    import com.acmerobotics.roadrunner.geometry.Pose2d;
    import com.acmerobotics.roadrunner.geometry.Vector2d;
    import com.acmerobotics.roadrunner.trajectory.Trajectory;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

    import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

    import com.acmerobotics.roadrunner.geometry.Pose2d;
    import com.acmerobotics.roadrunner.geometry.Vector2d;
    import com.acmerobotics.roadrunner.trajectory.Trajectory;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.robotcore.external.ClassFactory;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
    import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
    import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
    import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
    import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
    import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
    import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
    import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

    import java.util.ArrayList;
    import java.util.List;

    /**
    * This OpMode illustrates using the Vuforia localizer to determine positioning and orientation of
    * robot on the FTC field using a WEBCAM.  The code is structured as a LinearOpMode
    *
    * NOTE: If you are running on a Phone with a built-in camera, use the ConceptVuforiaFieldNavigation example instead of this one.
    * NOTE: It is possible to switch between multiple WebCams (eg: one for the left side and one for the right).
    *       For a related example of how to do this, see ConceptTensorFlowObjectDetectionSwitchableCameras
    *
    * When images are located, Vuforia is able to determine the position and orientation of the
    * image relative to the camera.  This sample code then combines that information with a
    * knowledge of where the target images are on the field, to determine the location of the camera.
    *
    * Finally, the location of the camera on the robot is used to determine the
    * robot's location and orientation on the field.
    *
    * To learn more about the FTC field coordinate model, see FTC_FieldCoordinateSystemDefinition.pdf in this folder
    *
    * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
    * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
    *
    * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
    * is explained below.
    */

@Autonomous(name="AutonomousBlue")
public class VuforiaAutonomousBlue extends LinearOpMode {

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code on the next line, between the double quotes.
         */
        private static final String VUFORIA_KEY =
                "ASg9akz/////AAABmUyMSYT980NPnVlwE+IUZvgcmQVycKh4y6qF5zwgIINrt/luYwZsjEBlHQR43ATLOgSb4zmCadznybzcf1EI0fTBLHFk0VArY96x/Cw6kulvlQF5BCmFHXqmWWulbDy7ASUWKVDK64OAbFSEJC0qqMZEYEQ/UHashdor5748WoqVfpnVs+8XeYMqZIDnnJpHHGbl1M4hGlzK0xVH96T1O0/hqsBAMd6XZjg+Whz04FziDdKSc6NVf65WXQop4m0rwbN+EnymPddCqnAj0xVVKefY8KuI7aJNDX/OUbDGUOWTb5hxl2KUbTYiUiGZQpqtLG2d8NpZ003naRIQRl0ev6+4yTFud9YyvAv2yNRgT82h";

        // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
        // We will define some constants and conversions here
        private static final float mmPerInch = 25.4f;
        private static final float mmTargetHeight = 6 * mmPerInch;          // the height of the center of the target image above the floor
        private static final float halfField = 72 * mmPerInch;
        private static final float halfTile = 12 * mmPerInch;
        private static final float oneAndHalfTile = 36 * mmPerInch;

        private SampleMecanumDrive drive;
        // Class Members
        private OpenGLMatrix lastLocation = null;
        private VuforiaLocalizer vuforia = null;
        private VuforiaTrackables targets = null;
        private WebcamName webcamName = null;
        private MecanumRobot robot;
        private List<VuforiaTrackable> allTrackables;

        private ElapsedTime runtime = new ElapsedTime();
        CheckConfig conf = null;

        VuforiaTrackable targetFound = null;

        private Pose2d startPose = new Pose2d(-37.33, 67.70, Math.toRadians(270));

        private boolean targetVisible = false;
        private String targetName = "";


        @Override
        public void runOpMode() {
            // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
            webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
            drive = new SampleMecanumDrive(hardwareMap, this);
            robot = new MecanumRobot();
            conf = new CheckConfig();
            conf.Init(this);

            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             * We can pass Vuforia the handle to a camera preview resource (on the RC screen);
             * If no camera-preview is desired, use the parameter-less constructor instead (commented out below).
             * Note: A preview window is required if you want to view the camera stream on the Driver Station Phone.
             */
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            robot.init(hardwareMap, this);
            parameters.vuforiaLicenseKey = VUFORIA_KEY;

            // We also indicate which camera we wish to use.
            parameters.cameraName = webcamName;

            // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
            parameters.useExtendedTracking = false;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Load the data sets for the trackable objects. These particular data
            // sets are stored in the 'assets' part of our application.
            targets = this.vuforia.loadTrackablesFromAsset("testq");

            // For convenience, gather together all the trackable objects in one easily-iterable collection */
            allTrackables = new ArrayList<VuforiaTrackable>();
            allTrackables.addAll(targets);

            /**
             * In order for localization to work, we need to tell the system where each target is on the field, and
             * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
             * Transformation matrices are a central, important concept in the math here involved in localization.
             * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
             * for detailed information. Commonly, you'll encounter transformation matrices as instances
             * of the {@link OpenGLMatrix} class.
             *
             * If you are standing in the Red Alliance Station looking towards the center of the field,
             *     - The X axis runs from your left to the right. (positive from the center to the right)
             *     - The Y axis runs from the Red Alliance Station towards the other side of the field
             *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
             *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
             *
             * Before being transformed, each target image is conceptually located at the origin of the field's
             *  coordinate system (the center of the field), facing up.
             */

            // Name and locate each trackable object
            allTrackables.get(1).setName("parking3");
            allTrackables.get(0).setName("parking2");

            // allTrackables.get(3).setName("parking4");
            //identifyTarget(0, "Red Audience Wall",   -halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0,  90);
            //        identifyTarget(1, "Red Rear Wall",        halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0, -90);
            //        identifyTarget(2, "Blue Audience Wall",  -halfField,   oneAndHalfTile, mmTargetHeight, 90, 0,  90);
            //        identifyTarget(3, "Blue Rear Wall",       halfField,   oneAndHalfTile, mmTargetHeight, 90, 0, -90);

            /*
             * Create a transformation matrix describing where the camera is on the robot.
             *
             * Info:  The coordinate frame for the robot looks the same as the field.
             * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
             * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
             *
             * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
             * with the wide (horizontal) axis of the camera aligned with the X axis, and
             * the Narrow (vertical) axis of the camera aligned with the Y axis
             *
             * But, this example assumes that the camera is actually facing forward out the front of the robot.
             * So, the "default" camera position requires two rotations to get it oriented correctly.
             * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out the right side of the robot)
             * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
             *
             * Finally the camera can be translated to its actual mounting position on the robot.
             *      In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
             */

            final float CAMERA_FORWARD_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
            final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
            final float CAMERA_LEFT_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

            OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

            /**  Let all the trackable listeners know where the camera is.  */
            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
            }

            /*
             * WARNING:
             * In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
             * This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
             * CONSEQUENTLY do not put any driving commands in this loop.
             * To restore the normal opmode structure, just un-comment the following line:
             */

            // waitForStart();

            /* Note: To use the remote camera preview:
             * AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
             * Tap the preview window to receive a fresh image.
             * It is not permitted to transition to RUN while the camera preview window is active.
             * Either press STOP to exit the OpMode, or use the "options menu" again, and select "Camera Stream" to close the preview window.
             */


            targets.activate();
            while (!isStopRequested()) {
                drive.setPoseEstimate(startPose);
                robot.claw.setPosition(.80);

                telemetry.addData("RUNTIME", runtime.seconds());
                telemetry.update();
                int count = 0;
                // check all the trackable targets to see which one (if any) is visible.
                targetVisible = false;
                targetFound = null;
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;
                        targetName = trackable.getName();
                        targetFound = trackable;
                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                    count++;
                }
                // Provide feedback as of which target its has found

                if (targetVisible) {
                    if (targetFound.equals(allTrackables.get(0))) {// parking 2
                        telemetry.addData("Robot is parking to %s", targetName);
                        telemetry.update();
                        waitForStart();
                        park();
                        break;
                    } else if (targetFound.equals(allTrackables.get(1))) {//parking 3
                        telemetry.addData("Robot is parking to %s", targetName);
                        telemetry.update();
                        waitForStart();
                        park();
                        break;
                    }
                } else if (runtime.seconds() > 10) {//parking 1
                    telemetry.addData("Parking 1 ", "");
                    telemetry.update();
                    waitForStart();
                    park();
                    break;
                }
            }
        }

        void park() {
            if(!conf.isLeft())// if robot in the right
                BandR();
            else
                BandL();
        }

        // right fuild side movement
        void BandR() {
            if (targetFound != null) {
                // if target is parking 2
                if (targetFound.equals(allTrackables.get(0))) {

                    robot.armUp(5);
                    Trajectory strafe = drive.trajectoryBuilder(new Pose2d(-37.33, 67.70, Math.toRadians(270.00)))
                            .strafeTo(new Vector2d(-64.89,67.70))
                            .build();
                    drive.followTrajectory(strafe);

                    Trajectory to_junc = drive.trajectoryBuilder(strafe.end())
                            .lineTo(new Vector2d(-64.89,24.22))
                            .splineTo(new Vector2d(-31.56, 9.56), Math.toRadians(-53.13))
                            .build();
                    drive.followTrajectory(to_junc);
                    robot.armUp(38);

                    Trajectory forward = drive.trajectoryBuilder(to_junc.end())
                            .forward(2)
                            .build();
                    drive.followTrajectory(forward);
                    robot.openClaw();
                    Trajectory back = drive.trajectoryBuilder(forward.end())
                            .back(6)
                            .build();
                    drive.followTrajectory(back);



                }
                // if target is parking 3
                else if (targetFound.equals(allTrackables.get(1))) {
                    robot.armUp(5);
                    Trajectory strafe = drive.trajectoryBuilder(new Pose2d(-37.33, 67.70, Math.toRadians(270.00)))
                            .strafeTo(new Vector2d(-64.89,67.70))
                            .build();
                    drive.followTrajectory(strafe);

                    Trajectory to_junc = drive.trajectoryBuilder(strafe.end())
                            .splineTo(new Vector2d(-58.67, 34.00), Math.toRadians(-41.63))
                            .build();
                  drive.followTrajectory(to_junc);
                    robot.armUp(13);

                    Trajectory forward = drive.trajectoryBuilder(to_junc.end())
                            .forward(3)
                            .build();
                    drive.followTrajectory(forward);
                    robot.openClaw();

                    Trajectory back = drive.trajectoryBuilder(forward.end())
                            .back(6)
                            .build();
                    drive.followTrajectory(back);

                }
                // if target is parking 1
            } else {
                robot.armUp(5);
                Trajectory untitled0 = drive.trajectoryBuilder(new Pose2d(-37.33, 67.70, Math.toRadians(270.00)))
                        .splineTo(new Vector2d(-14.07, 48.44), Math.toRadians(270.00))
                        .splineTo(new Vector2d(-7.56, 32.74), Math.toRadians(-51.63))
                        .build();
                drive.followTrajectory(untitled0);

                robot.armUp(36);

                Trajectory forward = drive.trajectoryBuilder(untitled0.end())
                        .forward(2)
                        .build();
                drive.followTrajectory(forward);
                robot.openClaw();
                Trajectory back = drive.trajectoryBuilder(forward.end())
                        .back(6)
                        .build();
                drive.followTrajectory(back);
            }
        }
        //left fuild side movement
            void BandL(){
            //change robot starting position
                Pose2d left_start = new Pose2d(37.33, 67.70, Math.toRadians(270));
                drive.setPoseEstimate(left_start);

                if (targetFound != null) {
                    // if target is parking 2
                    if (targetFound.equals(allTrackables.get(0))) {
                        robot.armUp(5);
                        Trajectory strafe = drive.trajectoryBuilder(new Pose2d(37.33, 67.70, Math.toRadians(270)))
                                .strafeTo(new Vector2d(64.00, 67.33))
                                .build();
                        drive.followTrajectory(strafe);

                        Trajectory to_junc = drive.trajectoryBuilder(strafe.end())
                                .lineTo(new Vector2d(64.00, 24.22))
                                .splineTo(new Vector2d(35.78, 10.89), Math.toRadians(221.42))
                                .build();
                        drive.followTrajectory(to_junc);
                        robot.armUp(38);

                        Trajectory forward = drive.trajectoryBuilder(to_junc.end())
                                .forward(3)
                                .build();
                        drive.followTrajectory(forward);

                        robot.openClaw();

                        Trajectory back = drive.trajectoryBuilder(forward.end())
                                .back(6)
                                .build();
                        drive.followTrajectory(back);


                    }
                    // if target is parking 3
                    else if (targetFound.equals(allTrackables.get(1))) {

                        robot.armUp(5);
                        Trajectory strafe = drive.trajectoryBuilder(new Pose2d(37.33, 67.70, Math.toRadians(270)))
                                .strafeTo(new Vector2d(12.33, 67.33))
                                .build();
                        drive.followTrajectory(strafe);

                        Trajectory go_to_junc = drive.trajectoryBuilder(strafe.end())
                                .splineTo(new Vector2d(6.00, 31.33), Math.toRadians(215.54))
                                .build();
                        drive.followTrajectory(go_to_junc);

                        robot.armUp(37);

                        Trajectory forward = drive.trajectoryBuilder(go_to_junc.end())
                                .forward(2)
                                .build();
                        drive.followTrajectory(forward);
                        robot.openClaw();

                        Trajectory back = drive.trajectoryBuilder(forward.end())
                                .back(6)
                                .build();
                        drive.followTrajectory(back);
                       }
                    // if target is parking 1
                } else {
                    robot.armUp(5);
                    Trajectory strafe = drive.trajectoryBuilder(new Pose2d(37.33, 67.70, Math.toRadians(270)))
                            .strafeTo(new Vector2d(64.00, 67.33))
                            .build();
                    drive.followTrajectory(strafe);

                    Trajectory go_to_junc = drive.trajectoryBuilder(strafe.end())
                            .splineTo(new Vector2d(58.44, 35.11), Math.toRadians(240))
                            .build();
                    drive.followTrajectory(go_to_junc);

                    robot.armUp(13);

                    Trajectory forward = drive.trajectoryBuilder(go_to_junc.end())
                            .forward(2)
                            .build();
                    drive.followTrajectory(forward);
                    robot.openClaw();

                    Trajectory back = drive.trajectoryBuilder(forward.end())
                            .back(6)
                            .build();
                    drive.followTrajectory(back);
            }
        }



//      void RandR(){
//        if (targetFound != null) {
//            // if target is parking 2
//            if (targetFound.equals(allTrackables.get(0)))
//            {robot.slideRight(30);}
//            // if target is parking 3
//            else if (targetFound.equals(allTrackables.get(1))){
//                robot.slideRight(30); robot.driveForwards(34);}
//            // if target is parking 1
//        } else {
//            robot.slideLeft(24);robot.driveForwards(45.5);}
//    }
//    void RandL(){
//        if (targetFound != null) {
//            // if target is parking 2
//            if (targetFound.equals(allTrackables.get(0)))
//            {robot.slideLeft(30);}
//            // if target is parking 3
//            else if (targetFound.equals(allTrackables.get(1))){
//                robot.slideLeft(30); robot.driveForwards(34);}
//            // if target is parking 1
//        } else {
//            robot.slideRight(24);robot.driveForwards(45.5);}
//    }
//
//    void BandL(){
//            if (targetFound != null) {
//                // if target is parking 2
//                if (targetFound.equals(allTrackables.get(0)))
//                {robot.slideLeft(30);}
//                // if target is parking 3
//                else if (targetFound.equals(allTrackables.get(1))){
//                    robot.slideLeft(30); robot.driveForwards(34);}
//                // if target is parking 1
//            } else {
//                robot.slideRight(24);robot.driveForwards(45.5);}
//    }

//            void level1(){
//               // robot.moveArm(50);
//                robot.driveForwards(5);
//                robot.openClaw();
//            }
//            void level2(){
//               // robot.moveArm(70);
//                robot.driveForwards(5);
//                robot.openClaw();
//            }
//            void level3(){
//               // robot.moveArm(90);
//                robot.driveForwards(5);
//                robot.openClaw();
//            }



    }




//    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
//        VuforiaTrackable aTarget = targets.get(targetIndex);
//        aTarget.setName(targetName);
//        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
//    }


        /* Initialize standard Hardware interfaces */
