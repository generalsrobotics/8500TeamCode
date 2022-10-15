  /* Copyright (c) 2017 FIRST. All rights reserved.
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

  import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
  import com.qualcomm.robotcore.hardware.LED;
  import com.qualcomm.robotcore.hardware.DigitalChannel;
  import java.util.Stack;
  import java.lang.annotation.Target;
  import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

  import com.qualcomm.robotcore.util.ElapsedTime;

  import com.qualcomm.robotcore.hardware.DcMotor;
  import com.qualcomm.robotcore.hardware.DcMotor;
  import com.qualcomm.robotcore.hardware.HardwareMap;
  import com.qualcomm.robotcore.hardware.Servo;
  import com.qualcomm.robotcore.util.ElapsedTime;

  /**
   * This is NOT an opmode.
   *
   * This class can be used to define all the specific hardware for a single robot.
   * In this case that robot is a Pushbot.
   * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
   *
   * This hardware class assumes the following device names have been configured on the robot:
   * Note:  All names are lower case and some have single spaces between words.
   *
   * Motor channel:  Left  drive motor:        "left_drive"
   * Motor channel:  Right drive motor:        "right_drive"
   * Motor channel:  Manipulator drive motor:  "left_arm"
   * Servo channel:  Servo to open left claw:  "left_hand"
   * Servo channel:  Servo to open right claw: "right_hand"
   */
  public class  MecanumRobot
  {
    /* Public OpMode members. */
    public DcMotor  frontLeft   = null;
    public DcMotor  frontRight  = null;
    public DcMotor  backLeft   = null;
    public DcMotor  backRight  = null;
    //public DcMotor  arm     = null;
    public DigitalChannel tSensor = null;
    public DcMotor  LED     = null;
    public Servo    claw    = null;
    public boolean LEDBlinking = false;
    public int blinkTime = 100;
    public double startPos;
    public static final double TURN_SPEED          =  0.5 ;
    public static final double FORWARD_SPEED       =  0.5 ;
    final double CLAW_OPEN = 0.0;
    final double CLAW_CLOSED =01.00;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;




    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     ARM_WHEEL_DIAMETER_INCHES   = 50.0 / 25.4 ;// arm diamiter
    static final double     WHEEL_DIAMETER_INCHES   = 100.0 / 25.4 ;     // 100 mm converted to inches For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     COUNTS_PER_INCH_FOR_ARM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (ARM_WHEEL_DIAMETER_INCHES * Math.PI);

    static final double     ARM_COUNTS_PER_MOTOR_REV    = 2786.2 ;    // eg: TETRIX Motor Encoder
    static final double     ARM_DRIVE_GEAR_REDUCTION    = 96.0 / 48.0 ;
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private LinearOpMode op = null;
    public static ElapsedTime runtime  = new ElapsedTime();
    private ElapsedTime ledTimer  = new ElapsedTime();

    /* Constructor */
    public MecanumRobot(){

    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode op) {
      this.op = op;

      // Save reference to Hardware map
      hwMap = ahwMap;

      // Define and Initialize Motors
      frontLeft = hwMap.dcMotor.get("frontLeft");
      backLeft = hwMap.dcMotor.get("backLeft");
      frontRight = hwMap.dcMotor.get("frontRight");
      backRight = hwMap.dcMotor.get("backRight");
     //arm = hwMap.dcMotor.get("arm");
     // tSensor = hwMap.get(DigitalChannel.class,"touch_sensor");
      // LED = hwMap.dcMotor.get("LED");


      ledTimer.reset();

      frontRight.setDirection(DcMotor.Direction.FORWARD);
      backRight.setDirection(DcMotor.Direction.FORWARD);
      frontLeft.setDirection(DcMotor.Direction.REVERSE);
      backLeft.setDirection(DcMotor.Direction.REVERSE);

      // Set all motors to zero power
      frontLeft.setPower(0);
      frontRight.setPower(0);
      backLeft.setPower(0);
      backRight.setPower(0);
    //  arm.setPower(0);

      // Set all motors to run without encoders.
      // May want to use RUN_USING_ENCODERS if encoders are installed.
      // Send telemetry message to signify robot waiting;
      op.telemetry.addData("Status", "Resetting Encoders");    //
      op.telemetry.update();

      frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     // arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

     // tSensor.setMode(DigitalChannel.Mode.INPUT);

      // Motors Positions
      double  backRightPos = backRight.getCurrentPosition();
      double  frontRightPos = backRight.getCurrentPosition();
      double  frontLeftPos = backRight.getCurrentPosition();
      double  backLeftPos = backRight.getCurrentPosition();
      //startPos = arm.getCurrentPosition();
      //double  armPos = arm.getCurrentPosition();




      frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


      //arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


      // Define and initialize ALL installed servos.
      //claw  = hwMap.get(Servo.class, "servo");
      // claw.setPosition(CLAW_OPEN);
    }




    void stop(){
      frontLeft.setPower(0);
      frontRight.setPower(0);
      backLeft.setPower(0);
      backRight.setPower(0);
    }

    // DRIVE METHODS
    void driveBackwards(double inches){
      driveBackwardsWithEncoder(inches);
    }
    void moveArm(double inches){
     // armEncoder(.5,inches, inches/2);
     // arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
//    boolean isPressed(){
//      return tSensor.getState() == false;
//    }

    void driveForwards(double inches){
      driveForwardsWithEncoder(inches);
    }

    //SLIDING METHODS
    void slideRight(double inches){
      // 2 inches = 0.12 sec
      //seconds = seconds * .06;
      slideRightWithEncoders(inches);
      stop();
    }
    void slideLeft(double inches){
      // 2 inches = 0.12 sec
      //seconds = seconds * .06;
      slideLeftWithEncoders(inches);

      stop();
    }
    // TURNING METHODS
    void turnLeft(double degree){
      degree = degree / 3.75; // TURN DEGREE IN TO INCHES
      encoderDrive(TURN_SPEED,-degree, degree, degree/2);
    }
    void turnRight(double degree){
      degree = degree / 3.75; // TURN DEGREE IN TO INCHES
      encoderDrive(TURN_SPEED,degree, -degree, degree/2);
    }
    //Claw Methods
    void openClaw()
    {


      claw.setPosition(0.0);
      op.telemetry.addData("Servo", "0.0");
      op.telemetry.update();
      runtime.reset();
      while (op.opModeIsActive() && (runtime.seconds() < 1.0))
      {
        op.telemetry.addData("Servo", "Open 0.0: %2.5f S Elapsed", runtime.seconds());
        op.telemetry.update();
      }
    }

    void closeClaw() {

      claw.setPosition(0.5);
      op.telemetry.addData("Servo", "1.0");
      op.telemetry.update();
      runtime.reset();
      while (op.opModeIsActive() && (runtime.seconds() < 1.0)) {
        op.telemetry.addData("Servo", "Close.0: %2.5f S Elapsed", runtime.seconds());
        op.telemetry.update();
      }
    }





    //ENCODER METHODS
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
      int frontLeftTarget;
      int frontRightTarget;

      int backLeftTarget;
      int backRightTarget;

      // Ensure that the opmode is still active
      if (op.opModeIsActive()) {

        // Determine new target position, and pass to motor controller
        frontLeftTarget = frontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        frontRightTarget = frontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        backLeftTarget = backLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        backRightTarget = backRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);
        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);


        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // reset the timeout time and start motion.
        runtime.reset();
        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));

        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));


        frontRight.setPower(Math.abs(speed));
        frontLeft.setPower(Math.abs(speed));


        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (op.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (frontLeft.isBusy() && frontRight.isBusy() &&
                        backLeft.isBusy() && backRight.isBusy())) {


          // Display it for the driver.
          op.telemetry.addData("frontLeft:", "Running to %7d :%7d", frontLeft.getCurrentPosition(), frontLeftTarget);
          op.telemetry.addData("frontRight:", "Running to %7d :%7d", frontRight.getCurrentPosition(), frontRightTarget);
          op.telemetry.addData("backLeft:", "Running to %7d :%7d", backRight.getCurrentPosition(), backLeftTarget);
          op.telemetry.addData("backRight:", "Running to %7d :%7d", backLeft.getCurrentPosition(), backRightTarget);


          op.telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


        // Turn off RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
      }
    }

    public void encoderSlide(double speed, double leftInches, double rightInches,
                             double timeoutS) {
      int frontLeftTarget;
      int frontRightTarget;

      int backLeftTarget;
      int backRightTarget;

      // Ensure that the opmode is still active
      if (op.opModeIsActive()) {

        // Determine new target position, and pass to motor controller
        frontLeftTarget = frontLeft.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        frontRightTarget = frontRight.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        backLeftTarget = backLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        backRightTarget = backRight.getCurrentPosition() + (int)(rightInches *  COUNTS_PER_INCH);

        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);
        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);



        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // reset the timeout time and start motion.
        runtime.reset();
        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));

        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));



        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (op.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (frontLeft.isBusy() && frontRight.isBusy() &&
                        backLeft.isBusy() && backRight.isBusy()) ) {


          // Display it for the driver.
          op.telemetry.addData("frontLeft:",  "Running to %7d :%7d", frontLeft.getCurrentPosition(),frontLeftTarget);
          op.telemetry.addData("frontRight:",  "Running to %7d :%7d", frontRight.getCurrentPosition(),frontRightTarget);
          op.telemetry.addData("backLeft:",  "Running to %7d :%7d", backRight.getCurrentPosition(), backLeftTarget);
          op.telemetry.addData("backRight:",  "Running to %7d :%7d", backLeft.getCurrentPosition(), backRightTarget);


          op.telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


        // Turn off RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
      }

    }
//    void armEncoder(double speed, double armInches, double timeoutS ){
//
//      int armTarget;
//      armTarget = arm.getCurrentPosition() + (int)(armInches * COUNTS_PER_INCH_FOR_ARM);
//
//      if (op.opModeIsActive()) {
//
//        // Determine new target position, and pass to motor controller
//        armTarget = arm.getCurrentPosition() + (int)(armInches * COUNTS_PER_INCH);
//
//
//        arm.setTargetPosition(armTarget);
//
//        // Turn On RUN_TO_POSITION
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//        // reset the timeout time and start motion.
//        runtime.reset();
//        arm.setPower(Math.abs(speed));
//
//
//
//        // keep looping while we are still active, and there is time left, and both motors are running.
//        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//        // its target position, the motion will stop.  This is "safer" in the event that the robot will
//        // always end the motion as soon as possible.
//        // However, if you require that BOTH motors have finished their moves before the robot continues
//        // onto the next step, use (isBusy() || isBusy()) in the loop test.
//        while (op.opModeIsActive() && (runtime.seconds() < timeoutS) &&(arm.isBusy() ) ) {
//
//
//          // Display it for the driver.
//          op.telemetry.addData("armMotor:",  "Running to %7d :%7d", arm.getCurrentPosition(),armTarget);
//
//          op.telemetry.update();
//        }
//
//        // Stop all motion;
//
//        arm.setPower(0);
//
//
//        // Turn off RUN_TO_POSITION
//
//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        //  sleep(250);   // optional pause after each move
//      }
//    }
    void slideRightWithEncoders(double inches){
      // Rule: encoderSlide(speed, inches(make negative to slideleft), inches( make negative to slideright), runtime)
      encoderSlide(FORWARD_SPEED, -inches, inches, inches/2);
    }
    void slideLeftWithEncoders(double inches){
      // Rule: encoderSlide(speed, inches(make negative to slideRight), inches( make negative to slideLeft), runtime)
      encoderSlide(FORWARD_SPEED, inches, -inches, inches/2);
    }
    void driveForwardsWithEncoder(double  distance) {

      encoderDrive(FORWARD_SPEED, distance, distance, distance / 2);


      stop();

    }
    void driveBackwardsWithEncoder(double  distance) {

      encoderDrive(FORWARD_SPEED, -distance, -distance, distance / 2);


      stop();

    }
  }
