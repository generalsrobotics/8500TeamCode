package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Disabled

@Autonomous(name="AutonomousBluetrac")
public class AutonomousBlue extends LinearOpMode {


    @Override
    public void runOpMode() {
        MecanumRobot robot = new MecanumRobot();



        waitForStart();
//        Trajectory slideRight = drive.trajectoryBuilder(startPose)
//                .strafeLeft(30)
//                .build();
//        drive.followTrajectory(slideRight);
//        Trajectory spline  = drive.trajectoryBuilder(slideRight.end())
//                .lineTo(new Vector2d(-12.5,50))
//                .splineTo(new Vector2d(-12.5,25),-.6)
//                .build();
//        drive.followTrajectory(spline);
        robot.armUp(39);
    }
}
