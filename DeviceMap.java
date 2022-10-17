package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DeviceMap {
    private final DcMotor frontLeft,backLeft,frontRight,backRight,arm;
    private final Servo claw;

    public DeviceMap(HardwareMap map){
        // define & initializing motors & Servos
        frontLeft = map.get(DcMotor.class,"frontLeft");
        backLeft = map.get(DcMotor.class,"backLeft");
        frontRight = map.get(DcMotor.class,"frontRight");
        backRight = map.get(DcMotor.class,"backRight");
        arm = map.get(DcMotor.class,"Arm");

        claw = map.get(Servo.class,"claw");
    }

    // MOTOR GETTERS
    public DcMotor getFrontLeft(){return frontLeft;}
    public DcMotor getBackLeft(){return backLeft;}
    public DcMotor getFrontRight(){return frontRight;}
    public DcMotor getBackRight(){return backRight;}
    public DcMotor getArm(){return arm;}

    // SERVO GETTERS
    public Servo getClaw(){return claw;}


}
