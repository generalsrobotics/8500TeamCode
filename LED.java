package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LED {
    public static DcMotor colorLed,powerLed;
     LED led;
     LinearOpMode op;
    public static ElapsedTime runtime = new ElapsedTime();

    public LED(HardwareMap hwmap,LinearOpMode op){
        DeviceMap map = new DeviceMap(hwmap);
        colorLed = map.getColorLed();
        powerLed = map.getPowerLed();
        this.op = op;
    }

    void ready(){
        powerGreenOn();
    }
    void powerGreenOn(){
        powerOn();
        colorLed.setPower(-1);
    }
    void powerBlueOn(){
        powerOn();
        colorLed.setPower(1);
    }
    void powerOn(){
        powerLed.setPower(1);
    }
    void powerOff() {
        powerLed.setPower(0);
    }
}

