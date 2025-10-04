package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    private final DcMotor launchMotor;
    private final DcMotor launchMotor2;

    private final Telemetry telemetry;

    Outtake(HardwareMap hardwareMap, Telemetry telemetry){
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        //launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Doesn't work on 1620 rpm 5203 motors

        launchMotor2 = hardwareMap.get(DcMotor.class, "launchMotor2");
        launchMotor2.setDirection(DcMotor.Direction.FORWARD);
        //launchMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //launchMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Doesn't work on 1620 rpm 5203 motors

        this.telemetry = telemetry;
    }

    //Spin wheels for launcher
    public void spinLauncher(double power){
        launchMotor.setPower(power);
        launchMotor2.setPower(power);

        //For Debugging
        //telemetry.addData("Launch Motor 1 Power: ", launchMotor.getPower());
        //telemetry.addData("Launch Motor 2 Power: ", launchMotor2.getPower());
    }

}
