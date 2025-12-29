package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    final CRServo intakeSweep;
    final double sweepPower = 1;
    final Servo intake;
    final double intakeDown = 0;
    final double intakeUp = .5;

    private final Telemetry telemetry;

    Intake(HardwareMap hardwareMap, Telemetry telemetry){
        intakeSweep = hardwareMap.get(CRServo.class, "hopper");
        intake = hardwareMap.get(Servo.class, "piston");
        lower();

        this.telemetry = telemetry;
    }

    void lower(){
        intake.setPosition(intakeDown);
    }

    void lift(){
        intake.setPosition(intakeUp);
    }

    void spinIntake(){
        intakeSweep.setPower(sweepPower);
    }

    void stopIntake(){
        intakeSweep.setPower(0);
    }
}
