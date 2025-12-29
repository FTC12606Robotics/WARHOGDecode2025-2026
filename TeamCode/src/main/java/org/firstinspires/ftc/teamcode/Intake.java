package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    final CRServo intakeSweep;
    final double sweepPower = 1;

    final Servo intake;
    private enum IntakeState {IDLE, EXTENDING, RETRACTING}
    private IntakeState intakeState = IntakeState.IDLE;
    private final ElapsedTime intakeTimer = new ElapsedTime();
    final double intakeDown = 0;
    final double intakeUp = .5;

    private final Telemetry telemetry;

    Intake(HardwareMap hardwareMap, Telemetry telemetry){
        intakeSweep = hardwareMap.get(CRServo.class, "hopper");
        intake = hardwareMap.get(Servo.class, "piston");
        lower();

        this.telemetry = telemetry;
    }

    //Call this every loop in TeleOp, to make it asynchronous!
    public void update(){
        updateIntake();
    }

    public void lower(){intake.setPosition(intakeDown);}

    public void lift(){intake.setPosition(intakeUp);}

    public double intakePosition(){
        return intake.getPosition();
    }

    public void runIntake(){
        if (intakePosition() >= .1){
            lower();
            intakeState = IntakeState.RETRACTING;
        }
        else{
            lift();
            intakeState = IntakeState.EXTENDING;
        }

        intakeTimer.reset();
    }

    private void updateIntake(){
        switch (intakeState) {
            case EXTENDING:
                if (intakeTimer.milliseconds() >= 300) {
                    lower();
                    intakeState = IntakeState.RETRACTING;
                    intakeTimer.reset();
                }

            case RETRACTING:
                if (intakeTimer.milliseconds() >= 300) {
                    intakeState = IntakeState.IDLE;
                }
                break;

            case IDLE:
            default:
                // nothing to do
                break;
        }
    }

    public void spinIntake(){intakeSweep.setPower(sweepPower);}

    public void stopIntake(){intakeSweep.setPower(0);}

}
