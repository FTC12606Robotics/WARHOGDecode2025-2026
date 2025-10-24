package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    private final DcMotor launchMotor;
    private final DcMotor launchMotor2;

    private final CRServo hopper;
    public enum HOPPERDIRECTION {RIGHT, LEFT}
    private final Servo piston;
    private final double pistonIn = 0;
    private final double pistonOut = .3;
    private final Servo rightPin;
    private final Servo leftPin;
    public enum PINS {RIGHT, LEFT, BOTH}
    private final double inPos = 10;
    private final double outPos = 40;

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

        rightPin = hardwareMap.get(Servo.class, "rightPin");
        leftPin = hardwareMap.get(Servo.class, "leftPin");
        hopper = hardwareMap.get(CRServo.class, "hopper");
        piston = hardwareMap.get(Servo.class, "piston");
        retractPiston();


        this.telemetry = telemetry;
    }

    //Spin wheels for launcher
    public void spinLauncher(double power){
        launchMotor.setPower(-power); //neg to spin opposite directions
        launchMotor2.setPower(power);

        //For Debugging
        //telemetry.addData("Launch Motor 1 Power: ", launchMotor.getPower());
        //telemetry.addData("Launch Motor 2 Power: ", launchMotor2.getPower());
    }

    // Return launch motor powers as doubles
    public double getSpinPower (int motor){
        if (motor == 1){
            return launchMotor.getPower();
        }
        else if (motor == 2){
            return launchMotor2.getPower();
        }
        else {
            return launchMotor.getPower();
        }
    }

    //For flicking the front pins
    public void flickPin(PINS pin) throws InterruptedException {

        switch (pin){
            case LEFT:
                leftPin.setPosition(outPos);
                sleep(10);
                leftPin.setPosition(inPos);
                break;
            case RIGHT:
                rightPin.setPosition(outPos);
                sleep(10);
                rightPin.setPosition(inPos);
                break;
            case BOTH:
                leftPin.setPosition(outPos);
                rightPin.setPosition(outPos);
                sleep(10);
                leftPin.setPosition(inPos);
                rightPin.setPosition(inPos);
        }
    }

    //Spin the hopper
    public void spinHopper(HOPPERDIRECTION direction, double speed){
        if (direction == HOPPERDIRECTION.LEFT){
            hopper.setPower(-speed);
        }
        else if (direction == HOPPERDIRECTION.RIGHT){
            hopper.setPower(speed);
        }
    }
    public void stopHopper(){
        hopper.setPower(0);
    }

    //Auto: Turn the hopper based on time and speed
    public void turnHopperTime(HOPPERDIRECTION direction, double speed, double mSeconds) throws InterruptedException {
        spinHopper(direction, speed);
        sleep((long)mSeconds);
        stopHopper();
    }

    public void turnHopperMag(HOPPERDIRECTION direction, double speed){
        //Turn the hopper based on magnet positions for meet 2
    }

    //Extend Piston
    public void extendPiston(){
        piston.setPosition(pistonOut);
    }

    //Retract Piston
    public void retractPiston(){
        piston.setPosition(pistonIn);
    }

    //Move Piston to any pos.
    public void runPiston(double pos){
        piston.setPosition(pos);
    }

    //Auto: Cycle the piston, for auto don't want to stop teleop
    public void runPiston() throws InterruptedException {
        extendPiston();
        sleep(1000);
        retractPiston();
    }

    //Give piston position
    public double pistonPosition(){
        return piston.getPosition();
    }
}
