package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    private final DcMotorEx launchMotor;
    private final DcMotorEx launchMotor2;

    public enum TICKSPEED {OFF(0), SLOW(2000), MEDIUM(2200), FAST(3000);
        private int value;

        private TICKSPEED(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    private final CRServo hopper;
    public enum HOPPERDIRECTION {RIGHT, LEFT}
    private enum HopperState {IDLE, TURNING}
    private HopperState hopperState = HopperState.IDLE;
    public DigitalChannel magSwitch;

    private final Servo piston;
    private final double pistonIn = 0;
    private final double pistonOut = .35;

    private final Servo rightPin;
    private final Servo leftPin;
    public enum PINS {RIGHT, LEFT, BOTH}
    private final double inPos = 10;
    private final double outPos = 40;

    private final Telemetry telemetry;

    Outtake(HardwareMap hardwareMap, Telemetry telemetry){
        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        launchMotor.setDirection(DcMotorEx.Direction.REVERSE);
        launchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launchMotor.setVelocityPIDFCoefficients(10.0, 3.0, 0.0, 12.0);
        //launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Doesn't work on 1620 rpm 5203 motors

        launchMotor2 = hardwareMap.get(DcMotorEx.class, "launchMotor2");
        launchMotor2.setDirection(DcMotorEx.Direction.FORWARD);
        launchMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launchMotor2.setVelocityPIDFCoefficients(10.0, 3.0, 0.0, 12.0);
        //launchMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //launchMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Doesn't work on 1620 rpm 5203 motors

        rightPin = hardwareMap.get(Servo.class, "rightPin");
        leftPin = hardwareMap.get(Servo.class, "leftPin");
        hopper = hardwareMap.get(CRServo.class, "hopper");
        piston = hardwareMap.get(Servo.class, "piston");
        retractPiston();

        magSwitch = hardwareMap.get(DigitalChannel.class, "magSwitch");
        magSwitch.setMode(DigitalChannel.Mode.INPUT);

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

    //Spin wheels for launcher
    public void spinLauncherVelocity(TICKSPEED power){
        launchMotor.setVelocity(power.getValue());
        launchMotor2.setVelocity(power.getValue());

        //For Debugging
        //telemetry.addData("Launch Motor 1 Power: ", launchMotor.getPower());
        //telemetry.addData("Launch Motor 2 Power: ", launchMotor2.getPower());
    }
    public void spinLauncherVelocity(double power){
        launchMotor.setVelocity(power);
        launchMotor2.setVelocity(power);

        //For Debugging
        //telemetry.addData("Launch Motor 1 Power: ", launchMotor.getPower());
        //telemetry.addData("Launch Motor 2 Power: ", launchMotor2.getPower());
    }

    // Return launch motor powers as doubles
    public double getSpinPower (int motor){
        if (motor == 1){
            //return launchMotor.getPower();
            return launchMotor.getVelocity();
        }
        else if (motor == 2){
            //return launchMotor2.getPower();
            return launchMotor2.getVelocity();
        }
        else {
            //return launchMotor.getPower();
            return launchMotor.getVelocity();
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

    //Auto: Turn the hopper based on magnet trigger
    public void turnHopperMagAuto(HOPPERDIRECTION direction, double speed) throws InterruptedException {
        boolean isTriggered = false;
        spinHopper(direction, speed);
        sleep(100);
        while (!isTriggered){
            isTriggered = !magSwitch.getState();
        }
        stopHopper();
    }

    //TeleOp: Turn the hopper based on magnet trigger
    public void turnHopperMag(HOPPERDIRECTION direction, double speed){
        spinHopper(direction, speed);
        hopperState = HopperState.TURNING;
    }

    // Call this every loop in TeleOp, to make it asynchronous!
    public void update() {
        switch (hopperState) {
            case TURNING:
                boolean isTriggered = !magSwitch.getState();
                if (isTriggered) {
                    stopHopper();
                    hopperState = HopperState.IDLE;
                }
                break;

            case IDLE:
            default:
                // nothing to do
                break;
        }
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
