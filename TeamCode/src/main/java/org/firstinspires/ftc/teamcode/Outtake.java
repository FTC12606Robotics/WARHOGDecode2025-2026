package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    private final DcMotorEx launchMotor;
    private final DcMotorEx launchMotor2;
    public enum TICKSPEED {OFF(0), SLOW(1800), MEDIUM(2000), FAST(3000);
        private final int value;
        TICKSPEED(int value) {
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
    private boolean hasLeftMagnet;
    public DigitalChannel magSwitch;

    private final Servo piston;
    private enum PistonState {IDLE, EXTENDING, RETRACTING}
    private PistonState pistonState = PistonState.IDLE;
    private final ElapsedTime pistonTimer = new ElapsedTime();
    private final double pistonIn = 0;
    private final double pistonOut = .35;

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

        hopper = hardwareMap.get(CRServo.class, "hopper");
        piston = hardwareMap.get(Servo.class, "piston");
        retractPiston();

        magSwitch = hardwareMap.get(DigitalChannel.class, "magSwitch");
        magSwitch.setMode(DigitalChannel.Mode.INPUT);

        this.telemetry = telemetry;
    }

    // Call this every loop in TeleOp, to make it asynchronous!
    public void update(){
        updateHopper();
        updatePiston();
    }

    //==========Launcher Logic==========

    //Spin wheels for launcher
    public void spinLauncher(double power){
        launchMotor.setPower(power);
        launchMotor2.setPower(power);

        //For Debugging
        //telemetry.addData("Launch Motor 1 Power: ", launchMotor.getPower());
        //telemetry.addData("Launch Motor 2 Power: ", launchMotor2.getPower());
    }

    //Spin wheels for launcher
    public void spinLauncherVelocity(double power){
        launchMotor.setVelocity(power);
        launchMotor2.setVelocity(power);

        //For Debugging
        //telemetry.addData("Launch Motor 1 Power: ", launchMotor.getPower());
        //telemetry.addData("Launch Motor 2 Power: ", launchMotor2.getPower());
    }
    public void spinLauncherVelocity(TICKSPEED power){      //To accept TICKSPEED as well
        spinLauncherVelocity(power.getValue());
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


    //==========HOPPER LOGIC==========

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
        hasLeftMagnet = false;
    }

    private void updateHopper() {
        switch (hopperState) {
            case TURNING:
                boolean isTriggered = !magSwitch.getState();

                /*if (isTriggered) {
                    stopHopper();
                    hopperState = HopperState.IDLE;
                }*/
                if (!hasLeftMagnet) {
                    if (!isTriggered) {
                        hasLeftMagnet = true;
                    }
                } else {
                    if (isTriggered) {
                        stopHopper();
                        hopperState = HopperState.IDLE;
                        hasLeftMagnet = false;
                    }
                }
                break;

            case IDLE:
            default:
                // nothing to do
                break;
        }
    }


    //============PISTON LOGIC============

    //Extend Piston
    public void extendPiston(){
        piston.setPosition(pistonOut);
    }

    //Retract Piston
    public void retractPiston(){
        piston.setPosition(pistonIn);
    }

    //Auto: Cycle the piston, for auto don't want to stop teleop
    public void runPiston() throws InterruptedException {
        extendPiston();
        sleep(1000);
        retractPiston();
    }

    public void runPistonTeleopAuto(){
        if (pistonPosition() >= .1){
            retractPiston();
            pistonState = PistonState.RETRACTING;
        }
        else{
            extendPiston();
            pistonState = PistonState.EXTENDING;
        }

        pistonTimer.reset();
    }

    //Give piston position
    public double pistonPosition(){
        return piston.getPosition();
    }

    private void updatePiston(){
        switch (pistonState) {
            case EXTENDING:
                if (pistonTimer.milliseconds() >= 250) {
                    retractPiston();
                    pistonState = PistonState.RETRACTING;
                    pistonTimer.reset();
                }

            case RETRACTING:
                if (pistonTimer.milliseconds() >= 250) {
                    pistonState = PistonState.IDLE;
                }
                break;

            case IDLE:
            default:
                // nothing to do
                break;
        }
    }
}
