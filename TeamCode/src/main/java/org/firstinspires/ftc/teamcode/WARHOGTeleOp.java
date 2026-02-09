package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="WARHOGTeleOp", group="")
public class WARHOGTeleOp extends LinearOpMode {
    public WARHOGTeleOp() throws InterruptedException {}

    @Override
    public void runOpMode() throws InterruptedException {

        //=====Set up classes=====
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry, "BNO055IMU");
        Outtake outtake = new Outtake(hardwareMap, telemetry);
        //AprilTagVision vision = new AprilTagVision(hardwareMap);

        //=====Set up variables=====
        double joyx, joyy, joyz, gas, brake, baseSpeed, launcherSpeed,
                hopperSpeed, hopperStickSpeed, hopperGasSpeed, pistonPos;
        boolean centricityToggle, resetDriveAngle, runPiston,
                spinFastToggle = false, spinMediumToggle = false, spinSlowToggle = false, turnHopperMagRight = false,
                turnHopperMagLeft = false, turning = false;

        Drivetrain.Centricity centricity = Drivetrain.Centricity.FIELD;

        baseSpeed = .4;
        hopperSpeed = .4;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        while (!isStarted() && !isStopRequested()) {

            try {
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);

                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
            }
            catch (Exception e) {
                // Swallow the possible exception, it should not happen as
                // currentGamepad1/2 are being copied from valid Gamepads.
            }
        }

        while(opModeIsActive()){
            //set up inputs
            try {
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);

                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
            }
            catch (Exception e) {
                // Swallow the possible exception, it should not happen as
                // currentGamepad1/2 are being copied from valid Gamepads.
            }
            telemetry.addData("angle", drivetrain.getIMUAngleData(Drivetrain.AngleType.HEADING)/PI*180);


            //=====Set up inputs=====

            //inputs that toggle the modes
            centricityToggle = currentGamepad1.dpad_down && !previousGamepad1.dpad_down; //change whether the drive is bot or field centric
            resetDriveAngle = currentGamepad1.dpad_up && !previousGamepad1.dpad_up; //use when the robot is facing away from you

            //toggle on and off the launch motors
            if (currentGamepad2.a && !previousGamepad2.a){
                spinFastToggle = false;
                spinMediumToggle = false;
                spinSlowToggle = !spinSlowToggle;
            }
            if (currentGamepad2.x && !previousGamepad2.x){
                spinSlowToggle = false;
                spinMediumToggle = false;
                spinFastToggle = !spinFastToggle;
            }
            if (currentGamepad2.y && !previousGamepad2.y){
                spinSlowToggle = false;
                spinFastToggle = false;
                spinMediumToggle = !spinMediumToggle;
            }

            //code to switch between field centric and bot centric drive
            if(centricityToggle){
                if(centricity==Drivetrain.Centricity.BOT){
                    centricity = Drivetrain.Centricity.FIELD;
                }
                else{
                    centricity = Drivetrain.Centricity.BOT;
                }
            }
            telemetry.addData("Centricity: ", centricity);

            runPiston = currentGamepad2.b && !previousGamepad2.b;

            //set up vectors
            joyx = currentGamepad1.left_stick_x;
            joyy = -currentGamepad1.left_stick_y;
            joyz = currentGamepad1.right_stick_x;
            gas = currentGamepad1.right_trigger*(1-baseSpeed);
            brake = -currentGamepad1.left_trigger*(baseSpeed);

            launcherSpeed = -currentGamepad2.left_stick_y;
            hopperStickSpeed = currentGamepad2.right_stick_x;
            hopperGasSpeed = currentGamepad2.left_trigger*(1-hopperSpeed);

            turnHopperMagLeft = currentGamepad2.dpad_left && !previousGamepad2.dpad_left;
            turnHopperMagRight = currentGamepad2.dpad_right && !previousGamepad2.dpad_right;

            pistonPos = outtake.pistonPosition();

            //limit launcher speed
            if (abs(launcherSpeed) > .9){
                launcherSpeed = .9;
            }

            //print vectors
            telemetry.addData("y: ", joyy);
            telemetry.addData("x: ", joyx);
            telemetry.addData("z: ", joyz);
            telemetry.addData("gas: ", gas);
            telemetry.addData("brake: ", brake);
            telemetry.addData("launch speed: ", outtake.getSpinPower(2));
            telemetry.addData("hopper speed: ", hopperSpeed);
            telemetry.addData("hopper gas: ", hopperGasSpeed);
            telemetry.addData("piston position: ", pistonPos);

            //set and print motor powers
            double[] motorPowers = drivetrain.driveVectors(centricity, joyx, joyy, joyz, baseSpeed+gas+brake);
            //For drive motor debugging
            //for (double line:motorPowers){telemetry.addLine( Double.toString(line) );}

            //reset the drive angle for field centricity
            if(resetDriveAngle){
                drivetrain.resetHeading();
            }

            //Launcher/Outtake
            if (spinFastToggle) {
                //outtake.spinLauncher(.8);
                outtake.spinLauncherVelocity(Outtake.TICKSPEED.FAST);
            }
            else if (spinMediumToggle) {
                //outtake.spinLauncher(.6);
                outtake.spinLauncherVelocity(Outtake.TICKSPEED.MEDIUM);
            }
            else if (spinSlowToggle) {
                //outtake.spinLauncher(.6);
                outtake.spinLauncherVelocity(Outtake.TICKSPEED.SLOW);
            }
            else{
                outtake.spinLauncherVelocity(launcherSpeed*2600); //Converted to ticks
            }

            //Spin Hopper
            //'turning' is the override TODO
            if (turning && hopperStickSpeed != 0) {
                outtake.turningOverride();
                turning = false;
            }

            if (hopperStickSpeed < 0) { //Using hopper stick speed really just to get direction
                outtake.spinHopper(Outtake.HOPPERDIRECTION.LEFT, hopperSpeed);
            }
            else if (hopperStickSpeed > 0){
                outtake.spinHopper(Outtake.HOPPERDIRECTION.RIGHT, hopperSpeed);
            }
            else if (turnHopperMagLeft) {
                outtake.turnHopperMag(Outtake.HOPPERDIRECTION.LEFT, .5);
                turning = true;
            }
            else if (turnHopperMagRight) {
                outtake.turnHopperMag(Outtake.HOPPERDIRECTION.RIGHT, .5);
                turning = true;
            }
            else if (!turning) { //Stop the hopper when stick is 0
                outtake.stopHopper();
            }

            //Piston extension
            if (runPiston){
                if (pistonPos >= .1){
                    outtake.retractPiston();
                }
                else{
                    outtake.extendPiston();
                }
                //outtake.runPistonTeleopAuto();
            }


            //end step
            telemetry.update();
            outtake.update();
        }

    }
}
