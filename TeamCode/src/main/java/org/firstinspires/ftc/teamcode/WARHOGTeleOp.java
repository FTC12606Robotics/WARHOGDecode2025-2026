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

        //=====Set up variables=====
        double joyx, joyy, joyz, gas, brake, baseSpeed, staticLaunchSpeed, launcherSpeed,
                hopperSpeed, hopperStickSpeed, hopperGasSpeed,pistonPos, launchTrigger;
        boolean centricityToggle, resetDriveAngle, rightFlick, leftFlick, runPiston, spinToggle;

        Drivetrain.Centricity centricity = Drivetrain.Centricity.FIELD;

        baseSpeed = .4;
        staticLaunchSpeed = .9;
        hopperSpeed = .3;

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
            resetDriveAngle = currentGamepad1.dpad_up; //use when the robot is facing away from you
            spinToggle = currentGamepad2.a && !previousGamepad2.a;

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

            //Pin Flickers
            leftFlick = currentGamepad2.left_bumper && !previousGamepad2.left_bumper;
            rightFlick = currentGamepad2.right_bumper && !previousGamepad2.right_bumper;
            launchTrigger = currentGamepad2.right_trigger;
            runPiston = currentGamepad2.b && !previousGamepad2.b;

            //set up vectors
            joyx = currentGamepad1.left_stick_x;
            joyy = -currentGamepad1.left_stick_y;
            joyz = -currentGamepad1.right_stick_x;
            gas = currentGamepad1.right_trigger*(1-baseSpeed);
            brake = -currentGamepad1.left_trigger*(baseSpeed);

            launcherSpeed = -currentGamepad2.left_stick_y;
            hopperStickSpeed = currentGamepad2.right_stick_x;
            hopperGasSpeed = currentGamepad2.left_trigger*(1-hopperSpeed);

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
            telemetry.addData("launch speed: ", outtake.getSpinPower(1));
            telemetry.addData("hopper speed: ", hopperSpeed);
            telemetry.addData("piston position: ", pistonPos);

            //set and print motor powers
            double[] motorPowers = drivetrain.driveVectors(centricity, joyx, joyy, joyz, baseSpeed+gas+brake);
            //For drive motor debugging
            /*for (double line:motorPowers){
                telemetry.addLine( Double.toString(line) );
            }*/

            //reset the angle
            if(resetDriveAngle){
                drivetrain.resetAngleData(Drivetrain.AngleType.HEADING);
            }

            //Launcher/Outtake
            if (!spinToggle) {
                outtake.spinLauncher(launcherSpeed);
            }
            else{
                outtake.spinLauncher(staticLaunchSpeed);
            }

            //Spin Hopper TEST
            if (hopperStickSpeed < 0) {
                outtake.spinHopper(Outtake.HOPPERDIRECTION.LEFT, hopperSpeed+hopperGasSpeed);
            }
            else if (hopperStickSpeed > 0){
                outtake.spinHopper(Outtake.HOPPERDIRECTION.RIGHT, hopperSpeed+hopperGasSpeed);
            }

            //Pin Flicker
            if(rightFlick) {
                outtake.flickPin(Outtake.PINS.RIGHT);
            }
            if(leftFlick){
                outtake.flickPin(Outtake.PINS.LEFT);
            }

            //Piston extension
            if (runPiston){
                if (pistonPos >= .1){
                    outtake.retractPiston();
                }
                else{
                    outtake.extendPiston();
                }
            }

            //Single button launch sequence.
            if (launchTrigger >= .1 && launchTrigger < .2){
                outtake.retractPiston();
                telemetry.addLine("Auto Launcher Status: RETRACTED");
            }
            else if (launchTrigger >= .2 && launchTrigger < .85){
                outtake.spinLauncher(staticLaunchSpeed);
                telemetry.addLine("Auto Launcher Status: ARMING");
            }
            else if (launchTrigger >= .85){
                //If launcher is sufficiently spinning
                if (outtake.getSpinPower(1) >= .95*staticLaunchSpeed){ //TODO come up with a better threshold number
                    outtake.extendPiston();
                    telemetry.addLine("Auto Launcher Status: FIRING");
                }
                else{
                    telemetry.addLine("Auto Launcher Status: WARNING: FIRING WHEN READY");
                }
            }


            //end step
            telemetry.update();
        }

    }
}
