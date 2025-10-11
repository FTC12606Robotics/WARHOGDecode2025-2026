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
        double joyx, joyy, joyz, gas, brake, baseSpeed, launcherSpeed, hopperSpeed, pistonPos, turnPiston;
        boolean centricityToggle, resetDriveAngle, rightFlick, leftFlick, runPiston;

        Drivetrain.Centricity centricity = Drivetrain.Centricity.FIELD;

        baseSpeed = .4;

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
            turnPiston = -currentGamepad2.right_stick_y;
            runPiston = currentGamepad2.b && !previousGamepad2.b;

            //set up vectors
            joyx = currentGamepad1.left_stick_x;
            joyy = -currentGamepad1.left_stick_y;
            joyz = -currentGamepad1.right_stick_x;
            gas = currentGamepad1.right_trigger*(1-baseSpeed);
            brake = -currentGamepad1.left_trigger*(baseSpeed);

            launcherSpeed = -currentGamepad2.left_stick_y;
            hopperSpeed = currentGamepad2.right_stick_x;

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
            telemetry.addData("launch speed: ", launcherSpeed);
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
            outtake.spinLauncher(launcherSpeed);

            //Spin Hopper TEST
            outtake.spinHopper(Outtake.HOPPERDIRECTION.LEFT, hopperSpeed);

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
            if (turnPiston!=0){
                outtake.runPiston(turnPiston);
            }


            //end step
            telemetry.update();
        }

    }
}
