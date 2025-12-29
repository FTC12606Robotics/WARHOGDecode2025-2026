package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="WARHOGTeleOpPushbot", group="")
public class WARHOGTeleOpPushbot extends LinearOpMode {
    public WARHOGTeleOpPushbot() throws InterruptedException {}

    @Override
    public void runOpMode() throws InterruptedException {

        //=====Set up classes=====
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry, "BHI260IMU");
        AprilTagVision aprilTagVision = new AprilTagVision(hardwareMap);

        //=====Set up variables=====
        double joyx, joyy, joyz, gas, brake, baseSpeed;
        boolean centricityToggle, resetDriveAngle;

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
            telemetry.addLine("Init complete");
            telemetry.update();
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


            //set up vectors
            joyx = currentGamepad1.left_stick_x;
            joyy = -currentGamepad1.left_stick_y;
            joyz = -currentGamepad1.right_stick_x;
            gas = currentGamepad1.right_trigger*(1-baseSpeed);
            brake = -currentGamepad1.left_trigger*(baseSpeed);

            //print vectors
            telemetry.addData("y: ", joyy);
            telemetry.addData("x: ", joyx);
            telemetry.addData("z: ", joyz);
            telemetry.addData("gas: ", gas);
            telemetry.addData("brake: ", brake);

            //set and print motor powers
            double[] motorPowers = drivetrain.driveVectors(centricity, joyx, joyy, joyz, baseSpeed+gas+brake);
            //For drive motor debugging
            /*for (double line:motorPowers){
                telemetry.addLine( Double.toString(line) );
            }*/

            //reset the drive angle for field centricity
            if(resetDriveAngle){
                drivetrain.resetHeading();
            }


            //==========April Tag Vision==========
            AprilTagDetection detection = aprilTagVision.getBestTag();

            if (detection != null) {
                telemetry.addData("Tag ID", detection.id);
                /*if (det.metadata != null) {
                    telemetry.addData("Range (m)", "%.2f", det.ftcPose.range);
                    telemetry.addData("Bearing (deg)", "%.1f", det.ftcPose.bearing);
                }*/
            } else {
                telemetry.addLine("No tag detected");
            }
            //==========April Tag Vision==========


            //end step
            telemetry.update();
        }

        //Stop vision at the end
        aprilTagVision.stop();
    }
}