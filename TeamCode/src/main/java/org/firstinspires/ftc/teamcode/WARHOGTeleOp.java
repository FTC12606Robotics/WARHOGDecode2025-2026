package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="WARHOGTeleOp", group="")
public class WARHOGTeleOp extends LinearOpMode {
    public WARHOGTeleOp() throws InterruptedException {}

    @Override
    public void runOpMode() throws InterruptedException {

        //=====Set up classes=====
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);

        //=====Set up variables=====
        double joyx, joyy, joyz, gas, baseSpeed, offset, modAngle;
        boolean centricityToggle, resetDriveAngle;

        offset = 0;
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

            if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
                offset-=90;
            }
            if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
                offset+=90;
            }
            if (offset==360){offset=0;}
            if (offset==-90){offset=270;}

            telemetry.addData("Angle Offset", offset);
            telemetry.update();
        }

        //drivetrain.setAngleOffset(offset); //We'll see if this works

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


            //set up vectors
            joyx = currentGamepad1.left_stick_x;
            joyy = -currentGamepad1.left_stick_y;
            joyz = -currentGamepad1.right_stick_x;
            gas = currentGamepad1.right_trigger*(1-baseSpeed);

            //print vectors
            telemetry.addData("y: ", joyy);
            telemetry.addData("x: ", joyx);
            telemetry.addData("z: ", joyz);
            telemetry.addData("gas: ", gas);

            //set and print motor powers
            double[] motorPowers = drivetrain.driveVectors(centricity, joyx, joyy, joyz, baseSpeed+gas);
            for (double line:motorPowers){
                telemetry.addLine( Double.toString(line) );
            }

            //reset the angle
            if(resetDriveAngle){
                drivetrain.resetAngleData(Drivetrain.AngleType.HEADING);
            }

            modAngle = (drivetrain.getIMUAngleData(Drivetrain.AngleType.HEADING)/PI*180)%360;    //********Reposition or take out these 2 lines if not needed, figure out what nod angle is for*********
            telemetry.addData("mod angle", modAngle);




            //end step
            telemetry.update();
        }

    }
}
