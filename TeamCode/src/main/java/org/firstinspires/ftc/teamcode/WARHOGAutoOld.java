package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Disabled
@Autonomous(name="WARHOGAutoOld", group="")
public class WARHOGAutoOld extends LinearOpMode {
    public WARHOGAutoOld() throws InterruptedException {}

    private enum MOSAIC {PPG, PGP, GPP, NONE} //PPG=23, PGP=22, GPP=21
    private enum STARTPOS {FAR, GOAL} //Launch zones
    private enum COLOR {RED, BLUE} //Start Color

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    boolean useCamera = true; //To decide to use the camera during a run or not

    double speed = .50;
    double startSleep = 0; //How many seconds to wait before starting the autonomous routine

    private MOSAIC mosaic = MOSAIC.NONE; //Set default
    private STARTPOS startPos = STARTPOS.GOAL; //Set default
    private COLOR color = COLOR.RED;


    @Override
    public void runOpMode() throws InterruptedException {

        //"BNO055IMU" for Main, "BHI260IMU" for Pushbot
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry, "BNO055IMU");
        AprilTagVision aprilTagVision = new AprilTagVision(hardwareMap);

        telemetry.setMsTransmissionInterval(50);

        //init loop
        while (!isStarted() && !isStopRequested()) {

            //set up inputs - have previous so that you can check rising edge
            try {
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);

                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
            } catch (Exception e) {
                // Swallow the possible exception, it should not happen as
                // currentGamepad1/2 are being copied from valid Gamepads.
            }

            //Override speed with driver hub
            if(currentGamepad1.y && !previousGamepad1.y){
                speed+=.05;
            }
            if(currentGamepad1.a && !previousGamepad1.a){
                speed-=.05;
            }
            if(speed>1){
                speed=1;
            }
            if(speed<.20){
                speed=.20;
            }

            //Override startSleep with driver hub
            if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                startSleep+=.5;
            }
            if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                startSleep-=.5;
            }
            if(startSleep>20){
                startSleep=20;
            }
            if(startSleep<0){
                startSleep=0;
            }

            //Set starting config
            if (currentGamepad1.b) {
                color = COLOR.RED;
            }
            if (currentGamepad1.x) {
                color = COLOR.BLUE;
            }
            if (currentGamepad1.left_bumper) {
                if (startPos == STARTPOS.GOAL) {
                    startPos = STARTPOS.FAR;
                } else if (startPos == STARTPOS.FAR) {
                    startPos = STARTPOS.GOAL;
                }
            }

            //For camera usage in decision making
            if (currentGamepad1.left_stick_button && !previousGamepad1.left_stick_button) {
                useCamera = !useCamera;
            }

            telemetry.addData("color (x/b)", color);
            telemetry.addData("launchPos (lbump)", startPos);
            telemetry.addData("Speed (a/y)", speed);
            telemetry.addData("startSleep (up/down)", startSleep);
            telemetry.addLine();
            telemetry.addData("Use Camera? (lsbtn)", useCamera);

            //===============April Tag Vision===============
            if (useCamera) {
                AprilTagDetection detection = aprilTagVision.getBestTag();

                if (detection != null) {
                    int id = detection.id;
                    telemetry.addData("Tag ID", detection.id);
                    /*if (det.metadata != null) {
                        telemetry.addData("Range (m)", "%.2f", det.ftcPose.range);
                        telemetry.addData("Bearing (deg)", "%.1f", det.ftcPose.bearing);
                    }*/

                    //Set Mosaic Arrangement
                    if (id == 21) {
                        mosaic = MOSAIC.GPP;
                    } else if (id == 22) {
                        mosaic = MOSAIC.PGP;
                    } else if (id == 23) {
                        mosaic = MOSAIC.PPG;
                    }
                } else {
                    telemetry.addLine("No tag detected");
                }
            } else {
                mosaic = MOSAIC.NONE;
            }

            telemetry.addData("Detected Mosaic pattern: ", mosaic);
            //===============April Tag Vision===============

            telemetry.update();
        }

        //Stop vision to save resources
        aprilTagVision.stop(); //TODO might want to take this out to be able to figure out mosiac after moving


        //=============Start command just came in================

        //Switchers for anything might go here


        //Start Wait
        sleep((long)((startSleep)*1000));

        //RED, BLUE, FAR, CLOSE

        telemetry.addLine("Auto Complete");
        telemetry.update();

    }
}