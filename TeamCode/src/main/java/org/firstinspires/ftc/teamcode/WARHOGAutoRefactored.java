package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="WARHOGAuto", group="")
public class WARHOGAutoRefactored extends OpMode {
    public WARHOGAutoRefactored() throws InterruptedException {}

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

    //==========FOR PEDROPATHING==========
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPoseCloseRed = new Pose(28.5, 128, Math.toRadians(180)); // first start Pose of our robot, close to goal
    private final Pose scorePoseCloseRed = new Pose(60, 85, Math.toRadians(135)); // first Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose endPoseCloseRed = new Pose(49, 135, Math.toRadians(0)); // First ending position outside of zone for points
    private final Pose startPoseFarRed = new Pose(37, 121, Math.toRadians(0)); // Second start position of our robot, far from goal
    private final Pose scorePoseFarRed = new Pose(43, 130, Math.toRadians(0)); // Second Scoring Pose of our robot.
    private final Pose endPoseFarRed = new Pose(49, 135, Math.toRadians(0)); // Second ending position outside of zone for points

    private final Pose startPoseCloseBlue = new Pose(28.5, 128, Math.toRadians(180)); // first start Pose of our robot, close to goal
    private final Pose scorePoseCloseBlue = new Pose(60, 85, Math.toRadians(135)); // first Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose endPoseCloseBlue = new Pose(49, 135, Math.toRadians(0)); // First ending position outside of zone for points
    private final Pose startPoseFarBlue = new Pose(37, 121, Math.toRadians(0)); // Second start position of our robot, far from goal
    private final Pose scorePoseFarBlue = new Pose(43, 130, Math.toRadians(0)); // Second Scoring Pose of our robot.
    private final Pose endPoseFarBlue = new Pose(49, 135, Math.toRadians(0)); // Second ending position outside of zone for points
    private Path scorePreloadCloseRed, scorePreloadFarRed, scorePreloadCloseBlue, scorePreloadFarBlue;
    private PathChain endCloseRed, endFarRed, endCloseBlue, endFarBlue;
    //==========FOR PEDROPATHING==========


    public void buildPaths() {
        //This is our scorePreloads path. We are using a BezierLine, which is a straight line.
        scorePreloadCloseRed = new Path(new BezierLine(startPoseCloseRed, scorePoseCloseRed));
        scorePreloadCloseRed.setLinearHeadingInterpolation(startPoseCloseRed.getHeading(), scorePoseCloseRed.getHeading());

        scorePreloadFarRed = new Path(new BezierLine(startPoseFarRed, scorePoseFarRed));
        scorePreloadFarRed.setLinearHeadingInterpolation(startPoseFarRed.getHeading(), scorePoseFarRed.getHeading());

        scorePreloadCloseBlue = new Path(new BezierLine(startPoseCloseBlue, scorePoseCloseBlue));
        scorePreloadCloseBlue.setLinearHeadingInterpolation(startPoseCloseBlue.getHeading(), scorePoseCloseBlue.getHeading());

        scorePreloadFarBlue = new Path(new BezierLine(startPoseFarBlue, scorePoseFarBlue));
        scorePreloadFarBlue.setLinearHeadingInterpolation(startPoseFarBlue.getHeading(), scorePoseFarBlue.getHeading());
        //Here is an example for Constant Interpolation
        //scorePreload.setConstantInterpolation(startPose.getHeading());

        //This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line.
        endCloseRed = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseCloseRed, endPoseCloseRed))
                .setLinearHeadingInterpolation(scorePoseCloseRed.getHeading(), endPoseCloseRed.getHeading())
                .build();
        //This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line.
        endFarRed = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseFarRed, endPoseFarRed))
                .setLinearHeadingInterpolation(scorePoseFarRed.getHeading(), endPoseFarRed.getHeading())
                .build();
        // This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line.
        endCloseBlue = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseCloseBlue, endPoseCloseBlue))
                .setLinearHeadingInterpolation(scorePoseCloseBlue.getHeading(), endPoseCloseBlue.getHeading())
                .build();
        // This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line.
        endFarBlue = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseFarBlue, endPoseFarBlue))
                .setLinearHeadingInterpolation(scorePoseFarBlue.getHeading(), endPoseFarBlue.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {

        //"BNO055IMU" for Main, "BHI260IMU" for Pushbot
        try {
            Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry, "BNO055IMU"); // TODO probably need a way to use these outside of init function
        } catch (InterruptedException e) {
            telemetry.addLine("Drivetrain failed to initialize");
            throw new RuntimeException(e);
        }
        AprilTagVision aprilTagVision = new AprilTagVision(hardwareMap);

        //Initialize for PedroPathing
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        //buildPaths();
        follower.setStartingPose(startPose1); //Based on the default start position set earlier, have to put it here after init of follower

        telemetry.setMsTransmissionInterval(50);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {

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
        if (currentGamepad1.y && !previousGamepad1.y) {
            speed += .05;
        }
        if (currentGamepad1.a && !previousGamepad1.a) {
            speed -= .05;
        }
        if (speed > 1) {
            speed = 1;
        }
        if (speed < .20) {
            speed = .20;
        }

        //Override startSleep with driver hub
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            startSleep += .5;
        }
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            startSleep -= .5;
        }
        if (startSleep > 20) {
            startSleep = 20;
        }
        if (startSleep < 0) {
            startSleep = 0;
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
                follower.setStartingPose(startPose2);
            } else if (startPos == STARTPOS.FAR) {
                startPos = STARTPOS.GOAL;
                follower.setStartingPose(startPose1);
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

    //=============Start command just came in================

    @Override
    public void start() {
        //Stop vision to save resources
        aprilTagVision.stop(); //TODO might want to take this out to be able to figure out mosiac after moving

        //start wait
        //sleep((long)((startSleep)*1000)); TODO FIX

        opmodeTimer.resetTimer();
        //setPathState(0);

        //RED, BLUE, FAR, CLOSE
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        //In case we keep the camera on during the auto loop
        //Disable it here.
        aprilTagVision.stop();
    }
}