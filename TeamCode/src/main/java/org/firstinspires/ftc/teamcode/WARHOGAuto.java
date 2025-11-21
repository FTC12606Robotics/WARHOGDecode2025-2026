package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;
import static java.lang.Math.PI;

import android.health.connect.datatypes.MealType;

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
public class WARHOGAuto extends OpMode {
    public WARHOGAuto() throws InterruptedException {}
    private Outtake outtake;
    private AprilTagVision aprilTagVision;

    private enum MOTIF {PPG, PGP, GPP, NONE} //PPG=23, PGP=22, GPP=21
    private enum STARTPOS {FAR, GOAL} //Launch zones
    private enum COLOR {RED, BLUE} //Start Color
    private enum LEAVE {YES, NO}

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    boolean useCamera = true; //To decide to use the camera during a run or not

    double speed = .50;
    double startSleep = 0; //How many seconds to wait before starting the autonomous routine

    double hopperRotationMSec = 1000; // approx milli seconds to rotate hopper 1 position at .2 speed

    //Set Default
    private MOTIF motif = MOTIF.NONE;
    private STARTPOS startPos = STARTPOS.GOAL;
    private COLOR color = COLOR.RED;
    private LEAVE leave = LEAVE.NO;

    //====================FOR PEDROPATHING====================
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPoseCloseRed = new Pose(123, 128, Math.toRadians(38)); // first start Pose of our robot, close to goal
    private final Pose checkPoseCloseRed = new Pose(90, 85, Math.toRadians(100)); // position to check mosaic pattern from obelisk
    private final Pose scorePoseCloseRed = new Pose(88, 90, Math.toRadians(49)); // first Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose endPoseCloseRed = new Pose(90, 134.5, Math.toRadians(90)); // First ending position outside of zone for points
    private final Pose startPoseFarRed = new Pose(87.5, 8, Math.toRadians(90)); // Second start position of our robot, far from goal
    private final Pose scorePoseFarRed = new Pose(86.5, 19, Math.toRadians(70)); // Second Scoring Pose of our robot.
    private final Pose endPoseFarRed = new Pose(110, 12, Math.toRadians(0)); // Second ending position outside of zone for points

    private final Pose startPoseCloseBlue = new Pose(21, 128, Math.toRadians(142)); // first start Pose of our robot, close to goal
    private final Pose checkPoseCloseBlue = new Pose(54, 85, Math.toRadians(80)); // position to check mosaic pattern from obelisk
    private final Pose scorePoseCloseBlue = new Pose(55, 90, Math.toRadians(140)); // first Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose endPoseCloseBlue = new Pose(54, 134.5, Math.toRadians(90)); // First ending position outside of zone for points
    private final Pose startPoseFarBlue = new Pose(57, 8, Math.toRadians(90)); // Second start position of our robot, far from goal
    private final Pose scorePoseFarBlue = new Pose(58, 19, Math.toRadians(115)); // Second Scoring Pose of our robot.
    private final Pose endPoseFarBlue = new Pose(42, 12, Math.toRadians(180)); // Second ending position outside of zone for points

    private Path scorePreloadCloseRed, scorePreloadFarRed, checkCloseRed, checkCloseBlue, scorePreloadCloseBlue, scorePreloadFarBlue, leaveCloseBlue, leaveCloseRed, leaveFarBlue, leaveFarRed;
    private PathChain endCloseRed, endFarRed, endCloseBlue, endFarBlue, checkScoreCloseRed, checkScoreCloseBlue;
    //====================FOR PEDROPATHING====================

    //Use the camera to detect april tags
    public void detectTag() {
        AprilTagDetection detection = null;
        if (aprilTagVision != null) {
            detection = aprilTagVision.getBestTag();
        }

        if (detection != null) {
            int id = detection.id;
            telemetry.addData("Tag ID", detection.id);
                /*if (det.metadata != null) {
                    telemetry.addData("Range (m)", "%.2f", det.ftcPose.range);
                    telemetry.addData("Bearing (deg)", "%.1f", det.ftcPose.bearing);
                }*/

            //Set Mosaic Arrangement
            if (id == 21) {
                motif = MOTIF.GPP;
            } else if (id == 22) {
                motif = MOTIF.PGP;
            } else if (id == 23) {
                motif = MOTIF.PPG;
            } else {
                motif = MOTIF.NONE; // In case we pick up the goal mosaics
            }
        } else {
            telemetry.addLine("No tag detected");
        }
    }

    public void shootArtifacts() throws InterruptedException{
        if (motif == MOTIF.PPG || motif == MOTIF.NONE){ //score accordingly, none default is ppg
            //spin left .5
            outtake.turnHopperMagAuto(Outtake.HOPPERDIRECTION.LEFT, .2);
            sleep(500);
            //launch
            outtake.runPiston();
            //continue spin left 1
            outtake.turnHopperMagAuto(Outtake.HOPPERDIRECTION.LEFT, .2);
            sleep(500);
            //launch
            outtake.runPiston();
            //continue spin left 1
            outtake.turnHopperMagAuto(Outtake.HOPPERDIRECTION.LEFT, .2);
            sleep(500);
            //launch
            outtake.runPiston();
        }
        else if (motif == MOTIF.GPP){
            //spin right .5
            outtake.turnHopperMagAuto(Outtake.HOPPERDIRECTION.RIGHT, .2);
            sleep(500);
            //launch
            outtake.runPiston();
            //continue spin right 1
            outtake.turnHopperMagAuto(Outtake.HOPPERDIRECTION.RIGHT, .2);
            sleep(500);
            //launch
            outtake.runPiston();
            //continue spin right 1
            outtake.turnHopperMagAuto(Outtake.HOPPERDIRECTION.RIGHT, .2);
            sleep(500);
            //launch
            outtake.runPiston();
        }
        else if (motif == MOTIF.PGP){
            //spin left .5
            outtake.turnHopperMagAuto(Outtake.HOPPERDIRECTION.LEFT, .2);
            sleep(500);
            //launch
            outtake.runPiston();
            // spin right 1
            outtake.turnHopperMagAuto(Outtake.HOPPERDIRECTION.RIGHT, .2);
            outtake.turnHopperMagAuto(Outtake.HOPPERDIRECTION.RIGHT, .2);
            sleep(500);
            //launch
            outtake.runPiston();
            //continue spin right 1
            outtake.turnHopperMagAuto(Outtake.HOPPERDIRECTION.RIGHT, .2);
            sleep(500);
            //launch
            outtake.runPiston();
        }
    }

    public void buildPaths() {
        //This is our scorePreloads go from start to score, check goes from start to check pos,
        scorePreloadCloseRed = new Path(new BezierLine(startPoseCloseRed, scorePoseCloseRed));
        scorePreloadCloseRed.setLinearHeadingInterpolation(startPoseCloseRed.getHeading(), scorePoseCloseRed.getHeading());

        checkCloseRed = new Path(new BezierLine(startPoseCloseRed, checkPoseCloseRed));
        checkCloseRed.setLinearHeadingInterpolation(startPoseCloseRed.getHeading(), checkPoseCloseRed.getHeading());

        scorePreloadFarRed = new Path(new BezierLine(startPoseFarRed, scorePoseFarRed));
        scorePreloadFarRed.setLinearHeadingInterpolation(startPoseFarRed.getHeading(), scorePoseFarRed.getHeading());

        scorePreloadCloseBlue = new Path(new BezierLine(startPoseCloseBlue, scorePoseCloseBlue));
        scorePreloadCloseBlue.setLinearHeadingInterpolation(startPoseCloseBlue.getHeading(), scorePoseCloseBlue.getHeading());

        checkCloseBlue = new Path(new BezierLine(startPoseCloseBlue, checkPoseCloseBlue));
        checkCloseBlue.setLinearHeadingInterpolation(startPoseCloseBlue.getHeading(), checkPoseCloseBlue.getHeading());

        scorePreloadFarBlue = new Path(new BezierLine(startPoseFarBlue, scorePoseFarBlue));
        scorePreloadFarBlue.setLinearHeadingInterpolation(startPoseFarBlue.getHeading(), scorePoseFarBlue.getHeading());

        leaveCloseBlue = new Path(new BezierLine(startPoseCloseBlue, endPoseCloseBlue));
        leaveCloseBlue.setLinearHeadingInterpolation(startPoseCloseBlue.getHeading(), endPoseCloseBlue.getHeading());

        leaveCloseRed = new Path(new BezierLine(startPoseCloseRed, endPoseCloseRed));
        leaveCloseRed.setLinearHeadingInterpolation(startPoseCloseRed.getHeading(), endPoseCloseRed.getHeading());

        leaveFarBlue = new Path(new BezierLine(startPoseFarBlue, endPoseFarBlue));
        leaveFarBlue.setLinearHeadingInterpolation(startPoseFarBlue.getHeading(), endPoseFarBlue.getHeading());

        leaveFarRed = new Path(new BezierLine(startPoseFarRed, endPoseFarRed));
        leaveFarRed.setLinearHeadingInterpolation(startPoseFarRed.getHeading(), endPoseFarRed.getHeading());
        //Here is an example for Constant Interpolation
        //scorePreload.setConstantInterpolation(startPose.getHeading());

        //check score goes from check position to scoring position
        checkScoreCloseRed = follower.pathBuilder()
                .addPath(new BezierLine(checkPoseCloseRed, scorePoseCloseRed))
                .setLinearHeadingInterpolation(checkPoseCloseRed.getHeading(), scorePoseCloseRed.getHeading())
                .build();

        checkScoreCloseBlue = follower.pathBuilder()
                .addPath(new BezierLine(checkPoseCloseBlue, scorePoseCloseBlue))
                .setLinearHeadingInterpolation(checkPoseCloseBlue.getHeading(), scorePoseCloseBlue.getHeading())
                .build();

        endCloseRed = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseCloseRed, endPoseCloseRed))
                .setLinearHeadingInterpolation(scorePoseCloseRed.getHeading(), endPoseCloseRed.getHeading())
                .build();

        endFarRed = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseFarRed, endPoseFarRed))
                .setLinearHeadingInterpolation(scorePoseFarRed.getHeading(), endPoseFarRed.getHeading())
                .build();

        endCloseBlue = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseCloseBlue, endPoseCloseBlue))
                .setLinearHeadingInterpolation(scorePoseCloseBlue.getHeading(), endPoseCloseBlue.getHeading())
                .build();

        endFarBlue = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseFarBlue, endPoseFarBlue))
                .setLinearHeadingInterpolation(scorePoseFarBlue.getHeading(), endPoseFarBlue.getHeading())
                .build();
    }

    public void autonomousPathUpdate() throws InterruptedException {
        double detectionStartTime;
        // Cases 0-3 for close red, cases 4-6 for far red, cases 7-10 for close blue, cases 11-13 for far blue
        switch (pathState) {
            //============Close Red===========
            case 0:
                if (useCamera){
                    if (leave == LEAVE.NO) {
                        follower.followPath(checkCloseRed);
                        setPathState(1);
                    }
                    if (leave == LEAVE.YES) {
                        follower.followPath(leaveCloseRed);
                        setPathState(3);
                    }
                }
                else{
                    if (leave == LEAVE.NO) {
                        follower.followPath(scorePreloadCloseRed);
                        setPathState(2);
                    }
                    if (leave == LEAVE.YES) {
                        follower.followPath(leaveCloseRed);
                        setPathState(3);
                    }
                }
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                //This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position
                if(!follower.isBusy()) {
                    //detect motif, for a max of 1 second
                    detectionStartTime = System.currentTimeMillis();
                    while (motif == MOTIF.NONE && ((System.currentTimeMillis() - detectionStartTime) / 1000) < 1){
                        detectTag();
                    }
                    aprilTagVision.stop();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(checkScoreCloseRed,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* TODO Score Artifacts */

                    //SpinLaunchMotors
                    outtake.spinLauncherVelocity(Outtake.TICKSPEED.SLOW); //Can change based on close/far now
                    shootArtifacts();
                    outtake.spinLauncherVelocity(Outtake.TICKSPEED.OFF);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(endCloseRed,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the ending position */
                if(!follower.isBusy()) {

                    setPathState(-1); //End route
                }
                break;

            //==========Far Red==========
            case 4:
                if (leave == LEAVE.NO){
                    follower.followPath(scorePreloadFarRed);
                    setPathState(5);
                }
                else if (leave == LEAVE.YES){
                    follower.followPath(leaveFarRed);
                    setPathState(6);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* TODO Score Sample */

                    //SpinLaunchMotors
                    outtake.spinLauncherVelocity(Outtake.TICKSPEED.MEDIUM); //Can change based on close/far now
                    shootArtifacts();
                    outtake.spinLauncherVelocity(Outtake.TICKSPEED.OFF);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(endFarRed,true); //TODO figure out how this works
                    setPathState(6);
                }
                break;
            case 6:
                //Get out of launch zone for points
                if(!follower.isBusy()) {
                    //If we want to setup for teleop after moving do here

                    setPathState(-1); //End route
                }
                break;

            //==========Close Blue==========
            case 7:
                if (useCamera){
                    if (leave == LEAVE.NO) {
                        follower.followPath(checkCloseBlue);
                        setPathState(8);
                    }
                    if (leave == LEAVE.YES) {
                        follower.followPath(leaveCloseBlue);
                        setPathState(10);
                    }
                }
                else{
                    if (leave == LEAVE.NO) {
                        follower.followPath(scorePreloadCloseBlue);
                        setPathState(9);
                    }
                    if (leave == LEAVE.YES) {
                        follower.followPath(leaveCloseBlue);
                        setPathState(10);
                    }
                }
                break;
            case 8:
                //This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position
                if(!follower.isBusy()) {
                    //detect motif, for a max of 1 second
                    detectionStartTime = System.currentTimeMillis();
                    while (motif == MOTIF.NONE && ((System.currentTimeMillis() - detectionStartTime) / 1000) < 1){
                        detectTag();
                    }
                    aprilTagVision.stop();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(checkScoreCloseBlue,true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* TODO Score Artifacts */

                    //SpinLaunchMotors
                    outtake.spinLauncherVelocity(Outtake.TICKSPEED.SLOW); //Can change based on close/far now
                    shootArtifacts();
                    outtake.spinLauncherVelocity(Outtake.TICKSPEED.OFF);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(endCloseBlue,true);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    //Todo anything at end

                    setPathState(-1); //End route
                }
                break;

            //==========Far Blue==========
            case 11:
                if (leave == LEAVE.NO){
                    follower.followPath(scorePreloadFarBlue);
                    setPathState(12);
                }
                else if (leave == LEAVE.YES){
                    follower.followPath(leaveFarBlue);
                    setPathState(13);
                }
                break;
            case 12:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* TODO Score Artifacts */

                    //SpinLaunchMotors
                    outtake.spinLauncherVelocity(Outtake.TICKSPEED.MEDIUM); //Can change based on close/far now
                    shootArtifacts();
                    outtake.spinLauncherVelocity(Outtake.TICKSPEED.OFF);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(endFarBlue,true); //TODO figure out how this works
                    setPathState(13);
                }
                break;
            case 13:
                //Get out of launch zone for points
                if(!follower.isBusy()) {
                    //If we want to setup for teleop after moving do here,  dup. w/6 might not need

                    setPathState(-1); //End route
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
        /*try {
            Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry, "BNO055IMU"); // TODO Do we even need drivtrain for auto?
        } catch (InterruptedException e) {
            telemetry.addLine("Drivetrain failed to initialize");
            throw new RuntimeException(e);
        }*/
        aprilTagVision = new AprilTagVision(hardwareMap);
        outtake = new Outtake(hardwareMap, telemetry);

        //Initialize for PedroPathing
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        //follower.setStartingPose(startPoseCloseRed); //Based on the default start position set earlier

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
        if (currentGamepad1.b && !previousGamepad1.b) {
            color = COLOR.RED;
        }
        if (currentGamepad1.x) {
            color = COLOR.BLUE;
        }
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            if (startPos == STARTPOS.GOAL) {
                startPos = STARTPOS.FAR;
            } else if (startPos == STARTPOS.FAR) {
                startPos = STARTPOS.GOAL;
            }
        }

        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            if (leave == LEAVE.NO) {
                leave = LEAVE.YES;
            } else if (leave == LEAVE.YES) {
                leave = LEAVE.NO;
            }
        }

        //For camera usage in decision making
        if (currentGamepad1.left_stick_button && !previousGamepad1.left_stick_button) {
            useCamera = !useCamera;
        }

        telemetry.addData("color (x/b)", color);
        telemetry.addData("launchPos (lbump)", startPos);
        telemetry.addData("leave (rbump)", leave);
        telemetry.addData("Speed (a/y)", speed);
        //telemetry.addData("startSleep (up/down)", startSleep);
        //telemetry.addData("Heading: ", follower.getHeading());
        telemetry.addLine();
        telemetry.addData("Use Camera? (lsbtn)", useCamera);

        //===============April Tag Vision===============
        if (useCamera) {
            detectTag();
        } else {
            motif = MOTIF.NONE;
        }

        telemetry.addData("Detected Motif pattern: ", motif);
        //===============April Tag Vision===============

        telemetry.update();
    }

    //=============Start command just came in================

    @Override
    public void start() {

        //start wait
        //sleep((long)((startSleep)*1000)); TODO FIX

        opmodeTimer.resetTimer();

        //Switch to paths based on alliance and start pos
        if (startPos == STARTPOS.GOAL){
            if (color == COLOR.BLUE){
                follower.setStartingPose(startPoseCloseBlue);
                setPathState(7);
            }
            else {
                follower.setStartingPose(startPoseCloseRed);
                setPathState(0);
            }
        }
        if (startPos == STARTPOS.FAR){
            //Stop vision to save resources
            aprilTagVision.stop(); // leave it on for checking when starting close

            if (color == COLOR.BLUE){
                follower.setStartingPose(startPoseFarBlue);
                setPathState(11);
            }
            else{
                follower.setStartingPose(startPoseFarRed);
                setPathState(4);
            }
        }

    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        try { //TODO see if this affects anything, it is because outtake needing to spin hopper based on time.
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading()*180/PI);
        //telemetry.addData("launcher speed", outtake.getSpinPower(1));
        telemetry.addData("Motif: ", motif);
        telemetry.update();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        //In case we keep the camera on during the auto loop, Disable it here/now.
        //aprilTagVision.stop();
    }
}