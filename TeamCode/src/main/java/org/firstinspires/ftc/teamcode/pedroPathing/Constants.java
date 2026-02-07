package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(7.620352)
            .forwardZeroPowerAcceleration(-36.213617734240536) //-31.663326054851762//-32.00933697634453//-37.032989351502074//-32.94547437274599//-37//-33.88136202931032//-40.7//-36.213617734240536
            .lateralZeroPowerAcceleration(-65.00713386333912) //-68.11815096864491//-64.5116452151177//-73.02245413472865//-61.00995862598752//-65.00713386333912//-67.50451142539538//
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0.01, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1.05, 0, 0, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.0005,0.6,0.01))
            .centripetalScaling(0.0005);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, .95, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(.8) //max drive power
            .xVelocity(48.886156938207435) //48.7566437007874//48.886156938207435//48.958174097256396
            .yVelocity(38.878132737527686) //38.878132737527686//39.295308991679995//38.86423762764518
            .rightFrontMotorName("rightFrontDrive")
            .rightRearMotorName("rightBackDrive")
            .leftRearMotorName("leftBackDrive")
            .leftFrontMotorName("leftFrontDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.5)
            .strafePodX(4.4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}