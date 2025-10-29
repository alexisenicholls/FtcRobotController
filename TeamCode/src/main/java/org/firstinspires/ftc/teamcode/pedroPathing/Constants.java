package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(1) //TODO go to this link to tune the robot (found on telemetry) https://pedropathing.com/docs/pathing/tuning/automatic
            .lateralZeroPowerAcceleration(1) //TODO again found on telemetry (same link as above) (tuning)
            .mass(5); //TODO set the mass kg


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf") //TODO set the correct names
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .xVelocity(1) //TODO set the correct velocity found on the link https://pedropathing.com/docs/pathing/tuning/automatic (number displayed on telemetry)
            .yVelocity(1) //TODO found on the link above (tuning same thing)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.001989436789)
            .strafeTicksToInches(.001989436789)
            .turnTicksToInches(.001989436789)
            .leftPodY(1) //TODO set the correct offsets
            .rightPodY(-1)
            .strafePodX(-2.5)
            .leftEncoder_HardwareMapName("leftFront") //TODO set the correct names
            .rightEncoder_HardwareMapName("rightRear")
            .strafeEncoder_HardwareMapName("rightFront")
            .forwardTicksToInches(1.1) //TODO set the correct multiplier https://pedropathing.com/docs/pathing/tuning/localization/three-wheel
            .strafeTicksToInches(1.1) //TODO same thing was the forward test found on the link
            .turnTicksToInches(1.1) //TODO same thing was the forward test found on the link (found on the link)
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);





}
