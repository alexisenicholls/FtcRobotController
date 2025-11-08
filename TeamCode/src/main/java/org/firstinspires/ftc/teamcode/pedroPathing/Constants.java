package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.16)
            .forwardZeroPowerAcceleration(-40.17214561629185)
            .lateralZeroPowerAcceleration(-54.53313144693408);


    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .leftPodY(3)
            .rightPodY(-3)
            .strafePodX(-2.5)
            .leftEncoder_HardwareMapName("br")
            .rightEncoder_HardwareMapName("bl")
            .strafeEncoder_HardwareMapName("fr")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP))
            .forwardTicksToInches(0.0020045042488761686)
            .strafeTicksToInches(0.00204900070779000633)
            .turnTicksToInches(0.0019284613625701373);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr") // 1 TODO set the correct names
            .rightRearMotorName("br") // 3
            .leftRearMotorName("bl") // 0
            .leftFrontMotorName("fl")// 2
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(57.5110963997171)
            .yVelocity(49.028104086235686)
            ;



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .threeWheelIMULocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)


                .build();
    }
}