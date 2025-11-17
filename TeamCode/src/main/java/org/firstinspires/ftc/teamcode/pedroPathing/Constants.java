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

// === Limelight Imports ===
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import java.util.List;

/**
 * Constants configuration for drivetrain, localization, follower, and Limelight vision.
 */
public class Constants {

    // === Limelight Constants ===
    public static Limelight3A limelight;

    // === Vision Tuning ===
    public static final double ROTATE_KP = 0.035;
    public static final double TX_DEADBAND = 1.5;
    public static final double MAX_ROTATE_SPEED = 0.6;
    public static final double DRIVE_SPEED = 0.5;

    // === Core Follower Constants ===
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.16)
            .forwardZeroPowerAcceleration(-40.17214561629185)
            .lateralZeroPowerAcceleration(-54.53313144693408);

    // === Localizer (Odo + IMU) Constants ===
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
            .IMU_Orientation(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP))
            .forwardTicksToInches(0.0020045042488761686)
            .strafeTicksToInches(0.00204900070779000633)
            .turnTicksToInches(0.0019284613625701373);

    // === Drivetrain Constants ===
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(57.5110963997171)
            .yVelocity(49.028104086235686);

    // === Intake Motor Constants (ADDED) ===
    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final DcMotorSimple.Direction INTAKE_DIRECTION =
            DcMotorSimple.Direction.REVERSE;

    // === Path Following Constraints ===
    public static PathConstraints pathConstraints =
            new PathConstraints(0.99, 100, 1, 1);

    // === Follower Builder ===
    public static Follower createFollower(HardwareMap hardwareMap) {

        // initialize limelight once
        if (limelight == null) {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);
            limelight.start();
        }

        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .threeWheelIMULocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    // === Limelight Helper ===
    public static LLResult getLatestVisionResult() {
        if (limelight == null) return null;
        return limelight.getLatestResult();
    }

    // === AprilTag Helper ===
    public static FiducialResult getPrimaryAprilTag() {
        LLResult result = getLatestVisionResult();
        if (result != null && result.isValid()) {
            List<FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                return fiducials.get(0);
            }
        }
        return null;
    }
}
