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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

// Limelight
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import java.util.List;

public class Constants {

    // === Limelight Constants ===
    public static Limelight3A limelight;

    // === Vision Tuning ===
    public static final double ROTATE_KP = 0.035;
    public static final double TX_DEADBAND = 1.5;
    public static final double MAX_ROTATE_SPEED = 0.6;
    public static final double DRIVE_SPEED = 0.5;

    // === Follower Constants ===
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.16)
            .forwardZeroPowerAcceleration(-40.17214561629185)
            .lateralZeroPowerAcceleration(-54.53313144693408);

    // === Localizer Constants ===
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

    // === Intake Motor Constants ===
    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final DcMotorSimple.Direction INTAKE_DIRECTION =
            DcMotorSimple.Direction.FORWARD;

    // === Indexer Motor Constants ===
    public static final String INDEXER_MOTOR_NAME = "feeder";
    public static final DcMotorSimple.Direction INDEXER_DIRECTION =
            DcMotorSimple.Direction.REVERSE;

    // === Shooter Motor Constants ===
    public static final String SHOOTER_MOTOR_NAME = "shooter";
    public static final DcMotorSimple.Direction SHOOTER_DIRECTION =
            DcMotorSimple.Direction.REVERSE;

    // === Servo Constants (NEW) ===
    public static final String SERVO_NAME = "servo";
    public static final Servo.Direction SERVO_DIRECTION = Servo.Direction.FORWARD;

    // === Shooter Specs ===
    public static final double SHOOTER_WHEEL_DIAMETER_INCH = 4.0;
    public static final double SHOOTER_GEAR_RATIO = (18.0 / 24.0);
    public static final double SHOOTER_MAX_MOTOR_RPM = 6000.0;

    public static final double SHOOTER_MAX_WHEEL_RPM =
            SHOOTER_MAX_MOTOR_RPM * SHOOTER_GEAR_RATIO;

    public static final double SHOOTER_kP = 20.0;
    public static final double SHOOTER_kI = 0.2;
    public static final double SHOOTER_kD = 1.5;
    public static final double SHOOTER_kF = 32767.0 / SHOOTER_MAX_WHEEL_RPM;

    // === Path Constraints ===
    public static PathConstraints pathConstraints =
            new PathConstraints(0.99, 100, 1, 1);

    // === Hardware Accessors ===
    public static DcMotorEx intakeMotor;
    public static DcMotorEx shooterMotor;
    public static DcMotorEx indexerMotor;
    public static Servo servo;   // <<< NEW SERVO

    // === Follower Builder ===
    public static Follower createFollower(HardwareMap hardwareMap) {

        // Setup limelight
        if (limelight == null) {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);
            limelight.start();
        }

        // Load motors
        intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);
        intakeMotor.setDirection(INTAKE_DIRECTION);

        shooterMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR_NAME);
        shooterMotor.setDirection(SHOOTER_DIRECTION);

        indexerMotor = hardwareMap.get(DcMotorEx.class, INDEXER_MOTOR_NAME);
        indexerMotor.setDirection(INDEXER_DIRECTION);

        // === NEW: Load servo ===
        servo = hardwareMap.get(Servo.class, SERVO_NAME);
        servo.setDirection(SERVO_DIRECTION);

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

    // === Tag Helper ===
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
