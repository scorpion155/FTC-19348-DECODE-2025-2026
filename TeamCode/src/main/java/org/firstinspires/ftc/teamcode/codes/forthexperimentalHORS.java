package org.firstinspires.ftc.teamcode.codes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelVersatile;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelVersatile.CalibrationPoint;
import org.firstinspires.ftc.teamcode.subsystems.TurretGoalAimer;

import java.util.Arrays;
import java.util.List;
@Disabled

@TeleOp(name = "Turret and Dynamic HORS", group = "Linear OpMode")
public class forthexperimentalHORS extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, turret, intakeMotor;
    private Servo clawServo, leftCompressionServo, rightCompressionServo;
    private Servo leftHoodServo, rightHoodServo;
    private Servo gateServo;

    private DriveController driveController;
    private Flywheel flywheel;
    private FlywheelVersatile flywheelVersatile;
    private TurretGoalAimer turretGoalAimer;

    private Follower follower;
    private Pose currentPose = new Pose();

    private IMU imu;   // ✅ BHI260IMU uses IMU interface

    private static final Pose BLUE_GOAL = new Pose(14, 134, 0);

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        shooter = hardwareMap.get(DcMotor.class, "shooter1");
        turret = hardwareMap.get(DcMotor.class, "aimMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        clawServo = hardwareMap.get(Servo.class, "c");
        leftCompressionServo = hardwareMap.get(Servo.class, "l");
        rightCompressionServo = hardwareMap.get(Servo.class, "r");
        leftHoodServo = hardwareMap.get(Servo.class, "lh");
        rightHoodServo = hardwareMap.get(Servo.class, "rh");
        gateServo = hardwareMap.get(Servo.class, "pushservo");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // =========================
        // ✅ BHI260 IMU INITIALIZATION
        // =========================
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(imuParams);

        // =========================

        driveController = new DriveController(
                frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive
        );

        flywheel = new Flywheel(shooter, telemetry);

        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(72, 72, 0));
            follower.update();
            currentPose = follower.getPose();
        } catch (Exception e) {
            follower = null;
        }

        List<CalibrationPoint> calibrationPoints = Arrays.asList(
                new CalibrationPoint(new Pose(48, 96, 135), 90.0),
                new CalibrationPoint(new Pose(60, 125, 0), 95.0),
                new CalibrationPoint(new Pose(60, 82, 0), 100.0),
                new CalibrationPoint(new Pose(72, 72, 0), 110.0),
                new CalibrationPoint(new Pose(52, 14, 0), 140.0)
        );

        flywheelVersatile = new FlywheelVersatile(
                flywheel, BLUE_GOAL, calibrationPoints, 90.0, 150.0
        );

        // ✅ Turret uses IMU interface (BHI260)
        turretGoalAimer = new TurretGoalAimer(turret, imu, telemetry);

        telemetry.addLine("Initialized (BHI260IMU)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (follower != null) {
                follower.update();
                currentPose = follower.getPose();
            }

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            driveController.setDrive(axial, lateral, yaw, 1.0);

            double targetRpm = flywheelVersatile.getFinalTargetRPM(currentPose);
            flywheel.setTargetRPM(targetRpm);
            flywheel.update(System.currentTimeMillis(), gamepad1.y);

            turretGoalAimer.update(
                    false,
                    0.0,
                    currentPose,
                    BLUE_GOAL
            );

            telemetry.update();
        }
    }
}
