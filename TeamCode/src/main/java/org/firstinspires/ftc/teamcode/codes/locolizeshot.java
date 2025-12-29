package org.firstinspires.ftc.teamcode.codes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
@Disabled

@TeleOp(name = "AdvancedPredictiveTurret")
public class locolizeshot extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Odometry / dead wheels (assume 3)
    private DcMotor leftWheel, rightWheel, backWheel;

    // Turret motor
    private DcMotor aimMotor;

    // Limelight
    private Limelight3A limelight;

    // IMU
    private IMU imu;
    private double lastHeading = 0;

    // Turret control
    private double turretAngle = 0;      // desired turret angle (deg)
    private double lastSeenTime = -1000;
    private double filteredTx = 0;

    // Constants
    private static final double DEADZONE = 1.5;        // degrees
    private static final double FILTER_ALPHA = 0.85;
    private static final double SEARCH_SWEEP_DEG = 20; // blind sweep amplitude (deg)
    private static final int TICKS_PER_REV = 537;      // motor encoder ticks
    private static final double TURNTABLE_GEAR_RATIO = 1.0;
    private static final double MAX_TURRET_POWER = 0.7;
    private static final double KP_DEG = 0.015;

    @Override
    public void runOpMode() {

        // ----------------- Hardware Init -----------------
        frontLeft  = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft   = hardwareMap.dcMotor.get("backLeft");
        backRight  = hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Dead wheels
        leftWheel = hardwareMap.dcMotor.get("frontLeft");
        rightWheel = hardwareMap.dcMotor.get("frontRight");
        backWheel = hardwareMap.dcMotor.get("shooter2");

        // Turret motor
        aimMotor = hardwareMap.dcMotor.get("aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aimMotor.setTargetPosition(aimMotor.getCurrentPosition());
        aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));

        waitForStart();

        lastHeading = imu.getRobotYawPitchRollAngles().getYaw();

        // Odometry last positions
        double lastLeft = leftWheel.getCurrentPosition();
        double lastRight = rightWheel.getCurrentPosition();
        double lastBack = backWheel.getCurrentPosition();

        while (opModeIsActive()) {

            double dt = 0.02; // approximate loop time

            // ------------------- Mecanum Drive -------------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double fl = y + x + rx;
            double fr = y - x - rx;
            double bl = y - x + rx;
            double br = y + x - rx;

            double max = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

            frontLeft.setPower(fl / max);
            frontRight.setPower(fr / max);
            backLeft.setPower(bl / max);
            backRight.setPower(br / max);

            // ------------------- Turret Tracking -------------------
            LLResult result = limelight.getLatestResult();

            // IMU heading change
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw();
            double headingDelta = normalizeAngle(currentHeading - lastHeading);
            lastHeading = currentHeading;

            // Odometry deltas
            double deltaLeft = leftWheel.getCurrentPosition() - lastLeft;
            double deltaRight = rightWheel.getCurrentPosition() - lastRight;
            double deltaBack = backWheel.getCurrentPosition() - lastBack;

            lastLeft = leftWheel.getCurrentPosition();
            lastRight = rightWheel.getCurrentPosition();
            lastBack = backWheel.getCurrentPosition();

            // Compute robot movement in robot frame (simplified)
            double deltaX = (deltaLeft + deltaRight) / 2.0;
            double deltaY = deltaBack; // approximate lateral movement

            // Predict target angle change based on robot movement
            double robotMovementAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
            turretAngle -= headingDelta + robotMovementAngle;

            // Vision update
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                filteredTx = FILTER_ALPHA * filteredTx + (1 - FILTER_ALPHA) * tx;

                if (Math.abs(filteredTx) > DEADZONE) {
                    turretAngle = filteredTx;
                }
                lastSeenTime = getRuntime();
            } else {
                double timeSinceSeen = getRuntime() - lastSeenTime;
                if (timeSinceSeen > 0.3) {
                    turretAngle = SEARCH_SWEEP_DEG * Math.sin(getRuntime() * 2.0);
                }
            }

            // Encoder target
            int targetTicks = (int) (turretAngle / 360.0 * TICKS_PER_REV * TURNTABLE_GEAR_RATIO);
            aimMotor.setTargetPosition(targetTicks);

            int errorTicks = targetTicks - aimMotor.getCurrentPosition();
            double power = KP_DEG * (errorTicks / (TICKS_PER_REV * TURNTABLE_GEAR_RATIO) * 360.0);
            power = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, power));

            aimMotor.setPower(power);

            // ------------------- Telemetry -------------------
            telemetry.addData("Seen?", result != null && result.isValid());
            telemetry.addData("tx", String.format("%.2f", filteredTx));
            telemetry.addData("TurretAngle", String.format("%.2f", turretAngle));
            telemetry.addData("CurrentTicks", aimMotor.getCurrentPosition());
            telemetry.addData("TargetTicks", targetTicks);
            telemetry.addData("MotorPower", power);
            telemetry.update();
        }
    }

    // ---------- Helpers ----------
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
