package org.firstinspires.ftc.teamcode.codes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled

@TeleOp(name="Turret IMU + PID + Limelight Safe", group="Test")
public class turret extends LinearOpMode {

    private DcMotor turretMotor;
    private IMU imu;

    // PID coefficients
    private final double kP = 0.015;
    private final double kI = 0.0;
    private final double kD = 0.002;

    private double integral = 0;
    private double lastError = 0;

    private double initialHeading = 0;
    private double initialTurretPos = 0;

    // Turret configuration
    private final double ticksPerRev = 537.7; // REV Core Hex example
    private final double turretGearRatio = 1.0; // adjust if gearing exists
    private final double toleranceTicks = 5; // stop motor within this range

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware init ---
        turretMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        telemetry.addLine("IMU initialized");
        telemetry.update();

        // Record initial positions
        initialHeading = getYaw();
        initialTurretPos = turretMotor.getCurrentPosition();

        waitForStart();

        while (opModeIsActive()) {

            // --- 1. Read robot yaw ---
            double robotYaw = getYaw();

            // --- 2. Read vision target ---
            boolean targetVisible = getLimelightTV() == 1; // TV = 1 if target detected
            double targetOffsetDeg = 0;
            if (targetVisible) {
                targetOffsetDeg = getLimelightTX(); // horizontal offset in degrees
            }

            // --- 3. Compute desired turret position in ticks ---
            double ticksPerDegree = ticksPerRev / 360.0 * turretGearRatio;
            double desiredTurretPos = initialTurretPos
                    - (robotYaw - initialHeading) * ticksPerDegree
                    + targetOffsetDeg * ticksPerDegree;

            // --- 4. PID control with deadzone ---
            double currentPos = turretMotor.getCurrentPosition();
            double error = desiredTurretPos - currentPos;

            if (Math.abs(error) < toleranceTicks || !targetVisible) {
                turretMotor.setPower(0); // hold position if target not visible
                integral = 0;
            } else {
                integral += error;
                double derivative = error - lastError;

                // Slow down near target
                double maxPower = 0.5;
                if (Math.abs(error) < 50) maxPower = 0.2;

                double power = kP * error + kI * integral + kD * derivative;
                power = Range.clip(power, -maxPower, maxPower);

                turretMotor.setPower(power);
                lastError = error;
            }

            // --- 5. Telemetry ---
            telemetry.addData("Yaw", robotYaw);
            telemetry.addData("Turret Encoder", currentPos);
            telemetry.addData("Target Visible", targetVisible);
            telemetry.addData("Target Offset", targetOffsetDeg);
            telemetry.addData("Error", error);
            telemetry.update();
        }
    }

    // --- Get robot yaw in degrees ---
    private double getYaw() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw();
    }

    // --- Placeholder: Limelight horizontal offset (TX) ---
    private double getLimelightTX() {
        // Replace with NetworkTables code:
        // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        // return table.getEntry("tx").getDouble(0);
        return 0;
    }

    // --- Placeholder: Limelight target visible (TV) ---
    private int getLimelightTV() {
        // Replace with NetworkTables code:
        // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        // return (int) table.getEntry("tv").getDouble(0);
        return 0; // default = no target
    }
}
