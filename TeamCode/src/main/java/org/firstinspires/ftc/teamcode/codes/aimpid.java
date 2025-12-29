package org.firstinspires.ftc.teamcode.codes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled
@TeleOp(name = "ApriltagAimOnly_PIDF", group = "Aiming")
public class aimpid extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;
    private DcMotor aimMotor;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // ----------------------------
    // YOUR PIDF VALUES
    // ----------------------------
    private double kP = 0.035;
    private double kI = 0.0;
    private double kD = 0.005;
    private double kF = 0.02;

    private double deadzone = 0.6;
    private double derivativeAlpha = 0.9;
    private double txFilterAlpha = 0.85;

    private double integralLimit = 200;
    private double maxErrorScale = 30;
    private double maxPowerDeltaPerSec = 1;
    private double stableHoldTime = 0.20;

    // PID state
    private double lastTx = 0;
    private double lastError = 0;
    private double integral = 0;
    private double filteredDerivative = 0;
    private double lastOutput = 0;
    private long lastTimeMs = 0;

    private boolean trackingEnabled = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // ----------------------------
        // INIT
        // ----------------------------
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9);

        imu = hardwareMap.get(IMU.class, "imu");

        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevHubOrientationOnRobot hubOrientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );

        imu.initialize(new IMU.Parameters(hubOrientation));

        telemetry.addLine("Aiming system ready. Press START.");
        telemetry.update();

        waitForStart();

        limelight.start();
        lastTimeMs = System.currentTimeMillis();

        // ----------------------------
        // MAIN LOOP
        // ----------------------------
        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Toggle tracking
            if (gamepad1.y) {
                trackingEnabled = !trackingEnabled;
                sleep(250);
                integral = 0;
                lastError = 0;
            }

            // Update orientation to Limelight
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                double tx = result.getTx();

                telemetry.addData("Tx", tx);
                telemetry.addData("Tracking", trackingEnabled);

                if (trackingEnabled) {
                    double power = runPIDFTracking(tx);
                    aimMotor.setPower(power);
                    telemetry.addData("AimPower", power);
                } else {
                    aimMotor.setPower(0);
                }

            } else {
                telemetry.addLine("NO TARGET");
                aimMotor.setPower(0);
            }

            telemetry.update();
        }

        aimMotor.setPower(0);
    }

    // ----------------------------------------------------------
    //  YOUR 100% EXACT PIDF + FILTERING
    // ----------------------------------------------------------
    private double runPIDFTracking(double tx) {

        long now = System.currentTimeMillis();
        double dt = (now - lastTimeMs) / 1000.0;
        if (dt <= 0) dt = 0.02;
        lastTimeMs = now;

        // TX FILTER
        double filteredTx = (txFilterAlpha * lastTx) + ((1 - txFilterAlpha) * tx);
        lastTx = filteredTx;

        // DEADZONE
        if (Math.abs(filteredTx) < deadzone)
            return 0;

        // INTEGRAL
        integral += filteredTx * dt;
        if (Math.abs(integral) > integralLimit)
            integral = Math.signum(integral) * integralLimit;

        // DERIVATIVE (smoothed)
        double rawDerivative = (filteredTx - lastError) / dt;
        filteredDerivative = derivativeAlpha * filteredDerivative +
                (1 - derivativeAlpha) * rawDerivative;
        lastError = filteredTx;

        // PIDF
        double pid = (kP * filteredTx) + (kI * integral) + (kD * filteredDerivative);
        double ff = kF * Math.signum(filteredTx);

        double output = pid + ff;

        // DISTANCE-BASED SCALING
        double scale = Math.min(Math.abs(filteredTx) / maxErrorScale, 1.0);
        output *= scale;

        // OUTPUT RAMP (no jumps)
        double maxDelta = maxPowerDeltaPerSec * dt;
        if (output - lastOutput > maxDelta) output = lastOutput + maxDelta;
        if (lastOutput - output > maxDelta) output = lastOutput - maxDelta;

        lastOutput = output;

        return output;
    }
}
