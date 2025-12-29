package org.firstinspires.ftc.teamcode.codes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled

@TeleOp(name = "AimMotor_NoOscillation", group = "Aiming")
public class reddit extends LinearOpMode {

    private DcMotor aimMotor;
    private Limelight3A limelight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Strong deadzone so no oscillation
    private static final double DEADZONE = 1.5;

    // Minimum torque for heavy gearing
    private static final double BASE_TORQUE = 0.12;

    // Power limit
    private static final double MAX_POWER = 0.45;

    // When close to center, reduce speed a lot
    private static final double SLOW_ZONE = 6;

    // Much weaker backlash compensation
    private static final double BACKLASH_FEEDFORWARD = 0.015;

    // Filter for smooth aiming
    private double filteredTx = 0;
    private static final double FILTER_ALPHA = 0.85;

    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        waitForStart();

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
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                double tx = result.getTx();

                // Low-pass filter to remove noise
                filteredTx = (FILTER_ALPHA * filteredTx) + (1 - FILTER_ALPHA) * tx;
                double error = filteredTx;

                telemetry.addData("tx", tx);
                telemetry.addData("filteredTx", filteredTx);

                // -------- DEADZONE --------
                if (Math.abs(error) <= DEADZONE) {
                    aimMotor.setPower(0);
                    telemetry.addData("Status", "Locked");
                    telemetry.update();
                    continue;
                }

                // -------- DYNAMIC SPEED CONTROL --------
                double scale;
                if (Math.abs(error) > SLOW_ZONE) {
                    // Far → move fast
                    scale = Math.min(Math.abs(error) / 18.0, 1.0);
                } else {
                    // Close → move very slow
                    scale = 0.25 * (Math.abs(error) / SLOW_ZONE);
                }

                double power = BASE_TORQUE + scale * (MAX_POWER - BASE_TORQUE);

                // Direction
                if (error < 0) power = -power;

                // Light backlash assistance
                if (Math.abs(error) < 4) {
                    power += (power > 0 ? BACKLASH_FEEDFORWARD : -BACKLASH_FEEDFORWARD);
                }

                aimMotor.setPower(power);
                telemetry.addData("Status", "Tracking");
                telemetry.addData("Power", power);
            }
            else {
                aimMotor.setPower(0);
                telemetry.addData("Status", "No tag");
            }

            telemetry.update();
        }
    }
}
