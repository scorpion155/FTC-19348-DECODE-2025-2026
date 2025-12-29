package org.firstinspires.ftc.teamcode.disFile;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled


@TeleOp(name = "🔵 BLUE-Tele PIDF")
public class shootPIDF extends LinearOpMode {

    // Mecanum motors
    private Limelight3A limelight;
    private DcMotor intakeMotor;
    private DcMotorEx shooter1;  // Changed to DcMotorEx for PIDF control
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor aimMotor;
    private IMU imu;
    private Servo sorterServo;
    private Servo shooterServo;
    private Servo pushservo;
    private RevColorSensorV3 colorSensor;

    // ---- PIDF Shooter Control ----
    private double highVelocity = 1500;  // Updated from 1500
    private double lowVelocity = 900;   // Updated from 900
    private double targetVelocity = 0;

    // Tuned PIDF values (adjust these after tuning)
    private double F = 0.0;
    private double P = 0.0;
    private double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    private int stepIndex = 1;

    // PIDF Tuning mode toggle
    private boolean pidfTuningMode = false;
    private boolean optionsLatch = false;

    // Button latches for PIDF adjustment
    private boolean yPressedLast = false;
    private boolean bPressedLast = false;

    // ---- Intake positions ----
    private final double[] INTAKE_POS = {-0.00, 0.31, 0.488};
    private int intakeIndex = 0;
    private boolean xPressed = false;

    // ---- Launch positions ----
    private final double[] LAUNCH_POS = {0.22, 0.40, 0.58};
    private int launchIndex = 0;

    // ---- Push servo ----
    private final double PUSHSERVO_ACTIVE_POS = 70.0 / 360.0;
    private final double PUSHSERVO_INACTIVE_POS = 0.0;

    // ---- Intake ----
    private final double INTAKE_POWER = 1.0;

    // Button latches
    private boolean circleLatch = false;
    private boolean dpadRightLatch = false;

    // Pushing state
    private boolean isPushing = false;

    // Turret constants
    private static final double DEADZONE = 2.0;
    private static final double MAX_TURRET_POWER = 0.35;
    private static final double FILTER_ALPHA = 0.85;
    private static final double KP = 0.025;

    private double filteredTx = 0;

    // Limelight distance parameters
    private static final double CAMERA_HEIGHT = 0.25;
    private static final double TARGET_HEIGHT = 0.90;
    private static final double CAMERA_ANGLE  = 20.0;

    // Servo mapping
    private static final double SERVO_MIN = 0.3;
    private static final double SERVO_MAX = 0.65;
    private static final double DIST_MIN  = 0.4;
    private static final double DIST_MAX  = 3.0;

    private static final double shooter_MIN = 0.6;
    private static final double shooter_MAX = 1;
    private static final double DIST_MIN_sh  = 0.4;
    private static final double DIST_MAX_sh  = 2.50;

    boolean shooterOn = false;
    boolean lastbuttom = false;

    static final int MAX_BALLS = 3;
    int ballCount = 0;
    boolean ballDetected = false;
    boolean intakeFull = false;

    @Override
    public void runOpMode() {

        // --- Drive Motors ---
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");  // DcMotorEx for PIDF
        sorterServo = hardwareMap.get(Servo.class, "sortservo");
        pushservo = hardwareMap.get(Servo.class, "pushservo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        // Setup shooter motor for velocity control
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        updatePIDF();

        sorterServo.setPosition(INTAKE_POS[0]);
        pushservo.setPosition(PUSHSERVO_INACTIVE_POS);

        // --- Turret Motor ---
        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Shooter Servo ---
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        shooterServo.setPosition(SERVO_MIN);

        // --- Limelight ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();

        telemetry.addLine("Press OPTIONS (PS) / START (Xbox) to toggle PIDF Tuning Mode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ========== PIDF TUNING MODE TOGGLE ==========
            // Press OPTIONS/START button to enter/exit tuning mode
            if (gamepad1.options && !optionsLatch) {
                pidfTuningMode = !pidfTuningMode;
                optionsLatch = true;
            }
            if (!gamepad1.options) {
                optionsLatch = false;
            }

            // ========== PIDF TUNING MODE ==========
            if (pidfTuningMode) {
                handlePIDFTuning();
                continue;  // Skip normal teleop when tuning
            }

            // ========== NORMAL TELEOP MODE ==========

            // ------------------ Mecanum Drive ------------------
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

            // ------------------ Color Sensor Ball Detection ------------------
            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();
            int brightness = r + g + b;

            boolean ballPresent = brightness > 320;
            boolean ballGone    = brightness < 280;

            if (ballGone) {
                ballDetected = false;
            }

            if (ballPresent && !ballDetected && !intakeFull) {
                ballDetected = true;
                ballCount++;

                if (ballCount >= MAX_BALLS) {
                    ballCount = MAX_BALLS;
                    intakeFull = true;
                } else {
                    intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
                    sorterServo.setPosition(INTAKE_POS[intakeIndex]);
                }
            }

            if (gamepad1.left_stick_button) {
                onBallReleased(3);
            }

            // ------------------ Push Servo ------------------
            if (gamepad1.right_bumper && !xPressed) {
                xPressed = true;
                pushservo.setPosition(PUSHSERVO_ACTIVE_POS);
                servoDelay(250);
                pushservo.setPosition(PUSHSERVO_INACTIVE_POS);
            } else if (!gamepad1.right_bumper) {
                xPressed = false;
            }

            // ------------------ Launch Stage ------------------
            if (gamepad1.left_bumper && !circleLatch) {
                circleLatch = true;
                launchIndex = (launchIndex + 1) % LAUNCH_POS.length;
                sorterServo.setPosition(LAUNCH_POS[launchIndex]);
            }
            if (!gamepad1.left_bumper) {
                circleLatch = false;
            }

            if (gamepad1.left_stick_button && !dpadRightLatch && !isPushing) {
                dpadRightLatch = true;
                isPushing = true;

                intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
                sorterServo.setPosition(INTAKE_POS[intakeIndex]);

                isPushing = false;
            }
            if (!gamepad1.left_stick_button) {
                dpadRightLatch = false;
            }

            // ------------------ Intake Motor ------------------
            if (gamepad1.right_trigger > 0.2) {
                intakeMotor.setPower(INTAKE_POWER);
            } else {
                intakeMotor.setPower(0);
            }

            // ------------------ Turret & Shooter Control ------------------
            double turretPower = 0;
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();

                filteredTx = FILTER_ALPHA * filteredTx + (1 - FILTER_ALPHA) * tx;

                if (Math.abs(filteredTx) > DEADZONE) {
                    turretPower = filteredTx * KP;
                }

                double angleRad = Math.toRadians(CAMERA_ANGLE + ty);
                double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleRad);

                double servoPos = SERVO_MIN + (distance - DIST_MIN) / (DIST_MAX - DIST_MIN) * (SERVO_MAX - SERVO_MIN);
                servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
                shooterServo.setPosition(servoPos);

                // Map distance to velocity instead of power
                double shooterVel = lowVelocity + (distance - DIST_MIN_sh) / (DIST_MAX_sh - DIST_MIN_sh) * (highVelocity - lowVelocity);
                shooterVel = Math.max(lowVelocity, Math.min(highVelocity, shooterVel));
                targetVelocity = shooterVel;

                if (gamepad1.right_stick_button && !lastbuttom) {
                    shooterOn = !shooterOn;
                }
                lastbuttom = gamepad1.right_stick_button;

                if (shooterOn) {
                    shooter1.setVelocity(targetVelocity);  // Use velocity control
                } else {
                    shooter1.setVelocity(0);
                }
            }

            turretPower = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, turretPower));
            aimMotor.setPower(turretPower);

            // ------------------ Telemetry ------------------
            telemetry.addData("🎯 Mode", "NORMAL (press OPTIONS for PIDF tuning)");
            telemetry.addData("tx", filteredTx);
            telemetry.addData("Turret Power", turretPower);
            telemetry.addData("Shooter Target Vel", targetVelocity);
            telemetry.addData("Shooter Current Vel", shooter1.getVelocity());
            telemetry.addData("Servo Position", shooterServo.getPosition());
            telemetry.addData("Ball Count", ballCount);
            telemetry.update();
        }
    }

    // ========== PIDF TUNING MODE HANDLER ==========
    private void handlePIDFTuning() {

        // Toggle velocity (Y button)
        if (gamepad1.y && !yPressedLast) {
            targetVelocity = (targetVelocity == highVelocity) ? lowVelocity : highVelocity;
        }
        yPressedLast = gamepad1.y;

        // Cycle step sizes (B button)
        if (gamepad1.b && !bPressedLast) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        bPressedLast = gamepad1.b;

        // Adjust F (D-Pad Left/Right)
        if (gamepad1.dpad_left) F -= stepSizes[stepIndex];
        if (gamepad1.dpad_right) F += stepSizes[stepIndex];

        // Adjust P (D-Pad Up/Down)
        if (gamepad1.dpad_up) P += stepSizes[stepIndex];
        if (gamepad1.dpad_down) P -= stepSizes[stepIndex];

        updatePIDF();
        shooter1.setVelocity(targetVelocity);

        double curVelocity = shooter1.getVelocity();
        double error = targetVelocity - curVelocity;

        // Telemetry for tuning
        telemetry.clearAll();
        telemetry.addData("🔧 Mode", "PIDF TUNING (press OPTIONS to exit)");
        telemetry.addLine();
        telemetry.addData("Target RPM", "%.0f", targetVelocity);
        telemetry.addData("Current RPM", "%.1f", curVelocity);
        telemetry.addData("Error", "%.1f", error);
        telemetry.addLine();
        telemetry.addData("P Gain", "%.4f", P);
        telemetry.addData("F Feedforward", "%.4f", F);
        telemetry.addData("Step Size", "%.4f", stepSizes[stepIndex]);
        telemetry.addLine();
        telemetry.addLine("Y: Toggle Speed  B: Next Step");
        telemetry.addLine("↑↓: P  ←→: F");
        telemetry.update();
    }

    private void updatePIDF() {
        if (shooter1 != null) {
            PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
            shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        }
    }

    public void onBallReleased(int ballsReleased) {
        ballCount -= ballsReleased;
        if (ballCount < 0) ballCount = 0;
        intakeFull = false;
        ballDetected = false;
    }

    private void servoDelay(long ms) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < ms) {
            // small busy wait
        }
    }
}