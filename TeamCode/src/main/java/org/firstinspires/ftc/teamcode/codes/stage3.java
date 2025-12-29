package org.firstinspires.ftc.teamcode.codes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled

@TeleOp(name="stage3", group="Stage2")
public class stage3 extends LinearOpMode {

    private DcMotor intakeMotor;
    private DcMotorEx shooter1;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo sorterServo;
    private Servo pushservo;
    private RevColorSensorV3 colorSensor;

    private DcMotor aimMotor;
    private Limelight3A limelight;
    private IMU imu;

    private final double[] INTAKE_POS = {0.00, 0.32, 0.49};
    private int intakeIndex = 0;
    private boolean xPressed = false;

    private final double[] LAUNCH_POS = {0.22, 0.40, 0.58};
    private int launchIndex = 0;

    private final double PUSHSERVO_ACTIVE_POS = 70.0 / 360.0;
    private final double PUSHSERVO_INACTIVE_POS = 0.0;

    private final double INTAKE_POWER = 1.0;

    private boolean ballDetected = false;
    private boolean circleLatch = false;
    private boolean dpadRightLatch = false;
    private boolean isPushing = false;

    private boolean trackingEnabled = false;
    private boolean isShooting = false;
    private long shootStartTime = 0;

    private final double kP_aim = 0.035;
    private final double kI_aim = 0.003;
    private final double kD_aim = 0.006;
    private final double INTEGRAL_LIMIT = 50.0;

    private double integral = 0;
    private double lastError = 0;
    private long lastTimeMs = 0;

    private final double GOAL_HEIGHT_M = 0.9845;
    private final double FLYWHEEL_RADIUS_M = 0.036;
    private final double G = 9.8;
    private double LAUNCH_ANGLE_DEG = 25.0;
    private final double DISTANCE_SCALE = 30666;
    private final double SHOOT_SPINUP_TIME = 1.0;

    private double[] taBuffer = new double[5];
    private int bufferIndex = 0;
    private double targetRPM = 0;

    private boolean prevY = false; // for tracking toggle

    @Override
    public void runOpMode() throws InterruptedException {

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        shooter1   = hardwareMap.get(DcMotorEx.class, "shooter1");
        sorterServo = hardwareMap.get(Servo.class, "sortservo");
        pushservo   = hardwareMap.get(Servo.class, "pushservo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");

        aimMotor   = hardwareMap.get(DcMotor.class, "aimMotor");
        limelight  = hardwareMap.get(Limelight3A.class, "limelight");
        imu        = hardwareMap.get(IMU.class, "imu");

        sorterServo.setPosition(INTAKE_POS[0]);
        pushservo.setPosition(PUSHSERVO_INACTIVE_POS);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setVelocity(0);
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setPower(0);

        IMU.Parameters imuParams = new IMU.Parameters(
                new com.qualcomm.hardware.rev.RevHubOrientationOnRobot(
                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParams);

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(9);
        limelight.start();

        telemetry.addLine("Stage 3 + Limelight Aimbot Ready.");
        telemetry.update();

        waitForStart();
        lastTimeMs = System.currentTimeMillis();

        while (opModeIsActive()) {

            // --- Mecanum drive ---
            double y  = gamepad1.left_stick_y;
            double x  = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower  = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // --- Tracking toggle ---
            if (gamepad1.y && !prevY) {
                trackingEnabled = !trackingEnabled;
                integral = 0;
                lastError = 0;
            }
            prevY = gamepad1.y;

            // --- Shooting control ---
            if (gamepad1.dpad_up) {
                if (!trackingEnabled) {
                    trackingEnabled = true;
                    isShooting = true;
                    shootStartTime = System.currentTimeMillis();
                }
            }
            if (gamepad1.dpad_down) {
                trackingEnabled = false;
                isShooting = false;
                shooter1.setVelocity(0);
                aimMotor.setPower(0);
            }

            if (!trackingEnabled) {
                if (gamepad1.triangle) {
                    shooter1.setPower(-1.0);
                } else if (!isShooting) {
                    shooter1.setPower(0.0);
                }
            }

            // --- Ball detection and sorter control ---
            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();
            int brightness = r + g + b;

            if (brightness > 300) {
                if (!ballDetected) {
                    intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
                    sorterServo.setPosition(INTAKE_POS[intakeIndex]);
                    ballDetected = true;
                }
            } else {
                ballDetected = false;
            }

            if (gamepad1.right_bumper && !xPressed) {
                xPressed = true;
                pushservo.setPosition(PUSHSERVO_ACTIVE_POS);
                servoDelay(250);
                pushservo.setPosition(PUSHSERVO_INACTIVE_POS);
            } else if (!gamepad1.right_bumper) {
                xPressed = false;
            }

            if (gamepad1.left_bumper && !circleLatch) {
                circleLatch = true;
                launchIndex = (launchIndex + 1) % LAUNCH_POS.length;
                sorterServo.setPosition(LAUNCH_POS[launchIndex]);
            }
            if (!gamepad1.left_bumper) circleLatch = false;

            if (gamepad1.dpad_right && !dpadRightLatch && !isPushing) {
                dpadRightLatch = true;
                isPushing = true;
                intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
                sorterServo.setPosition(INTAKE_POS[intakeIndex]);
                isPushing = false;
            }
            if (!gamepad1.dpad_right) dpadRightLatch = false;

            // --- Intake ---
            if (gamepad1.right_trigger > 0.2) {
                intakeMotor.setPower(INTAKE_POWER);
            } else {
                intakeMotor.setPower(0);
            }

            // --- Tracking / PID aiming ---
            if (trackingEnabled) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                limelight.updateRobotOrientation(orientation.getYaw());

                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    double tx = result.getTx();
                    double ta = result.getTa();

                    // Distance smoothing
                    taBuffer[bufferIndex] = ta;
                    bufferIndex = (bufferIndex + 1) % 5;
                    double avgTa = 0;
                    for (int i = 0; i < 5; i++) avgTa += taBuffer[i];
                    avgTa /= 5.0;

                    double distCm = DISTANCE_SCALE * Math.sqrt(1.0 / avgTa);
                    double distM = distCm / 100.0;

                    // Physics-based RPM calculation
                    double alphaRad = Math.toRadians(LAUNCH_ANGLE_DEG);
                    double cosAlpha = Math.cos(alphaRad);
                    double tanAlpha = Math.tan(alphaRad);

                    double numerator = G * distM * distM;
                    double denominatorPhysics = 2 * cosAlpha * cosAlpha * (distM * tanAlpha - GOAL_HEIGHT_M);
                    double v0 = (denominatorPhysics > 0) ? Math.sqrt(numerator / denominatorPhysics) : 0;

                    targetRPM = (v0 * 60.0) / (2 * Math.PI * FLYWHEEL_RADIUS_M);
                    targetRPM = Math.max(500, Math.min(targetRPM, 6000));

                    // Spin-up
                    if (isShooting) {
                        long shootElapsed = System.currentTimeMillis() - shootStartTime;
                        if (shootElapsed < SHOOT_SPINUP_TIME * 1000) {
                            double spinupProgress = shootElapsed / (SHOOT_SPINUP_TIME * 1000.0);
                            shooter1.setVelocity(-targetRPM * spinupProgress);
                        } else {
                            shooter1.setVelocity(-targetRPM);
                        }
                    } else {
                        shooter1.setVelocity(0);
                    }

                    // --- PID aim ---
                    runAimPID(tx);

                    telemetry.addData("Target X", tx);
                    telemetry.addData("Target Area", ta);
                    telemetry.addData("Distance (cm)", distCm);
                    telemetry.addData("RPM", targetRPM);

                } else {
                    telemetry.addData("Limelight", "No target");
                    shooter1.setVelocity(0);
                    aimMotor.setPower(0);
                }
            } else {
                aimMotor.setPower(0);
            }

            telemetry.addData("Intake Slot", intakeIndex);
            telemetry.addData("Launch Slot", launchIndex);
            telemetry.addData("Sorter Position", sorterServo.getPosition());
            telemetry.addData("R", r);
            telemetry.addData("G", g);
            telemetry.addData("B", b);
            telemetry.addData("Brightness", brightness);
            telemetry.addData("trackingEnabled", trackingEnabled);
            telemetry.addData("isShooting", isShooting);
            telemetry.update();
        }

        limelight.stop();
    }

    private void servoDelay(long ms) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < ms) {
            idle();
        }
    }

    private void runAimPID(double error) {
        long nowMs = System.currentTimeMillis();
        double dt = (nowMs - lastTimeMs) / 1000.0;
        if (dt <= 0) dt = 0.02;
        lastTimeMs = nowMs;

        // Deadzone to stop tiny oscillations
        double deadzone = 0.5;
        if (Math.abs(error) < deadzone) {
            aimMotor.setPower(0);
            integral = 0;
            lastError = 0;
            return;
        }

        // PID calculation
        integral += error * dt;
        if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
        if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;

        double derivative = (error - lastError) / dt;

        double output = kP_aim * error + kI_aim * integral + kD_aim * derivative;

        // Scale output for smooth motion
        double maxError = 15.0;
        double scale = Math.min(Math.abs(error) / maxError, 1.0);
        output *= scale;

        // Clamp power
        if (output > 1) output = 1;
        if (output < -1) output = -1;

        aimMotor.setPower(-output); // invert if motor direction is opposite

        lastError = error;
    }
}
