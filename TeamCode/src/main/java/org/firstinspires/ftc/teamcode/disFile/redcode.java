package org.firstinspires.ftc.teamcode.disFile;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled

@TeleOp(name = "🔴 RED-Tele ")
public class redcode extends LinearOpMode {

    // Mecanum motors
    private Limelight3A limelight;
    private DcMotor intakeMotor;
    private DcMotor shooter1;
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

    // ---- Intake positions ----
    private final double[] INTAKE_POS = {-0.00, 0.31, 0.488};
    private int intakeIndex = 0;
    private boolean xPressed = false;

    // ---- Launch positions ----
    private final double[] LAUNCH_POS = {0.22, 0.40, 0.58};
    private int launchIndex = 0;

    // ---- Push servo ----  (tuned for your goBILDA 2000-0025-0504)
    private final double PUSHSERVO_ACTIVE_POS = 70.0 / 360.0;
    private final double PUSHSERVO_INACTIVE_POS = 0.0;

    // ---- Intake ----
    private final double INTAKE_POWER = 1.0;

    // Anti-spam for sensor
//    private boolean ballDetected = false;

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

    // Limelight distance parameters (adjust for your setup)
    private static final double CAMERA_HEIGHT = 0.25;   // meters
    private static final double TARGET_HEIGHT = 0.90;   // meters
    private static final double CAMERA_ANGLE  = 20.0;   // degrees

    // Servo mapping
    private static final double SERVO_MIN = 0.3;  // minimum angle
    private static final double SERVO_MAX = 0.55;  // maximum angle
    private static final double DIST_MIN  = 0.4;  // meters (close)
    private static final double DIST_MAX  = 3.0;  // meters (far)

    private static final double shooter_MIN = 0.7;  // minimum angle
    private static final double shooter_MAX = 1;  // maximum angle
    private static final double DIST_MIN_sh  = 0.4;  // meters (close)
    private static final double DIST_MAX_sh  = 2.50;  // meters (far)
    boolean shooterOn = false;    // Shooter running state
    boolean lastbuttom = false; // Last frame state of the button


    static final int MAX_BALLS = 3;

    int ballCount = 0;

    boolean ballDetected = false;
    boolean intakeFull = false;


    @Override
    public void runOpMode() {

        // --- Drive Motors ---
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        sorterServo = hardwareMap.get(Servo.class, "sortservo");
        pushservo = hardwareMap.get(Servo.class, "pushservo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        shooter1.setDirection(DcMotor.Direction.REVERSE);


        sorterServo.setPosition(INTAKE_POS[0]);
        pushservo.setPosition(PUSHSERVO_INACTIVE_POS);

        // --- Turret Motor ---
        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Shooter Servo ---
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        shooterServo.setPosition(SERVO_MIN); // default start

        // --- Limelight ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);

        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
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

//            if (gamepad1.triangle) {
//                shooter1.setPower(-1.0);
//            } else {
//                shooter1.setPower(0.0);
//            }

            // --------------------------------------------------
            // 🔵 INTAKE AUTO USING COLOR SENSOR (UNCHANGED)
            // --------------------------------------------------
            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();
            int brightness = r + g + b;

            boolean ballPresent = brightness > 320;
            boolean ballGone    = brightness < 280;

            /* --- SENSOR CLEAR → RE-ARM --- */
            if (ballGone) {
                ballDetected = false;
            }

            /* --- BALL ENTERING --- */
            if (ballPresent && !ballDetected && !intakeFull) {

                ballDetected = true;
                ballCount++;

                if (ballCount >= MAX_BALLS) {
                    ballCount = MAX_BALLS;
                    intakeFull = true;      // LOCK rotation
                } else {
                    intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
                    sorterServo.setPosition(INTAKE_POS[intakeIndex]);
                }
            }
            if (gamepad1.left_stick_button) {   // shooter fires
                onBallReleased(3);
            }

//            if (!intakeFull) {
//
//                if (brightness > 320 && !ballDetected) {   // ball just arrived
//                    ballDetected = true;
//
//                    ballCount++;
//
//                    if (ballCount >= MAX_BALLS) {
//                        intakeFull = true;     // LOCK SYSTEM
//                        ballCount = MAX_BALLS;
//                    } else {
//                        intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
//                        sorterServo.setPosition(INTAKE_POS[intakeIndex]);
//                    }
//                }
//
//                if (brightness < 280) {        // ball left sensor
//                    ballDetected = false;
//                }
//            }

//            if (brightness > 300) {   // ball detected threshold (tune if needed)
//                if (!ballDetected) { // trigger only once per ball
//                    intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
//                    sorterServo.setPosition(INTAKE_POS[intakeIndex]);
//                    ballDetected = true;
//                }
//            } else {
//                ballDetected = false;
//            }

//            if (brightness > 300) {   // ball detected threshold (tune if needed)
//                if (!ballDetected) { // trigger only once per ball
//                    intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
//                    sorterServo.setPosition(INTAKE_POS[intakeIndex]);
//                    ballDetected = true;
//                }
//            }  else if (brightness > 500) {
//                ballDetected = false;
//            } else {
//                ballDetected = false;
//            }
            if (gamepad1.right_bumper && !xPressed) {
                xPressed = true;
                pushservo.setPosition(PUSHSERVO_ACTIVE_POS);
                servoDelay(250);
                pushservo.setPosition(PUSHSERVO_INACTIVE_POS);
            } else if (!gamepad1.right_bumper) {
                xPressed = false;
            }

            // --------------------------------------------------
            // 🔴 LAUNCH STAGE (Manual) - CIRCLE (latch) fixed
            // --------------------------------------------------
            if (gamepad1.left_bumper && !circleLatch) {
                circleLatch = true;

                // Move sorter to next launch slot (one press -> one step)
                launchIndex = (launchIndex + 1) % LAUNCH_POS.length;
                sorterServo.setPosition(LAUNCH_POS[launchIndex]);
            }
            if (!gamepad1.left_bumper) {
                circleLatch = false;
            }

            // --------------------------------------------------
            // DPAD RIGHT → Push servo (UP → DOWN) with latch and protection
            // --------------------------------------------------
            if (gamepad1.left_stick_button && !dpadRightLatch && !isPushing) {
                dpadRightLatch = true;
                isPushing = true;

                // Perform push action (blocking short durations)
                // allow return

                // After pushing, optionally advance intakeIndex so next ball will be in place.
                // If you do not want the sorter to advance automatically after push, remove next two lines.
                intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
                sorterServo.setPosition(INTAKE_POS[intakeIndex]);

                isPushing = false;
            }
            if (!gamepad1.left_stick_button) {
                dpadRightLatch = false;
            }

            // --------------------------------------------------
            // INTAKE MOTOR
            // --------------------------------------------------
            if (gamepad1.right_trigger > 0.2) {
                intakeMotor.setPower(INTAKE_POWER);
            } else {
                intakeMotor.setPower(0);
            }


            // ------------------ Turret Control (Limelight) ------------------
            double turretPower = 0;
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();

                // Low-pass filter for smoothness
                filteredTx = FILTER_ALPHA * filteredTx + (1 - FILTER_ALPHA) * tx;

                if (Math.abs(filteredTx) > DEADZONE) {
                    turretPower = filteredTx * KP;
                }

                // --- Calculate distance ---
                double angleRad = Math.toRadians(CAMERA_ANGLE + ty);
                double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleRad);

                // --- Map distance to servo ---
                double servoPos = SERVO_MIN + (distance - DIST_MIN) / (DIST_MAX - DIST_MIN) * (SERVO_MAX - SERVO_MIN);
                servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
                shooterServo.setPosition(servoPos);

                double shooterPower = shooter_MIN + (distance - DIST_MIN_sh) / (DIST_MAX_sh - DIST_MIN_sh) * (shooter_MAX - shooter_MIN);
                shooterPower = Math.max(shooter_MIN, Math.min(shooter_MAX, shooterPower));
                if (gamepad1.right_stick_button && !lastbuttom) {
                    shooterOn = !shooterOn; // Toggle state
                }
                lastbuttom = gamepad1.right_stick_button;

// 3. Set motor power based on toggle state
                if (shooterOn) {
                    shooter1.setPower(shooterPower);
                } else {
                    shooter1.setPower(0);
                }



            }

            // Clamp power
            turretPower = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, turretPower));
            aimMotor.setPower(turretPower);

            // ------------------ Telemetry ------------------
            telemetry.addData("tx", filteredTx);
            telemetry.addData("Turret Power", turretPower);
            telemetry.addData("Servo Position", shooterServo.getPosition());
            telemetry.addData("shooter power", shooter1.getPower());
            telemetry.addData("Distance (m)", result != null && result.isValid() ?
                    String.format("%.2f", (TARGET_HEIGHT - CAMERA_HEIGHT)/Math.tan(Math.toRadians(CAMERA_ANGLE + result.getTy()))) : "N/A");
            telemetry.update();
        }



    }

    public void onBallReleased(int ballsReleased) {

        ballCount -= ballsReleased;
        if (ballCount < 0) ballCount = 0;

        intakeFull = false;
        ballDetected = false;   // ⭐ THIS IS THE FIX ⭐
    }
    // ---- Safe busy-wait delay (short) ----
    private void servoDelay(long ms) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < ms) {
            // small busy wait; keep short to avoid watchdog issues
        }
    }
}
