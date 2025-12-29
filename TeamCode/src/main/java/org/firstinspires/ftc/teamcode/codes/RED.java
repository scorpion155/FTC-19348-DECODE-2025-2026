package org.firstinspires.ftc.teamcode.codes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled

@TeleOp(name = "RED")
public class RED extends OpMode {

    // Mecanum motors
    private Limelight3A limelight;
    private DcMotor intakeMotor;
    private DcMotor shooter1;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor aimMotor;
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

    // ---- Push servo ----
    private final double PUSHSERVO_ACTIVE_POS = 70.0 / 360.0;
    private final double PUSHSERVO_INACTIVE_POS = 0.0;
    private long pushStartTime = 0;
    private boolean isPushing = false;

    // ---- Intake ----
    private final double INTAKE_POWER = 1.0;

    private boolean ballDetected = false;
    private boolean circleLatch = false;
    private boolean dpadRightLatch = false;

    // Turret constants
    private static final double DEADZONE = 2.0;
    private static final double MAX_TURRET_POWER = 0.40;
    private static final double FILTER_ALPHA = 0.85;
    private static final double KP = 0.03;
    private double filteredTx = 0;

    // Limelight distance parameters
    private static final double CAMERA_HEIGHT = 0.3;   // meters
    private static final double TARGET_HEIGHT = 1.10;   // meters
    private static final double CAMERA_ANGLE  = 15;   // degrees

    // Servo mapping
    private static final double SERVO_MIN = 0.3;
    private static final double SERVO_MAX = 0.61;
    private static final double DIST_MIN  = 0.4;
    private static final double DIST_MAX  = 3.0;

    // ***********************
    // Shooter auto-RPM control
    // ***********************
    private static final double SHOOTER_PWR_MIN = 0.65;   // close shot
    private static final double SHOOTER_PWR_MAX = 1.00;   // far shot
    private double shooterPowerTarget = 0;

    @Override
    public void init() {

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        sorterServo = hardwareMap.get(Servo.class, "sortservo");
        pushservo = hardwareMap.get(Servo.class, "pushservo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");

        // Reverse right side
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        sorterServo.setPosition(INTAKE_POS[0]);
        pushservo.setPosition(PUSHSERVO_INACTIVE_POS);

        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setDirection(DcMotor.Direction.REVERSE);

        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        shooterServo.setPosition(SERVO_MIN);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }

    @Override
    public void loop() {

        // ------------------ Mecanum Drive ------------------
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeft.setPower((y + x + rx) / denominator);
        backLeft.setPower((y - x + rx) / denominator);
        frontRight.setPower((y - x - rx) / denominator);
        backRight.setPower((y + x - rx) / denominator);

        // --------------------------------------------------
        // 🔵 INTAKE AUTO USING COLOR SENSOR (UNCHANGED)
        // --------------------------------------------------
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        int brightness = r + g + b;



        if (brightness > 300) {   // ball detected threshold (tune if needed)
            if (!ballDetected) { // trigger only once per ball
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
        if (gamepad1.dpad_right && !dpadRightLatch && !isPushing) {
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
        if (!gamepad1.dpad_right) {
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

        // ------------------ Limelight Tracking + Distance ------------------
        double turretPower = 0;
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double tx = result.getTx();
            double ty = result.getTy();

            filteredTx = FILTER_ALPHA * filteredTx + (1 - FILTER_ALPHA) * tx;

            if (Math.abs(filteredTx) > DEADZONE)
                turretPower = filteredTx * KP;

            // ---- Distance Calculation ----
            double angleRad = Math.toRadians(CAMERA_ANGLE + ty);
            double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleRad);

            // ---- Auto Servo Angle ----
            double servoPos = SERVO_MIN +
                    (distance - DIST_MIN) / (DIST_MAX - DIST_MIN) * (SERVO_MAX - SERVO_MIN);

            servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
            shooterServo.setPosition(servoPos);

            // ---- Auto Shooter RPM based on distance ----
            double normalized = (distance - DIST_MIN) / (DIST_MAX - DIST_MIN);
            normalized = Math.max(0, Math.min(1, normalized));

            shooterPowerTarget = SHOOTER_PWR_MIN +
                    normalized * (SHOOTER_PWR_MAX - SHOOTER_PWR_MIN);
        }

        // ---- Manual Shooter Activation ----
        if (gamepad1.triangle) {
            shooter1.setPower(-shooterPowerTarget);
        } else {
            shooter1.setPower(-0);
        }

        // ---- Apply turret motor ----
        turretPower = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, turretPower));
        aimMotor.setPower(turretPower);

        // ------------------ Telemetry ------------------
        telemetry.addData("Shooter Target Power", shooterPowerTarget);
        telemetry.addData("Shooter Active", gamepad1.triangle);
        telemetry.addData("Servo Angle", shooterServo.getPosition());
        telemetry.addData("Filtered tx", filteredTx);
        telemetry.addData("Turret Power", turretPower);
        telemetry.update();
    }
    // ---- Safe busy-wait delay (short) ----
    private void servoDelay(long ms) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < ms) {
            // small busy wait; keep short to avoid watchdog issues
        }
    }
}
