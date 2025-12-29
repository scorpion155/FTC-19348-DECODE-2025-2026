package org.firstinspires.ftc.teamcode.codes;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
@Disabled

@TeleOp(name="redFull", group="Stage2")
public class redFull extends OpMode {

    // ---- Hardware ----
    private DcMotor intakeMotor;
    private DcMotor shooter1;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor aimMotor;
    private Limelight3A limelight;
    private IMU imu;
    private Servo sorterServo;
    private Servo pushservo;
    private RevColorSensorV3 colorSensor;

    // ---- Intake positions ----
    private final double[] INTAKE_POS = {0.00, 0.32, 0.49};
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
    private boolean ballDetected = false;

    // Button latches
    private boolean circleLatch = false;
    private boolean dpadRightLatch = false;

    // Pushing state
    private boolean isPushing = false;


    @Override
    public void init() {

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        sorterServo = hardwareMap.get(Servo.class, "sortservo");
        pushservo = hardwareMap.get(Servo.class, "pushservo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");

        // Ensure safe initial positions
        sorterServo.setPosition(INTAKE_POS[0]);
        pushservo.setPosition(PUSHSERVO_INACTIVE_POS);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        //aming with camera
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9);

        imu = hardwareMap.get(IMU.class, "imu");
        aimMotor = hardwareMap.dcMotor.get("aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setPower(0);

        RevHubOrientationOnRobot hubOrientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );

        imu.initialize(new IMU.Parameters(hubOrientation));

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
    }

    @Override
    public void loop() {

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

        if (gamepad1.triangle) {
            shooter1.setPower(-1.0);
        } else {
            shooter1.setPower(0.0);
        }

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

        // --------------------------------------------------
        // TELEMETRY (debugging helps diagnose auto-moves)
        // --------------------------------------------------
        telemetry.addData("Intake Slot", intakeIndex);
        telemetry.addData("Launch Slot", launchIndex);
        telemetry.addData("Sorter Position", sorterServo.getPosition());
        telemetry.addData("R", r);
        telemetry.addData("G", g);
        telemetry.addData("B", b);
        telemetry.addData("Brightness", brightness);
        telemetry.addData("gamepad.dpad_right", gamepad1.dpad_right);
        telemetry.addData("gamepad.circle", gamepad1.circle);
        telemetry.addData("isPushing", isPushing);
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
