package allcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
@Disabled

@TeleOp(name="BallSorter_FullSystem")
public class stageTest extends OpMode {

    // ---- Hardware ----
    private DcMotor intakeMotor;
    private Servo sorterServo;
    private Servo lifterServo;
    private RevColorSensorV3 colorSensor;

    // ---- Intake positions ----
    private final double[] INTAKE_POS = {0.00, 0.31, 0.48};
    private int intakeIndex = 0;

    // ---- Launch positions ----
    private final double[] LAUNCH_POS = {0.22, 0.39, 0.57};
    private int launchIndex = 0;

    // ---- Push servo ----
    private final double PUSH_UP = 0.70;
    private final double PUSH_DOWN = 0.10;

    // ---- Intake ----
    private final double INTAKE_POWER = 1.0;

    // Anti-spam for sensor
    private boolean ballDetected = false;

    @Override
    public void init() {

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        sorterServo = hardwareMap.get(Servo.class, "sortservo");
        lifterServo = hardwareMap.get(Servo.class, "pushservo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");

        sorterServo.setPosition(INTAKE_POS[0]);
        lifterServo.setPosition(PUSH_DOWN);

        telemetry.addLine("System Ready");
    }

    @Override
    public void loop() {

        // --------------------------------------------------
        // 🔵 INTAKE AUTO USING COLOR SENSOR
        // --------------------------------------------------
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        int brightness = r + g + b;

        if (brightness > 300) {   // ball detected
            if (!ballDetected) { // trigger only once
                intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
                sorterServo.setPosition(INTAKE_POS[intakeIndex]);
                ballDetected = true;
            }
        } else {
            ballDetected = false;
        }

        // --------------------------------------------------
        // 🔴 LAUNCH STAGE (Manual)
        // --------------------------------------------------

        // CIRCLE → Sorter next launch slot
        if (gamepad1.circle) {
            launchIndex = (launchIndex + 1) % LAUNCH_POS.length;
            sorterServo.setPosition(LAUNCH_POS[launchIndex]);
        }

        // DPAD RIGHT → Push servo (UP → DOWN)
        if (gamepad1.dpad_right) {
            lifterServo.setPosition(PUSH_UP);
            servoDelay(250);
            lifterServo.setPosition(PUSH_DOWN);
            servoDelay(250);
        }

        // --------------------------------------------------
        // INTAKE MOTOR
        // --------------------------------------------------
        if (gamepad1.right_trigger > 0.2) {
            intakeMotor.setPower(INTAKE_POWER);
        }
        else if (gamepad1.left_trigger > 0.2) {
            intakeMotor.setPower(0);
        }

        // --------------------------------------------------
        // TELEMETRY
        // --------------------------------------------------
        telemetry.addData("Intake Slot", intakeIndex);
        telemetry.addData("Launch Slot", launchIndex);
        telemetry.addData("Sorter Position", sorterServo.getPosition());
        telemetry.addData("Push Servo", lifterServo.getPosition());
        telemetry.addData("R", r);
        telemetry.addData("G", g);
        telemetry.addData("B", b);
        telemetry.addData("Brightness", brightness);
        telemetry.update();
    }

    // ---- Safe delay for OpMode ----
    private void servoDelay(long ms) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < ms) {
            // non-blocking, legal in OpMode
        }
    }
}
