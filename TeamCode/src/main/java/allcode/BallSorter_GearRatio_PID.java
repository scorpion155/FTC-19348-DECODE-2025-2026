package allcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
@Disabled

@TeleOp(name="BallSorter_GearRatio_PID", group="Linear Opmode")
public class BallSorter_GearRatio_PID extends LinearOpMode {

    private RevColorSensorV3 ColorSensor;
    private DcMotor intake;
    private Servo sortservo;

    // gear info (for your reference)
    // servo gear teeth = 100, fan gear teeth = 36
    // driver->driven ratio = N_servo / N_fan = 100/36 ≈ 2.777...
    // servo needed = fanAngle * (N_fan / N_servo) = fanAngle * 0.36

    // safety offset to avoid mechanical end-stops (adjustable)
    private final double SAFETY_OFFSET = 0.05;

    // computed servo positions (safe)
    private final double SLOT_A = 0.00;  // 0° fan -> servo ≈ 0.00 -> +offset
    private final double SLOT_B = 0.30;  // 120° fan -> servo ≈ 0.144 -> +offset
    private final double SLOT_C = 0.45;  // 240° fan -> servo ≈ 0.288 -> +offset

    private double[] positions = {SLOT_A, SLOT_B, SLOT_C};

    // PID constants (starting values — tune if necessary)
    private double Kp = 0.9;
    private double Ki = 0.0;
    private double Kd = 0.06;

    private double integral = 0.0;
    private double lastError = 0.0;

    // ball detection thresholds (tune to your lighting)
    private int BRIGHTNESS_HIGH = 350; // trigger threshold (when ball is present)
    private int BRIGHTNESS_LOW  = 200; // reset threshold (when ball leaves)

    private int currentSlot = 0;
    private boolean ballDetected = false;

    @Override
    public void runOpMode() {

        ColorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");
        sortservo  = hardwareMap.get(Servo.class, "sortservo");
        intake = hardwareMap.get(DcMotor.class, "intake");


        // start at safe slot A
        sortservo.setPosition(positions[currentSlot]);

        telemetry.addLine("Sorter ready. Using gear ratio -> servo positions set.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            int r = ColorSensor.red();
            int g = ColorSensor.green();
            int b = ColorSensor.blue();
            int brightness = r + g + b;

            // ---------- Ball detection (anti double trigger) ----------
            if (!ballDetected && brightness > BRIGHTNESS_HIGH) {
                ballDetected = true;
                moveToNextSlotPID();
            }

            if (ballDetected && brightness < BRIGHTNESS_LOW) {
                ballDetected = false;
            }

            // manual reset to slot A (circle button)
            if (gamepad1.circle) {
                currentSlot = 0;
                moveToSlotPID(currentSlot);
            }
            if (gamepad1.triangle) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }

            // telemetry
            telemetry.addData("R", r);
            telemetry.addData("G", g);
            telemetry.addData("B", b);
            telemetry.addData("Brightness", brightness);
            telemetry.addData("Slot", currentSlot);
            telemetry.update();
        }
    }

    // wrapper: increment slot and move
    private void moveToNextSlotPID() {
        currentSlot = (currentSlot + 1) % positions.length;
        moveToSlotPID(currentSlot);
    }

    // move to a slot using a fast approach then PID micro-adjust
    private void moveToSlotPID(int slotIndex) {
        double target = positions[slotIndex];

        // 1) fast coarse move to 90% of the distance (fast approach)
        double current = sortservo.getPosition();
        double fastPos = current + (target - current) * 0.9;
        clampAndSet(fastPos);
        sleep(150); // let the servo move fast

        // 2) PID micro-adjust phase for precise landing
        executePIDToTarget(target);
    }

    // clamp within safe range and set
    private void clampAndSet(double pos) {
        double min = 0.02;  // lower bound safe
        double max = 0.98;  // upper bound safe
        if (pos < min) pos = min;
        if (pos > max) pos = max;
        sortservo.setPosition(pos);
    }

    // PID loop that nudges servo until error is very small
    private void executePIDToTarget(double targetPos) {
        // reset integrator
        integral = 0.0;
        lastError = targetPos - sortservo.getPosition();

        // loop until precise or timeout
        long startTime = System.currentTimeMillis();
        long timeoutMs = 1200; // safety timeout

        while (opModeIsActive()) {
            double current = sortservo.getPosition();
            double error = targetPos - current;

            // break if within small threshold
            if (Math.abs(error) <= 0.003) break;

            // anti-windup: only integrate small errors
            integral += error * 0.02;

            double derivative = (error - lastError) / 0.02;

            double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

            double newPos = current + output;

            // clamp
            if (newPos < 0.02) newPos = 0.02;
            if (newPos > 0.98) newPos = 0.98;

            sortservo.setPosition(newPos);

            lastError = error;

            // small delay
            sleep(20);

            // timeout safety
            if (System.currentTimeMillis() - startTime > timeoutMs) break;
        }

        // final snap to target (safe clamp)
        clampAndSet(targetPos);
    }
}
