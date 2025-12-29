package allcode;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled

@TeleOp(name="BallSorterMultiLift", group="Linear Opmode")
public class test1 extends LinearOpMode {

    // Hardware
    private RevColorSensorV3 colorSensor;
    private Servo sorterServo;
    private Servo lifterServo;
    private DcMotor intake;

    // Sorter slots
    private double[] positions = {0.00, 0.6, 0.8};
    private int currentSlot = 0;

    // Ball detection debounce
    private boolean ballDetected = false;

    // Lifter servo
    private double lifterUpPos = 0.23;   // 70°
    private double lifterDownPos = 0.00;
    private boolean lifterButtonPressed = false;

    // Optional: launch position for rear lifter (if needed)
    private double launchSorterPos = 0.20;

    @Override
    public void runOpMode() {

        // Map hardware
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");
        sorterServo = hardwareMap.get(Servo.class, "sortservo");
        lifterServo = hardwareMap.get(Servo.class, "pushservo");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Initial positions
        sorterServo.setPosition(positions[currentSlot]);
        lifterServo.setPosition(lifterDownPos);

        telemetry.addLine("Multi-Lift BallSorter Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ======================================
            // 1) COLOR SENSOR BALL DETECTION (optional)
            // ======================================
            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();
            int brightness = r + g + b;

            if (!ballDetected && brightness > 350) {
                ballDetected = true;
                // sorter moves automatically on lift, so optional
            }

            if (ballDetected && brightness < 200) {
                ballDetected = false;
            }

            // ======================================
            // 2) LIFTER MULTI-LIFT MODE
            // ======================================
            if (gamepad1.square && !lifterButtonPressed) {
                lifterButtonPressed = true;

                // 1) Lift ball
                lifterServo.setPosition(lifterUpPos);
                sleep(250);

                // 2) Return down
                lifterServo.setPosition(lifterDownPos);
                sleep(150);

                // 3) Move sorter to next slot
                currentSlot = (currentSlot + 1) % positions.length;
                sorterServo.setPosition(positions[currentSlot]);
                sleep(150); // allow servo to settle
            }

            // Reset debounce
            if (!gamepad1.square) {
                lifterButtonPressed = false;
            }
            if (gamepad1.right_bumper) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }
            // ======================================
            // 3) Telemetry
            // ======================================
            telemetry.addData("R", r);
            telemetry.addData("G", g);
            telemetry.addData("B", b);
            telemetry.addData("Brightness", brightness);
            telemetry.addData("Current Slot", currentSlot);
            telemetry.addData("Lifter Up", lifterServo.getPosition() == lifterUpPos);
            telemetry.update();
        }
    }
}