package allcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled

@TeleOp(name="BallSorter", group="Linear Opmode")
public class testservo extends LinearOpMode {

    private DcMotor intake;
    private ColorSensor ColorSensor;
    private Servo sortservo;

    // 3 slots for 3 balls (0°, 120°, 240° servo positions)
    private double[] positions = {0.21, 0.53, 0.85};
    private int currentSlot = 0;

    @Override
    public void runOpMode() {

        intake = hardwareMap.get(DcMotor.class, "intake");
        ColorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        sortservo = hardwareMap.servo.get("sortservo");

        sortservo.setPosition(positions[currentSlot]);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            int r = ColorSensor.red();
            int g = ColorSensor.green();
            int b = ColorSensor.blue();

            int brightness = r + g + b;

            // a ball is detected when brightness becomes HIGH
            if (brightness > 350) {      // adjust if needed
                moveToNextSlot();
                sleep(600);              // wait for ball to pass
            }
            if (gamepad1.circle) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }

            telemetry.addData("R", r);
            telemetry.addData("G", g);
            telemetry.addData("B", b);
            telemetry.addData("Brightness", brightness);
            telemetry.addData("Slot", currentSlot);
            telemetry.update();
        }
    }

    private void moveToNextSlot() {
        currentSlot = (currentSlot + 1) % 3;
        sortservo.setPosition(positions[currentSlot]);
    }
}