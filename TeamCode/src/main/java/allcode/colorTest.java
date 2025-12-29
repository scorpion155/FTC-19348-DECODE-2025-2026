package allcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled

@TeleOp(name = "Servotest", group = "LinearOpMode")
public class colorTest extends LinearOpMode {

    private Servo sortservo;
    private ColorSensor ColorSensor;

    // Adjust these servo positions to your mechanismc
    private final double GREEN_SLOT   = 0.00;
    private final double PURPLE1_SLOT = 0.30;
    private final double PURPLE2_SLOT = 0.45;

    // Used to alternate the two purple spaces
    private boolean usePurpleSlot1 = true;

    @Override
    public void runOpMode() {

        sortservo = hardwareMap.get(Servo.class, "sortservo");
        ColorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor ");

        waitForStart();

        while (opModeIsActive()) {

            // Read RGB values
            int red = ColorSensor.red();
            int green = ColorSensor.green();
            int blue = ColorSensor.blue();

            // Color detection
            boolean isGreen  = (green > red && green > blue && green > 80);

            // ✔ Corrected purple detection based on #9107FF
            boolean isPurple = (blue > 200 && green > 120 && red > 80);

            if (isGreen) {
                sortservo.setPosition(GREEN_SLOT);
            }

            else if (isPurple) {
                if (usePurpleSlot1) {
                    sortservo.setPosition(PURPLE1_SLOT);
                } else {
                    sortservo.setPosition(PURPLE2_SLOT);
                }
                usePurpleSlot1 = !usePurpleSlot1;
            }

            // Telemetry to help you test
            telemetry.addData("R", red);
            telemetry.addData("G", green);
            telemetry.addData("B", blue);
            telemetry.addData("PurpleSlot1?", usePurpleSlot1);
            telemetry.update();

            sleep(150); // small delay so we do not detect the same ball multiple times
        }
    }
}