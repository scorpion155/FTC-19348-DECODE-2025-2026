package allcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled

@TeleOp(name = "Sorter_NextCorner")
public class Sorter_NextCorner extends LinearOpMode {

    private Servo sortservo;


    private final double CORNER1 = 0.00;
    private final double CORNER2 = 0.30;
    private final double CORNER3 = 0.45;

    private int currentCorner = 0;
    private boolean bWasPressed = false;

    @Override
    public void runOpMode() {

        sortservo = hardwareMap.get(Servo.class, "sortservo");

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.circle && !bWasPressed) {
                currentCorner++;
                if (currentCorner > 2) {
                    currentCorner = 0;
                }
                if (currentCorner == 0) {
                    sortservo.setPosition(CORNER1);
                } else if (currentCorner == 1) {
                    sortservo.setPosition(CORNER2);
                } else if (currentCorner == 2) {
                    sortservo.setPosition(CORNER3);
                }
                bWasPressed = true;
            }
            if (!gamepad1.circle) {
                bWasPressed = false;
            }
            telemetry.addData("Current Corner", currentCorner + 1);
            telemetry.update();
        }
    }
}
