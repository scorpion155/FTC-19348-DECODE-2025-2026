package allcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled

@TeleOp(name="Stage2_SorterAndPusher", group="Stage2")
public class stagetwo extends LinearOpMode {

    private Servo sorterServo;
    private Servo pushServo;

    // Intake slots (empty spaces)
    private final double[] INTAKE_POS = {0.00, 0.30, 0.45};

    // Launch position (opposite side)
    private final double[] LAUNCH_POS = {0.21, 0.53, 0.85};

    private int currentSlot = 0;

    // goBILDA 2000-0025-0504 servo push angles
    private final double PUSH_UP_POS = 0.23;
    private final double PUSH_DOWN_POS = 0.00;

    // Debounce flags
    private boolean circlePressedPrev = false;
    private boolean rbPressedPrev = false;

    @Override
    public void runOpMode() throws InterruptedException {

        sorterServo = hardwareMap.get(Servo.class, "sortservo");
        pushServo   = hardwareMap.get(Servo.class, "pushservo");

        sorterServo.setPosition(INTAKE_POS[currentSlot]);
        pushServo.setPosition(PUSH_DOWN_POS);

        telemetry.addLine("Stage 2 Ready");
        telemetry.addLine("CIRCLE = next empty slot");
        telemetry.addLine("RIGHT BUMPER = push only");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ----------- CIRCLE → Move sorter to next intake slot -----------
            if (gamepad1.circle && !circlePressedPrev) {
                circlePressedPrev = true;

                // move to next slot
                currentSlot = (currentSlot + 1) % INTAKE_POS.length;

                sorterServo.setPosition(INTAKE_POS[currentSlot]);
                sleep(180);  // settle time
            }
            if (!gamepad1.circle) circlePressedPrev = false;

            // ----------- RIGHT BUMPER → Push only (no sorter movement) -----------
            if (gamepad1.right_bumper && !rbPressedPrev) {
                rbPressedPrev = true;

                pushServo.setPosition(PUSH_UP_POS);
                sleep(250);

                pushServo.setPosition(PUSH_DOWN_POS);
                sleep(130);
            }
            if (!gamepad1.right_bumper) rbPressedPrev = false;

            telemetry.addData("Current Slot", currentSlot);
            telemetry.addData("Sorter Pos", sorterServo.getPosition());
            telemetry.addData("Push Pos", pushServo.getPosition());
            telemetry.update();
        }
    }
}
