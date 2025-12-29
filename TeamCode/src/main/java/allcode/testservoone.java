package allcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled

@TeleOp(name="PS4 Servo Toggle", group="Examples")
public class testservoone extends LinearOpMode {

    private Servo intakeServo;



    private boolean servoState = false;
    private boolean lastPress = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Correct servo name
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        intakeServo.setPosition(0.0);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean press = gamepad1.dpad_right;

            if (press && !lastPress) {
                servoState = !servoState;
                intakeServo.setPosition(servoState ? 0.32 : 0.0);
            }

            lastPress = press;

            telemetry.addData("PS4 Left", gamepad1.dpad_right);
            telemetry.addData("Servo Pos", intakeServo.getPosition());
            telemetry.addData("State", servoState);
            telemetry.update();
        }
    }
}