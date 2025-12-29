package allcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled

@TeleOp(name="DecodeCode", group="Examples")

public class DecodeCode extends LinearOpMode {

    private DcMotor intake;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo intakeServo;
    private boolean servoState = false;
    private boolean lastPress = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Correct servo name
        intake = hardwareMap.get(DcMotor.class, "intake");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        intakeServo.setPosition(0.0);

        waitForStart();

        while (opModeIsActive()) {
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


            if (gamepad1.right_bumper) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }

            boolean press = gamepad1.dpad_down;
            if (press && !lastPress) {
                servoState = !servoState;
                intakeServo.setPosition(servoState ? 0.37 : 0.0);
            }
            lastPress = press;
        }
    }
}
