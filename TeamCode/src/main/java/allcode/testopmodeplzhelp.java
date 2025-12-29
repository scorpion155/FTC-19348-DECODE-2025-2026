package allcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled

@TeleOp(name = "Testopmodeplzhelp", group = "LinearOpMode")
public class testopmodeplzhelp extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo sortservo;
    private Servo pushservo;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor intake;

    private int servoPositionState = 0;
    private final double MAX_SERVO_POSITION = 1.0;

    private final double PUSHSERVO_ACTIVE_POS = 70.0 / 360.0;
    private final double PUSHSERVO_INACTIVE_POS = 0.0;

    private boolean lbPressed = false;
    private boolean rbPressed = false;
    private boolean xPressed = false;
    private final double CORNER1 = 0.00;
    private final double CORNER2 = 0.30;
    private final double CORNER3 = 0.45;

    private int currentCorner = 0;
    private boolean bWasPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.get(DcMotor.class, "intake");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        sortservo = hardwareMap.get(Servo.class, "sortservo");
        pushservo = hardwareMap.get(Servo.class, "pushservo");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        sortservo = hardwareMap.get(Servo.class, "sortservo");


        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        shooter1.setDirection(DcMotor.Direction.REVERSE); // Reverse the left shooter motor

        sortservo.setPosition(0.0);
        pushservo.setPosition(PUSHSERVO_INACTIVE_POS);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

            if (gamepad1.right_bumper && !bWasPressed) {
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
            if (!gamepad1.right_bumper) {
                bWasPressed = false;
            }

            if (gamepad1.x && !xPressed) {
                xPressed = true;
                pushservo.setPosition(PUSHSERVO_ACTIVE_POS);
                sleep(250);
                pushservo.setPosition(PUSHSERVO_INACTIVE_POS);
            } else if (!gamepad1.x) {
                xPressed = false;
            }

            if (gamepad1.triangle) {
                shooter1.setPower(1.0);
            } else {
                shooter1.setPower(0.0);
            }

            if (gamepad1.circle) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }

            telemetry.addData("Shooter 1 Power", shooter1.getPower());
            telemetry.addData("Shooter 2 Power", shooter2.getPower());
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Current Corner", currentCorner + 1);

            telemetry.update();
        }
    }
}