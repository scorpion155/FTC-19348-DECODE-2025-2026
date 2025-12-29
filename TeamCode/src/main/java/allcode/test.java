package allcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
@Disabled

@TeleOp(name = "Limelight AprilTag Tracker", group = "Sensor")
public class test extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor trackingMotor;

    private static final double TRACKING_KP = 0.03;
    private static final double TX_DEADBAND = 1.0;
    private static final double MAX_TRACKING_POWER = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        trackingMotor = hardwareMap.get(DcMotor.class, "trackingMotor");

        trackingMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        trackingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(50);
        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play to start tracking.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            LLStatus status = limelight.getStatus();

            telemetry.addData("Limelight FPS", (int)status.getFps());

            if (result.isValid() && !result.getFiducialResults().isEmpty()) {
                double tx = result.getTx();

                Pose3D botpose = result.getBotpose();

                double distanceToTag = botpose.getPosition().z;
                double strafeToTag = botpose.getPosition().x;

                telemetry.addData("AprilTag", "Visible");
                telemetry.addData("Distance (Z)", "%.2f in", distanceToTag);
                telemetry.addData("Strafe (X)", "%.2f in", strafeToTag);
                telemetry.addData("Angle (tx)", "%.2f deg", tx);

                double motorPower = 0.0;
                if (Math.abs(tx) > TX_DEADBAND) {
                    motorPower = -tx * TRACKING_KP;
                }

                motorPower = Math.max(-MAX_TRACKING_POWER, Math.min(motorPower, MAX_TRACKING_POWER));
                trackingMotor.setPower(motorPower);
                telemetry.addData("Motor Power", "%.2f", motorPower);

            } else {
                telemetry.addData("AprilTag", "Not Visible");
                trackingMotor.setPower(0.0);
            }

            telemetry.update();
        }
        trackingMotor.setPower(0.0);
        limelight.stop();
    }
}