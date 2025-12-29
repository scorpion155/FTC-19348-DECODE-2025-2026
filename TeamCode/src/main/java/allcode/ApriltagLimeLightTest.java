package allcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled

@TeleOp(name = "ApriltagLimeLightTestWithAiming_ScaledPID", group = "Pedro Pathing")
public class ApriltagLimeLightTest extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;
    private DcMotor aimMotor;

    // PID constants
    private final double kI = 0.0;
    private double integral = 0;
    private double lastError = 0;
    private long lastTimeMs = 0;

    private boolean trackingEnabled = false;

    @Override
    public void runOpMode() {

        // ----- INIT -----
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9);

        imu = hardwareMap.get(IMU.class, "imu");
        aimMotor = hardwareMap.dcMotor.get("aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setPower(0);

        RevHubOrientationOnRobot hubOrientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );

        imu.initialize(new IMU.Parameters(hubOrientation));

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        limelight.start();

        lastTimeMs = System.currentTimeMillis();

        // ----- LOOP -----
        while (opModeIsActive()) {

            // toggle tracking with Y button
            if (gamepad1.y) {
                trackingEnabled = !trackingEnabled;
                sleep(200); // debounce
                integral = 0;
                lastError = 0;
            }

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());

            LLResult llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {

                Pose3D botpose = llResult.getBotpose_MT2();
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                double z = botpose.getPosition().z;

                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
                telemetry.addData("Botpose X", x);
                telemetry.addData("Botpose Y", y);
                telemetry.addData("Botpose Z", z);

                // ---- PID aiming ----
                if (trackingEnabled) {
                    double tx = llResult.getTx(); // horizontal offset
                    runAimPID(tx);
                    telemetry.addData("AimPower", aimMotor.getPower());
                } else {
                    aimMotor.setPower(0);
                }

            } else {
                telemetry.addLine("No valid Limelight result");
                aimMotor.setPower(0);
            }

            telemetry.addData("Tracking Enabled", trackingEnabled);
            telemetry.update();
        }

        aimMotor.setPower(0);
    }

    private void runAimPID(double tx) {
        long nowMs = System.currentTimeMillis();
        double dt = (nowMs - lastTimeMs) / 1000.0;
        if (dt <= 0) dt = 0.02;
        lastTimeMs = nowMs;

        double error = tx; // horizontal offset

        // ---- Deadzone to stop tiny oscillations ----
        double deadzone = 0.5;
        if (Math.abs(error) < deadzone) {
            aimMotor.setPower(0);
            return;
        }

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        // PID constants
        double kP = 0.035;  // slightly lower for smooth control
        double kD = 0.006;  // damping for vibration

        double output = (kP * error) + (kI * integral) + (kD * derivative);

        // ---- Scale output based on distance to target ----
        double maxError = 15.0;  // full power when error = 15 deg
        double scale = Math.min(Math.abs(error) / maxError, 1.0);
        output *= scale;

        // clamp power
        double maxPower = 1;
        if (output > maxPower) output = maxPower;
        if (output < -maxPower) output = -maxPower;

        aimMotor.setPower(-output); // invert if needed
    }
}
