package org.firstinspires.ftc.teamcode.codes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled

@Configurable
@TeleOp(name = "ApriltagLimeLight_PIDF_Adjustable", group = "Pedro Pathing")
public class pidfAim extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;
    private DcMotor aimMotor;

    // PIDF constants adjustable from dashboard
    public static double kP = 0.035;
    public static double kI = 0.0;
    public static double kD = 0.006;
    public static double kF = 0.0;          // Feedforward term

    public static double maxPower = 1.0;    // Maximum motor power
    public static double deadzone = 0.5;    // Deadzone to stop tiny oscillations

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

            // Toggle tracking with Y button
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

                // ---- PIDF aiming ----
                if (trackingEnabled) {
                    double tx = llResult.getTx(); // horizontal offset
                    runAimPIDF(tx);
                    telemetry.addData("AimPower", aimMotor.getPower());
                } else {
                    aimMotor.setPower(0);
                }

            } else {
                telemetry.addLine("No valid Limelight result");
                aimMotor.setPower(0);
            }

            telemetry.addData("Tracking Enabled", trackingEnabled);
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.addData("Deadzone", deadzone);
            telemetry.addData("MaxPower", maxPower);
            telemetry.update();
        }

        aimMotor.setPower(0);
    }

    private void runAimPIDF(double tx) {
        long nowMs = System.currentTimeMillis();
        double dt = (nowMs - lastTimeMs) / 1000.0;
        if (dt <= 0) dt = 0.02;
        lastTimeMs = nowMs;

        double error = tx; // horizontal offset

        // ---- Deadzone ----
        if (Math.abs(error) < deadzone) {
            aimMotor.setPower(0);
            integral = 0;
            lastError = 0;
            return;
        }

        // ---- PIDF calculations ----
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = (kP * error) + (kI * integral) + (kD * derivative) + (kF * Math.signum(error));

        // ---- Scale output based on distance to target ----
        double maxError = 15.0; // full power when error = 15 deg
        double scale = Math.min(Math.abs(error) / maxError, 1.0);
        output *= scale;

        // Clamp output
        if (output > maxPower) output = maxPower;
        if (output < -maxPower) output = -maxPower;

        aimMotor.setPower(-output); // invert if needed
    }
}
