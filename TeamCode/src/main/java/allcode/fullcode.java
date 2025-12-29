package allcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
@Disabled

/**
 * Stage 3: Limelight A3 PID aiming for shooter.
 */
@TeleOp(name="Stage3_LimelightAiming", group="Pro")
public class fullcode extends LinearOpMode {

    private DcMotor aimMotor;
    private LimelightI2C limelight;

    // PID constants
    private double kP = 0.025;
    private double kI = 0.0;
    private double kD = 0.004;
    private double integral = 0;
    private double lastError = 0;
    private long lastTimeMs = 0;

    private boolean trackingEnabled = false;

    // Hardware names
    private final String AIM_MOTOR_NAME = "aimMotor";
    private final String LIMELIGHT_I2C_NAME = "limelight";

    @Override
    public void runOpMode() {

        // --- Hardware ---
        aimMotor = hardwareMap.get(DcMotor.class, AIM_MOTOR_NAME);
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Limelight I2C helper
        limelight = new LimelightI2C(hardwareMap, LIMELIGHT_I2C_NAME);

        telemetry.addLine("Stage 3 ready. Press PLAY");
        telemetry.update();
        waitForStart();
        lastTimeMs = System.currentTimeMillis();

        while(opModeIsActive()) {

            // Toggle tracking with Y button
            if (gamepad1.y) {
                trackingEnabled = !trackingEnabled;
                sleep(200);
                integral = 0;
                lastError = 0;
            }

            if (trackingEnabled) {
                if (limelight.hasTarget()) {
                    double tx = limelight.getTx(); // horizontal offset in degrees
                    runAimPID(tx);
                } else {
                    aimMotor.setPower(0.0);
                }
            } else {
                aimMotor.setPower(0.0);
            }

            // Telemetry
            telemetry.addData("Tracking Enabled", trackingEnabled);
            telemetry.addData("Target Found", limelight.hasTarget());
            telemetry.addData("TX", limelight.getTx());
            telemetry.addData("Tag ID", limelight.getTagID());
            telemetry.update();
        }

        aimMotor.setPower(0.0);
    }

    /** PID control for aim motor using tx from Limelight */
    private void runAimPID(double tx) {
        long nowMs = System.currentTimeMillis();
        double dt = (nowMs - lastTimeMs)/1000.0;
        if (dt <= 0) dt = 0.02;
        lastTimeMs = nowMs;

        double error = tx;
        integral += error * dt;
        double derivative = (error - lastError)/dt;
        lastError = error;

        double output = (kP*error) + (kI*integral) + (kD*derivative);

        // Clamp output
        if (output > 0.6) output = 0.6;
        if (output < -0.6) output = -0.6;

        aimMotor.setPower(-output); // invert if needed
    }

    /**
     * Simple Limelight I2C helper
     * Works for Limelight A3 FTC
     */
    private static class LimelightI2C {
        private I2cDeviceSynch device;

        public LimelightI2C(com.qualcomm.robotcore.hardware.HardwareMap hw, String deviceName) {
            device = hw.get(I2cDeviceSynch.class, deviceName);
            device.engage();
        }

        public boolean hasTarget() {
            return device.read8(0) == 1;
        }

        public double getTx() {
            return device.read8(1)/10.0;
        }

        public double getTy() {
            return device.read8(2)/10.0;
        }

        public int getTagID() {
            return device.read8(3) & 0xFF;
        }
    }
}
