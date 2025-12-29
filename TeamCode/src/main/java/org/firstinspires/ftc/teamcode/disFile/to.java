package org.firstinspires.ftc.teamcode.disFile;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled

@TeleOp(name = "to", group = "Teleop")
public class to extends OpMode {

    private DcMotorEx aimMotor;

    // REV HD Hex Motor encoder: 28 ticks per motor shaft revolution
    private static final double TICKS_PER_MOTOR_REV = 28.0;

    // Your gear ratio: large 89-tooth gear on turret, small 29-tooth pinion on motor
    private static final double GEAR_RATIO = 89.0 / 29.0;  // ≈ 3.0689655 : 1

    // Ticks per full turret (output) revolution
    private static final double TICKS_PER_OUTPUT_REV = TICKS_PER_MOTOR_REV * GEAR_RATIO;

    // Ticks per degree of turret rotation
    private static final double TICKS_PER_DEGREE = TICKS_PER_OUTPUT_REV / 360.0;

    // Speed settings (adjust these to your liking)
    private static final double SLOW_SPEED   = 0.15;  // D-pad up/down for fine control
    private static final double NORMAL_SPEED = 0.30;  // D-pad left/right default
    private static final double FAST_SPEED   = 0.50;  // Hold both up+left/right for faster

    @Override
    public void init() {
        // Map your motor - make sure the name matches your robot config!
        aimMotor = hardwareMap.get(DcMotorEx.class, "aimMotor");

        // Set up encoder reading
        aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aimMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // BRAKE = holds position when not moving (good for turret)
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Turret Manual Control READY");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("  D-pad Left  ←  Turn Left");
        telemetry.addLine("  D-pad Right →  Turn Right");
        telemetry.addLine("  D-pad Up    ↑  Faster");
        telemetry.addLine("  D-pad Down  ↓  Slower");
        telemetry.addLine("");
        telemetry.addLine("Live encoder position shown below");
        telemetry.update();
    }

    @Override
    public void loop() {
        double power = 0.0;

        // Base direction from left/right
        if (gamepad1.dpad_left) {
            power = -NORMAL_SPEED;
        } else if (gamepad1.dpad_right) {
            power = NORMAL_SPEED;
        }

        // Speed modifiers
        if (gamepad1.dpad_up) {
            if (power != 0) {
                // Make it faster if already moving left/right
                power = power > 0 ? FAST_SPEED : -FAST_SPEED;
            } else {
                // If only up pressed, move slowly forward (optional)
                power = SLOW_SPEED;
            }
        } else if (gamepad1.dpad_down) {
            if (power != 0) {
                // Make it slower
                power = power > 0 ? SLOW_SPEED : -SLOW_SPEED;
            } else {
                power = -SLOW_SPEED;
            }
        }

        // Apply power to motor
        aimMotor.setPower(power);

        // Live encoder feedback
        int ticks = aimMotor.getCurrentPosition();
        double turretRevs = ticks / TICKS_PER_OUTPUT_REV;
        double turretDegrees = ticks / TICKS_PER_DEGREE;

        telemetry.addData("Turret Power", "%.2f", power);
        telemetry.addData("Raw Encoder Ticks", ticks);
        telemetry.addData("Turret Revolutions", "%.3f", turretRevs);
        telemetry.addData("Turret Degrees", "%.1f°", turretDegrees);
        telemetry.addLine("");
        telemetry.addLine("→ Use D-pad to move turret");
        telemetry.addLine("→ Values update live as you move!");
        telemetry.update();
    }

    @Override
    public void stop() {
        aimMotor.setPower(0);
    }
}