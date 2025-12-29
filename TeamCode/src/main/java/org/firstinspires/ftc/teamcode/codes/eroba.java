package org.firstinspires.ftc.teamcode.codes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled

@TeleOp(name="BLUE", group="TeleOp")
public class eroba extends LinearOpMode {

    private DcMotor turret;
    private Limelight3A limelight;
    private IMU imu;
    private static final double TURRET_MANUAL_POWER = 0.25;  // CHANGED: reduced from 0.6 to 0.25
    // Turret auto-tracking
    private static final double DEADZONE = 2.0;


    @Override
    public void runOpMode() {

        turret   = hardwareMap.get(DcMotor.class, "aimMotor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9);
        limelight.start();
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setPower(0);
        waitForStart();
        limelight.start();



        while (opModeIsActive()) {

            // =======================
            // TURRET CONTROL (Auto + Manual)
            // =======================
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            LLResult llResult = limelight.getLatestResult();

            boolean tagDetected = (llResult != null && llResult.isValid());
            double turretCmd = 0.0;

            if (tagDetected) {
                // AUTO TRACKING when AprilTag detected
                double tx = llResult.getTx();
                turretCmd = tx * 0.03;
                turretCmd = Math.max(-0.6, Math.min(0.6, turretCmd));
                if (Math.abs(tx) < DEADZONE) {
                    turretCmd = 0;
                }
            } else {
                // MANUAL CONTROL when no tag (gamepad2)
                boolean rb2 = gamepad2.right_bumper;
                boolean lb2 = gamepad2.left_bumper;

                if (rb2) {
                    turretCmd = TURRET_MANUAL_POWER;  // Rotate right (0.25 power)
                } else if (lb2) {
                    turretCmd = -TURRET_MANUAL_POWER; // Rotate left (0.25 power)
                }
            }

            turret.setPower(turretCmd);

            // =======================
            // Speed Adjustment (gamepad2)
            // =======================
            boolean a2 = gamepad2.a;
            boolean b2 = gamepad2.b;


            if (tagDetected) {
                telemetry.addData("Turret", "AUTO TRACKING (Tx=%.2f, Power=%.2f)",
                        llResult.getTx(), turretCmd);
            } else {
                telemetry.addData("Turret", "MANUAL MODE (Power=%.2f)", turretCmd);
            }

            telemetry.update();
        }

        limelight.stop();
    }


}