package org.firstinspires.ftc.teamcode.codes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Disabled

@TeleOp(name="TurretShooterTrackingFixedHeading", group="Shooter")
public class turretimu extends LinearOpMode {

    private DcMotor turretMotor;
    private BNO055IMU imu;
    private Limelight3A limelight;

    private double kP = 0.02;
    private double lastError = 0;
    private double integral = 0;

    private double targetHeading = 0;

    @Override
    public void runOpMode() {

        turretMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.loggingEnabled = false;
        imuParams.mode = BNO055IMU.SensorMode.IMU;
        imuParams.calibrationDataFile = "BNO055IMUCalibration.json";
        imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(imuParams);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Hardware initialized. Press Play.");
        telemetry.update();
        waitForStart();

        // ------------------------------
        // Initial heading
        // ------------------------------
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        targetHeading = angles.firstAngle;

        // ------------------------------
        // Main loop
        // ------------------------------
        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            double turretPower = 0;

            // Read IMU heading correctly
            try {
                Orientation currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double currentHeading = currentAngles.firstAngle;

                if (result != null && result.isValid()) {
                    double tx = result.getTx();

                    double error = tx;
                    integral += error * 0.02;
                    double derivative = (error - lastError) / 0.02;

                    turretPower = kP * error + integral * 0 + derivative * 0;
                    lastError = error;

                    targetHeading = currentHeading;

                } else {
                    double headingDelta = currentHeading - targetHeading;
                    turretPower = -headingDelta * 0.05;
                    if (Math.abs(turretPower) < 0.05) turretPower = 0;
                }

                turretMotor.setPower(turretPower);

                telemetry.addData("Turret Power", turretPower);
                telemetry.addData("Target Visible", result != null && result.isValid());
                if (result != null) {
                    telemetry.addData("TX", result.getTx());
                    telemetry.addData("TY", result.getTy());
                }
                telemetry.addData("IMU Heading", currentHeading);
                telemetry.update();

            } catch (Exception e) {
                telemetry.addLine("IMU not ready yet");
                telemetry.update();
            }

            sleep(20);
        }
    }
}
