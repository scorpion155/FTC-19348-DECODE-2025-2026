package org.firstinspires.ftc.teamcode.codes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
@Disabled

@TeleOp(name = "Turret IMU Auto Aim (STABLE)", group = "Turret")
public class turret1 extends OpMode {

    /* ================= HARDWARE ================= */

    private DcMotor turret;
    private BNO055IMU imu;

    /* ================= LIMITS ================= */
    private static final int TURRET_MIN = -600;
    private static final int TURRET_MAX =  600;

    /* ================= TUNING ================= */
    private static final double TICKS_PER_RAD = 380.0;
    private static final double KP = 1.1;
    private static final double KI = 0.04;
    private static final double KD = 0.25;
    private static final double FF_GAIN = 0.03;

    /* ================= STATE ================= */
    private double integral = 0;
    private double lastError = 0;
    private double lastHeading = 0;
    private long lastTimeMs = 0;

    private int turretZero;
    private double headingZero;

    /* ================= INIT ================= */

    @Override
    public void init() {

        turret = hardwareMap.get(DcMotor.class, "aimMotor");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /* ---------- LEGACY IMU (BHI260AP SAFE) ---------- */
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled = false;

        imu.initialize(params);

        turretZero = turret.getCurrentPosition();
        headingZero = getHeadingRad();
        lastHeading = headingZero;
        lastTimeMs = System.currentTimeMillis();

        telemetry.addLine("Turret Auto Aim READY (NO CRASH)");
        telemetry.update();
    }

    /* ================= LOOP ================= */

    @Override
    public void loop() {

        boolean manual = gamepad1.right_bumper;
        double manualPower = gamepad1.right_stick_x;

        if (manual) {
            turret.setPower(clamp(manualPower, -1, 1));
            resetPID();
            return;
        }

        double heading = getHeadingRad();
        double headingDelta = normalize(heading - headingZero);

        int desiredTicks = (int) (turretZero - headingDelta * TICKS_PER_RAD);
        desiredTicks = clamp(desiredTicks, TURRET_MIN, TURRET_MAX);

        int currentTicks = turret.getCurrentPosition();
        double error = desiredTicks - currentTicks;

        long now = System.currentTimeMillis();
        double dt = Math.max(0.001, (now - lastTimeMs) / 1000.0);
        lastTimeMs = now;

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double pid = KP * error + KI * integral + KD * derivative;

        double headingRate = (heading - lastHeading) / dt;
        lastHeading = heading;

        double power = pid - headingRate * FF_GAIN;
        power = clamp(power, -1.0, 1.0);

        if ((currentTicks >= TURRET_MAX && power > 0) ||
                (currentTicks <= TURRET_MIN && power < 0)) {
            power = 0;
        }

        turret.setPower(power);
    }

    /* ================= HELPERS ================= */

    private void resetPID() {
        integral = 0;
        lastError = 0;
        lastTimeMs = System.currentTimeMillis();
    }

    private double getHeadingRad() {
        Orientation o = imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.RADIANS
        );
        return o.firstAngle;
    }

    private static double normalize(double a) {
        while (a <= -Math.PI) a += 2 * Math.PI;
        while (a >  Math.PI) a -= 2 * Math.PI;
        return a;
    }

    private static int clamp(int v, int min, int max) {
        return Math.max(min, Math.min(max, v));
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
