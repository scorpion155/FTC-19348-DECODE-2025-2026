package org.firstinspires.ftc.teamcode.subsystems;

/*
  TurretController.java
  ---------------------
  Encapsulates all turret rotation / tracking logic (PID + IMU feedforward + encoder mapping).
*/

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretController {

    // Hardware references
    private final DcMotor turretMotor;
    private final IMU imu;
    private final Telemetry telemetry; // optional, may be null

    // Turret encoder hard limits
    public static final int TURRET_MIN_POS = -600;
    public static final int TURRET_MAX_POS = 600;

    // ticks-per-radian mapping
    private static final double BASE_TICKS_PER_RADIAN = TURRET_MAX_POS / Math.PI;
    private static final double TICKS_PER_RADIAN_SCALE = 2;
    private static final double TICKS_PER_RADIAN = BASE_TICKS_PER_RADIAN * TICKS_PER_RADIAN_SCALE;

    // PID & control gains
    private static final double TURRET_KP = 1.3;
    private static final double TURRET_KI = 0.09;
    private static final double TURRET_KD = 0.3;
    private static final double TURRET_MAX_POWER = 1.0;

    // Feedforward and smoothing/filtering
    private static final double FF_GAIN = 0.030;
    private static final double POWER_SMOOTH_ALPHA = 0.96;
    private static final double DERIV_FILTER_ALPHA = 0.40;

    // Deadband & anti-windup
    private static final int SMALL_DEADBAND_TICKS = 4;
    private static final double INTEGRAL_CLAMP = 200.0;

    // Internal state
    private double turretIntegral = 0.0;
    private int lastErrorTicks = 0;
    private long lastTimeMs = -1L;
    private double lastAppliedPower = 0.0;
    private double lastDerivative = 0.0;

    // Heading / encoder reference
    private double headingReferenceRad = 0.0;
    private double lastHeadingRad = 0.0;
    private int turretEncoderReference = 0;

    private boolean manualActiveLast = false;

    // telemetry values
    private int lastDesiredTicks = 0;
    private int lastErrorReported = 0;
    private double lastPidOut = 0.0;
    private double lastFf = 0.0;

    public TurretController(DcMotor turretMotor, IMU imu, Telemetry telemetry) {
        this.turretMotor = turretMotor;
        this.imu = imu;
        this.telemetry = telemetry;

        captureReferences();
        resetPidState();
    }

    public void captureReferences() {
        headingReferenceRad = getHeadingRadians();
        lastHeadingRad = headingReferenceRad;
        turretEncoderReference = turretMotor.getCurrentPosition();
    }

    public void resetPidState() {
        turretIntegral = 0.0;
        lastErrorTicks = 0;
        lastTimeMs = System.currentTimeMillis();
        lastAppliedPower = 0.0;
        lastDerivative = 0.0;
        manualActiveLast = false;
    }

    public void update(boolean manualNow, double manualPower) {
        long nowMs = System.currentTimeMillis();

        if (manualNow) {
            applyManualPower(manualPower);
            turretIntegral = 0.0;
            lastErrorTicks = 0;
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            manualActiveLast = true;
            publishTelemetry();
            return;
        }

        if (manualActiveLast && !manualNow) {
            captureReferences();
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            lastAppliedPower = 0.0;
        }
        manualActiveLast = false;

        double currentHeadingRad = getHeadingRadians();
        double headingDelta = normalizeAngle(currentHeadingRad - headingReferenceRad);

        double angularVel = 0.0;
        if (lastTimeMs > 0) {
            double dtHeading = Math.max(0.0001, (nowMs - lastTimeMs) / 1000.0);
            double headingDeltaSinceLast = normalizeAngle(currentHeadingRad - lastHeadingRad);
            angularVel = headingDeltaSinceLast / dtHeading;
        }

        lastHeadingRad = currentHeadingRad;

        double desiredTicksDouble = turretEncoderReference - headingDelta * TICKS_PER_RADIAN;
        int desiredTicks = (int)Math.round(desiredTicksDouble);

        if (desiredTicks > TURRET_MAX_POS) desiredTicks = TURRET_MAX_POS;
        if (desiredTicks < TURRET_MIN_POS) desiredTicks = TURRET_MIN_POS;

        int currentTicks = turretMotor.getCurrentPosition();
        int errorTicks = desiredTicks - currentTicks;

        long dtMs = (lastTimeMs < 0) ? 20 : Math.max(1, nowMs - lastTimeMs);
        double dt = dtMs / 1000.0;
        lastTimeMs = nowMs;

        if (Math.abs(errorTicks) > SMALL_DEADBAND_TICKS) {
            if (lastErrorTicks != 0 &&
                    ((errorTicks > 0 && lastErrorTicks < 0) ||
                            (errorTicks < 0 && lastErrorTicks > 0))) {
                turretIntegral *= 0.5;
            }
            turretIntegral += errorTicks * dt;
        } else {
            turretIntegral *= 0.90;
        }

        if (turretIntegral > INTEGRAL_CLAMP) turretIntegral = INTEGRAL_CLAMP;
        if (turretIntegral < -INTEGRAL_CLAMP) turretIntegral = -INTEGRAL_CLAMP;

        double rawDerivative = (errorTicks - lastErrorTicks) / Math.max(1e-4, dt);
        double derivativeFiltered =
                DERIV_FILTER_ALPHA * rawDerivative + (1.0 - DERIV_FILTER_ALPHA) * lastDerivative;

        double pidOut = TURRET_KP * errorTicks
                + TURRET_KI * turretIntegral
                + TURRET_KD * derivativeFiltered;

        double ff = -angularVel * FF_GAIN;

        double cmdPower = pidOut + ff;

        if (Math.abs(errorTicks) <= SMALL_DEADBAND_TICKS) cmdPower = 0.0;

        if (cmdPower > TURRET_MAX_POWER) cmdPower = TURRET_MAX_POWER;
        if (cmdPower < -TURRET_MAX_POWER) cmdPower = -TURRET_MAX_POWER;

        double applied =
                POWER_SMOOTH_ALPHA * lastAppliedPower
                        + (1.0 - POWER_SMOOTH_ALPHA) * cmdPower;

        if ((currentTicks >= TURRET_MAX_POS && applied > 0.0) ||
                (currentTicks <= TURRET_MIN_POS && applied < 0.0)) {
            applied = 0.0;
        }

        turretMotor.setPower(applied);

        lastErrorTicks = errorTicks;
        lastAppliedPower = applied;
        lastDerivative = derivativeFiltered;

        lastDesiredTicks = desiredTicks;
        lastErrorReported = errorTicks;
        lastPidOut = pidOut;
        lastFf = ff;

        publishTelemetry();
    }

    private void applyManualPower(double manualPower) {
        int currentTicks = turretMotor.getCurrentPosition();
        double requested = manualPower;

        if ((currentTicks >= TURRET_MAX_POS && requested > 0.0) ||
                (currentTicks <= TURRET_MIN_POS && requested < 0.0)) {
            requested = 0.0;
        }

        if (requested > 1.0) requested = 1.0;
        if (requested < -1.0) requested = -1.0;

        turretMotor.setPower(requested);

        lastDesiredTicks = turretEncoderReference;
        lastErrorReported = lastDesiredTicks - currentTicks;
        lastPidOut = 0.0;
        lastFf = 0.0;
    }

    private double getHeadingRadians() {
        if (imu == null) return 0.0;
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        return -ypr.getYaw(AngleUnit.RADIANS);
    }

    private static double normalizeAngle(double angle) {
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }

    public int getLastDesiredTicks() { return lastDesiredTicks; }
    public int getLastErrorTicks() { return lastErrorReported; }
    public double getLastPidOut() { return lastPidOut; }
    public double getLastFf() { return lastFf; }
    public double getLastAppliedPower() { return lastAppliedPower; }

    private void publishTelemetry() {
        if (telemetry == null) return;
    }
}
