package org.firstinspires.ftc.teamcode.disFile;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled

@Autonomous(name = "aa", group = "Autonomous")
@Configurable
public class aa extends OpMode {

    private Servo pushServo;
    private Servo sorterServo;

    private RevColorSensorV3 colorSensor;
    private NormalizedColorSensor launchSensor;

    private DcMotor shooter1;
    private DcMotor aimMotor;
    private DcMotor intakeMotor;

    private Servo shooterServo;

    private final double[] INTAKE_POS = {0.00, 0.32, 0.49};
    private final double[] LAUNCH_POS = {0.22, 0.40, 0.58};

    private boolean launchMode = false;

    static final int MAX_BALLS = 3;
    static final int BALL_DETECT = 320;
    static final int BALL_CLEAR = 280;

    int ballCount = 0;
    int intakeIndex = 0;
    boolean ballDetected = false;
    boolean intakeFull = false;

    public static double SHOOTER_POWER = 1.0;
    public static double SHOOTER_SPINUP_SECONDS = 1.0;
    private boolean shooterSpinupStarted = false;
    private ElapsedTime shooterSpinupTimer = new ElapsedTime();

    public static boolean AIM_RESET_ENCODER_ON_INIT = true;
    public static boolean AIM_DIRECTION_REVERSE = true;

    public static double LEFT_TILT_POWER = 0.7;
    public static int LEFT_TILT_TICKS = 450;
    public static double LEFT_TILT_DIVIDE = 4.5;
    public static int LEFT_TILT_TOL_TICKS = 6;
    public static double LEFT_TILT_TIMEOUT_S = 1.2;

    private int leftTiltTargetTicks = 0;
    private boolean leftTiltStarted = false;
    private ElapsedTime leftTiltTimer = new ElapsedTime();

    public static float GREEN_HUE_TARGET = 180f;
    public static float PURPLE_HUE_TARGET = 112f;
    public static float HUE_TOL = 18f;

    public static float MIN_SAT = 0.35f;
    public static float MIN_VAL = 0.12f;
    public static float LAUNCH_MIN_ALPHA = 0.05f;
    public static int LAUNCH_CONFIRM_SAMPLES = 4;

    private int greenHits = 0;
    private int purpleHits = 0;

    private enum BallColor { GREEN, PURPLE, UNKNOWN }

    private final BallColor[] requiredMotif = { BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN };

    private BallColor verifiedLaunchColor = BallColor.UNKNOWN;
    private boolean launchBallPresent = false;

    private static final double PUSH_OUT = 1.0;
    private static final double PUSH_HOME = 0.0;

    private static final double PUSH_OUT_TIME = 0.10;
    private static final double PUSH_HOME_TIME = 0.20;

    public static double SLOT_SETTLE_TIME = 0.25;

    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;

    private ElapsedTime path3Timer = new ElapsedTime();
    private boolean path3DelayStarted = false;
    private static final double PATH3_DELAY_SECONDS = 6.0;

    public static boolean USE_PATH3 = true;

    private boolean launching = false;
    private int ballsToLaunch = 0;

    private int motifStep = 0;
    private int scanSlot = 0;
    private int scanAttempts = 0;

    private enum PushState {
        IDLE,
        CHECK_COLOR,
        SWITCH_SLOT,
        PUSH_OUT,
        PUSH_BACK
    }

    private PushState pushState = PushState.IDLE;
    private ElapsedTime pushTimer = new ElapsedTime();

    public enum PathState {
        PATH_1, PATH_2, PATH_3_TILT, PATH_3, PATH_4, PATH_5, DONE
    }

    private PathState pathState = PathState.PATH_1;
    private Paths paths;

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5;

        public Paths(Follower follower, PathConstraints slow) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(59.0, 15.0), new Pose(55.0, 31.0)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(55.0, 31.0), new Pose(28.0, 31.0)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(28.0, 31.0), new Pose(49.0, 34.0), new Pose(58.0, 26.0)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(58.0, 26.0), new Pose(60.0, 40.0)))
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.0, 40.0), new Pose(45.0, 45.0)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }

    private boolean inHueBand(float hue, float target, float tol) {
        float d = Math.abs(hue - target);
        d = Math.min(d, 360f - d);
        return d <= tol;
    }

    private void updateLaunchHSV() {
        NormalizedRGBA c = launchSensor.getNormalizedColors();
        int argb = c.toColor();
        float[] hsv = new float[3];
        Color.colorToHSV(argb, hsv);

        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        launchBallPresent = (c.alpha >= LAUNCH_MIN_ALPHA);

        if (!launchBallPresent || sat < MIN_SAT || val < MIN_VAL) {
            verifiedLaunchColor = BallColor.UNKNOWN;
            greenHits = 0;
            purpleHits = 0;
        } else {
            if (inHueBand(hue, GREEN_HUE_TARGET, HUE_TOL)) greenHits++;
            else greenHits = 0;

            if (inHueBand(hue, PURPLE_HUE_TARGET, HUE_TOL)) purpleHits++;
            else purpleHits = 0;

            if (greenHits >= LAUNCH_CONFIRM_SAMPLES) verifiedLaunchColor = BallColor.GREEN;
            else if (purpleHits >= LAUNCH_CONFIRM_SAMPLES) verifiedLaunchColor = BallColor.PURPLE;
            else verifiedLaunchColor = BallColor.UNKNOWN;
        }

        telemetry.addData("H", hue);
        telemetry.addData("S", sat);
        telemetry.addData("V", val);
        telemetry.addData("A", c.alpha);
    }

    private void startLeftTilt() {
        leftTiltTargetTicks = (int) Math.round(LEFT_TILT_TICKS / LEFT_TILT_DIVIDE);

        aimMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setTargetPosition(leftTiltTargetTicks);
        aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        aimMotor.setPower(Math.abs(LEFT_TILT_POWER));

        leftTiltStarted = true;
        leftTiltTimer.reset();
    }

    private boolean leftTiltReached() {
        int err = leftTiltTargetTicks - aimMotor.getCurrentPosition();
        return Math.abs(err) <= LEFT_TILT_TOL_TICKS;
    }

    private boolean leftTiltTimedOut() {
        return leftTiltTimer.seconds() >= LEFT_TILT_TIMEOUT_S;
    }

    private void holdLeftTilt() {
        aimMotor.setPower(0.0);
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updateStateMachine() {
        switch (pathState) {

            case PATH_1:
                follower.followPath(paths.Path1, true);
                setIntakeMode();
                intake(0);
                follower.setMaxPower(1);
                setPathState(PathState.PATH_2);
                break;

            case PATH_2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    intake(1);
                    follower.setMaxPower(0.4);
                    setPathState(USE_PATH3 ? PathState.PATH_3_TILT : PathState.PATH_4);
                }
                break;

            case PATH_3_TILT:
                if (!leftTiltStarted) startLeftTilt();

                if (leftTiltReached() || leftTiltTimedOut()) {
                    holdLeftTilt();

                    intakeFull = false;
                    ballDetected = false;
                    intakeIndex = 0;
                    sorterServo.setPosition(INTAKE_POS[intakeIndex]);
                    intake(0);

                    follower.followPath(paths.Path3, true);
                    follower.setMaxPower(1);
                    setPathState(PathState.PATH_3);

                    leftTiltStarted = false;
                }
                break;

            case PATH_3:
                if (!follower.isBusy()) {
                    setPathState(PathState.PATH_4);
                    path3DelayStarted = false;
                }
                break;

            case PATH_4:
                if (!follower.isBusy()) {

                    if (!shooterSpinupStarted) {
                        shooterSpinupStarted = true;
                        shooterSpinupTimer.reset();
                        shooter1.setPower(SHOOTER_POWER);
                    }

                    if (shooterSpinupStarted
                            && shooterSpinupTimer.seconds() >= SHOOTER_SPINUP_SECONDS
                            && !launching) {
                        setLaunchMode();
                        startLaunchMotif(3);
                    }

                    if (!path3DelayStarted) {
                        path3Timer.reset();
                        path3DelayStarted = true;

                        intakeFull = false;
                        ballDetected = false;
                        intakeIndex = 0;
                        sorterServo.setPosition(INTAKE_POS[intakeIndex]);
                        intake(0);
                    }

                    if (path3Timer.seconds() >= PATH3_DELAY_SECONDS) {
                        follower.followPath(paths.Path5, true);
                        setPathState(PathState.PATH_5);
                        follower.setMaxPower(1);
                        path3DelayStarted = false;
                    }
                }
                break;

            case PATH_5:
                if (!follower.isBusy()) setPathState(PathState.DONE);
                break;

            case DONE:
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        if (pathTimer != null) pathTimer.reset();
    }

    @Override
    public void init() {
        pushServo = hardwareMap.get(Servo.class, "pushservo");
        pushServo.setPosition(PUSH_HOME);

        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        shooterServo.setPosition(0.45);

        sorterServo = hardwareMap.get(Servo.class, "sortservo");

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");

        launchSensor = hardwareMap.get(NormalizedColorSensor.class, "launchsen");
        launchSensor.setGain(4);

        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setDirection(AIM_DIRECTION_REVERSE ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        aimMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aimMotor.setPower(0);

        if (AIM_RESET_ENCODER_ON_INIT) {
            aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            aimMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        sorterServo.setPosition(INTAKE_POS[0]);

        PathConstraints slow = new PathConstraints(20, 60);

        pathTimer = new ElapsedTime();
        opModeTimer = new ElapsedTime();
        follower = Constants.createFollower(hardwareMap);

        paths = new Paths(follower, slow);
        follower.setPose(new Pose(59.0, 15.0, Math.toRadians(180)));
    }

    @Override
    public void start() {
        opModeTimer.reset();

        shooterSpinupStarted = false;
        shooterSpinupTimer.reset();

        launching = false;
        launchMode = false;
        pushState = PushState.IDLE;

        greenHits = 0;
        purpleHits = 0;

        shooter1.setPower(0);
        aimMotor.setPower(0);

        leftTiltStarted = false;

        setPathState(PathState.PATH_1);
    }

    @Override
    public void loop() {
        int brightness = colorSensor.red() + colorSensor.green() + colorSensor.blue();
        if (!launchMode) updateBallSensor(brightness);

        if (launchMode) {
            updateLaunchHSV();
            sorterServo.setPosition(LAUNCH_POS[scanSlot]);
        } else {
            launchBallPresent = false;
            verifiedLaunchColor = BallColor.UNKNOWN;
            greenHits = 0;
            purpleHits = 0;
        }

        follower.update();
        updateStateMachine();
        updateLauncherMotif();

        Pose p = follower.getPose();
        telemetry.addData("Path", pathState);
        telemetry.addData("X", p.getX());
        telemetry.addData("Y", p.getY());
        telemetry.addData("Hdg", Math.toDegrees(p.getHeading()));

        telemetry.addData("BallCount", ballCount);
        telemetry.addData("LaunchMode", launchMode);
        telemetry.addData("Present", launchBallPresent);
        telemetry.addData("Color", verifiedLaunchColor);

        telemetry.addData("AimPos", aimMotor.getCurrentPosition());
        telemetry.addData("LeftTiltTarget", leftTiltTargetTicks);

        telemetry.addData("Spin", shooterSpinupStarted);
        telemetry.addData("SpinT", shooterSpinupTimer.seconds());

        telemetry.update();
    }

    public void intake(double power) {
        intakeMotor.setPower(power);
    }

    public void updateBallSensor(int brightness) {
        boolean ballPresent = brightness > BALL_DETECT;
        boolean ballGone = brightness < BALL_CLEAR;

        if (ballGone) ballDetected = false;

        if (ballPresent && !ballDetected && !intakeFull) {
            ballDetected = true;
            ballCount++;

            if (ballCount >= MAX_BALLS) {
                ballCount = MAX_BALLS;
                intakeFull = true;
            } else {
                intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
                sorterServo.setPosition(INTAKE_POS[intakeIndex]);
            }
        }
    }

    public void startLaunchMotif(int balls) {
        if (launching || balls <= 0) return;

        ballsToLaunch = Math.min(balls, ballCount);
        if (ballsToLaunch <= 0) return;

        launching = true;
        launchMode = true;

        motifStep = 0;
        scanSlot = 0;
        scanAttempts = 0;

        pushServo.setPosition(PUSH_HOME);
        sorterServo.setPosition(LAUNCH_POS[scanSlot]);

        pushState = PushState.CHECK_COLOR;
        pushTimer.reset();
    }

    private void switchToNextSlotAndSettle() {
        scanSlot = (scanSlot + 1) % MAX_BALLS;
        sorterServo.setPosition(LAUNCH_POS[scanSlot]);
        pushState = PushState.SWITCH_SLOT;
        pushTimer.reset();
    }

    private void stopLaunching() {
        launching = false;
        launchMode = false;
        pushState = PushState.IDLE;
        shooter1.setPower(0);
        pushServo.setPosition(PUSH_HOME);
    }

    public void updateLauncherMotif() {
        if (!launching) return;

        if (motifStep >= requiredMotif.length || ballsToLaunch <= 0 || ballCount <= 0) {
            stopLaunching();
            return;
        }

        switch (pushState) {
            case CHECK_COLOR: {
                BallColor expected = requiredMotif[motifStep];

                if (!launchBallPresent) {
                    scanAttempts++;
                    if (scanAttempts >= MAX_BALLS) {
                        stopLaunching();
                        return;
                    }
                    switchToNextSlotAndSettle();
                    return;
                }

                if (verifiedLaunchColor == BallColor.UNKNOWN) {
                    if (pushTimer.seconds() >= SLOT_SETTLE_TIME) {
                        scanAttempts++;
                        if (scanAttempts >= MAX_BALLS) {
                            stopLaunching();
                            return;
                        }
                        switchToNextSlotAndSettle();
                    }
                    return;
                }

                if (verifiedLaunchColor != expected) {
                    scanAttempts++;
                    if (scanAttempts >= MAX_BALLS) {
                        stopLaunching();
                        return;
                    }
                    switchToNextSlotAndSettle();
                    return;
                }

                pushServo.setPosition(PUSH_OUT);
                pushState = PushState.PUSH_OUT;
                pushTimer.reset();
                return;
            }

            case SWITCH_SLOT:
                if (pushTimer.seconds() >= SLOT_SETTLE_TIME) {
                    pushState = PushState.CHECK_COLOR;
                    pushTimer.reset();
                }
                break;

            case PUSH_OUT:
                if (pushTimer.seconds() >= PUSH_OUT_TIME) {
                    pushServo.setPosition(PUSH_HOME);
                    pushState = PushState.PUSH_BACK;
                    pushTimer.reset();
                }
                break;

            case PUSH_BACK:
                if (pushTimer.seconds() >= PUSH_HOME_TIME) {
                    motifStep++;
                    ballsToLaunch--;
                    ballCount--;
                    scanAttempts = 0;
                    switchToNextSlotAndSettle();
                }
                break;

            case IDLE:
                break;
        }
    }

    public void setIntakeMode() {
        launchMode = false;
        sorterServo.setPosition(INTAKE_POS[intakeIndex]);
    }

    public void setLaunchMode() {
        launchMode = true;
        sorterServo.setPosition(LAUNCH_POS[scanSlot]);
    }

    @Override
    public void stop() {
        shooter1.setPower(0);
        intakeMotor.setPower(0);
        aimMotor.setPower(0);
        pushServo.setPosition(PUSH_HOME);
    }
}