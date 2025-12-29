package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "🔵 BLUE-AUTO ", group = "Autonomous")
@Configurable
public class BLUEAU extends OpMode {
    private Servo pushServo;
    private Servo sorterServo;
    private RevColorSensorV3 colorSensor;
    private DcMotorEx shooter1;
    private DcMotor aimMotor;
    private DcMotor intakeMotor;
    private Servo shooterServo;
    private Limelight3A limelight;

    // Alliance toggle (kept explicit so you can reuse for RED later if you want)
    private static final boolean IS_BLUE = true;

    boolean slowPathStarted = false;

    private final double[] INTAKE_POS = {0.00, 0.32, 0.49};
    private final double[] LAUNCH_POS = {0.22, 0.40, 0.58};
    private boolean launchMode = false;

    private boolean shooterSpunUp = false;
    private int launchIndex = 0;

    private static final double AIM_OFFSET_POWER = 0.5;

    private static final double CAMERA_HEIGHT = 0.3;
    private static final double TARGET_HEIGHT = 1.10;
    private static final double CAMERA_ANGLE = 15;

    private static final double SERVO_MIN = 0.3;
    private static final double SERVO_MAX = 0.50;
    private static final double DIST_MIN = 0.4;
    private static final double DIST_MAX = 3.0;

    private static final double RPM_MIN = 0;
    private static final double RPM_MAX = 1800;
    private double targetShooterRPM = 900;

    private boolean useFixedShooterVelocity = false;

    static final int MAX_BALLS = 3;

    int ballCount = 0;
    int intakeIndex = 0;

    boolean intakeFull = false;
    boolean ballDetected = false;

    static final int BALL_DETECT = 280;
    static final int BALL_CLEAR = 250;

    ElapsedTime ballTimer = new ElapsedTime();
    ElapsedTime servoMoveTimer = new ElapsedTime();

    ElapsedTime forceIndexTimer = new ElapsedTime();
    boolean useForceIndex = false;
    static final double FORCE_INDEX_INTERVAL = 0.6;

    static final double BALL_COOLDOWN = 0.15;
    static final double REARM_TIMEOUT = 0.20;
    static final double SERVO_SETTLE_TIME = 0.08;

    private static final double PUSH_HOME = 0.0;

    private static final double SETTLE_TIME = 0.45;
    private static final double PUSH_OUT_TIME = 0.1;
    private static final double PUSH_HOME_TIME = 0.2;

    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;

    private ElapsedTime path3Timer = new ElapsedTime();
    private boolean path3DelayStarted = false;
    private static final double PATH3_DELAY_SECONDS = 4.5;

    private static final double PATH1_DELAY_SECONDS = 8;
    public static boolean USE_PATH3 = true;

    private boolean launching = false;
    private int ballsToLaunch = 0;

    private boolean servoMoving = false;
    private boolean initAimComplete = false;

    enum LaunchState {
        IDLE, ROTATE, WAIT_SETTLE, PUSH_UP, WAIT_UP, PUSH_DOWN, WAIT_DOWN
    }

    LaunchState launchState = LaunchState.IDLE;
    ElapsedTime launchTimer = new ElapsedTime();

    static final double PUSH_UP_POS = 1;
    static final double PUSH_DOWN_POS = 0.0;

    static final double PUSH_UP_TIME = 0.25;
    static final double PUSH_DOWN_TIME = 0.35;

    public enum PathState {
        PATH_1, PATH_2, PATH_3, PATH_4, PATH_5, PATH_6, PATH_7, PATH_8, PATH_9, DONE
    }

    private PathState pathState = PathState.PATH_1;
    private Paths paths;

    private boolean aimHoldEnabled = false;
    private int aimTargetTicks = 0;

    // Mirror RED aim setpoints onto BLUE by negating them
    private int aimAlliance(int redTicks) {
        return IS_BLUE ? -redTicks : redTicks;
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

        public Paths(Follower follower) {

            // BLUE-mirrored (180° rotated) version of your original RED paths
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.000, 15.000), new Pose(50.000, 31.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 31.000), new Pose(28.000, 31.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(28.000, 31.000),
                                    new Pose(49.000, 34.000),
                                    new Pose(58.000, 26.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.000, 26.000), new Pose(47.000, 50.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.000, 50.000), new Pose(28.000, 50.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(28.000, 50.000), new Pose(47.000, 67.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.000, 67.000), new Pose(28.000, 67.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(28.000, 67.000), new Pose(47.000, 67.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.000, 67.000), new Pose(47.000, 67.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();


        }
    }

    private void resetBallState() {
        ballCount = 0;
        intakeIndex = 0;
        ballDetected = false;
        intakeFull = false;
        launchMode = false;
        launching = false;
        servoMoving = false;
        useForceIndex = false;
        sorterServo.setPosition(INTAKE_POS[0]);
        ballTimer.reset();
        forceIndexTimer.reset();
    }

    private void updateShooterVelocity() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleRad = Math.toRadians(CAMERA_ANGLE + ty);
            double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleRad);

            targetShooterRPM = RPM_MIN + (distance - DIST_MIN) / (DIST_MAX - DIST_MIN)
                    * (RPM_MAX - RPM_MIN);

            targetShooterRPM = Math.max(RPM_MIN, Math.min(RPM_MAX, targetShooterRPM));

            shooter1.setVelocity(targetShooterRPM);

            double servoPos = SERVO_MIN + (distance - DIST_MIN) / (DIST_MAX - DIST_MIN)
                    * (SERVO_MAX - SERVO_MIN);
            servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
            shooterServo.setPosition(servoPos);
        } else {
            shooter1.setVelocity(targetShooterRPM);
        }
    }

    private void setShooterVelocity(double rpm) {
        targetShooterRPM = rpm;
        shooter1.setVelocity(rpm);
    }

    private void stopShooter() {
        shooter1.setVelocity(0);
    }

    public void updateStateMachine() {
        switch (pathState) {

            case PATH_1:
                if (!initAimComplete) {
                    aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    aimTargetTicks = aimAlliance(99); // mirrored for BLUE
                    aimMotor.setTargetPosition(aimTargetTicks);
                    aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    aimMotor.setPower(AIM_OFFSET_POWER);
                    aimHoldEnabled = true;
                    initAimComplete = true;
                }

                if (!follower.isBusy()) {
                    setShooterVelocity(RPM_MAX);

                    if (!path3DelayStarted) {
                        path3Timer.reset();
                        path3DelayStarted = true;
                        shooterSpunUp = false;
                    }

                    if (path3Timer.seconds() >= 3.0 && !shooterSpunUp) {
                        setLaunchMode();
                        startLaunch(3);
                        shooterSpunUp = true;
                    }

                    if (path3Timer.seconds() >= PATH1_DELAY_SECONDS) {
                        stopShooter();
                        resetBallState();

                        follower.followPath(paths.Path1, true);
                        follower.setMaxPower(1);
                        setPathState(PathState.PATH_2);
                        intake(1);
                        path3DelayStarted = false;
                        shooterSpunUp = false;
                    }
                }
                break;

            case PATH_2:
                if (!follower.isBusy()) {
                    setIntakeMode();
                    aim(aimAlliance(0));

                    useForceIndex = true;
                    forceIndexTimer.reset();

                    follower.followPath(paths.Path2, true);
                    intake(1);
                    follower.setMaxPower(0.4);
                    setPathState(USE_PATH3 ? PathState.PATH_3 : PathState.PATH_4);
                }
                break;

            case PATH_3:
                if (!follower.isBusy()) {
                    useForceIndex = false;
                    resetBallState();
                    intake(0);
                    setShooterVelocity(1400);
                    follower.followPath(paths.Path3, true);
                    setPathState(PathState.PATH_4);
                    follower.setMaxPower(1);
                    path3DelayStarted = false;
                    aim(aimAlliance(-45));
                }
                break;

            case PATH_4:
                if (!follower.isBusy()) {
                    setLaunchMode();
                    startLaunch(3);

                    if (!path3DelayStarted) {
                        path3Timer.reset();
                        path3DelayStarted = true;
                    }

                    if (path3Timer.seconds() >= PATH3_DELAY_SECONDS) {
                        resetBallState();
                        follower.followPath(paths.Path4, true);
                        setPathState(PathState.PATH_5);
                        stopShooter();
                        follower.setMaxPower(1);
                        path3DelayStarted = false;
                    }
                }
                break;

            case PATH_5:
                if (!follower.isBusy()) {
                    setIntakeMode();

                    useForceIndex = true;
                    forceIndexTimer.reset();

                    follower.followPath(paths.Path5, true);
                    follower.setMaxPower(0.4);
                    setPathState(PathState.PATH_6);
                    intake(1);
                    aim(aimAlliance(0));
                }
                break;

            case PATH_6:
                if (!follower.isBusy()) {
                    useForceIndex = false;
                    follower.followPath(paths.Path6, true);
                    setPathState(PathState.PATH_7);
                    setShooterVelocity(300);
                    useFixedShooterVelocity = true;
                    follower.setMaxPower(1);
                    aim(aimAlliance(70));
                }
                break;

            case PATH_7:
                if (!follower.isBusy()) {
                    setLaunchMode();
                    startLaunch(3);

                    if (!path3DelayStarted) {
                        path3Timer.reset();
                        path3DelayStarted = true;
                    }

                    if (path3Timer.seconds() >= PATH3_DELAY_SECONDS) {
                        resetBallState();
                        setIntakeMode();
                        useFixedShooterVelocity = false;

                        useForceIndex = true;
                        forceIndexTimer.reset();

                        follower.followPath(paths.Path7, true);
                        intake(0);
                        follower.setMaxPower(0.4);
                        setPathState(PathState.PATH_8);
                        path3DelayStarted = false;
                    }
                }
                break;

            case PATH_8:
                if (!follower.isBusy()) {
                    useForceIndex = false;
                    resetBallState();
                    intake(0);
                    setShooterVelocity(1200);
                    follower.followPath(paths.Path8, true);
                    setPathState(PathState.PATH_9);
                    follower.setMaxPower(1);
                    path3DelayStarted = false;
                }
                break;

            case PATH_9:
                if (!follower.isBusy()) {
                    setLaunchMode();
                    startLaunch(3);

                    if (!path3DelayStarted) {
                        path3Timer.reset();
                        path3DelayStarted = true;
                    }

                    if (path3Timer.seconds() >= PATH3_DELAY_SECONDS) {
                        follower.followPath(paths.Path9, true);
                        setPathState(PathState.DONE);
                        stopShooter();
                        follower.setMaxPower(1);
                        path3DelayStarted = false;
                    }
                }
                break;

            case DONE:
                if (!follower.isBusy()) telemetry.addLine("AUTO COMPLETE ✔");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        if (pathTimer != null) pathTimer.reset();
    }

    @Override
    public void init() {
        try {
            pushServo = hardwareMap.get(Servo.class, "pushservo");
            pushServo.setPosition(PUSH_HOME);

            shooterServo = hardwareMap.get(Servo.class, "shooterServo");
            shooterServo.setPosition(SERVO_MIN);

            sorterServo = hardwareMap.get(Servo.class, "sortservo");
            sorterServo.setPosition(INTAKE_POS[0]);

            colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");

            shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
            shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

            PIDFCoefficients pidf = new PIDFCoefficients(5, 0, 0, 19);
            shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

            intakeMotor = hardwareMap.get(DcMotor.class, "intake");

            aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
            aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            aimMotor.setDirection(DcMotor.Direction.REVERSE);
            aimMotor.setPower(0);

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(2);
            limelight.start();

            pathTimer = new ElapsedTime();
            opModeTimer = new ElapsedTime();

            follower = Constants.createFollower(hardwareMap);

            // BLUE start pose (180° rotated from RED)
            follower.setPose(new Pose(85.0, 15.0, Math.toRadians(0)));

            paths = new Paths(follower);

            telemetry.addLine("✅ 🔵 BLUE ALLIANCE AUTO");
            telemetry.addLine("⚙ Ready to start");
            telemetry.addLine("═══════════════════════════════");
            telemetry.addLine("🎯 CORRECTED PIDF");
            telemetry.addLine("   P=5, I=0, D=0, F=19");
            telemetry.addLine("🚀 RPM Range: " + (int) RPM_MIN + "-" + (int) RPM_MAX);
            telemetry.addLine("📐 Servo Range: " + SERVO_MIN + "-" + SERVO_MAX);
            telemetry.addLine("🔧 PATH_7: FIXED 300 RPM");
            telemetry.addLine("📷 Limelight: PIPELINE 2");
            telemetry.addLine("═══════════════════════════════");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addLine("❌ Init Error: " + e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void start() {
        opModeTimer.reset();
        setPathState(PathState.PATH_1);

        ballCount = 3;
        intakeIndex = 2;
        ballDetected = false;
        intakeFull = true;
        initAimComplete = false;

        ballTimer.reset();
        servoMoveTimer.reset();
        forceIndexTimer.reset();
    }

    @Override
    public void loop() {
        try {
            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();
            int brightness = r + g + b;

            if (!launchMode && !servoMoving) {
                updateBallSensor(brightness);
            }

            if (servoMoving && servoMoveTimer.seconds() > SERVO_SETTLE_TIME) {
                servoMoving = false;
            }

            if (launchMode) {
                sorterServo.setPosition(LAUNCH_POS[intakeIndex]);
                if (!useFixedShooterVelocity) {
                    updateShooterVelocity();
                }
            }

            if (aimHoldEnabled) {
                if (aimMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    aimMotor.setTargetPosition(aimTargetTicks);
                    aimMotor.setPower(AIM_OFFSET_POWER);
                }
            }

            follower.update();
            updateStateMachine();
            updateLauncher();

            telemetry.addData("🔵 Alliance", "BLUE");
            telemetry.addData("State", pathState);
            telemetry.addData("Balls", ballCount);
            telemetry.addData("Idx", intakeIndex);
            telemetry.addData("Bright", brightness);
            telemetry.addData("AimPos", aimMotor.getCurrentPosition());
            telemetry.addData("━━━━━━━━━━━━━━━━━━", "");
            telemetry.addData("🎯 Target RPM", (int) targetShooterRPM);
            telemetry.addData("⚙ Actual RPM", (int) shooter1.getVelocity());
            telemetry.addData("📐 Servo Pos", String.format("%.2f", shooterServo.getPosition()));
            telemetry.addData("🔧 Fixed Mode", useFixedShooterVelocity);
            telemetry.update();

        } catch (Exception e) {
            telemetry.addLine("Loop Error: " + e.getMessage());
            telemetry.update();
        }
    }

    public void intake(double power) {
        intakeMotor.setPower(power);
    }

    public void aim(int target) {
        aimTargetTicks = target;
        aimHoldEnabled = true;
        aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        aimMotor.setTargetPosition(aimTargetTicks);
        aimMotor.setPower(AIM_OFFSET_POWER);
    }

    public void updateBallSensor(int brightness) {
        if (useForceIndex && !servoMoving && ballCount < MAX_BALLS) {
            if (forceIndexTimer.seconds() > FORCE_INDEX_INTERVAL) {
                ballCount++;

                if (ballCount >= MAX_BALLS) {
                    ballCount = MAX_BALLS;
                    intakeFull = true;
                    intakeIndex = MAX_BALLS - 1;
                } else {
                    intakeIndex = ballCount - 1;
                }

                sorterServo.setPosition(INTAKE_POS[intakeIndex]);
                servoMoving = true;
                servoMoveTimer.reset();
                forceIndexTimer.reset();
            }
        }

        boolean ballPresent = brightness > BALL_DETECT;
        boolean ballCleared = brightness < BALL_CLEAR;

        if (ballCleared || (ballDetected && ballTimer.seconds() > REARM_TIMEOUT)) {
            ballDetected = false;
        }

        if (ballPresent && !ballDetected && !launchMode && !intakeFull && !useForceIndex && ballTimer.seconds() > BALL_COOLDOWN) {
            ballDetected = true;
            ballTimer.reset();
            ballCount++;

            if (ballCount >= MAX_BALLS) {
                ballCount = MAX_BALLS;
                intakeFull = true;
                intakeIndex = MAX_BALLS - 1;
            } else {
                intakeIndex = ballCount - 1;
            }

            sorterServo.setPosition(INTAKE_POS[intakeIndex]);
            servoMoving = true;
            servoMoveTimer.reset();
        }
    }

    public void startLaunch(int balls) {
        if (launching || balls <= 0) return;
        ballsToLaunch = Math.min(balls, ballCount);
        launching = true;
        launchMode = true;
    }

    public void updateLauncher() {
        if (!launchMode) {
            launchState = LaunchState.IDLE;
            return;
        }

        switch (launchState) {
            case IDLE:
                sorterServo.setPosition(LAUNCH_POS[intakeIndex]);
                pushServo.setPosition(PUSH_DOWN_POS);
                launchState = LaunchState.ROTATE;
                break;

            case ROTATE:
                sorterServo.setPosition(LAUNCH_POS[intakeIndex]);
                launchTimer.reset();
                launchState = LaunchState.WAIT_SETTLE;
                break;

            case WAIT_SETTLE:
                if (launchTimer.seconds() > SETTLE_TIME) {
                    launchState = LaunchState.PUSH_UP;
                }
                break;

            case PUSH_UP:
                pushServo.setPosition(PUSH_UP_POS);
                launchTimer.reset();
                launchState = LaunchState.WAIT_UP;
                break;

            case WAIT_UP:
                if (launchTimer.seconds() > PUSH_UP_TIME) {
                    launchState = LaunchState.PUSH_DOWN;
                }
                break;

            case PUSH_DOWN:
                pushServo.setPosition(PUSH_DOWN_POS);
                launchTimer.reset();
                launchState = LaunchState.WAIT_DOWN;
                break;

            case WAIT_DOWN:
                if (launchTimer.seconds() > PUSH_DOWN_TIME) {
                    intakeIndex = (intakeIndex + 1) % LAUNCH_POS.length;
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    public void setIntakeMode() {
        launchMode = false;
        sorterServo.setPosition(INTAKE_POS[intakeIndex]);
    }

    public void setLaunchMode() {
        launchMode = true;
        sorterServo.setPosition(LAUNCH_POS[intakeIndex]);
    }

    @Override
    public void stop() {
        if (aimMotor != null) aimMotor.setPower(0);
        if (shooter1 != null) stopShooter();
        if (intakeMotor != null) intakeMotor.setPower(0);
    }
}