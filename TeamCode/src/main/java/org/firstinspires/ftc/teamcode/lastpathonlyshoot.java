package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
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

@Autonomous(name = "🔵 BLUE-AUTO (Path6-7 Only FixedRPM)", group = "Autonomous")
@Configurable
public class lastpathonlyshoot extends OpMode {

    private Servo pushServo;
    private Servo sorterServo;
    private RevColorSensorV3 colorSensor;
    private DcMotorEx shooter1;
    private DcMotor aimMotor;
    private DcMotor intakeMotor;
    private Servo shooterServo;
    private Limelight3A limelight;

    private final double[] INTAKE_POS = {0.00, 0.32, 0.49};
    private final double[] LAUNCH_POS = {0.22, 0.40, 0.58};
    private boolean launchMode = false;

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

    // Keep the same fixed velocity style as your PATH_6
    private static final double FIXED_RPM_67 = 1000;
    private double targetShooterRPM = FIXED_RPM_67;
    private boolean useFixedShooterVelocity = true;

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

    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;

    private ElapsedTime shootDelayTimer = new ElapsedTime();
    private boolean shootDelayStarted = false;

    // keep same delay constant you used in the big code
    private static final double PATH3_DELAY_SECONDS = 4.5;

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

    // Only Path6 and Path7
    public enum PathState {
        PATH_6, PATH_7, DONE
    }

    private PathState pathState = PathState.PATH_6;
    private Paths paths;

    private boolean aimHoldEnabled = false;
    private int aimTargetTicks = 0;

    public static class Paths {
        public PathChain Path6, Path7;

        public Paths(Follower follower) {

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(28.000, 50.000), new Pose(47.000, 67.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(47.000, 67.000), new Pose(28.000, 67.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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

    // Kept for compatibility (but not used when useFixedShooterVelocity=true)
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

            case PATH_6:
                // Aim in init exactly like your original "init aim complete" pattern, but target=+70
                if (!initAimComplete) {
                    aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    aimTargetTicks = 70;

                    aimMotor.setTargetPosition(aimTargetTicks);
                    aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    aimMotor.setPower(AIM_OFFSET_POWER);
                    aimHoldEnabled = true;

                    initAimComplete = true;
                }

                if (!follower.isBusy()) {
                    useForceIndex = false;

                    // Start Path6
                    follower.followPath(paths.Path6, true);
                    follower.setMaxPower(1);

                    // Fixed velocity for Path6
                    useFixedShooterVelocity = true;
                    setShooterVelocity(FIXED_RPM_67);

                    setPathState(PathState.PATH_7);
                }
                break;

            case PATH_7:
                if (!follower.isBusy()) {
                    // While we're about to shoot, keep same fixed velocity too
                    useFixedShooterVelocity = true;
                    setShooterVelocity(FIXED_RPM_67);

                    setLaunchMode();
                    startLaunch(3);

                    if (!shootDelayStarted) {
                        shootDelayTimer.reset();
                        shootDelayStarted = true;
                    }

                    // After shooting delay, run Path7 (same structure as your big code)
                    if (shootDelayTimer.seconds() >= PATH3_DELAY_SECONDS) {
                        resetBallState();
                        setIntakeMode();

                        follower.followPath(paths.Path7, true);
                        intake(0);
                        follower.setMaxPower(0.4);

                        // stop shooter after starting Path7 (same idea as your original)
                        stopShooter();

                        shootDelayStarted = false;
                        setPathState(PathState.DONE);
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

            // Keep your BLUE start pose
            follower.setPose(new Pose(59.0, 15.0, Math.toRadians(180)));

            paths = new Paths(follower);

            telemetry.addLine("✅ 🔵 BLUE AUTO Path6-7 ONLY");
            telemetry.addLine("🎯 Init aim: 70 ticks");
            telemetry.addLine("🚀 Fixed RPM (6&7): " + (int) FIXED_RPM_67);
            telemetry.update();

        } catch (Exception e) {
            telemetry.addLine("❌ Init Error: " + e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void start() {
        opModeTimer.reset();
        setPathState(PathState.PATH_6);

        // same preload behavior as your original
        ballCount = 3;
        intakeIndex = 2;
        ballDetected = false;
        intakeFull = true;
        initAimComplete = false;

        ballTimer.reset();
        servoMoveTimer.reset();
        forceIndexTimer.reset();

        shootDelayStarted = false;
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

                // only update Limelight RPM when NOT fixed
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

        if (ballPresent && !ballDetected && !launchMode && !intakeFull && !useForceIndex
                && ballTimer.seconds() > BALL_COOLDOWN) {
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
