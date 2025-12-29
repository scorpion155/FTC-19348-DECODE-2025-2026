//package org.firstinspires.ftc.teamcode;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "🔴 RED - PGP", group = "Autonomous")
//@Configurable
//public class rpgp extends OpMode {
//    private Servo pushServo;
//    private Servo sorterServo;
//    private DcMotorEx shooter1;
//    private DcMotor aimMotor;
//    private DcMotor intakeMotor;
//    private Servo shooterServo;
//    private Limelight3A limelight;
//
//    private final double[] INTAKE_POS = {0.22, 0.40, 0.58};
//    private final double[] LAUNCH_POS = {0.00, 0.32, 0.49};
//    private boolean launchMode = false;
//
//    private boolean shooterSpunUp = false;
//    private int launchIndex = 0;
//
//    private static final double AIM_OFFSET_POWER = 0.5;
//
//    private static final double CAMERA_HEIGHT = 0.3;
//    private static final double TARGET_HEIGHT = 1.10;
//    private static final double CAMERA_ANGLE = 15;
//
//    private static final double SERVO_MIN = 0.3;
//    private static final double SERVO_MAX = 0.61;
//    private static final double DIST_MIN = 0.4;
//    private static final double DIST_MAX = 3.0;
//
//    private static final double TARGET_VELOCITY = 2000;
//    private static final double VELOCITY_TOLERANCE = 50;
//    private static final int AIM_TOLERANCE_TICKS = 5;
//
//    private ElapsedTime velocityStableTimer = new ElapsedTime();
//    private ElapsedTime aimStableTimer = new ElapsedTime();
//    private static final double VELOCITY_STABLE_TIME = 0.3;
//    private static final double AIM_STABLE_TIME = 0.2;
//    private boolean velocityStable = false;
//    private boolean aimStable = false;
//
//    static final int MAX_BALLS = 3;
//    int ballCount = 0;
//    int intakeIndex = 0;
//
//    private static final double PUSH_HOME = 0.0;
//    private static final double SETTLE_TIME = 0.45;
//    private static final double PUSH_OUT_TIME = 0.1;
//    private static final double PUSH_HOME_TIME = 0.2;
//
//    private Follower follower;
//    private ElapsedTime pathTimer, opModeTimer;
//
//    private ElapsedTime path3Timer = new ElapsedTime();
//    private boolean path3DelayStarted = false;
//    private static final double PATH3_DELAY_SECONDS = 4.5;
//
//    private static final double PATH1_DELAY_SECONDS = 8;
//    public static boolean USE_PATH3 = true;
//
//    private boolean launching = false;
//    private boolean initAimComplete = false;
//
//    private ElapsedTime intakeTimer = new ElapsedTime();
//    private static final double BALL_INTAKE_INTERVAL = 0.8;
//    private int ballsToIntake = 0;
//    private boolean intaking = false;
//
//    enum LaunchState {
//        IDLE, ROTATE, WAIT_SETTLE, PUSH_UP, WAIT_UP, PUSH_DOWN, WAIT_DOWN
//    }
//
//    LaunchState launchState = LaunchState.IDLE;
//    ElapsedTime launchTimer = new ElapsedTime();
//
//    static final double PUSH_UP_POS = 1;
//    static final double PUSH_DOWN_POS = 0.0;
//
//    static final double PUSH_UP_TIME = 0.25;
//    static final double PUSH_DOWN_TIME = 0.35;
//
//    public enum PathState {
//        PATH_1, PATH_2, PATH_3, PATH_4, PATH_5, PATH_6, PATH_7, PATH_8, PATH_9, DONE
//    }
//
//    private PathState pathState = PathState.PATH_1;
//    private Paths paths;
//
//    private boolean aimHoldEnabled = false;
//    private int aimTargetTicks = 0;
//
//    public static class Paths {
//        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;
//
//        public Paths(Follower follower) {
//
//            Path1 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(59.000, 129.000), new Pose(50.000, 113.000)))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .build();
//
//            Path2 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(50.000, 113.000), new Pose(28.000, 113.000)))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .build();
//
//            Path3 = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(28.000, 113.000),
//                            new Pose(49.000, 110.000),
//                            new Pose(58.000, 118.000)))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(250))
//                    .build();
//
//            Path4 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(58.000, 118.000), new Pose(47.000, 97.000))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(180))
//                    .build();
//
//            Path5 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(47.000, 97.000), new Pose(28.000, 97.000))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .build();
//
//            Path6 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(28.000, 94.000), new Pose(47.000, 77.000)))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .build();
//
//            Path7 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(47.000, 77.000), new Pose(28.000, 77.000)))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .build();
//
//            Path8 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(28.000, 77.000), new Pose(47.000, 77.000)))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .build();
//
//            Path9 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(47.000, 77.000), new Pose(47.000, 77.000)))
//                    .setTangentHeadingInterpolation()
//                    .build();
//        }
//    }
//
//    private void resetBallState() {
//        ballCount = 0;
//        intakeIndex = 0;
//        launchMode = false;
//        launching = false;
//        intaking = false;
//        ballsToIntake = 0;
//        velocityStable = false;
//        aimStable = false;
//        sorterServo.setPosition(INTAKE_POS[0]);
//        velocityStableTimer.reset();
//        aimStableTimer.reset();
//    }
//
//    private void startIntake(int balls) {
//        ballsToIntake = balls;
//        intaking = true;
//        intakeTimer.reset();
//    }
//
//    private void updateIntakeSequence() {
//        if (!intaking || ballsToIntake <= 0) return;
//
//        int ballsIntaked = (int) (intakeTimer.seconds() / BALL_INTAKE_INTERVAL);
//
//        if (ballsIntaked > 0 && ballsIntaked <= ballsToIntake && ballCount < ballsIntaked) {
//            ballCount = ballsIntaked;
//            intakeIndex = ballCount - 1;
//            sorterServo.setPosition(INTAKE_POS[intakeIndex]);
//
//            if (ballCount >= ballsToIntake) {
//                intaking = false;
//                ballsToIntake = 0;
//            }
//        }
//    }
//
//    public void updateStateMachine() {
//        switch (pathState) {
//
//            case PATH_1:
//                if (!initAimComplete) {
//                    aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    aimTargetTicks = 95;
//                    aimMotor.setTargetPosition(aimTargetTicks);
//                    aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    aimMotor.setPower(AIM_OFFSET_POWER);
//                    aimHoldEnabled = true;
//                    initAimComplete = true;
//                }
//
//                if (!follower.isBusy()) {
//                    setShooterVelocity(TARGET_VELOCITY);
//
//                    if (!path3DelayStarted) {
//                        path3Timer.reset();
//                        path3DelayStarted = true;
//                        shooterSpunUp = false;
//                    }
//
//                    if (path3Timer.seconds() >= 3.0 && !shooterSpunUp) {
//                        setLaunchMode();
//                        startLaunch(3);
//                        shooterSpunUp = true;
//                    }
//
//                    if (path3Timer.seconds() >= PATH1_DELAY_SECONDS) {
//                        setShooterVelocity(0);
//                        resetBallState();
//
//                        follower.followPath(paths.Path1, true);
//                        follower.setMaxPower(1);
//                        setPathState(PathState.PATH_2);
//                        intake(1);
//                        path3DelayStarted = false;
//                        shooterSpunUp = false;
//                    }
//                }
//                break;
//
//            case PATH_2:
//                if (!follower.isBusy()) {
//                    setIntakeMode();
//                    aim(0);
//
//                    startIntake(3);
//
//                    follower.followPath(paths.Path2, true);
//                    intake(1);
//                    follower.setMaxPower(0.4);
//                    setPathState(USE_PATH3 ? PathState.PATH_3 : PathState.PATH_4);
//                }
//                break;
//
//            case PATH_3:
//                if (!follower.isBusy()) {
//                    intaking = false;
//                    resetBallState();
//                    intake(0);
//                    setShooterVelocity(TARGET_VELOCITY);
//                    follower.followPath(paths.Path3, true);
//                    setPathState(PathState.PATH_4);
//                    follower.setMaxPower(1);
//                    path3DelayStarted = false;
//                    aim(-45);
//                }
//                break;
//
//            case PATH_4:
//                if (!follower.isBusy()) {
//                    setLaunchMode();
//                    startLaunch(3);
//
//                    if (!path3DelayStarted) {
//                        path3Timer.reset();
//                        path3DelayStarted = true;
//                    }
//
//                    if (path3Timer.seconds() >= PATH3_DELAY_SECONDS) {
//                        resetBallState();
//                        setShooterVelocity(0);
//                        follower.followPath(paths.Path4, true);
//                        setPathState(PathState.PATH_5);
//                        follower.setMaxPower(1);
//                        path3DelayStarted = false;
//                    }
//                }
//                break;
//
//            case PATH_5:
//                if (!follower.isBusy()) {
//                    setIntakeMode();
//
//                    startIntake(3);
//
//                    follower.followPath(paths.Path5, true);
//                    follower.setMaxPower(0.4);
//                    setPathState(PathState.PATH_6);
//                    intake(1);
//                    aim(0);
//                }
//                break;
//
//            case PATH_6:
//                if (!follower.isBusy()) {
//                    intaking = false;
//                    setShooterVelocity(TARGET_VELOCITY);
//                    follower.followPath(paths.Path6, true);
//                    setPathState(PathState.PATH_7);
//                    follower.setMaxPower(1);
//                    aim(70);
//                }
//                break;
//
//            case PATH_7:
//                if (!follower.isBusy()) {
//                    setLaunchMode();
//                    startLaunch(3);
//
//                    if (!path3DelayStarted) {
//                        path3Timer.reset();
//                        path3DelayStarted = true;
//                    }
//
//                    if (path3Timer.seconds() >= PATH3_DELAY_SECONDS) {
//                        resetBallState();
//                        setShooterVelocity(0);
//                        setIntakeMode();
//
//                        startIntake(3);
//
//                        follower.followPath(paths.Path7, true);
//                        intake(1);
//                        follower.setMaxPower(0.4);
//                        setPathState(PathState.PATH_8);
//                        path3DelayStarted = false;
//                    }
//                }
//                break;
//
//            case PATH_8:
//                if (!follower.isBusy()) {
//                    intaking = false;
//                    resetBallState();
//                    intake(0);
//                    setShooterVelocity(TARGET_VELOCITY);
//                    follower.followPath(paths.Path8, true);
//                    setPathState(PathState.PATH_9);
//                    follower.setMaxPower(1);
//                    path3DelayStarted = false;
//                }
//                break;
//
//            case PATH_9:
//                if (!follower.isBusy()) {
//                    setLaunchMode();
//                    startLaunch(3);
//
//                    if (!path3DelayStarted) {
//                        path3Timer.reset();
//                        path3DelayStarted = true;
//                    }
//
//                    if (path3Timer.seconds() >= PATH3_DELAY_SECONDS) {
//                        follower.followPath(paths.Path9, true);
//                        setPathState(PathState.DONE);
//                        setShooterVelocity(0);
//                        follower.setMaxPower(1);
//                        path3DelayStarted = false;
//                    }
//                }
//                break;
//
//            case DONE:
//                if (!follower.isBusy()) telemetry.addLine("AUTO COMPLETE ✔");
//                break;
//        }
//    }
//
//    public void setPathState(PathState newState) {
//        pathState = newState;
//        if (pathTimer != null) pathTimer.reset();
//    }
//
//    @Override
//    public void init() {
//        try {
//            pushServo = hardwareMap.get(Servo.class, "pushservo");
//            pushServo.setPosition(PUSH_HOME);
//
//            shooterServo = hardwareMap.get(Servo.class, "shooterServo");
//            shooterServo.setPosition(SERVO_MIN);
//
//            sorterServo = hardwareMap.get(Servo.class, "sortservo");
//            sorterServo.setPosition(INTAKE_POS[0]);
//
//            shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
//            shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
//
//            aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
//            aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            aimMotor.setDirection(DcMotor.Direction.REVERSE);
//            aimMotor.setPower(0);
//
//            limelight = hardwareMap.get(Limelight3A.class, "limelight");
//            limelight.pipelineSwitch(3);
//            limelight.start();
//
//            pathTimer = new ElapsedTime();
//            opModeTimer = new ElapsedTime();
//
//            follower = Constants.createFollower(hardwareMap);
//            follower.setPose(new Pose(59.0, 129.0, Math.toRadians(180)));
//
//            paths = new Paths(follower);
//
//            telemetry.addLine("✅ 🔴 RED - ALL PGP");
//            telemetry.addLine("⚠️ PRELOAD: 0=PURPLE, 1=GREEN, 2=PURPLE");
//            telemetry.addLine("📊 ALL LAUNCHES = PGP");
//            telemetry.update();
//
//        } catch (Exception e) {
//            telemetry.addLine("❌ Init Error: " + e.getMessage());
//            telemetry.update();
//        }
//    }
//
//    @Override
//    public void start() {
//        opModeTimer.reset();
//        setPathState(PathState.PATH_1);
//
//        ballCount = 3;
//        intakeIndex = 2;
//        initAimComplete = false;
//
//        velocityStableTimer.reset();
//        aimStableTimer.reset();
//    }
//
//    @Override
//    public void loop() {
//        try {
//            updateIntakeSequence();
//
//            if (launchMode) {
//                sorterServo.setPosition(LAUNCH_POS[intakeIndex]);
//            }
//
//            updateStabilityChecks();
//
//            if (aimHoldEnabled) {
//                if (aimMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
//                    aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    aimMotor.setTargetPosition(aimTargetTicks);
//                    aimMotor.setPower(AIM_OFFSET_POWER);
//                }
//            }
//
//            LLResult result = limelight.getLatestResult();
//            if (result != null && result.isValid()) {
//                double ty = result.getTy();
//                double angleRad = Math.toRadians(CAMERA_ANGLE + ty);
//                double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleRad);
//
//                double servoPos = SERVO_MIN + (distance - DIST_MIN) / (DIST_MAX - DIST_MIN) * (SERVO_MAX - SERVO_MIN);
//                servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
//                shooterServo.setPosition(servoPos);
//            }
//
//            follower.update();
//            updateStateMachine();
//            updateLauncher();
//
//            telemetry.addData("🔴 Alliance", "RED - PGP");
//            telemetry.addData("State", pathState);
//            telemetry.addData("Balls", ballCount);
//            telemetry.addData("Idx", intakeIndex);
//            telemetry.addData("Intaking", intaking);
//            telemetry.addData("AimPos", aimMotor.getCurrentPosition());
//            telemetry.addData("Velocity", shooter1.getVelocity());
//            telemetry.addData("VelStable", velocityStable);
//            telemetry.addData("AimStable", aimStable);
//            telemetry.update();
//
//        } catch (Exception e) {
//            telemetry.addLine("Loop Error: " + e.getMessage());
//            telemetry.update();
//        }
//    }
//
//    private void updateStabilityChecks() {
//        double currentVelocity = shooter1.getVelocity();
//        double velocityError = Math.abs(currentVelocity - TARGET_VELOCITY);
//
//        if (velocityError <= VELOCITY_TOLERANCE && TARGET_VELOCITY > 0) {
//            if (velocityStableTimer.seconds() >= VELOCITY_STABLE_TIME) {
//                velocityStable = true;
//            }
//        } else {
//            velocityStable = false;
//            velocityStableTimer.reset();
//        }
//
//        int aimError = Math.abs(aimMotor.getCurrentPosition() - aimTargetTicks);
//
//        if (aimError <= AIM_TOLERANCE_TICKS && aimHoldEnabled) {
//            if (aimStableTimer.seconds() >= AIM_STABLE_TIME) {
//                aimStable = true;
//            }
//        } else {
//            aimStable = false;
//            aimStableTimer.reset();
//        }
//    }
//
//    public void setShooterVelocity(double velocity) {
//        shooter1.setVelocity(velocity);
//        velocityStable = false;
//        velocityStableTimer.reset();
//    }
//
//    public void intake(double power) {
//        intakeMotor.setPower(power);
//    }
//
//    public void aim(int target) {
//        aimTargetTicks = target;
//        aimHoldEnabled = true;
//        aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        aimMotor.setTargetPosition(aimTargetTicks);
//        aimMotor.setPower(AIM_OFFSET_POWER);
//        aimStable = false;
//        aimStableTimer.reset();
//    }
//
//    public void startLaunch(int balls) {
//        if (launching || balls <= 0) return;
//        launching = true;
//        launchMode = true;
//    }
//
//    public void updateLauncher() {
//        if (!launchMode) {
//            launchState = LaunchState.IDLE;
//            return;
//        }
//
//        switch (launchState) {
//            case IDLE:
//                sorterServo.setPosition(LAUNCH_POS[intakeIndex]);
//                pushServo.setPosition(PUSH_DOWN_POS);
//                launchState = LaunchState.ROTATE;
//                velocityStableTimer.reset();
//                aimStableTimer.reset();
//                break;
//
//            case ROTATE:
//                sorterServo.setPosition(LAUNCH_POS[intakeIndex]);
//                launchTimer.reset();
//                launchState = LaunchState.WAIT_SETTLE;
//                break;
//
//            case WAIT_SETTLE:
//                if (launchTimer.seconds() > SETTLE_TIME && velocityStable && aimStable) {
//                    launchState = LaunchState.PUSH_UP;
//                }
//                break;
//
//            case PUSH_UP:
//                pushServo.setPosition(PUSH_UP_POS);
//                launchTimer.reset();
//                launchState = LaunchState.WAIT_UP;
//                velocityStableTimer.reset();
//                break;
//
//            case WAIT_UP:
//                if (launchTimer.seconds() > PUSH_UP_TIME) {
//                    launchState = LaunchState.PUSH_DOWN;
//                }
//                break;
//
//            case PUSH_DOWN:
//                pushServo.setPosition(PUSH_DOWN_POS);
//                launchTimer.reset();
//                launchState = LaunchState.WAIT_DOWN;
//                break;
//
//            case WAIT_DOWN:
//                if (launchTimer.seconds() > PUSH_DOWN_TIME) {
//                    intakeIndex = (intakeIndex + 1) % LAUNCH_POS.length;
//                    launchState = LaunchState.IDLE;
//                }
//                break;
//        }
//    }
//
//    public void setIntakeMode() {
//        launchMode = false;
//        sorterServo.setPosition(INTAnb KE_POS[intakeIndex]);
//    }
//
//    public void setLaunchMode() {
//        launchMode = true;
//        sorterServo.setPosition(LAUNCH_POS[intakeIndex]);
//    }
//
//    @Override
//    public void stop() {
//        if (aimMotor != null) aimMotor.setPower(0);
//        if (shooter1 != null) shooter1.setVelocity(0);
//        if (intakeMotor != null) intakeMotor.setPower(0);
//    }
//}