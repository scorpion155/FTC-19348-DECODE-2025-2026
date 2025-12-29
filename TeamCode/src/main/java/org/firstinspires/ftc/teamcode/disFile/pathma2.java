package org.firstinspires.ftc.teamcode.disFile;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled

@Autonomous(name = "PedroPathing 3 ball intake fix v2", group = "Autonomous")
@Configurable
public class pathma2 extends OpMode {
    private Servo pushServo;
    private Servo sorterServo;
    private RevColorSensorV3 colorSensor;
    private DcMotor shooter1;
    private DcMotor aimMotor;
    private DcMotor intakeMotor;
    private Servo shooterServo;
    private Limelight3A limelight;

    //    Path fastPath, slowPath;
    boolean slowPathStarted = false;
    PathConstraints slow;

    private final double[] INTAKE_POS ={0.22, 0.40, 0.58};// {0.00, 0.32, 0.49};//;  //
    private final double[] LAUNCH_POS =   {0.00, 0.32, 0.49};// {0.22, 0.40, 0.58};// //
    private boolean launchMode = false;

    private boolean shooterSpunUp = false;



    //    private final double[] INTAKE_POS = {0.22, 0.40, 0.58};
    private boolean xPressed = false;

    //    private final double[] LAUNCH_POS = {0.22, 0.40, 0.58};
    private int launchIndex = 0;

    private static final double DEADZONE = 2.0;
    private static final double MAX_TURRET_POWER = 0.40;
    private static final double FILTER_ALPHA = 0.85;
    private static final double KP = 0.03;
    private double filteredTx = 0;

    private static final double CAMERA_HEIGHT = 0.3;
    private static final double TARGET_HEIGHT = 1.10;
    private static final double CAMERA_ANGLE  = 15;

    private static final double SERVO_MIN = 0.3;
    private static final double SERVO_MAX = 0.61;
    private static final double DIST_MIN  = 0.4;
    private static final double DIST_MAX  = 3.0;

    private static final double SHOOTER_PWR_MIN = 0.65;
    private static final double SHOOTER_PWR_MAX = 1.00;
    private double shooterPowerTarget = 0;

    //    static final int MAX_BALLS = 3;
//    int ballCount = 0;
    int intakeIndex = 0;
    boolean ballDetected = false;
//    boolean intakeFull = false;

    static final int MAX_BALLS = 3;

    int ballCount = 0;
    int slotIndex = 0;

    boolean ballLocked = false;
    boolean intakeFull = false;

//    ElapsedTime ballTimer = new ElapsedTime();

    static final int BALL_DETECT = 320;
//    static final double BALL_REARM_TIME = 0.35;
//    static final double BALL_REARM_TIME = 0.45;


    //    static final int BALL_DETECT = 350;
    static final int BALL_CLEAR  = 200;

//    ElapsedTime ballTimer = new ElapsedTime();

    static final double BALL_REARM_TIME = 0.35; // seconds
    static final double SETTLE_TIME = 0.45; // was 0.35


    private static final double PUSH_OUT = 1;
    private static final double PUSH_HOME = 0.0;

    private static final double PUSH_OUT_TIME  = 0.1;
    private static final double PUSH_HOME_TIME = 0.2;
    private static final double SORTER_MOVE_TIME = 0.60;

    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;

    private ElapsedTime path3Timer = new ElapsedTime();
    private boolean path3DelayStarted = false;
    private static final double PATH3_DELAY_SECONDS = 4.5;// 5 seconds delay

    private ElapsedTime path1Timer = new ElapsedTime();
    private boolean path1DelayStarted = false;
    private static final double PATH1_DELAY_SECONDS = 8;
    public static boolean USE_PATH3 = true; // toggle to enter other course

    private boolean launching = false;
    private int ballsToLaunch = 0;

    ElapsedTime ballTimer = new ElapsedTime();
    static final double BALL_COOLDOWN = 0.35; // seconds

    private enum PushState {
        IDLE,
        PUSH_OUT,
        PUSH_BACK,
        MOVE_SORTER,
        SETTLE
    }

    enum LaunchState {
        IDLE,
        ROTATE,
        WAIT_SETTLE,   // ⭐ NEW
        PUSH_UP,
        WAIT_UP,
        PUSH_DOWN,
        WAIT_DOWN
    }

//    static final double SETTLE_TIME = 0.35; // seconds (tune 0.25–0.5)


    LaunchState launchState = LaunchState.IDLE;
    ElapsedTime launchTimer = new ElapsedTime();

    static final double PUSH_UP_POS = 1;
    static final double PUSH_DOWN_POS = 0.0;

    static final double PUSH_UP_TIME = 0.25;     // seconds
    static final double PUSH_DOWN_TIME = 0.35;
    static final double ROTATE_TIME = 0.40;


    private PushState pushState = PushState.IDLE;
    private ElapsedTime pushTimer = new ElapsedTime();

    public enum PathState {
        PATH_1, PATH_2, PATH_3, PATH_4, PATH_5, PATH_6, PATH_7, PATH_8, PATH_9, DONE
    }

    private PathState pathState = PathState.PATH_1;
    private Paths paths;

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

        public Paths(Follower follower, PathConstraints slow) {

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

    // === STATE MACHINE ===
    public void updateStateMachine() {
        switch (pathState) {

            case PATH_1:
                if (!follower.isBusy()) {

                    // 🔥 START SHOOTER AT FULL SPEED
                    driveForward(-1); // shooter1 full power

                    // ⏱️ START SPIN-UP TIMER
                    if (!path3DelayStarted) {
                        path3Timer.reset();
                        path3DelayStarted = true;
                        shooterSpunUp = false;
                    }

                    // ⏳ WAIT 3 SECONDS FOR SHOOTER TO REACH SPEED
                    if (path3Timer.seconds() >= 3.0 && !shooterSpunUp) {
                        setLaunchMode();
                        startLaunch(3);
                        shooterSpunUp = true;
                    }

                    // ▶️ AFTER FIRING DELAY → MOVE ON
                    if (path3Timer.seconds() >= PATH1_DELAY_SECONDS) {

                        driveForward(0); // optional: stop shooter after firing
                        setIntakeMode();

                        follower.followPath(paths.Path1, true);
                        follower.setMaxPower(1);
//                        follower.followPath(paths.Path2, true);
                        setPathState(PathState.PATH_2);
                        intake(1);
                        path3DelayStarted = false;
                        shooterSpunUp = false;

                    }
                }
                break;

//            case PATH_1:
//
//                if (!follower.isBusy()) {
//                    driveForward(-1);
//
//                    // 🔴 STOP AND FIRE (same as PATH_4)
//                    setLaunchMode();
//                    startLaunch(3);
//
//                    if (!path3DelayStarted) {
//                        path3Timer.reset();
//                        path3DelayStarted = true;
//                    }
//
//                    // ⏱️ WAIT UNTIL ALL 3 BALLS ARE FIRED
//                    if (path3Timer.seconds() >= PATH3_DELAY_SECONDS) {
//
//                        // ▶️ NOW MOVE TO PATH 2
//                        setIntakeMode();
//                        follower.followPath(paths.Path2, true);
//                        intake(1);
//                        follower.setMaxPower(1);
//                        driveForward(0);
//
//                        setPathState(PathState.PATH_2);
//                        path3DelayStarted = false;
//                    }
//                }
//                break;
//                follower.followPath(paths.Path1, true);
//                setIntakeMode();
//                intake(0);
//                follower.setMaxPower(1);
//                setPathState(PathState.PATH_2);
//                break;

            case PATH_2:
                if (!follower.isBusy()) {

                    setIntakeMode();
                    follower.followPath(paths.Path2, true);
                    intake(1);
                    follower.setMaxPower(0.4);
                    setPathState(USE_PATH3 ? PathState.PATH_3 : PathState.PATH_4);
                }
                break;

            case PATH_3:
                if (!follower.isBusy()) {
                    intakeFull = false;
                    ballDetected = false;
                    intakeIndex = 0;
                    sorterServo.setPosition(INTAKE_POS[intakeIndex]);
                    intake(0);
                    driveForward(-1);
                    follower.followPath(paths.Path3, true);
                    setPathState(PathState.PATH_4);
                    follower.setMaxPower(1);
                    path3DelayStarted = false;



                }
                break;


//            case PATH_3:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path3, true);
//                    intake(0);
//                    setPathState(PathState.PATH_4);
//                    follower.setMaxPower(1);
//                    driveForward(1);
//
//
//
//                    // Reset sorter so it can accept more balls
//                    intakeFull = false;
//                    ballDetected = false;
//                    intakeIndex = 0;
//                    sorterServo.setPosition(INTAKE_POS[intakeIndex]);
//                    intake(0);
//                    updateBallSensor(colorSensor.alpha());
//                    onBallReleased(ballCount);
//
//                }
//                break;

            case PATH_4:
                if (!follower.isBusy()) {

                    setLaunchMode();
                    startLaunch(3);

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
                        follower.followPath(paths.Path4, true);
                        setPathState(PathState.PATH_5);
                        driveForward(0);
                        follower.setMaxPower(1);
                        path3DelayStarted = false;
                    }
                }


                break;

            case PATH_5:
                if (!follower.isBusy()) {
                    setIntakeMode();
                    follower.followPath(paths.Path5, true);
                    follower.setMaxPower(0.4);
                    setPathState(PathState.PATH_6);
                    intake(1);




                }
                break;

            case PATH_6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    setPathState(PathState.PATH_7);
                    driveForward(-1);
                    follower.setMaxPower(1);
                }

            case PATH_7:
                if (!follower.isBusy()) {
                    setLaunchMode();
                    startLaunch(3);

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
                        setIntakeMode();
                        follower.followPath(paths.Path7, true);
                        intake(1);
                        follower.setMaxPower(0.4);
                        setPathState(USE_PATH3 ? PathState.PATH_7 : PathState.PATH_8);
                        follower.followPath(paths.Path7, true);
                        setPathState(PathState.PATH_8);
                        path3DelayStarted = false;
                    }



                }
                break;

            case PATH_8:
                if (!follower.isBusy()) {

                    intakeFull = false;
                    ballDetected = false;
                    intakeIndex = 0;
                    sorterServo.setPosition(INTAKE_POS[intakeIndex]);
                    intake(0);
                    driveForward(-1);
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
                        intakeFull = false;
                        ballDetected = false;
                        intakeIndex = 0;
                        sorterServo.setPosition(INTAKE_POS[intakeIndex]);
                        intake(0);
                    }

                    if (path3Timer.seconds() >= PATH3_DELAY_SECONDS) {
                        follower.followPath(paths.Path9, true);
                        setPathState(PathState.DONE);

                        driveForward(0);
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

        pushServo = hardwareMap.get(Servo.class, "pushservo");
        pushServo.setPosition(PUSH_HOME);
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        shooterServo.setPosition(SERVO_MIN);
        sorterServo = hardwareMap.get(Servo.class, "sortservo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        sorterServo.setPosition(INTAKE_POS[0]);

        PathConstraints fast = new PathConstraints(60, 120);
        PathConstraints slow = new PathConstraints(20, 60);

        pathTimer = new ElapsedTime();
        opModeTimer = new ElapsedTime();
        follower = Constants.createFollower(hardwareMap);

        paths = new Paths(follower, slow);
        follower.setPose(new Pose(59.0, 15.0, Math.toRadians(180)));

        telemetry.addLine("Init complete.");
    }

    @Override
    public void start() {
        opModeTimer.reset();
        setPathState(PathState.PATH_1);
    }


    @Override
    public void loop() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        int brightness = r + g + b;

//        if (brightness > 300) {
//            if (!ballDetected) {
//                intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
//                sorterServo.setPosition(INTAKE_POS[intakeIndex]);
//                ballDetected = true;
//            }
//        } else ballDetected = false;






        if (!launchMode) {
            updateBallSensor(brightness);
        }




//        if (launchMode) {
//            sorterServo.setPosition(LAUNCH_POS[intakeIndex]);
//            return;
//        }

        // ⭐ LOCK SORTER POSITION DURING LAUNCH
//        if (launchMode) {
//            sorterServo.setPosition(LAUNCH_POS[slotIndex]);
//        }

//        if (!launchMode) {
//            updateBallSensor(brightness);
//        }
//
// ⭐ LOCK SORTER DURING LAUNCH (THIS LINE)
        if (launchMode) {
            sorterServo.setPosition(LAUNCH_POS[intakeIndex]);
        }




        double turretPower = 0;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();
            filteredTx = FILTER_ALPHA * filteredTx + (1 - FILTER_ALPHA) * tx;
            if (Math.abs(filteredTx) > DEADZONE) turretPower = filteredTx * KP;

            double angleRad = Math.toRadians(CAMERA_ANGLE + ty);
            double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleRad);

            double servoPos = SERVO_MIN + (distance - DIST_MIN) / (DIST_MAX - DIST_MIN) * (SERVO_MAX - SERVO_MIN);
            servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
            shooterServo.setPosition(servoPos);
        }
        turretPower = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, turretPower));
        aimMotor.setPower(turretPower);

        follower.update();
        updateStateMachine();
        updateLauncher();

        Pose p = follower.getPose();
        telemetry.addData("State", pathState);
        telemetry.addData("X", p.getX());
        telemetry.addData("Y", p.getY());
        telemetry.addData("Heading°", Math.toDegrees(p.getHeading()));
        telemetry.addData("Path Time", pathTimer.seconds());
        telemetry.update();
    }

    public void driveForward(double power) { shooter1.setPower(power); }
    public void intake(double power) { intakeMotor.setPower(power); }

    public void updateBallSensor(int brightness) {

        boolean ballPresent = brightness > BALL_DETECT;

        // Detect new ball by cooldown, NOT by clear
        if (ballPresent
                && !launchMode
                && !intakeFull
                && ballTimer.seconds() > BALL_COOLDOWN) {

            ballTimer.reset();
            ballCount++;

            if (ballCount >= MAX_BALLS) {
                ballCount = MAX_BALLS;
                intakeFull = true;
                intakeIndex = MAX_BALLS - 1;
            } else {
                intakeIndex = ballCount;
            }

            sorterServo.setPosition(INTAKE_POS[intakeIndex]);
        }
    }










    public void onBallReleased(int ballsReleased) {
        ballCount -= ballsReleased;
        if (ballCount < 0) ballCount = 0;
        intakeFull = false;
        ballDetected = false;
    }

    public void startLaunch(int balls) {
        if (launching || balls <= 0) return;

        ballsToLaunch = Math.min(balls, ballCount);
        launching = true;
        launchMode = true;   // ⭐ DISABLE SENSOR CONTROL

        pushState = PushState.PUSH_OUT;
        pushTimer.reset();
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

                launchTimer.reset();   // reset BEFORE waiting
                launchState = LaunchState.WAIT_SETTLE;
                break;



            case WAIT_SETTLE:
                // ⭐ LET BALL FALL & CENTER
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
                    // Advance to next ball AFTER arm is clear
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
        telemetry.addLine("Stopped.");
        telemetry.update();
    }


}
