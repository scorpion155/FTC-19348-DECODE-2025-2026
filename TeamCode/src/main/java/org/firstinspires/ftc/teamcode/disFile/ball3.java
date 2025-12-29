package org.firstinspires.ftc.teamcode.disFile;

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled

@Autonomous(name = "PedroPathing GPP Only", group = "Autonomous")
@Configurable
public class ball3 extends OpMode {
    private Servo pushServo;
    private Servo sorterServo;
    private RevColorSensorV3 colorSensor;
    private DcMotor shooter1;
    private DcMotor aimMotor;
    private DcMotor intakeMotor;
    private Servo shooterServo;

    // Original positions from your code
    private final double[] INTAKE_POS = {0.22, 0.40, 0.58};
    private final double[] LAUNCH_POS = {0.00, 0.32, 0.49};

    // Push servo positions
    private static final double PUSH_OUT = 1.0;
    private static final double PUSH_HOME = 0.0;
    private static final double PUSH_OUT_TIME = 0.1;
    private static final double PUSH_HOME_TIME = 0.2;

    // Shooter power
    private static final double SHOOTER_POWER = 0.85;

    // Sorter timing
    private static final double SORTER_SETTLE_TIME = 0.25;

    // === Color classification ===
    private enum BallColor {
        UNKNOWN, GREEN, PURPLE
    }

    // === GPP sequence tracking ===
    private int gppStage = 0;
    private boolean sorterSearching = true;
    private ElapsedTime sorterTimer = new ElapsedTime();
    private int intakeIndex = 0;

    // === Push/Fire state ===
    private enum PushState {
        IDLE,
        PUSH_OUT,
        PUSH_BACK,
        SETTLE
    }

    private PushState pushState = PushState.IDLE;
    private ElapsedTime pushTimer = new ElapsedTime();
    private int gppBallsLeft = 0;

    // === PATH FOLLOWER ===
    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;
    private ElapsedTime path3Timer = new ElapsedTime();
    private boolean path3DelayStarted = false;
    private static final double PATH3_DELAY_SECONDS = 6.0;
    public static boolean USE_PATH3 = true;

    public enum PathState {
        PATH_1, PATH_2, PATH_3, PATH_4, PATH_5, DONE
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

    // === STATE MACHINE ===
    public void updateStateMachine() {
        switch (pathState) {
            case PATH_1:
                follower.followPath(paths.Path1, true);
                intake(0);
                follower.setMaxPower(1);
                setPathState(PathState.PATH_2);
                break;

            case PATH_2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    intake(1);
                    follower.setMaxPower(0.4);
                    setPathState(USE_PATH3 ? PathState.PATH_3 : PathState.PATH_4);
                }
                break;

            case PATH_3:
                if (!follower.isBusy()) {
                    intakeIndex = 0;
                    sorterServo.setPosition(INTAKE_POS[intakeIndex]);
                    intake(0);
                    follower.followPath(paths.Path3, true);
                    setPathState(PathState.PATH_4);
                    follower.setMaxPower(1);
                    path3DelayStarted = false;
                }
                break;

            case PATH_4:
                if (!follower.isBusy()) {
                    // Start GPP color sorting and firing
                    sorterSearching = true;
                    gppStage = 0;
                    sorterTimer.reset();

                    if (!path3DelayStarted) {
                        path3Timer.reset();
                        path3DelayStarted = true;
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
                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
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
        shooterServo.setPosition(0.3);
        sorterServo = hardwareMap.get(Servo.class, "sortservo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setDirection(DcMotor.Direction.REVERSE);

        sorterServo.setPosition(INTAKE_POS[0]);

        PathConstraints fast = new PathConstraints(60, 120);
        PathConstraints slow = new PathConstraints(20, 60);

        pathTimer = new ElapsedTime();
        opModeTimer = new ElapsedTime();
        sorterTimer = new ElapsedTime();
        pushTimer = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower, slow);
        follower.setPose(new Pose(59.0, 15.0, Math.toRadians(180)));

        telemetry.addLine("Init complete.");
    }

    @Override
    public void start() {
        opModeTimer.reset();
        pathTimer.reset();
        sorterTimer.reset();
        pushTimer.reset();
        setPathState(PathState.PATH_1);
    }

    @Override
    public void loop() {
        // Read raw color values
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        // Run GPP color-order search (works during path following)
        updateGPPSequence();

        // Handle push + shooter (fires during path following)
        updateLauncher();

        // Path following
        follower.update();
        updateStateMachine();

        Pose p = follower.getPose();

        telemetry.addData("State", pathState);
        telemetry.addData("X", p.getX());
        telemetry.addData("Y", p.getY());
        telemetry.addData("Heading°", Math.toDegrees(p.getHeading()));
        telemetry.addData("Path Time", pathTimer.seconds());
        telemetry.addData("R", r);
        telemetry.addData("G", g);
        telemetry.addData("B", b);
        telemetry.addData("GPP Stage", gppStage);
        telemetry.addData("Intake Index", intakeIndex);
        telemetry.addData("Balls Left", gppBallsLeft);
        telemetry.addData("Push State", pushState);
        telemetry.update();
    }

    /**
     * Detects color from the Rev Color Sensor.
     * Tune these thresholds for your specific Green and Purple game elements.
     */
    private BallColor readBallColor() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        // GREEN detection: green channel dominant
        if (g > r && g > b && g > 80) {
            return BallColor.GREEN;
        }
        // PURPLE detection: strong red AND blue, weak green
        else if (b > g && r > g && r > 60 && b > 60) {
            return BallColor.PURPLE;
        }
        // Default unknown
        else {
            return BallColor.UNKNOWN;
        }
    }

    /**
     * Main GPP sequence state machine.
     * Rotates sorter through INTAKE_POS until G-P-P is found.
     */
    private void updateGPPSequence() {
        if (!sorterSearching) return;

        // Wait for sorter to settle at current position
        if (sorterTimer.seconds() < SORTER_SETTLE_TIME) return;

        BallColor color = readBallColor();

        switch (gppStage) {
            case 0:
                // Waiting for Green
                if (color == BallColor.GREEN) {
                    gppStage = 1;
                    sorterTimer.reset();
                } else {
                    // Not green, rotate to next slot
                    intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
                    sorterServo.setPosition(INTAKE_POS[intakeIndex]);
                    sorterTimer.reset();
                }
                break;

            case 1:
                // Waiting for first Purple
                if (color == BallColor.PURPLE) {
                    gppStage = 2;
                    sorterTimer.reset();
                } else if (color == BallColor.GREEN) {
                    gppStage = 1;
                    sorterTimer.reset();
                } else {
                    gppStage = 0;
                    intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
                    sorterServo.setPosition(INTAKE_POS[intakeIndex]);
                    sorterTimer.reset();
                }
                break;

            case 2:
                // Waiting for second Purple
                if (color == BallColor.PURPLE) {
                    // SUCCESS: G-P-P sequence found!
                    sorterSearching = false;
                    startGPPFire();
                } else if (color == BallColor.GREEN) {
                    gppStage = 1;
                    sorterTimer.reset();
                } else {
                    gppStage = 0;
                    intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
                    sorterServo.setPosition(INTAKE_POS[intakeIndex]);
                    sorterTimer.reset();
                }
                break;
        }
    }

    /**
     * Called when G-P-P sequence is found.
     */
    private void startGPPFire() {
        shooter1.setPower(SHOOTER_POWER);
        gppBallsLeft = 3;
        pushState = PushState.PUSH_OUT;
        pushTimer.reset();
    }

    /**
     * Push servo state machine.
     * Fires 3 balls using LAUNCH_POS positions.
     */
    public void updateLauncher() {
        if (gppBallsLeft <= 0 && pushState == PushState.IDLE) {
            return;
        }

        switch (pushState) {
            case PUSH_OUT:
                pushServo.setPosition(PUSH_OUT);
                if (pushTimer.seconds() >= PUSH_OUT_TIME) {
                    pushState = PushState.PUSH_BACK;
                    pushTimer.reset();
                }
                break;

            case PUSH_BACK:
                pushServo.setPosition(PUSH_HOME);
                if (pushTimer.seconds() >= PUSH_HOME_TIME) {
                    gppBallsLeft--;
                    if (gppBallsLeft > 0) {
                        // Move sorter to next launch slot
                        intakeIndex = (intakeIndex + 1) % LAUNCH_POS.length;
                        sorterServo.setPosition(LAUNCH_POS[intakeIndex]);
                        pushState = PushState.PUSH_OUT;
                        pushTimer.reset();
                    } else {
                        // Done firing all 3 balls
                        shooter1.setPower(0.0);
                        pushState = PushState.IDLE;

                        // Resume searching
                        sorterSearching = true;
                        gppStage = 0;
                        sorterTimer.reset();
                    }
                }
                break;

            case SETTLE:
                break;

            case IDLE:
                break;
        }
    }

    public void driveForward(double power) {
        shooter1.setPower(power);
    }

    public void intake(double power) {
        intakeMotor.setPower(power);
    }

    @Override
    public void stop() {
        shooter1.setPower(0);
        pushServo.setPosition(PUSH_HOME);
        telemetry.addLine("Stopped.");
        telemetry.update();
    }
}