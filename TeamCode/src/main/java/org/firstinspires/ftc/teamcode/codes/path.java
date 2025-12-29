package org.firstinspires.ftc.teamcode.codes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
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

@Autonomous(name = "PedroPathing Auto – 3 Paths", group = "Autonomous")
@Configurable
public class path extends OpMode {
    private Servo sorterServo;
    private RevColorSensorV3 colorSensor;
    private DcMotor shooter1;
    private DcMotor aimMotor;
    private DcMotor intakeMotor;
    private Servo shooterServo;


    private Limelight3A limelight;



    Path fastPath, slowPath;
    boolean slowPathStarted = false;

    PathConstraints slow;





    private boolean ballDetected = false;
    private final double[] INTAKE_POS = {0.00, 0.32, 0.49};
    private int intakeIndex = 0;
    private boolean xPressed = false;

    // ---- Launch positions ----
    private final double[] LAUNCH_POS = {0.22, 0.40, 0.58};
    private int launchIndex = 0;

    // Turret constants
    private static final double DEADZONE = 2.0;
    private static final double MAX_TURRET_POWER = 0.40;
    private static final double FILTER_ALPHA = 0.85;
    private static final double KP = 0.03;
    private double filteredTx = 0;

    // Limelight distance parameters
    private static final double CAMERA_HEIGHT = 0.3;   // meters
    private static final double TARGET_HEIGHT = 1.10;   // meters
    private static final double CAMERA_ANGLE  = 15;   // degrees

    // Servo mapping
    private static final double SERVO_MIN = 0.3;
    private static final double SERVO_MAX = 0.61;
    private static final double DIST_MIN  = 0.4;
    private static final double DIST_MAX  = 3.0;

    private static final double SHOOTER_PWR_MIN = 0.65;   // close shot
    private static final double SHOOTER_PWR_MAX = 1.00;   // far shot
    private double shooterPowerTarget = 0;

    private Follower follower;

    private ElapsedTime pathTimer, opModeTimer;

    // === STATES ===
    public enum PathState {
        PATH_1,
        PATH_2,
        PATH_3,
        PATH_4,
        PATH_5,
        PATH_6,
        PATH_7,
        PATH_8,
        DONE
    }

    private PathState pathState = PathState.PATH_1;

    // === PATH STORAGE ===
    private Paths paths;

    // === PATH CLASS YOU PROVIDED ===
    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;


        public Paths(Follower follower,PathConstraints slow) {

            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(59.000, 15.000),
                                    new Pose(55.000, 30.000),
                                    new Pose(43.000, 35.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(43.000, 35.000),
                                    new Pose(40.000, 40.000),
                                    new Pose(38.000, 45.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(35.000, 35.000),
                                    new Pose(49.000, 33.000),
                                    new Pose(58.000, 23.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))

                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(58.000, 23.000),
                                    new Pose(59.000, 45.000),
                                    new Pose(45.000, 45.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))


                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.000, 45.000), new Pose(35.000, 45.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(35.000, 45.000), new Pose(46.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(46.000, 60.000), new Pose(34.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(34.000, 60.000), new Pose(46.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();



        }


    }

    // === STATE MACHINE LOGIC ===
    public void updateStateMachine() {

        switch (pathState) {

            case PATH_1:
                follower.followPath(paths.Path1, true);
                intake(1);


                setPathState(PathState.PATH_2);


                break;

            case PATH_2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    follower.setMaxPower(0.3);

                    setPathState(PathState.PATH_3);

                }
                break;

            case PATH_3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    setPathState(PathState.PATH_4);
                    follower.setMaxPower(1);

                    driveForward(1);



                }
                break;

            case PATH_4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    setPathState(PathState.PATH_5);
                    driveForward(0);


                }
                break;
            case PATH_5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    follower.setMaxPower(0.3);
                    setPathState(PathState.PATH_6);
                }
                break;
            case PATH_6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    setPathState(PathState.PATH_7);
                }
                break;
            case PATH_7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, true);
                    setPathState(PathState.PATH_8);
                }
                break;

            case PATH_8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, true);
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                if (!follower.isBusy()) {
                    telemetry.addLine("AUTO COMPLETE ✔");
                }
                break;
        }
    }

    // === STATE CHANGE ===
    public void setPathState(PathState newState) {
        pathState = newState;
        if (pathTimer != null) pathTimer.reset();
    }

    // === INIT ===
    @Override
    public void init() {
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

        PathConstraints fast = new PathConstraints(60, 120); // fast path
        PathConstraints slow = new PathConstraints(20, 60);  // slow path

        telemetry.addLine("Initializing...");

        pathTimer = new ElapsedTime();
        opModeTimer = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);




        // build your 3 paths
        paths = new Paths(follower, slow);



        // Set the robot starting pose to match Path1 start
        follower.setPose(new Pose(59.000, 15.000, Math.toRadians(180)));

        telemetry.addLine("Init complete.");
    }

    // === START ===
    @Override
    public void start() {
        opModeTimer.reset();
        setPathState(PathState.PATH_1);

    }

    // === LOOP ===
    @Override
    public void loop() {


        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        int brightness = r + g + b;



        if (brightness > 300) {   // ball detected threshold (tune if needed)
            if (!ballDetected) { // trigger only once per ball
                intakeIndex = (intakeIndex + 1) % INTAKE_POS.length;
                sorterServo.setPosition(INTAKE_POS[intakeIndex]);
                ballDetected = true;
            }
        } else {
            ballDetected = false;
        }

        double turretPower = 0;
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double tx = result.getTx();
            double ty = result.getTy();

            filteredTx = FILTER_ALPHA * filteredTx + (1 - FILTER_ALPHA) * tx;

            if (Math.abs(filteredTx) > DEADZONE)
                turretPower = filteredTx * KP;

            // ---- Distance Calculation ----
            double angleRad = Math.toRadians(CAMERA_ANGLE + ty);
            double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleRad);

            // ---- Auto Servo Angle ----
            double servoPos = SERVO_MIN +
                    (distance - DIST_MIN) / (DIST_MAX - DIST_MIN) * (SERVO_MAX - SERVO_MIN);

            servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
            shooterServo.setPosition(servoPos);

//            // ---- Auto Shooter RPM based on distance ----
//            double normalized = (distance - DIST_MIN) / (DIST_MAX - DIST_MIN);
//            normalized = Math.max(0, Math.min(1, normalized));
//
//            shooterPowerTarget = SHOOTER_PWR_MIN +
//                    normalized * (SHOOTER_PWR_MAX - SHOOTER_PWR_MIN);
        }

        // ---- Apply turret motor ----
        turretPower = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, turretPower));
        aimMotor.setPower(turretPower);


        follower.update();
        updateStateMachine();

//        if (follower.getRemainingDistance() < 10 && !slowPathStarted) {
//            follower.followPath(slowPath);
//            slowPathStarted = true;
//        }

        Pose p = follower.getPose();

        telemetry.addData("State", pathState);
        telemetry.addData("X", p.getX());
        telemetry.addData("Y", p.getY());
        telemetry.addData("Heading°", Math.toDegrees(p.getHeading()));
        telemetry.addData("Path Time", pathTimer.seconds());
        telemetry.update();
    }

    public void driveForward(double power) {
        shooter1.setPower(1);

    }
    public void intake(double power) {
        intakeMotor.setPower(1);

    }

    @Override
    public void stop() {
        telemetry.addLine("Stopped.");
        telemetry.update();
    }
}

