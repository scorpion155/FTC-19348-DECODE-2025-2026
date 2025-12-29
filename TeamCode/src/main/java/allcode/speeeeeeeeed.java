package allcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled

@TeleOp(name = "stephen curry", group = "tryhard")
public class speeeeeeeeed extends LinearOpMode {
    private Limelight3A limelight;
    private DcMotorEx shooter;
    private DcMotor aimMotor;
    private IMU imu;
    private Servo pushservo;

    private boolean squarePressed = false;
    private boolean circlePressed = false;
    private boolean trianglePressed = false;
    private boolean crossPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean isSpecialMove = false;
    private boolean trackingEnabled = false;

    private long specialStartTime = 0;
    private final double kI = 0.0;
    private double integral = 0;
    private double lastError = 0;
    private long lastTimeMs = 0;

    private final double GOAL_HEIGHT_M = 0.9845;
    private final double FLYWHEEL_RADIUS_M = 0.036;
    private final double G = 9.8;
    private double LAUNCH_ANGLE_DEG = 25.0;
    private final double DISTANCE_SCALE = 30666;
    private final double SHOOT_SPINUP_TIME = 1.0;

    private double[] taBuffer = new double[5];
    private int bufferIndex = 0;
    private double targetRPM = 0;
    private long shootStartTime = 0;
    private boolean isShooting = false;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        imu = hardwareMap.get(IMU.class, "imu");
        pushservo = hardwareMap.get(Servo.class, "pushservo");

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocity(0);

        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setPower(0);

        pushservo.setPosition(0.0);

        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(hubOrientation));

        limelight.pipelineSwitch(9);
        limelight.setPollRateHz(100);
        limelight.start();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        lastTimeMs = System.currentTimeMillis();
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            boolean crossNow = gamepad1.cross;
            boolean circleNow = gamepad1.circle;
            boolean squareNow = gamepad1.square;
            boolean triangleNow = gamepad1.triangle;
            boolean dpadUpNow = gamepad1.dpad_up;
            boolean dpadDownNow = gamepad1.dpad_down;
            boolean rightBumperNow = gamepad1.right_bumper;

            if (dpadUpNow && !dpadUpPressed) {
                trackingEnabled = true;
                isShooting = true;
                shootStartTime = System.currentTimeMillis();
            }
            dpadUpPressed = dpadUpNow;

            if (dpadDownNow && !dpadDownPressed) {
                trackingEnabled = false;
                isShooting = false;
                shooter.setVelocity(0);
                aimMotor.setPower(0);
            }
            dpadDownPressed = dpadDownNow;

            if (rightBumperNow) {
                pushservo.setPosition(0.23);
            } else {
                pushservo.setPosition(0.0);
            }

            if (crossNow && !crossPressed && !isSpecialMove) {
                isSpecialMove = true;
                specialStartTime = System.currentTimeMillis();
                crossPressed = true;
            } else if (!crossNow) {
                crossPressed = false;
            }

            if (circleNow && !circlePressed && !isSpecialMove) {
                isSpecialMove = true;
                specialStartTime = System.currentTimeMillis();
                circlePressed = true;
            } else if (!circleNow) {
                circlePressed = false;
            }

            if (squareNow && !squarePressed && !isSpecialMove) {
                isSpecialMove = true;
                specialStartTime = System.currentTimeMillis();
                squarePressed = true;
            } else if (!squareNow) {
                squarePressed = false;
            }

            if (triangleNow && !trianglePressed && !isSpecialMove) {
                isSpecialMove = true;
                specialStartTime = System.currentTimeMillis();
                trianglePressed = true;
            } else if (!triangleNow) {
                trianglePressed = false;
            }

            if (isSpecialMove) {
                long elapsed = System.currentTimeMillis() - specialStartTime;
                if (elapsed < 300) {
                    double k = 0.6;
                    if (circlePressed) {
                        y = 0;
                        x = 0;
                        rx = k;
                    } else if (squarePressed) {
                        y = 0;
                        x = 0;
                        rx = -k;
                    } else if (trianglePressed) {
                        y = k;
                        x = 0;
                        rx = k;
                    } else if (crossPressed) {
                        y = -k;
                        x = 0;
                        rx = -k;
                    }
                } else {
                    isSpecialMove = false;
                    crossPressed = false;
                    circlePressed = false;
                    squarePressed = false;
                    trianglePressed = false;
                }
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());

            if (trackingEnabled) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    double tx = result.getTx();
                    double ty = result.getTy();
                    double ta = result.getTa();

                    taBuffer[bufferIndex] = ta;
                    bufferIndex = (bufferIndex + 1) % 5;
                    double avgTa = 0;
                    for (int i = 0; i < 5; i++) avgTa += taBuffer[i];
                    avgTa /= 5.0;

                    double distCm = DISTANCE_SCALE * Math.sqrt(1.0 / avgTa);
                    double distM = distCm / 100.0;

                    double alphaRad = Math.toRadians(LAUNCH_ANGLE_DEG);
                    double cosAlpha = Math.cos(alphaRad);
                    double tanAlpha = Math.tan(alphaRad);

                    double numerator = G * distM * distM;
                    double denominatorPhysics = 2 * cosAlpha * cosAlpha * (distM * tanAlpha - GOAL_HEIGHT_M);
                    double v0 = (denominatorPhysics > 0) ? Math.sqrt(numerator / denominatorPhysics) : 0;

                    targetRPM = (v0 * 60.0) / (2 * Math.PI * FLYWHEEL_RADIUS_M);
                    targetRPM = Math.max(500, Math.min(targetRPM, 6000));

                    if (isShooting) {
                        long shootElapsed = System.currentTimeMillis() - shootStartTime;
                        if (shootElapsed < SHOOT_SPINUP_TIME * 1000) {
                            double spinupProgress = shootElapsed / (SHOOT_SPINUP_TIME * 1000.0);
                            shooter.setVelocity(-targetRPM * spinupProgress);
                        } else {
                            shooter.setVelocity(-targetRPM);
                            telemetry.addData("aoooo aooo aoooo", "maxvastapen");
                        }
                    } else {
                        shooter.setVelocity(0);
                    }

                    if (trackingEnabled) {
                        runAimPID(tx);
                        telemetry.addData("AimPower", aimMotor.getPower());
                    } else {
                        aimMotor.setPower(0);
                    }

                    telemetry.addData("Target X", tx);
                    telemetry.addData("Target Y", ty);
                    telemetry.addData("Target Area", ta);
                    telemetry.addData("Distance (cm)", distCm);
                    telemetry.addData("Target RPM", targetRPM);
                    telemetry.addData("Angle (deg)", LAUNCH_ANGLE_DEG);
                } else {
                    telemetry.addData("Limelight", "No Targets");
                    shooter.setVelocity(0);
                    aimMotor.setPower(0);
                    targetRPM = 0;
                }
            } else {
                shooter.setVelocity(0);
                aimMotor.setPower(0);
            }

            telemetry.addData("Tracking Enabled", trackingEnabled);
            telemetry.addData("Shooting", isShooting);
            telemetry.addData("PushServo Pos", pushservo.getPosition());
            telemetry.update();
        }
        aimMotor.setPower(0);
        shooter.setVelocity(0);
        pushservo.setPosition(0.0);
    }

    private void runAimPID(double tx) {
        long nowMs = System.currentTimeMillis();
        double dt = (nowMs - lastTimeMs) / 1000.0;
        if (dt <= 0) dt = 0.02;
        lastTimeMs = nowMs;

        double error = tx;

        double deadzone = 0.5;
        if (Math.abs(error) < deadzone) {
            aimMotor.setPower(0);
            return;
        }

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double kP = 0.035;
        double kD = 0.006;

        double output = (kP * error) + (kI * integral) + (kD * derivative);

        double maxError = 15.0;
        double scale = Math.min(Math.abs(error) / maxError, 1.0);
        output *= scale;

        double maxPower = 0.9;
        if (output > maxPower) output = maxPower;
        if (output < -maxPower) output = -maxPower;

        aimMotor.setPower(-output);
    }
}