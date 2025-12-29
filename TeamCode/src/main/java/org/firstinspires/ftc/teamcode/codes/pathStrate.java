//package org.firstinspires.ftc.teamcode.codes;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathBuilder;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.paths.PathConstraints;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//@Disabled
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "PedroPathing Auto toll", group = "Autonomous")
//@Configurable
//public class pathStrate extends OpMode {
//
//
//    Path fastPath, slowPath;
//    boolean slowPathStarted = false;
//
//    PathConstraints slow;
//
//
//
//
//
//
//    public Path pathFast;
//    public Path pathSlow;
//
//    private Follower follower;
//
//    private ElapsedTime pathTimer, opModeTimer;
//
//    // === STATES ===
//    public enum PathState {
//        PATH_1,
//        PATH_2,
//        PATH_3,
//        PATH_4,
//        PATH_5,
//        PATH_6,
//        PATH_7,
//        PATH_8,
//        DONE
//    }
//
//    private PathState pathState = PathState.PATH_1;
//
//    // === PATH STORAGE ===
//    private Paths paths;
//
//
//    // === PATH CLASS YOU PROVIDED ===
//    public static class Paths {
//
//        public PathChain Path1;
//        public PathChain Path2;
//        public PathChain Path3;
//        public PathChain Path4;
//        public PathChain Path5;
//        public PathChain Path6;
//        public PathChain Path7;
//        public PathChain Path8;
//
//
//        public Paths(Follower follower,
//                     PathConstraints normal,
//                     PathConstraints slow) {
//
//            Path1 = new PathBuilder(normal)
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(56.0, 8.0),
//                                    new Pose(56.0, 40.0)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(
//                            Math.toRadians(90),
//                            Math.toRadians(90)
//                    )
//                    .build();
//
//            // -------- SLOW PATH (40 -> 80) --------
//            Path1 = new PathBuilder(slow)
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(56.0, 40.0),
//                                    new Pose(56.0, 80.0)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(
//                            Math.toRadians(90),
//                            Math.toRadians(90)
//                    )
//                    .build();
//
//
//
//
//        }
//
//
//    }
//
//    // === STATE MACHINE LOGIC ===
//    public void updateStateMachine() {
//
//        switch (pathState) {
//
//            case PATH_1:
//                follower.followPath(paths.Path1, true);
//                setPathState(PathState.PATH_2);
//
//
//                break;
//
//            case PATH_2:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path2, true);
//                    setPathState(PathState.PATH_3);
//
//                }
//                break;
//
//            case PATH_3:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path3, true);
//                    setPathState(PathState.PATH_4);
//
//
//
//                }
//                break;
//
//            case PATH_4:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path4, true);
//                    setPathState(PathState.PATH_5);
//
//
//                }
//                break;
//            case PATH_5:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path5, true);
//                    setPathState(PathState.PATH_6);
//                }
//                break;
//            case PATH_6:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path6, true);
//                    setPathState(PathState.PATH_7);
//                }
//                break;
//            case PATH_7:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path7, true);
//                    setPathState(PathState.PATH_8);
//                }
//                break;
//
//            case PATH_8:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path8, true);
//                    setPathState(PathState.DONE);
//                }
//                break;
//
//            case DONE:
//                if (!follower.isBusy()) {
//                    telemetry.addLine("AUTO COMPLETE ✔");
//                }
//                break;
//        }
//    }
//
//    // === STATE CHANGE ===
//    public void setPathState(PathState newState) {
//        pathState = newState;
//        if (pathTimer != null) pathTimer.reset();
//    }
//
//    // === INIT ===
//    @Override
//    public void init() {
//
//        PathConstraints normal = new PathConstraints(
//                1.0,   // max velocity
//                1.0,   // max acceleration
//                1.0,   // max angular velocity
//                1.0    // max angular acceleration
//        );
//
//        PathConstraints slow = new PathConstraints(
//                0.3,   // slow velocity
//                0.3,   // slow acceleration
//                0.5,
//                0.5
//        );
//
//
//
//
//        telemetry.addLine("Initializing...");
//
//        pathTimer = new ElapsedTime();
//        opModeTimer = new ElapsedTime();
//
//        follower = Constants.createFollower(hardwareMap);
//
//
//
//
//        // build your 3 paths
//        paths = new Paths(follower, slow);
//
//
//
//        // Set the robot starting pose to match Path1 start
//        follower.setPose(new Pose(59.000, 15.000, Math.toRadians(90)));
//
//        telemetry.addLine("Init complete.");
//    }
//
//    // === START ===
//    @Override
//    public void start() {
//        opModeTimer.reset();
//        setPathState(PathState.PATH_1);
//
//    }
//
//    // === LOOP ===
//    @Override
//    public void loop() {
//
//
//
//
//
//        follower.update();
//        updateStateMachine();
//
////        if (follower.getRemainingDistance() < 10 && !slowPathStarted) {
////            follower.followPath(slowPath);
////            slowPathStarted = true;
////        }
//
//        Pose p = follower.getPose();
//
//        telemetry.addData("State", pathState);
//        telemetry.addData("X", p.getX());
//        telemetry.addData("Y", p.getY());
//        telemetry.addData("Heading°", Math.toDegrees(p.getHeading()));
//        telemetry.addData("Path Time", pathTimer.seconds());
//        telemetry.update();
//    }
//
//
//
//    @Override
//    public void stop() {
//        telemetry.addLine("Stopped.");
//        telemetry.update();
//    }
//}
