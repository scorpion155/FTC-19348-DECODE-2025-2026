package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10)
            .forwardZeroPowerAcceleration(-47.212261068685635)
            .lateralZeroPowerAcceleration(-59.65118795870824)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.09,0,0.001, 0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.01, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.7, 0,0.0001,0.6, 0.015))
            .centripetalScaling(0.0005)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1,
            1);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(36.916355653237346)
            .yVelocity(26.742881428824216);


//    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
//            .forwardTicksToInches(.001989436789)
//            .strafeTicksToInches(.001989436789)
//            .turnTicksToInches(.001989436789)
//            .leftPodY(6.6)
//            .rightPodY(-6.6)
//            .strafePodX(-8.8)
//            .leftEncoder_HardwareMapName("frontLeft")
//            .rightEncoder_HardwareMapName("frontRight")
//            .strafeEncoder_HardwareMapName("shooter2")
//            .leftEncoderDirection(Encoder.FORWARD)
//            .rightEncoderDirection(Encoder.REVERSE)
//            .strafeEncoderDirection(Encoder.FORWARD);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.001989436789)
            .strafeTicksToInches(.001989436789)
            .turnTicksToInches(.001989436789)
            .leftPodY(6.1)
            .rightPodY(-6.1)
            .strafePodX(-7.7)
            .leftEncoder_HardwareMapName("frontLeft")
            .rightEncoder_HardwareMapName("frontRight")
            .strafeEncoder_HardwareMapName("shooter2")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }







}
