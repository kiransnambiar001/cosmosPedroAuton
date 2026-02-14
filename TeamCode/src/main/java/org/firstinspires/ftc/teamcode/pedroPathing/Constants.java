package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.7) // in kg
            .useSecondaryHeadingPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(0.75, 0, 0.025, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.9, 0, 0.03, 0.015))
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.125, 0, 0.01, 0.03))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.01, 0.02))
            .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0, 0.0015, 0.6, 0.03)) // original p: 0.025
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.0005, 0.6, 0.03))
            .forwardZeroPowerAcceleration(-63.114980111444126)
            .lateralZeroPowerAcceleration(-86.82421475789788);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(53.34483271818632)
            .yVelocity(42.170307426716725)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.00303159999713800063)
            .strafeTicksToInches(0.00299472692945746415)
            .turnTicksToInches(0.00269519038878365556)
            .leftPodY(5.25) // offset from center of rotation INCHES
            .rightPodY(-5.25) // offset from center of rotation INCHES
            .strafePodX(1.265) // offset from center of rotation INCHES
            .leftEncoder_HardwareMapName("backLeft")
            .rightEncoder_HardwareMapName("backRight")
            .strafeEncoder_HardwareMapName("frontLeft")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE);
//            .IMU_HardwareMapName("imu")
//            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
