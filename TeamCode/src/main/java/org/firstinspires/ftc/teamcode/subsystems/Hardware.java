package org.firstinspires.ftc.teamcode.subsystems;

// motors

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware {
    public static final double WHEEL_DIAMETER_CM = 9.6;//change this
    public static final double OUTTAKE_TPR = 537.7;
    public static final double OUTTAKE_MAX_RPM = 312;
    public DcMotor frontLeft, frontRight, backLeft, backRight, intakeMotor;
    public DcMotorEx outtakeMotor;
    public IMU imu;
    public ElapsedTime timer;
    public CRServo storageLeft, storageRight;
    // Init hardwareMaps

    public void initialize(HardwareMap hardwareMap, boolean isPedro) {
        if (!isPedro) {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.FORWARD);
        }
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        imu = hardwareMap.get(IMU.class, "imu");
        storageLeft = hardwareMap.get(CRServo.class, "storageLeft");
        storageRight = hardwareMap.get(CRServo.class, "storageRight");
        // Set motor zero power behavior to brake instead of move freely

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // Set directions for each motor

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        outtakeMotor.setDirection(DcMotor.Direction.FORWARD);
        storageLeft.setDirection(CRServo.Direction.REVERSE);
        storageRight.setDirection(CRServo.Direction.FORWARD);

        // pidf constants for outtake motor
//        outtakeMotor.setVelocityPIDFCoefficients(0.01d,0d,0d,0.00052d);
        // TODO: f value is 0.013

        // Initialize the IMU
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        timer = new ElapsedTime();
        timer.reset();
    }
//    public void initialize(HardwareMap hardwareMap, boolean isAuton) {
//        initialize(hardwareMap);
//
//        if (isAuton) {
//            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }

}