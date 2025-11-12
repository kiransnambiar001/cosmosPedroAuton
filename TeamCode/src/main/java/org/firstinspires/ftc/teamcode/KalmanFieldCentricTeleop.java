package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode.robot;
//
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.arcrobotics.ftclib.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // For linear OpModes
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp; // For TeleOp OpModes
//
//import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
//import com.arcrobotics.ftclib.gamepad.TriggerReader;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//
//@TeleOp(name="FieldCentric TeleOp", group="TeleOp")
//
//public class KalmanFieldCentricTeleop extends LinearOpMode {
//
//
//    // Create hardware object
//    Hardware robotHardware = new Hardware(gamepad1, gamepad2);
//
//    HeadingKalmanFilter kalmanFilter = new HeadingKalmanFilter(0); // the initial heading of the robot is 0
//
//    ToggleButtonReader b1Reader;
//    TriggerReader rt1Reader;
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        // use hardware class to initialize everything (code in Hardware file)
//        robotHardware.initialize(hardwareMap);
//
//        float speedMultiplier = 1.0f;
//        boolean slowMode = false;
//
//        b1Reader = new ToggleButtonReader(robotHardware.pad1, GamepadKeys.Button.B);
//        rt1Reader = new TriggerReader(robotHardware.pad1, GamepadKeys.Trigger.RIGHT_TRIGGER);
//
//
//        // update telemetry to show INITIALIZED status
//        telemetry.addData("Status", "INITIALIZED");
//        telemetry.update();
//
//        // wait for user to press start button
//        waitForStart();
//
//        double lastTime = robotHardware.timer.seconds();
//
//
//        // start OpMode loop
//        while (opModeIsActive()) {
//            double currentTime = robotHardware.timer.seconds();
//            double dt = currentTime - lastTime; // in seconds
//
//            // odometry
//            robotHardware.odometry.updatePose();
//            Pose2d currentPose = robotHardware.odometry.getPose();
//            double odomHeading = currentPose.getHeading(); // in radians
//
//            // imu
//            double imuHeading = -robotHardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//            // Kalman Filter
//            kalmanFilter.predict(dt);
//            kalmanFilter.update(imuHeading, odomHeading);
//            double kalmanHeading = kalmanFilter.getHeading();
//
//            // get data from controller
//            double ly = -(robotHardware.pad1.getLeftY()); // forward/backward driving
//            double lx = robotHardware.pad1.getLeftX(); // strafing
//            double rx = robotHardware.pad1.getRightX(); // turning
//
//            // slow mode
//            if (rt1Reader.isDown()) {
//                slowMode = !slowMode;
//                speedMultiplier = (float) (slowMode ? 0.3 : 1.0);
//            }
//
//            // field centric calculations
//            double adjustedLy = 0;
//            double adjustedLx = 0;
//            if (b1Reader.getState()) {
//                adjustedLx = -ly * Math.sin(kalmanHeading) + lx * Math.cos(kalmanHeading);
//                adjustedLy = ly * Math.cos(kalmanHeading) + lx * Math.sin(kalmanHeading);
//            }
//            // Calculate motor powers
//            double frontLeftPower = (adjustedLy + adjustedLx + rx) * speedMultiplier;
//            double frontRightPower = (adjustedLy - adjustedLx - rx) * speedMultiplier;
//            double backLeftPower = (adjustedLy - adjustedLx + rx) * speedMultiplier;
//            double backRightPower = (adjustedLy + adjustedLx - rx) * speedMultiplier;
//
//            // limit max motor power
//            double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
//            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
//            maxPower = Math.max(maxPower, Math.abs(backRightPower));
//
//            // If the calculated max power is greater than 1.0, scale all powers down proportionally.
//            if (maxPower > 1.0) {
//                frontLeftPower /= maxPower;
//                frontRightPower /= maxPower;
//                backLeftPower /= maxPower;
//                backRightPower /= maxPower;
//            }
//
//            // set motor power based on values
//            robotHardware.frontLeft.setPower(frontLeftPower);
//            robotHardware.frontRight.setPower(frontRightPower);
//            robotHardware.backLeft.setPower(backLeftPower);
//            robotHardware.backRight.setPower(backRightPower);
//
//
//            // Telemetry
//            telemetry.addData("Status", "RUNNING");
//            telemetry.addData("Odometry Heading", odomHeading);
//            telemetry.addData("IMU Heading", imuHeading);
//            telemetry.addData("Kalman Heading", kalmanHeading);
//            telemetry.addData("Slow Mode", (slowMode ? "On" : "Off"));
//            telemetry.update();
//
//            // update values
//            b1Reader.readValue();
//            rt1Reader.readValue();
//        }
//    }
//}