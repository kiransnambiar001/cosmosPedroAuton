//package org.firstinspires.ftc.teamcode;
//
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.arcrobotics.ftclib.gamepad.TriggerReader;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//
//@TeleOp(name="RobotCentric TeleOp", group="TeleOp")
//
//public class RobotCentricTeleop extends LinearOpMode {
//
//
//    // Create hardware object
//    Hardware robotHardware;
//    GamepadEx pad1;
//    GamepadEx pad2;
//    TriggerReader rt1Reader;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        robotHardware.initialize(hardwareMap);
//
//        rt1Reader = new TriggerReader(pad1, GamepadKeys.Trigger.RIGHT_TRIGGER);
//
//        float speedMultiplier = 1.0f;
//        boolean slowMode = false;
//
//        // update telemetry to show INITIALIZED status
//        telemetry.addData("Status", "INITIALIZED");
//        telemetry.update();
//
//        // wait for user to press start button
//        waitForStart();
//
//
//        // start OpMode loop
//        while (opModeIsActive()) {
//
//            // get data from controller
//            double ly = -(pad1.getLeftY()); // forward/backward driving
//            double lx = pad1.getLeftX(); // strafing
//            double rx = pad1.getRightX(); // turning
//
//            if (rt1Reader.isDown()) {
//                slowMode = !slowMode;
//                speedMultiplier = (float) (slowMode ? 0.3 : 1.0);
//            }
//
//            // Calculate motor powers
//            double frontLeftPower = (ly + lx + rx)*speedMultiplier;
//            double frontRightPower = (ly - lx - rx)*speedMultiplier;
//            double backLeftPower = (ly - lx + rx)*speedMultiplier;
//            double backRightPower = (ly + lx - rx)*speedMultiplier;
//
//            // limit max motor power
//            double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
//            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
//            maxPower = Math.max(maxPower, Math.abs(backRightPower));
//
//            // If the calculated max power is greater than 1.0, scale all powers down proportionally.
//            if (maxPower > 1.0) {
//                frontLeftPower  /= maxPower;
//                frontRightPower /= maxPower;
//                backLeftPower   /= maxPower;
//                backRightPower  /= maxPower;
//            }
//
//            // set motor power based on values
//            robotHardware.frontLeft.setPower(frontLeftPower);
//            robotHardware.frontRight.setPower(frontRightPower);
//            robotHardware.backLeft.setPower(backLeftPower);
//            robotHardware.backRight.setPower(backRightPower);
//
//            rt1Reader.readValue();
//
//            /* Telemetry
//            telemetry.addData("Status", "RUNNING");
//            telemetry.addData("FrontLeft Motor Power", frontLeftPower);
//            telemetry.addData
//            ("FrontRight Motor Power", frontRightPower);
//            telemetry.addData("BackLeft Motor Power", backLeftPower);
//            telemetry.addData("BackRight Motor Power", backRightPower);
//            telemetry.update();*/
//        }
//    }
//}