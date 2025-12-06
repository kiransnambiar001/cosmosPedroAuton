//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//@TeleOp
//public class FeedForwardTuner extends LinearOpMode {
//    double initVoltage = 0.0f;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        double power = 0.0f;
//        Gamepad prev = new Gamepad();
//        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
//        initVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
//        waitForStart();
//        prev.copy(gamepad1);
//        while (opModeIsActive()) {
//            if (gamepad1.left_bumper && !prev.left_bumper) {
//                power += 0.01f;
//            }
//            if (gamepad1.right_bumper && !prev.right_bumper) {
//                power -= 0.01f;
//            }
//            shooter.setPower(power);
//            prev.copy(gamepad1);
//
//            telemetry.addData("Power", power);
//            telemetry.addData("Motor Voltage", initVoltage - hardwareMap.voltageSensor.iterator().next().getVoltage());
//            telemetry.addData("Original Voltage", initVoltage);
//            telemetry.addData("Motor Velocity", shooter.getVelocity());
//            telemetry.update();
//        }
//    }
//}