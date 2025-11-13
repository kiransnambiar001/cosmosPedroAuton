package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Main TeleOp", group="LinearOpMode")
public class MainTeleOp extends LinearOpMode {

    // Create hardware object
    Hardware robotHardware = new Hardware();
    Intake robotIntake;
    Storage robotStorage;
    Outtake robotOuttake;

    private double prevFrontLeftPower = 0.0;
    private double prevFrontRightPower = 0.0;
    private double prevBackLeftPower = 0.0;
    private double prevBackRightPower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware
        robotHardware.initialize(hardwareMap);
        robotIntake = new Intake(robotHardware);
        robotStorage = new Storage(robotHardware);
        robotOuttake = new Outtake(robotHardware);

        initializeDrivetrainForTeleOp();

        telemetry.addData("Status", "INITIALIZED");
        telemetry.addData("Drive Mode", "Ready for TeleOp");
        telemetry.update();

        waitForStart();

        boolean fieldCentric = true;

        boolean home1prevState = false;
        boolean options1prevState = false;
        boolean options2prevState = false;
        boolean dpu2prevState = false;
        boolean dpd2prevState = false;
        boolean a2prevState = false;
        boolean b2prevState = false;
        boolean y2prevState = false;

        // Auto shoot sequence tracking
        boolean isIntakeRunning = false;
        boolean isAutoShooting = false;
        double spoolUpEndTime = 0;
        double outtakePower = 0;

        robotHardware.imu.resetYaw();

        // Start OpMode loop
        while (opModeIsActive()) {
            //      Gamepad 1 inputs
            double ly1 = -gamepad1.left_stick_y; // forward/backward driving
            double lx1 = gamepad1.left_stick_x; // strafing
            double rx1 = gamepad1.right_stick_x; // turning
            boolean home1state = gamepad1.guide; // reset yaw value on gyro
            boolean options1state = gamepad1.options; // field centric toggle
            double lt1state = gamepad1.left_trigger; // slow mode

            //      Gamepad 2 inputs
            double ly2 = gamepad2.left_stick_y;
            double ry2 = -gamepad2.right_stick_y;
            double lt2state = gamepad2.left_trigger;
            double rt2state = gamepad2.right_trigger;
            boolean rb2state = gamepad2.right_bumper;
            boolean a2state = gamepad2.a; // storage on/off
            boolean b2state = gamepad2.b; // outtake preset for close shoot
            boolean y2state = gamepad2.y; // outtake preset for far shoot
            boolean options2state = gamepad2.options;
            boolean dpu2 = gamepad2.dpad_up; //
            boolean dpd2 = gamepad2.dpad_down;

            double imuHeading = robotHardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            //      Drivetrain Control
            //Field centric toggle
            if (home1state && !home1prevState && fieldCentric) {
                robotHardware.imu.resetYaw();
            } home1prevState = home1state;

            if (options1state && !options1prevState) {
                fieldCentric = !fieldCentric;
            } options1prevState = options1state;

            updateDriveBase(ly1, lx1, rx1, lt1state, imuHeading, fieldCentric);

            //      Intake Control
            robotIntake.update();

            if (ly2 >= 0.3) {
                robotIntake.run(1.0);
            } else if (ly2 <= -0.3) {
                robotIntake.run(-1.0);
            } else {
                if (!robotIntake.isTimedRunActive) {robotIntake.run(0.0);}
                robotIntake.run(0.0);
                if (a2state && !a2prevState) {robotIntake.runForTime(1.0, 5.0);}
            }
            a2prevState = a2state;

            //      Storage Control
            robotStorage.update();

            if (rt2state >= 0.3 && !rb2state) {
                robotStorage.run(1.0);
            } else if (rb2state && rt2state < 0.3) {
                robotStorage.run(-1.0);
            } else {
                robotStorage.run(0.0);
            }

            //      Outtake presets & auto shoot
            final double spoolUpTime = 5000;
            final double storageTime = 3000 ;
            double currentOuttakePower;

            // Start auto shoot sequence
            if (((b2state && !b2prevState) || (y2state && !y2prevState)) && !isAutoShooting) {
                isAutoShooting = true;
                spoolUpEndTime = robotHardware.timer.milliseconds() + spoolUpTime;
            }

            if (isAutoShooting) {
                // Check if spool up time has passed
                if (robotHardware.timer.milliseconds() >= spoolUpEndTime) {
                    robotStorage.runForTime(1.0, storageTime / 1000.0);
                    isAutoShooting = false;
                }

                // Set outtake power based on which preset is active
                if (b2state) {
                    currentOuttakePower = robotOuttake.setPreset("close");
                } else if (y2state) {
                    currentOuttakePower = robotOuttake.setPreset("far");
                } else {
                    isAutoShooting = false;
                    currentOuttakePower = robotOuttake.setPreset("idle");
                }
            } else {
                // Manual preset control
                if (b2state) {
                    currentOuttakePower = robotOuttake.setPreset("close");
                } else if (y2state) {
                    currentOuttakePower = robotOuttake.setPreset("far");
                } else {
                    currentOuttakePower = robotOuttake.setPreset("idle");
                }
            }

            // Fine tune active preset
            if (dpu2 && !dpu2prevState) {
                robotOuttake.tuneActivePreset(0.03);
            } else if (dpd2 && !dpd2prevState) {
                robotOuttake.tuneActivePreset(-0.03);
            }

            robotOuttake.run(currentOuttakePower);

            // Reset outtake presets
            if (options2state && !options2prevState) {
                robotOuttake.reset();
            }

            // Update previous states
            options2prevState = options2state;
            dpu2prevState = dpu2;
            dpd2prevState = dpd2;
            b2prevState = b2state;
            y2prevState = y2state;

            telemetry.addData("Status", "Running");
            telemetry.addData("Field Centric", fieldCentric ? "ON" : "OFF");
            telemetry.addData("Auto-Shooting", isAutoShooting ? "ACTIVE" : "IDLE");
            telemetry.addData("Outtake Motor Power", currentOuttakePower);
            telemetry.addData("Target Velocity (tps)", robotOuttake.getTargetTps());
            telemetry.addData("Actual Velocity (tps)", robotHardware.outtakeMotor.getVelocity());
            telemetry.addData("IMU Heading (deg)", Math.toDegrees(imuHeading));
            telemetry.update();
        }
    }

    private void initializeDrivetrainForTeleOp() {
        robotHardware.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotHardware.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardware.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardware.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardware.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robotHardware.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robotHardware.frontLeft.setPower(0);
        robotHardware.frontRight.setPower(0);
        robotHardware.backLeft.setPower(0);
        robotHardware.backRight.setPower(0);
    }

    private void updateDriveBase(double ly, double lx, double rx, double lt1state, double imuHeading, boolean fieldCentric) {
        double speedMultiplier = (lt1state > 0.5) ? 0.3 : 1.0;

        double adjLy, adjLx;

        if (fieldCentric) {
            adjLx = lx * Math.cos(-imuHeading) - ly * Math.sin(-imuHeading);
            adjLy = lx * Math.sin(-imuHeading) + ly * Math.cos(-imuHeading);
        } else {
            adjLx = lx;
            adjLy = ly;
        }

        double frontLeftPower = (adjLy + adjLx + rx) * speedMultiplier;
        double frontRightPower = (adjLy - adjLx - rx) * speedMultiplier;
        double backLeftPower = (adjLy - adjLx + rx) * speedMultiplier;
        double backRightPower = (adjLy + adjLx - rx) * speedMultiplier;

        double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Acceleration smoothing
        final double RAMP_RATE = 0.05;

        prevFrontLeftPower += Math.max(-RAMP_RATE, Math.min(RAMP_RATE, frontLeftPower - prevFrontLeftPower));
        prevFrontRightPower += Math.max(-RAMP_RATE, Math.min(RAMP_RATE, frontRightPower - prevFrontRightPower));
        prevBackLeftPower += Math.max(-RAMP_RATE, Math.min(RAMP_RATE, backLeftPower - prevBackLeftPower));
        prevBackRightPower += Math.max(-RAMP_RATE, Math.min(RAMP_RATE, backRightPower - prevBackRightPower));

        robotHardware.frontLeft.setPower(prevFrontLeftPower);
        robotHardware.frontRight.setPower(prevFrontRightPower);
        robotHardware.backLeft.setPower(prevBackLeftPower);
        robotHardware.backRight.setPower(prevBackRightPower);
    }
}