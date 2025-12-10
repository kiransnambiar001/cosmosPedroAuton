package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.CRServoStorage;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@TeleOp(name="Main TeleOp", group="LinearOpMode")
public class MainTeleOp extends LinearOpMode
{

    // Create hardware object
    Hardware robotHardware = new Hardware();
    Intake robotIntake;
    CRServoStorage robotStorage;
    Outtake robotOuttake;

    private double prevFrontLeftPower = 0.0;
    private double prevFrontRightPower = 0.0;
    private double prevBackLeftPower = 0.0;
    private double prevBackRightPower = 0.0;
    private final double fcoefficient = 0.0001;

    //auto shoot
    final double storageTime = 1900;
    double spoolUpTime = 0;
    double shootingTime = 0;
    double gracePeriod = 2500;
    double spoolUpEndTime = 0;
    double shootingEndTime = 0;
    double autoShootSequenceEndTime = 0;
    boolean isAutoShooting = false;
    String autoShootPreset = "close";
    String autoShootState;


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware
        robotHardware.initialize(hardwareMap, false);
        robotIntake = new Intake(robotHardware);
        robotStorage = new CRServoStorage(robotHardware);
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
        boolean x2prevState = false;

        // Auto shoot sequence tracking
        boolean isIntakeRunning = false;
        boolean isAutoShooting = false;
        double spoolUpEndTime = 0;
        double currentOuttakePower = 0;

        boolean farToggle = false;
        boolean closeToggle = false;

        // Start OpMode loop
        while (opModeIsActive()) {
            //      Gamepad 1 inputs
            double ly1 = -gamepad1.left_stick_y; // forward/backward driving
            double lx1 = gamepad1.left_stick_x; // strafing
            double rx1 = gamepad1.right_stick_x/2; // turning (decrease by factor of 2)
            boolean home1state = gamepad1.guide; // reset yaw value on gyro
            boolean options1state = gamepad1.options; // field centric toggle
            double lt1state = gamepad1.left_trigger; // slow mode

            //      Gamepad 2 controls
            double ly2 = gamepad2.left_stick_y; //Intake(up = in, down = out)
            double ry2 = -gamepad2.right_stick_y; // N/A
            double lt2state = gamepad2.left_trigger; //N/A
            double rt2state = gamepad2.right_trigger; //Storage in
            boolean rb2state = gamepad2.right_bumper; //Storage out
            boolean a2state = gamepad2.a; // run intake for 5 secs
            boolean b2state = gamepad2.b; // outtake preset for close shoot
            boolean y2state = gamepad2.y; // outtake preset for far shoot
            boolean x2state = gamepad2.x; // reverse outtake motor
            boolean options2state = gamepad2.options; // reset the fine adjustments of the outtake
            boolean dpu2 = gamepad2.dpad_up; // outtake speed +0.3
            boolean dpd2 = gamepad2.dpad_down; // outtake speed -0.3

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
                if (a2state && !a2prevState) {
                    robotIntake.runForTime(1.0, 5.0);
                }
                if (!robotIntake.isTimedRunActive) {
                    robotIntake.run(0.0);
                }
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
            if (b2state && !b2prevState) {
                if(!isAutoShooting)
                    initializeAutoShoot("close");
                else
                {
                    autoShootSequenceEndTime = robotHardware.timer.milliseconds() + shootingTime + gracePeriod;
                    if (autoShootState.equals("gracePeriod"))
                        robotStorage.runForTime(1.0, storageTime);
                }
            } else if (y2state && !y2prevState) {
                if(!isAutoShooting)
                    initializeAutoShoot("far");
                else {
                    autoShootSequenceEndTime = robotHardware.timer.milliseconds() + shootingTime + gracePeriod;
                    if (autoShootState.equals("gracePeriod"))
                        robotStorage.runForTime(1.0, storageTime);
                }
            }
            if (isAutoShooting) {
                runAutoShoot();
            }
                // End of auto shoot sequence

                // Fine tune active preset
                if (dpu2 && !dpu2prevState) {
                    robotOuttake.tuneActivePreset(0.03);
                } else if (dpd2 && !dpd2prevState) {
                    robotOuttake.tuneActivePreset(-0.03);
                }

                // Reset outtake presets
                if (options2state && !options2prevState) {
                    robotOuttake.reset();
                }

                // Updates
                robotOuttake.update();
                robotStorage.update();
                robotIntake.update();

                options2prevState = options2state;
                home1prevState = home1state;
                options1prevState = options1state;
                dpu2prevState = dpu2;
                dpd2prevState = dpd2;
                b2prevState = b2state;
                y2prevState = y2state;
                a2prevState = a2state;

                telemetry.addData("Status", "Running");
                telemetry.addData("Field Centric", fieldCentric ? "ON" : "OFF");
                telemetry.addData("Auto-Shooting", isAutoShooting ? "ACTIVE" : "IDLE");
                telemetry.addData("Outtake Motor Power", robotOuttake.getPower());
                telemetry.addData("Target Velocity (tps)", robotOuttake.getTargetTps());
                telemetry.addData("Actual Velocity (tps)", robotHardware.outtakeMotor.getVelocity());
                telemetry.addData("IMU Heading (deg)", Math.toDegrees(imuHeading));
                telemetry.addData("F Coefficient", fcoefficient);
                telemetry.update();
        }
    }
    private void initializeAutoShoot(String currentPreset)
    {
        autoShootPreset = currentPreset;
        if(autoShootPreset.equals("close"))
            spoolUpTime = 2000;
        else if(autoShootPreset.equals("far"))
            spoolUpTime = 4000;
        shootingTime = 1200;
        spoolUpEndTime = robotHardware.timer.milliseconds() + spoolUpTime;
        shootingEndTime = spoolUpEndTime + shootingTime;
        autoShootSequenceEndTime = shootingEndTime + gracePeriod;
        isAutoShooting = true;
        autoShootState = "spoolingUp";
    }
    private void runAutoShoot()
    {
        robotOuttake.run(autoShootPreset);
        if(robotHardware.timer.milliseconds() >= spoolUpEndTime && robotHardware.timer.milliseconds() < shootingEndTime)
        {
            autoShootState = "shooting";
            robotStorage.runForTime(1.0, storageTime);
        }

        if(robotHardware.timer.milliseconds() >= shootingEndTime && robotHardware.timer.milliseconds() < autoShootSequenceEndTime)
        {
            robotStorage.run(0.0);
            autoShootState = "gracePeriod";
        }

        if(robotHardware.timer.milliseconds() >= autoShootSequenceEndTime)
        {
            robotOuttake.run("idle");
            robotStorage.run(0.0);
            isAutoShooting = false;
            autoShootState = "idle";
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

        robotHardware.imu.resetYaw();
    }

    private void updateDriveBase(double ly, double lx, double rx, double lt1state, double imuHeading, boolean fieldCentric) {
        double speedMultiplier = (lt1state > 0.5) ? 0.3 : 1.0;
        final double rxMultiplier = 0.75;
        rx *= rxMultiplier;
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
        final double RAMP_RATE = 0.075;

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
