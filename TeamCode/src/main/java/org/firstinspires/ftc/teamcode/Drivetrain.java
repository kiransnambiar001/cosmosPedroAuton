package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain {
    private final Hardware hw;
    private static final double TICKS_PER_REV = 537.7;
    private static final double TICKS_PER_CM = TICKS_PER_REV / (Hardware.WHEEL_DIAMETER_CM * Math.PI);
    private static final double TIMEOUT_SECONDS = 5.0;

    private ElapsedTime runtime = new ElapsedTime();

    public Drivetrain(Hardware hw) {
        this.hw = hw;
        initializeMotors();
    }

    private void initializeMotors() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior
        hw.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void move(double cm, double power) {
        if (cm == 0) return;

        // Calculate target position
        int ticks = (int) (cm * TICKS_PER_CM);

        // Reset encoders to ensure clean start
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target positions for all motors
        hw.frontLeft.setTargetPosition(ticks);
        hw.frontRight.setTargetPosition(ticks);
        hw.backLeft.setTargetPosition(ticks);
        hw.backRight.setTargetPosition(ticks);

        // Switch to RUN_TO_POSITION mode
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power (always positive in RUN_TO_POSITION)
        double absPower = Math.abs(power);
        hw.frontLeft.setPower(absPower);
        hw.frontRight.setPower(absPower);
        hw.backLeft.setPower(absPower);
        hw.backRight.setPower(absPower);

        // Wait for movement to complete with timeout
        runtime.reset();
        while (isBusy() && runtime.seconds() < TIMEOUT_SECONDS) {
            // Optional: Add telemetry here for debugging
        }

        // Stop all motors
        stopMotors();

        // Return to encoder mode for future operations
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turn(double angle, double power) {
        if (angle == 0) return;

        // Switch to run without encoder for turning
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset IMU yaw
        hw.imu.resetYaw();

        // Determine turn direction
        double turnPower = angle > 0 ? Math.abs(power) : -Math.abs(power);

        // Set motor powers for turning
        hw.frontLeft.setPower(turnPower);
        hw.backLeft.setPower(turnPower);
        hw.frontRight.setPower(-turnPower);
        hw.backRight.setPower(-turnPower);

        // Wait for turn to complete with timeout
        runtime.reset();
        while (Math.abs(getYaw()) < Math.abs(angle) && runtime.seconds() < TIMEOUT_SECONDS) {
            // Optional: Add telemetry here for debugging
        }

        // Stop all motors
        stopMotors();

        // Return to encoder mode
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopMotors() {
        hw.frontLeft.setPower(0);
        hw.frontRight.setPower(0);
        hw.backLeft.setPower(0);
        hw.backRight.setPower(0);
    }

    private void setPower(double power) {
        hw.frontLeft.setPower(power);
        hw.frontRight.setPower(power);
        hw.backLeft.setPower(power);
        hw.backRight.setPower(power);
    }

    private void setMode(DcMotor.RunMode mode) {
        hw.frontLeft.setMode(mode);
        hw.frontRight.setMode(mode);
        hw.backLeft.setMode(mode);
        hw.backRight.setMode(mode);
    }


    private boolean isBusy() {
        return hw.frontLeft.isBusy() ||
                hw.frontRight.isBusy() ||
                hw.backLeft.isBusy() ||
                hw.backRight.isBusy();
    }


    private double getYaw() {
        return hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public int[] getMotorPositions() {
        return new int[] {
                hw.frontLeft.getCurrentPosition(),
                hw.frontRight.getCurrentPosition(),
                hw.backLeft.getCurrentPosition(),
                hw.backRight.getCurrentPosition()
        };
    }
}