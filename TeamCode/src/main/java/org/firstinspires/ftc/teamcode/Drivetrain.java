package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain {
    private final Hardware robotHardware;
    private static final double TICKS_PER_REV = 537.7;
    private static final double TICKS_PER_CM = TICKS_PER_REV / (Hardware.WHEEL_DIAMETER_CM * Math.PI);
    private static final double TIMEOUT_SECONDS = 5.0;

    private final ElapsedTime runtime = new ElapsedTime();

    public Drivetrain(Hardware hardware)
    {
        robotHardware = hardware;
        initializeMotors();
    }

    private void initializeMotors()
    {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior
        robotHardware.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void move(double cm, double power)
    {
        if (cm == 0) return;

        int ticks = (int) (cm * TICKS_PER_CM);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotHardware.frontLeft.setTargetPosition(ticks);
        robotHardware.frontRight.setTargetPosition(ticks);
        robotHardware.backLeft.setTargetPosition(ticks);
        robotHardware.backRight.setTargetPosition(ticks);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double absPower = Math.abs(power);
        robotHardware.frontLeft.setPower(absPower);
        robotHardware.frontRight.setPower(absPower);
        robotHardware.backLeft.setPower(absPower);
        robotHardware.backRight.setPower(absPower);

        runtime.reset();
        while (isBusy() && runtime.seconds() < TIMEOUT_SECONDS) {
        }

        stopMotors();

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turn(double angle, double power)
    {
        if (angle == 0) return;

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robotHardware.imu.resetYaw();

        double turnPower = angle > 0 ? Math.abs(power) : -Math.abs(power);

        robotHardware.frontLeft.setPower(turnPower);
        robotHardware.backLeft.setPower(turnPower);
        robotHardware.frontRight.setPower(-turnPower);
        robotHardware.backRight.setPower(-turnPower);

        runtime.reset();
        while (Math.abs(getYaw()) < Math.abs(angle) && runtime.seconds() < TIMEOUT_SECONDS)
        {
        }

        stopMotors();

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopMotors()
    {
        robotHardware.frontLeft.setPower(0);
        robotHardware.frontRight.setPower(0);
        robotHardware.backLeft.setPower(0);
        robotHardware.backRight.setPower(0);
    }

    private void setPower(double power)
    {
        robotHardware.frontLeft.setPower(power);
        robotHardware.frontRight.setPower(power);
        robotHardware.backLeft.setPower(power);
        robotHardware.backRight.setPower(power);
    }

    private void setMode(DcMotor.RunMode mode)
    {
        robotHardware.frontLeft.setMode(mode);
        robotHardware.frontRight.setMode(mode);
        robotHardware.backLeft.setMode(mode);
        robotHardware.backRight.setMode(mode);
    }


    private boolean isBusy()
    {
        return robotHardware.frontLeft.isBusy() ||
                robotHardware.frontRight.isBusy() ||
                robotHardware.backLeft.isBusy() ||
                robotHardware.backRight.isBusy();
    }


    private double getYaw()
    {
        return robotHardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public int[] getMotorPositions()
    {
        return new int[] {
                robotHardware.frontLeft.getCurrentPosition(),
                robotHardware.frontRight.getCurrentPosition(),
                robotHardware.backLeft.getCurrentPosition(),
                robotHardware.backRight.getCurrentPosition()
        };
    }
}