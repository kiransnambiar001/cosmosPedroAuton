package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {

    private final Hardware robotHardware;

    private final double startingCloseShotPower = 0.32;
    private final double startingFarShotPower = 0.53;
    private final double idlePower = 0.2;
    private final double maxPower = 0.8;
    private double  closeShotPower = 0.42;
    private double farShotPower = 0.63;
    private double targetTps = 0.0;
    double stopTimeMs;
    double power = idlePower;
    private String currentPreset = "";
    public boolean isTimedRunActive = false;
    private final ElapsedTime timer;

    public Outtake(Hardware hardware, double p, double i, double d, double f) {
        hardware.outtakeMotor.setVelocityPIDFCoefficients(p,i,d,f);
        robotHardware = hardware;
        timer = robotHardware.timer;
    }
    public void tuneActivePreset(double tuneAmount) {
        if (currentPreset.equals("close")) {
            closeShotPower += tuneAmount;
        } else if (currentPreset.equals("far")) {
            farShotPower += tuneAmount;
        }
        closeShotPower = Math.max(idlePower, Math.min(maxPower, closeShotPower));
        farShotPower = Math.max(idlePower, Math.min(maxPower, farShotPower));
    }
    public void run(double power)
    {
        double targetRpm = power * Hardware.OUTTAKE_MAX_RPM;
        targetTps = (targetRpm / 60.0) * Hardware.OUTTAKE_TPR;
        robotHardware.outtakeMotor.setVelocity(targetTps);

        isTimedRunActive = false;
    }
    public void run(String presetName) {
        if (presetName.equals("close"))
        {
            currentPreset = "close";
            power = closeShotPower;
        } else if (presetName.equals("far"))
        {
            currentPreset = "far";
            power = farShotPower;
        } else if (presetName.equals("idle"))
        {
            currentPreset = "idle";
            power = idlePower;
        }
        double targetRpm = power * Hardware.OUTTAKE_MAX_RPM;
        targetTps = (targetRpm / 60.0) * Hardware.OUTTAKE_TPR;
        isTimedRunActive = false;

        robotHardware.outtakeMotor.setVelocity(targetTps);
    }

    public void runForTime(String preset, double durationMs) {
        stopTimeMs = timer.milliseconds() + durationMs;
        if (!isTimedRunActive) {
            isTimedRunActive = true;
            run(preset);
        }
    }

    public void runForTime(double power, double durationMs) {
        stopTimeMs = timer.milliseconds() + durationMs;
        if (!isTimedRunActive) {
            isTimedRunActive = true;
            run(power);
        }
    }
    public boolean update()
    {
        if (isTimedRunActive && timer.milliseconds() >= stopTimeMs) {
            run("idle");
            isTimedRunActive = false;
            return true;
        }
        return false;
    }
    public double getCurrentTps()
    {
        return power;
    }
    public double getTargetTps()
    {
        return targetTps;
    }

    public String getCurrentPreset() {return currentPreset;}

    public void reset()
    {
        closeShotPower = startingCloseShotPower;
        farShotPower = startingFarShotPower;
    }
}