package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {

    private final Hardware robotHardware;

    private final double startingCloseShotPower = 0.32;
    private final double startingFarShotPower = 0.53;
    private final double idlePower = 0.05;
    private final double maxPower = 0.8;
    private double  closeShotPower = 0.32;
    private double farShotPower = 0.53;
    private double targetTps = 0.0;
    double stopTimeMs;
    double power = idlePower;
    private String currentPreset = "";
    private boolean isTimedRunActive = false;
    private final ElapsedTime timer;

    public Outtake(Hardware hardware) {
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

        robotHardware.outtakeMotor.setVelocity(targetTps);
    }
    public void runForTime(String preset, double durationMs) {
        stopTimeMs = timer.milliseconds() + durationMs;
        if (!isTimedRunActive) {
            isTimedRunActive = true;
            run(preset);
        }
    }
    public void update()
    {
        if (isTimedRunActive && timer.milliseconds() >= stopTimeMs)
            run("idle");
    }
    public double getPower()
    {
        return power;
    }
    public double getTargetTps()
    {
        return targetTps;
    }
    public void reset()
    {
        closeShotPower = startingCloseShotPower;
        farShotPower = startingFarShotPower;
    }
}