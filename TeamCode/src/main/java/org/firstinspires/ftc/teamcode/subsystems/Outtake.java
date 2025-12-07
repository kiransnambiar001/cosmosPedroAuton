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

    // runForTime() variables
    private ElapsedTime timer = new ElapsedTime();
    public boolean isRunningForTime = false;
    public double runTime = 0;

    private String currentPreset = "";

    public Outtake(Hardware hardware) {
        robotHardware = hardware;
    }

    public double setPreset(String presetName) {
        currentPreset = presetName; // Remember which preset is active

        if (presetName.equals("close")) {
            return closeShotPower;
        } else if (presetName.equals("far")) {
            return farShotPower;
        } else {
            return idlePower;
        }
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

    public void run(double powerPercentage) {
        double targetRpm = powerPercentage * Hardware.OUTTAKE_MAX_RPM; //maxrpm=6000
        targetTps = ((targetRpm / 60.0) * 28); //* Hardware.OUTTAKE_TPR

        robotHardware.outtakeMotor.setVelocity(targetTps);
    }
    public double getTargetTps()
    {
        return targetTps;
    }

    public void runForTime(double millis, double powerPercentage) {
        timer.reset();
        runTime = millis;
        isRunningForTime = true;
        this.run(powerPercentage);
    }

    public boolean update() {
        if (isRunningForTime && timer.milliseconds() >= runTime) {isRunningForTime = false; this.run(0); return true;}
        return false;
    }

    public boolean abortRunForTime() {
        if (isRunningForTime) {isRunningForTime = false; this.run(0); return true;}
        return false;
    }

    public void reset()
    {
        closeShotPower = startingCloseShotPower;
        farShotPower = startingFarShotPower;
    }
}