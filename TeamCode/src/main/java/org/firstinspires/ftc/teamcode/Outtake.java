package org.firstinspires.ftc.teamcode.robot;

public class Outtake {

    private final Hardware robotHardware;
    private final double startingCloseShotPower = 0.32;
    private final double startingFarShotPower = 0.53;
    private final double idlePower = 0.07;
    private final double maxPower = 0.8;
    private double  closeShotPower = 0.32;
    private double farShotPower = 0.53;
    private double targetTps = 0.0;

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
        double targetRpm = powerPercentage * Hardware.OUTTAKE_MAX_RPM;
        targetTps = (targetRpm / 60.0) * Hardware.OUTTAKE_TPR;

        robotHardware.outtakeMotor.setVelocity(targetTps);
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