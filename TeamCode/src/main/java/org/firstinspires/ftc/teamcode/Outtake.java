package org.firstinspires.ftc.teamcode;

public class Outtake {

    private final Hardware robotHardware;
    private final double startingCloseShotPower = 0.32;
    private final double startingFarShotPower = 0.53;
    private final double idlePower = 0.07

            ;
    private final double maxPower = 0.8;
    private double  closeShotPower = 0.32;
    private double farShotPower = 0.53;
    private double targetTps = 0.0;

    private String currentPreset = "";

    public Outtake(Hardware hardware) {
        this.robotHardware = hardware;
    }

    public double setPreset(String presetName) {
        this.currentPreset = presetName; // Remember which preset is active

        if (presetName.equals("close")) {
            return this.closeShotPower;
        } else if (presetName.equals("far")) {
            return this.farShotPower;
        } else {
            return this.idlePower;
        }
    }

    public void tuneActivePreset(double tuneAmount) {
        if (this.currentPreset.equals("close")) {
            this.closeShotPower += tuneAmount;
        } else if (this.currentPreset.equals("far")) {
            this.farShotPower += tuneAmount;
        }

        this.closeShotPower = Math.max(idlePower, Math.min(maxPower, this.closeShotPower));
        this.farShotPower = Math.max(idlePower, Math.min(maxPower, this.farShotPower));
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
        this.closeShotPower = startingCloseShotPower;
        this.farShotPower = startingFarShotPower;
    }
}
