package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class SolversOuttake {

    private final Hardware robotHardware;

    public PIDFController pidf;
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

    public SolversOuttake(Hardware hardware, double p, double i, double d, double f) {
        robotHardware = hardware;
        pidf = new PIDFController(p, i, d, f);
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

//    public void run(double powerPercentage) {
//        double targetRpm = powerPercentage * Hardware.OUTTAKE_MAX_RPM; //maxrpm=6000
//        targetTps = ((targetRpm / 60.0) * 28); //* Hardware.OUTTAKE_TPR
//
//        robotHardware.outtakeMotor.setVelocity(targetTps);
//    }

    public void setTargetTps(double powerPercentage) {
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
        this.setTargetTps(powerPercentage);
    }

    public boolean update() {
        boolean stoppedIsRunningForTime = false;
        if (isRunningForTime && timer.milliseconds() >= runTime) {isRunningForTime = false; this.setTargetTps(0); stoppedIsRunningForTime = true;}
        double output = pidf.calculate(robotHardware.outtakeMotor.getCurrentPosition(), this.getTargetTps());
        robotHardware.outtakeMotor.setVelocity(output);
        return stoppedIsRunningForTime;
    }

    public boolean abortRunForTime() {
        if (isRunningForTime) {isRunningForTime = false; this.setTargetTps(0); return true;}
        return false;
    }

    public void reset()
    {
        closeShotPower = startingCloseShotPower;
        farShotPower = startingFarShotPower;
    }
}