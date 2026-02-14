package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.PresetPoses;

@Configurable
public class Outtake {

    private final Hardware robotHardware;
    public static double  closeShotPower = 0.46;
    public static double farShotPower = 0.66;

    private final double startingCloseShotPower = closeShotPower;
    private final double startingFarShotPower = farShotPower;
    private final double idlePower = 0.2;
    private final double maxPower = 0.8;
    private final double tpsTolerance = 50;


    private double targetTps = 0.0;
    double stopTimeMs;
    double power = idlePower;
    private String currentPreset = "";
    public boolean isTimedRunActive = false;
    private final ElapsedTime timer;

    public Outtake(Hardware hardware, double p, double i, double d, double f) {
        hardware.outtakeMotor.setVelocityPIDFCoefficients(p,i,d,f);
        robotHardware = hardware;
        //PresetPoses poses;
        //Pose goalPose;
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
    /** power percentage from 0.0-1.0, and the function turns the percentage into tps**/
    public void run(double power)
    {
        //ouble distance = currentPose.distanceFrom(goalPose) + powerDistOffset;
        //double dampingFactor = 1.0- (distance * 0.002);
        double targetRpm = power   * Hardware.OUTTAKE_MAX_RPM;
        targetTps = (targetRpm / 60.0) * Hardware.OUTTAKE_TPR;
        robotHardware.outtakeMotor.setVelocity(targetTps);

        isTimedRunActive = false;
    }
    /** three presets: close, far, and idle **/
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
    public boolean isUpToSpeed()
    {
        return Math.abs(robotHardware.outtakeMotor.getVelocity() - targetTps) < tpsTolerance;
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
        return robotHardware.outtakeMotor.getVelocity();
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