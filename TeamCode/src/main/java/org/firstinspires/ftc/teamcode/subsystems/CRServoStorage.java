package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class CRServoStorage {

    private final Hardware robotHardware;
    private final ElapsedTime timer;
    private final ElapsedTime jitterTimer = new ElapsedTime(0);
    public boolean isTimedRunActive = false;

    // jitter
    private static boolean jitter = true;
    private static double jitterInterval = 1000;
    private static double jitterPower = 0;
    private static boolean jitterState = false;

    private double stopTime;

    public CRServoStorage(Hardware hardware) {
        robotHardware = hardware;
        timer = robotHardware.timer;
    }
    public void runForTime(double power, double durationMs) {
        if (!isTimedRunActive) {
            isTimedRunActive = true;
            this.jitter = false;
            stopTime = timer.milliseconds() + durationMs;
            robotHardware.storageLeft.setPower(power);
            robotHardware.storageRight.setPower(power);
        }
    }

    public void runForTime(double power, double durationMs, boolean jitter, double interval) {
        if (!isTimedRunActive) {
            isTimedRunActive = true; this.jitter = jitter; jitterInterval = interval;
            stopTime = timer.milliseconds() + durationMs;
            robotHardware.storageLeft.setPower(power);
            robotHardware.storageRight.setPower(power);
        }
    }


    public void run(double power) {
        isTimedRunActive = false;
        robotHardware.storageLeft.setPower(power);
        robotHardware.storageRight.setPower(power);


    }
    public boolean getState()
    {
        if(robotHardware.storageLeft.getPower() == 0 && robotHardware.storageRight.getPower() == 0)
            return false;
        else
            return true;
    }

    public boolean update() {
        if (isTimedRunActive && jitter) {
            if (jitterTimer.milliseconds() > jitterInterval) {
                jitterTimer.reset();
                if (jitterState) {this.run(0);} else {this.run(jitterPower);}
            }
        }
        if (isTimedRunActive && robotHardware.timer.milliseconds() >= stopTime) {
            run(0);
            isTimedRunActive = false;
            return true;
        }
        return false;
    }
}