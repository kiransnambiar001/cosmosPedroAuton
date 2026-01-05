package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class CRServoStorage {

    private final Hardware robotHardware;
    private final ElapsedTime timer;
    public boolean isTimedRunActive = false;

    private double stopTime;

    public CRServoStorage(Hardware hardware) {
        robotHardware = hardware;
         timer = robotHardware.timer;
    }
    public void runForTime(double power, double durationMs) {
        if (!isTimedRunActive) {
            isTimedRunActive = true;
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
        if (isTimedRunActive && robotHardware.timer.milliseconds() >= stopTime) {
            run(0);
            isTimedRunActive = false;
            return true;
        }
        return false;
    }
}