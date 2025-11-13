package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Storage {

    private final Hardware robotHardware;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean isTimedRunActive = false;
    private double stopTime = 0;

    public Storage(Hardware hardware) {
        robotHardware = hardware;
    }

    public void runForTime(double power, double durationMs) {
        if (!isTimedRunActive) {
            isTimedRunActive = true;
            timer.reset();
            stopTime = timer.milliseconds() + durationMs;
            robotHardware.storage.setPower(power);
        }
    }


    public void run(double power) {
        isTimedRunActive = false;
        robotHardware.storage.setPower(power);
    }

    public void update() {
        if (isTimedRunActive && timer.milliseconds() >= stopTime) {
            run(0);
        }
    }
}