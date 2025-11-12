package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Storage {

    private final Hardware robotHardware;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean isTimedRunActive = false;
    private double stopTime = 0;

    public Storage(Hardware hardware) {
        this.robotHardware = hardware;
    }

    // This method now correctly takes duration in SECONDS.
    // This method now correctly takes duration in MILLISECONDS.
    public void runForTime(double power, double durationMs) {
        if (!isTimedRunActive) {
            this.isTimedRunActive = true;
            this.timer.reset();
            this.stopTime = timer.milliseconds() + durationMs;
            this.robotHardware.storage.setPower(power);
        }
    }


    public void run(double power) {
        this.isTimedRunActive = false;
        this.robotHardware.storage.setPower(power);
    }

    public void update() {
        // The check now correctly uses .milliseconds()
        if (isTimedRunActive && timer.milliseconds() >= stopTime) {
            run(0);
        }
    }
}
