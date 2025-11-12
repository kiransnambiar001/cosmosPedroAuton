package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    private final Hardware robotHardware;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean state = false;
    public boolean isTimedRunActive = false;
    private double stopTimeInSeconds = 0;

    public Intake(Hardware hardware) {
        this.robotHardware = hardware;
    }

    public void runForTime(double power, double durationSeconds) {
        if (!isTimedRunActive) {
            this.isTimedRunActive = true;
            this.timer.reset();
            this.stopTimeInSeconds = durationSeconds;
            this.robotHardware.intakeMotor.setPower(power);
        }
    }

    public void run(double power) {
        this.isTimedRunActive = false;
        this.robotHardware.intakeMotor.setPower(power);
    }

    public void update()
    {
        if (isTimedRunActive && timer.seconds() >= stopTimeInSeconds)
            run(0);
    }
    public boolean getState()
    {
        return state;
    }
    public void toggle()
    {
        state = !state;
    }
}
