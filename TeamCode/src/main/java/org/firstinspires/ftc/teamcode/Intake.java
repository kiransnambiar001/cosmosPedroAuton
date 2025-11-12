package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    private final Hardware robotHardware;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean state = false;
    public boolean isTimedRunActive = false;
    private double stopTimeInSeconds = 0;

    public Intake(Hardware hardware) {
        robotHardware = hardware;
    }

    public void runForTime(double power, double durationSeconds) {
        if (!isTimedRunActive) {
            isTimedRunActive = true;
            timer.reset();
            stopTimeInSeconds = durationSeconds;
            robotHardware.intakeMotor.setPower(power);
        }
    }

    public void run(double power) {
        isTimedRunActive = false;
        robotHardware.intakeMotor.setPower(power);
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
