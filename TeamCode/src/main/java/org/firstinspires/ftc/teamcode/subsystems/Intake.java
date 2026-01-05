package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    private final Hardware robotHardware;
    private final ElapsedTime timer;
    //private boolean state = false;
    public boolean isTimedRunActive = false;
    private double stopTimeMs = 0;

    public Intake(Hardware hardware) {
        robotHardware = hardware;
        timer = robotHardware.timer;
    }

    public void runForTime(double power, double durationMs) {
         stopTimeMs = timer.milliseconds() + durationMs;
        if (!isTimedRunActive) {
            isTimedRunActive = true;
            robotHardware.intakeMotor.setPower(power);
        }
    }

    public void run(double power) {
        isTimedRunActive = false;
        robotHardware.intakeMotor.setPower(power);
    }

    public boolean update()
    {
        if (isTimedRunActive && timer.milliseconds() >= stopTimeMs) {
            run(0);
            isTimedRunActive = false;
            return true;
        }
        return false;
    }
//    public boolean getState()
//    {
//        return state;
//    }
//    public void toggle()
//    {
//        state = !state;
//    }
}
