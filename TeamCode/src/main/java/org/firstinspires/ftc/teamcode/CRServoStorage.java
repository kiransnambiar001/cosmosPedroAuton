package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class CRServoStorage {
    public CRServo servoLeft;
    public CRServo servoRight;
    private double speed = 5;


    public boolean isBusy = false;
    private double previousTime = 0;
    private double duration = 0;
    private ElapsedTime timer = new ElapsedTime();



    public CRServoStorage(Hardware robotHardware) {
        this.servoLeft = robotHardware.storageLeft;
        this.servoRight = robotHardware.storageRight;
    }


    public void run(double speed) {
        this.speed = speed;
        servoLeft.setPower(this.speed); servoRight.setPower(this.speed);
    }


    public void runForTime(double time) {
        this.isBusy = true;

        servoLeft.setPower(this.speed);
        servoRight.setPower(this.speed);


        this.duration = time;
        this.timer.reset();
    }


    boolean turnOffAllMotors() { // returns if the function overrided a runForTime() function
        servoLeft.setPower(0); servoRight.setPower(0);
        if (this.isBusy) {this.isBusy = false; return true;}
        else {return false;}
    }


    public void update() {
        if (this.timer.milliseconds() >= this.duration && isBusy) {
            servoLeft.setPower(0); servoRight.setPower(0);
            this.isBusy = false;
        }
    }
}



