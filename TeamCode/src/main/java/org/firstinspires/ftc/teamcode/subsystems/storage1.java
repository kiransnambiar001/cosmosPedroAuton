//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//public class CRServoStorage {
//    public CRServo storageLeft;
//    public CRServo storageRight;
//    private double speed = 5;
//
//
//    public boolean isBusy = false;
//    private double previousTime = 0;
//    private double duration = 0;
//    private ElapsedTime timer = new ElapsedTime();
//
//
//
//    public CRServoStorage(Hardware robotHardware) {
//        this.storageLeft = robotHardware.storageLeft;
//        this.storageRight = robotHardware.storageRight;
//    }
//
//
//    public void run(double speed) {
//        this.speed = speed;
//        storageLeft.setPower(this.speed); storageRight.setPower(this.speed);
//    }
//
//
//    public void runForTime(double time) {
//        this.isBusy = true;
//
//        storageLeft.setPower(this.speed);
//        storageRight.setPower(this.speed);
//
//
//        this.duration = time;
//        this.timer.reset();
//    }
//
//
//    boolean turnOffAllMotors() { // returns if the function overrided a runForTime() function
//        storageLeft.setPower(0); storageRight.setPower(0);
//        if (this.isBusy) {this.isBusy = false; return true;}
//        else {return false;}
//    }
//
//
//    public void update() {
//        if (this.timer.milliseconds() >= this.duration && isBusy) {
//            storageLeft.setPower(0); storageRight.setPower(0);
//            this.isBusy = false;
//        }
//    }
//}
//
//
//
