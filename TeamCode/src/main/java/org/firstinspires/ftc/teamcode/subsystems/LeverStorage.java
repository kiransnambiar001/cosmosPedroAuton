//package org.firstinspires.ftc.teamcode.subsystems;
//
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Configurable
//public class LeverStorage {
//
//    private final Hardware robotHardware;
//    private final ElapsedTime timer = new ElapsedTime();
//    //private boolean state = false;
//
//    public static double storageShootingPos = -1;
//    public static double storageIntakingPos = 0.4;
//    public static double leverOpenPos = -1;
//    public static double leverClosedPos = 0.4;
//    private static boolean isBusy = false;
//
//
//    private static boolean isPushingBall = false;
//    private static int ballPushState = 0;
//    private static int ballsToPush = 0;
//
//    private Servo gate;
//    private Servo lever;
//
//    public LeverStorage(Hardware hardware) {
//        robotHardware = hardware;
//        gate = robotHardware.gateServo;
//        lever = robotHardware.leverServo;
//        this.setStorageState(false);
//        this.setLeverState(true);
//    }
//
//    public void setStorageState(boolean isIntaking) {
//        if (isIntaking) {gate.setPosition(storageIntakingPos);}
//        else {gate.setPosition(storageShootingPos);}
//    }
//
//    public void setStorageState(double position) {
//        gate.setPosition(position);
//    }
//
//    public void setLeverState(boolean isClosed) {
//        if (isClosed) {lever.setPosition(leverClosedPos);}
//        else {lever.setPosition(leverOpenPos);}
//    }
//
//    public void setLeverState(double position) {
//        lever.setPosition(position);
//    }
//
//    public boolean stopAutomatic() {
//        if (isBusy) {
//            ballPushState = 0; isBusy = false; isPushingBall = false; ballsToPush = 0;
//            return true;
//        } else {return false;}
//    }
//
//    public void pushBall(boolean dontInterrupt) {
//        if (!dontInterrupt || !isBusy) {
//            isBusy = true;
//            isPushingBall = true;
//            ballPushState = 0;
//            ballsToPush = 1;
//        }
//    }
//    public void pushBall(boolean dontInterrupt, int ballsToPush) {
//        if (!dontInterrupt || !isBusy) {
//            isBusy = true;
//            isPushingBall = true;
//            ballPushState = 0;
//            this.ballsToPush = ballsToPush;
//        }
//    }
//    public void pushBall() {
//        isBusy = true;
//        isPushingBall = true;
//        ballPushState = 0;
//        ballsToPush = 1;
//    }
//    public void pushBall(int ballsToPush) {
//        isBusy = true;
//        isPushingBall = true;
//        ballPushState = 0;
//        this.ballsToPush = ballsToPush;
//    }
//
//
//    public void update() {
//        if (isBusy) {
//            if (isPushingBall) {
//                switch (ballPushState) {
//
//                    // open storage to accept ball
//                    case 0:
//                        this.setStorageState(true);
//                        timer.reset();
//                        ballPushState = 1;
//                        break;
//
//                    // let ball fall down
//                    case 1:
//                        if (timer.milliseconds() >= 200) {
//                            this.setLeverState(false);
//                            timer.reset();
//                            ballPushState = 2;
//                        }
//                        break;
//
//                    // let no more balls fall down
//                    case 2:
//                        if (timer.milliseconds() >= 300) {
//                            this.setLeverState(true);
//                            timer.reset();
//                            ballPushState = 3;
//                        }
//                        break;
//
//                    // push the ball into outtake
//                    // if there are no more balls to be pushed, STOP
//                    // if there are more balls to be pushed, go to state 4
//                    case 3:
//                        if(timer.milliseconds() >= 500) {
//                            this.setStorageState(false);
//                            ballsToPush--;
//                            if (ballsToPush > 0) {
//                                ballPushState = 4;
//                            } else {
//                                isPushingBall = false;
//                                isBusy = false;
//                                ballPushState = 0;
//                            }
//                            timer.reset();
//                        }
//                        break;
//
//                    // finish pushing the ball before getting the next one
//                    case 4:
//                        if (timer.milliseconds() >= 300) {
//                            ballPushState = 0;
//                            timer.reset();
//                        }
//
//                }
//            }
//        }
//
//    }
//
//
//
//
//}
