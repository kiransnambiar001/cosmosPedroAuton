package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class ServoStorage {

    // Servo Positions
    private static double storageLShootingPos = 0;
    private static double storageLLoadingPos = 0.5;
    private static double storageLIntakingPos = 1;
    private static double storageRShootingPos = 1;
    private static double storageRLoadingPos = 0.55;
    private static double storageRIntakingPos = 0;

    // Timing and State
    private final ElapsedTime storageTimer = new ElapsedTime();
    public static double[] cycleDelay = {650, 550};
    private int safetyTime = 5000;
    private double cycleStartTime = 0;
    private int cycleState = 0;
    private boolean cycleOn = false;
    private boolean isIntCycle = false;
    private double previousTime = 0;

    public static double distanceToTopBall = 5.0; // cm, distance when one ball is loaded and touching the polycarb

    public Servo storageL, storageR;
    private final Rev2mDistanceSensor storageDistance;


    public ServoStorage(Hardware hardware) {
        storageL = hardware.servoStorageLeft;
        storageR = hardware.servoStorageRight;

        storageDistance = (Rev2mDistanceSensor) hardware.storageDistanceSensor;

        storageTimer.reset();
    }

    /** position: -1 is intaking, 0 is loading, 1 is shooting**/
    public void setPos(int position) {
        switch (position) {
            case -1:
                storageL.setPosition(storageLIntakingPos);
                storageR.setPosition(storageRIntakingPos);
                break;
            case 0:
                storageL.setPosition(storageLLoadingPos);
                storageR.setPosition(storageRLoadingPos);
                break;
            case 1:
                storageL.setPosition(storageLShootingPos);
                storageR.setPosition(storageRShootingPos);
                break;
        }
    }

    /**
     * cycle will cycle between shooting and loading continuously.
    **/
    public void cycle(boolean on){
        cycleOn = on;
        isIntCycle = false;
        if (!on) {
            storageL.setPosition(storageLLoadingPos);
            storageR.setPosition(storageRLoadingPos);
        }
    }

    /**
     * Starts a finite cycle to shoot all loaded balls.
     */
    public void cycle(int balls){
        cycleOn = true;
        isIntCycle = true;
        cycleState = 0;
        cycleStartTime = storageTimer.milliseconds();
    }


    /**
     * Updates the servo positions based on the current cycle state.
     * @return true if a finite (shoot all) cycle has just completed, false otherwise.
     */
    public boolean update() {
        boolean justFinished = false;
        if (!cycleOn) {
            return false;
        }

        switch (cycleState) {
            case 0: // State 0: Move to Loading Position
                storageL.setPosition(storageLLoadingPos);
                storageR.setPosition(storageRLoadingPos);
                previousTime = storageTimer.milliseconds();
                cycleState = 1;
                break;

            case 1: // State 1: wait for servos, then Check Distance
                if (previousTime + cycleDelay[0] <= storageTimer.milliseconds()) {
                    if (isIntCycle) {
                        double distance = storageDistance.getDistance(DistanceUnit.CM);
                        boolean isTimedOut = (storageTimer.milliseconds() - cycleStartTime) > safetyTime;
                        if (distance > distanceToTopBall || isTimedOut) {
                            cycleOn = false;
                            isIntCycle = false;
                            justFinished = true;
                            cycleState = 0;
                            break;
                        }
                    }
                    cycleState = 2;
                }
                break;

            case 2:
                storageL.setPosition(storageLShootingPos);
                storageR.setPosition(storageRShootingPos);
                previousTime = storageTimer.milliseconds();
                cycleState = 3;
                break;

            case 3:
                if (previousTime + cycleDelay[1] <= storageTimer.milliseconds()) {
                    cycleState = 0;
                }
                break;
        }

        return justFinished;
    }

}
