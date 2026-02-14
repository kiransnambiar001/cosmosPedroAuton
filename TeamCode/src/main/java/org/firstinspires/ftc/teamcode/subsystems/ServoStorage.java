package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class ServoStorage {

    //private boolean state = false;

    private static double storageLShootingPos = 0;
    private static double storageLLoadingPos = 0.5;
    private static double storageLIntakingPos = 0.775;
    private static double storageRShootingPos = 1;
    private static double storageRLoadingPos = 0.3;
    private static double storageRIntakingPos = 0.025;
    private ElapsedTime storageTimer = new ElapsedTime();
    public static double[] cycleDelay = {650, 650};
    private int cycleState = 0;
    private boolean cycleOn = false;
    private int cycleCount = 0;
    private boolean isIntCycle = false;
    private double previousTime = 0;

    public Servo storageL, storageR;


    public ServoStorage(Hardware hardware) {
        storageL = hardware.servoStorageLeft;
        storageR = hardware.servoStorageRight;
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
     * cycle will cycle between shooting and loading
     * cycleDelay[0]: delay between loading and shooting pos
     * cycleDelay[1]: delay between shooting and loading pos
    **/
    public void cycle(boolean on){
        cycleOn = on;
        if (!on) {
            storageL.setPosition(storageLLoadingPos);
            storageR.setPosition(storageRLoadingPos);
            isIntCycle = false;
        }
    }

    public void cycle(int balls){
        cycleOn = true;
        isIntCycle = true;
        cycleCount = balls;
        previousTime = storageTimer.milliseconds();
        cycleState = 0;
        storageL.setPosition(storageLShootingPos);
        storageR.setPosition(storageRShootingPos);
    }


    /** returns true if the int cycle finished **/
    public boolean update() {
        boolean justFinished = false;
        if (cycleOn && !isIntCycle) {
            switch (cycleState) {
                case 0:
                    if (previousTime + cycleDelay[0] <= storageTimer.milliseconds()) {
                        storageL.setPosition(storageLLoadingPos);
                        storageR.setPosition(storageRLoadingPos);
                        previousTime = storageTimer.milliseconds();
                        cycleState = 1;
                    }
                    break;
                case 1:
                    if (previousTime + cycleDelay[1] <= storageTimer.milliseconds()) {
                        storageL.setPosition(storageLShootingPos);
                        storageR.setPosition(storageRShootingPos);
                        previousTime = storageTimer.milliseconds();
                        cycleState = 0;
                    }
                    break;
            }
        }
        else if (cycleOn) {
            switch (cycleState) {
                case 0:
                    if (previousTime + cycleDelay[0] <= storageTimer.milliseconds()) {
                        storageL.setPosition(storageLLoadingPos);
                        storageR.setPosition(storageRLoadingPos);
                        previousTime = storageTimer.milliseconds();
                        cycleState = 1;
                    }
                    break;
                case 1:
                    if (previousTime + cycleDelay[1] <= storageTimer.milliseconds()) {
                        storageL.setPosition(storageLShootingPos);
                        storageR.setPosition(storageRShootingPos);
                        previousTime = storageTimer.milliseconds();
                        cycleCount--;
                        if (cycleCount <= 0) {
                            cycleOn = false;
                            isIntCycle = false;
                            justFinished = true;
                        }
                        cycleState = 0;
                    }
                    break;
            }
        }

        return justFinished;
    }

}