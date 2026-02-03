package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class ServoStorage {

    private final Hardware robotHardware;
    //private boolean state = false;

    public static double storageShootingPos = -1;
    public static double storageIntakingPos = 0.4;

    public Servo storageL, storageR;


    public ServoStorage(Hardware hardware) {
        robotHardware = hardware;
        storageL = robotHardware.sStorageLeft;
        storageR = robotHardware.sStorageRight;
    }

    public void setPos(boolean isIntaking) {
        if (isIntaking) {
            storageL.setPosition(storageShootingPos);
            storageR.setPosition(storageShootingPos);
        } else {
            storageL.setPosition(storageIntakingPos);
            storageR.setPosition(storageIntakingPos);
        }
    }
}