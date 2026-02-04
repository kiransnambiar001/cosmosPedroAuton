package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class ServoStorage {

    //private boolean state = false;

    public static double storageLShootingPos = -1;
    public static double storageLIntakingPos = 1;
    public static double storageRShootingPos = 1;
    public static double storageRIntakingPos = -1;

    public Servo storageL, storageR;


    public ServoStorage(Hardware hardware) {
        storageL = hardware.servoStorageLeft;
        storageR = hardware.servoStorageRight;
    }

    public void setPos(boolean isIntaking) {
        if (isIntaking) {
            storageL.setPosition(storageLShootingPos);
            storageR.setPosition(storageRShootingPos);
        } else {
            storageL.setPosition(storageLIntakingPos);
            storageR.setPosition(storageRIntakingPos);
        }
    }
}