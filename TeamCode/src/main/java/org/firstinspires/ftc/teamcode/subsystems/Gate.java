package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Gate {

    private final Hardware robotHardware;
    //private boolean state = false;

    public static double gateOpenPos = -1;
    public static double gateClosedPos = 0.4;

    public Gate(Hardware hardware) {
        robotHardware = hardware;
    }

    public void setGateState(boolean isClosed) {
        if (isClosed) {robotHardware.gateServo.setPosition(gateClosedPos);}
        else {robotHardware.gateServo.setPosition(gateOpenPos);}
    }
}
