package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Gate {

    private final Hardware robotHardware;
    //private boolean state = false;

    public static double gateClosedPos = -1;
    public static double gateOpenPos = 0.4;

    public Gate(Hardware hardware) {
        robotHardware = hardware;
    }

    public void setGateState(String state) {
        if (state.equalsIgnoreCase("close")) {robotHardware.gateServo.setPosition(gateClosedPos);}
        else if (state.equalsIgnoreCase("open")) {robotHardware.gateServo.setPosition(gateOpenPos);}
    }
}
