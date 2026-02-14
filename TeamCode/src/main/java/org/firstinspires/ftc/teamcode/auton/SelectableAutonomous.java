package org.firstinspires.ftc.teamcode.auton;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import java.util.ArrayList;

/**
 * This is the Tuning class. It contains a selection menu for various tuning OpModes.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 6/26/2025
 */
@Configurable
@TeleOp(name = "SELECTABLE - Autonomous", group = "TeleOp")
public class SelectableAutonomous extends SelectableOpMode {

    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();

    public SelectableAutonomous() {
        super("Select a Tuning OpMode", s -> {
            s.folder("1 Pair", l -> {
                l.add("GOAL SIDE BLUE - 1 Pair OutOfWay", RobotAutonGoalSideBluePreloaded::new);
                l.add("GOAL SIDE RED - 1 Pair OutOfWay", RobotAutonGoalSideRedPreloaded::new);
            });
            s.folder("2 Pair", l -> {
                l.add("GOAL SIDE BLUE - 2 Pair", RobotAutonGoalSideBlue2Pair::new);
                l.add("GOAL SIDE RED - 2 Pair", RobotAutonGoalSideRed2Pair::new);
                l.add("FAR SIDE BLUE - 2 Pair", RobotAutonFarBlue::new);
                l.add("FAR SIDE RED - 2 Pair", RobotAutonFarRed::new);
            });
            s.folder("3 or 4 Pair", l -> {
                l.add("GOAL SIDE BLUE - 3 Pair", RobotAutonGoalSideBlue3Pair::new);
                l.add("GOAL SIDE RED - 3 Pair", RobotAutonGoalSideRed3Pair::new);

                l.add("GOAL SIDE BLUE - 4 Pair Gate", RobotAutonGoalSideBlue4PairGate::new);
                l.add("GOAL SIDE RED - 4 Pair Gate", RobotAutonGoalSideRed4PairGate::new);

                l.add("GOAL SIDE BLUE - 4 Pair", RobotAutonGoalSideBlue4Pair::new);
                l.add("GOAL SIDE RED - 4 Pair", RobotAutonGoalSideRed4Pair::new);
            });
            s.folder("Move Forward (requires relocalization in teleop)", l -> {
                l.add("Move Forward (intake is front)", MoveForwardAuton::new);
            });
        });
    }

    @Override
    public void onSelect() {

    }
}