package org.firstinspires.ftc.teamcode.teleop;




import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.annotation.SuppressLint;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PresetPoses;
import org.firstinspires.ftc.teamcode.subsystems.CRServoStorage;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.FileController;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.ServoStorage;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * This is the Tuning class. It contains a selection menu for various tuning OpModes.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 6/26/2025
 */
@Configurable
@TeleOp(name = "SELECTABLE - PedroPathing TeleOp", group = "TeleOp")
public class SelectableTeleOp extends SelectableOpMode {

    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();

    public SelectableTeleOp() {
        super("Select a Tuning OpMode", l -> {
            l.add("FAR SIDE - BLUE", FarSideBlue::new);
            l.add("FAR SIDE - RED", FarSideRed::new);
            l.add("GOAL SIDE - BLUE", GoalSideBlue::new);
            l.add("GOAL SIDE - RED", GoalSideRed::new);
            l.add("OUT OF WAY - BLUE", OOWBlue::new);
            l.add("OUT OF WAY - RED", OOWRed::new);
            l.add("AUTO END POSE - ANY", EndPoseTeleop::new);
        });
    }

    @Override
    public void onSelect() {

    }
}

abstract class BaseTeleop extends OpMode {
    Hardware robotHardware = new Hardware();
    Intake robotIntake;
    ServoStorage robotStorage;
    Outtake robotOuttake;
    Gate gate;
    boolean gateIsClosed = false;
    public GamepadManager g1Manager, g2Manager;

    public String initMsg;

    // pp vars

    public PresetPoses poses;
    public static Pose currentPose;
    private Follower follower;
    public boolean autoDriving = false;
    private Supplier<PathChain> toShootPosePath;
    private Function<Pose, PathChain> toLaunchLinePath;
    private TelemetryManager panelsTelemetry;
    public double tpsTolerance = 50;


    // input vars
    boolean fieldCentric = false;
    double fcOffset;

    public Pose holdingPose = new Pose(0,0,0);

    // Auto shoot
    double initialPower;
    double powerOffset = 0;
    public static double slowModeMultiplier = 0.3;

    // toggles
    boolean ballCamToggle = false;
    PIDFController headingPIDFController = new PIDFController(new PIDFCoefficients(0,0,0,0));
    boolean slowMode = false;
    boolean offToggle = false;

    // for button wasPressed detection
    boolean home1prevState = false;
    boolean options1prevState = false;
    boolean dpd1prevState = false;
    boolean dpr1prevState = false;
    boolean lt1prevState = false;
    boolean rb1prevState = false;
    boolean lb1prevState = false;
    boolean b1prevState = false;
    boolean dpu1prevState = false;
    boolean x1prevState = false;
    boolean y1prevState = false;
    boolean dpu2prevState = false; // tune preset increment
    boolean dpd2prevState = false; // tune preset decrement
    boolean a2prevState = false;
    boolean dpr2prevState = false;
    boolean dpl2prevState = false;
    boolean lb2prevState = false;
    boolean lt2prevstate = false;
    boolean a1prevState = false;
    boolean b2prevState = false;
    boolean y2prevState = false;
    boolean x2prevState = false;

    private void log(String caption, Object... text) {
        if (text.length == 1) {
            telemetry.addData(caption, text[0]);
            panelsTelemetry.debug(caption + ": " + text[0]);
        } else if (text.length >= 2) {
            StringBuilder message = new StringBuilder();
            for (int i = 0; i < text.length; i++) {
                message.append(text[i]);
                if (i < text.length - 1) message.append(" ");
            }
            telemetry.addData(caption, message.toString());
            panelsTelemetry.debug(caption + ": " + message);
        }
    }

    public void drawOnlyCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }
    public void draw() {
        Drawing.drawDebug(follower);
    }

    @Override
    public void init() {
        setPresets();
        // Initialize hardware
        robotHardware.initialize(hardwareMap,true, false);
        robotIntake = new Intake(robotHardware);
        robotStorage = new ServoStorage(robotHardware);
        robotOuttake = new Outtake(robotHardware, 200d, 0d, 0d, 13.989d);
        gate = new Gate(robotHardware);

//        poses = new PresetPoses(startPose, false);
        powerOffset = 0;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(poses.startPose);
        follower.update();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        Drawing.init();


        toShootPosePath = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, poses.closeShootPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, poses.closeShootPose.getHeading(), 0.8))
                .build();

        toLaunchLinePath = (Pose targetPose) -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, targetPose::getHeading, 0.8))
                .build();


        g1Manager = PanelsGamepad.INSTANCE.getFirstManager();
        g2Manager = PanelsGamepad.INSTANCE.getSecondManager();

        log("Status", "INITIALIZED");
        log(initMsg);
        panelsTelemetry.update(telemetry);

        drawOnlyCurrent();

    }

    @Override
    public void init_loop() {
        follower.update();
        log("Status", "INITIALIZED");
        log(initMsg);
        panelsTelemetry.update(telemetry);
        drawOnlyCurrent();

    }

    @Override
    public void start() {


        follower.update();
        robotHardware.imu.resetYaw();
        follower.startTeleOpDrive();


    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        headingPIDFController.setCoefficients(follower.constants.coefficientsHeadingPIDF);
        currentPose = follower.getPose();
        Gamepad g1 = g1Manager.asCombinedFTCGamepad(gamepad1);
        Gamepad g2 = g2Manager.asCombinedFTCGamepad(gamepad2);


        //      Gamepad 1 inputs
        double ly1 = -g1.left_stick_y; // forward/backward driving
        double lx1 = -g1.left_stick_x; // strafing
        double rx1 = g1.right_stick_x / 2; // turning (decrease by factor of 2)
        slowMode = g1.right_trigger >= 0.3;
        boolean home1state = g1.guide; // reset yaw value on gyro
        boolean options1state = g1.options; // field centric toggle
        boolean dpd1state = g1.dpad_down; // go to closest shoot pose
        boolean dpr1state = g1.dpad_right;
        boolean b1state = g1.b; // ball cam toggle
        boolean dpu1state = g1.dpad_up; // go to closest launch line pose
        boolean x1state = g1.x; // hp localize
        boolean a1state = g1.a; // left localize
        boolean y1state = g1.y; // abort autonomous drive
        boolean lt1state = g1.left_trigger > 0.3; // hold pose

        boolean home1wP = home1state && !home1prevState;
        boolean options1wP = options1state && !options1prevState;
        boolean dpd1wP = dpd1state && !dpd1prevState;
        boolean b1wP = b1state && !b1prevState;
        boolean dpu1wP = dpu1state && !dpu1prevState;
        boolean x1wP = x1state && !x1prevState;
        boolean y1wP = y1state && !y1prevState;
        boolean a1wP = a1state && !a1prevState;
        boolean dpr1wP = dpr1state && !dpr1prevState;

        //      Gamepad 2 inputs
        double ly2 = g2.left_stick_y; // robot intake run
        boolean rt2state = g2.right_trigger > 0.3; // storage forward
        boolean rb2state = g2.right_bumper; // storage reverse
        boolean a2state = g2.a; // outtake idle on/off
        boolean b2state = g2.b; // outtake preset for close shoot
        boolean y2state = g2.y; // outtake preset for far shoot
        boolean x2state = g2.x; // set outtake power based on power tables
        boolean dpu2state = g2.dpad_up; // tune preset increment
        boolean dpd2state = g2.dpad_down; // tune preset decrement
        boolean dpr2state = g2.dpad_right;
        boolean dpl2state = g2.dpad_left;
        boolean lb2state = g2.left_bumper; // gate toggle
        boolean lt2state = g2.left_trigger > 0.3;

        boolean a2wP = a2state && !a2prevState;
        boolean dpu2wP = dpu2state && !dpu2prevState;
        boolean dpd2wP = dpd2state && !dpd2prevState;
        boolean dpr2wP = dpr2state && !dpr2prevState;
        boolean dpl2wP = dpl2state && !dpl2prevState;
        boolean lb2wP = lb2state && !lb2prevState;
        boolean lt1wP = lt1state && !lt1prevState;
        boolean lt2wp = lt2state && !lt2prevstate;


        double imuHeading = robotHardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // rumble outtake feedback
        if (robotOuttake.getCurrentPreset().equals("idle")) {gamepad2.stopRumble();}
        else if (Math.abs(robotHardware.outtakeMotor.getVelocity() - robotOuttake.getTargetTps()) < tpsTolerance) {
            gamepad2.rumble(-1); // infinite
        }
        else {gamepad2.stopRumble();}

        //      Drivetrain Control
        //Field centric toggle
        if (home1wP && fieldCentric) {robotHardware.imu.resetYaw();}
        if (options1wP) {fieldCentric = !fieldCentric;}

        if (!autoDriving) {
            if (slowMode) {ly1*=slowModeMultiplier; lx1*=slowModeMultiplier; rx1*=slowModeMultiplier;}

            if (fieldCentric) { // field centric
                if (ballCamToggle) {

                    double targetHeading = poses.getAngleTowardsGoal(currentPose) - Math.toRadians(180);

                    double error = MathFunctions.getTurnDirection(follower.getPose().getHeading(), targetHeading)
                            * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), targetHeading);

                    headingPIDFController.updateError(error);

                    follower.setTeleOpDrive(ly1, lx1, headingPIDFController.run(), false, fcOffset);

                }
                else {follower.setTeleOpDrive(ly1, lx1, -rx1, false, fcOffset);} // normal field centric
            }
            else {follower.setTeleOpDrive(ly1, lx1, -rx1, true);} // rbt centric
        }

        if (!poses.isRed && x1wP) {
            follower.setPose(PresetPoses.LOCALIZE_POSE_RIGHT);
            powerOffset = 0;
        }
        else if (poses.isRed && x1wP) {
            follower.setPose(PresetPoses.LOCALIZE_POSE_LEFT);
            powerOffset = 0;
        }

        if (!poses.isRed && a1wP) {
            follower.setPose(PresetPoses.CLOSE_LOCALIZE_POSE_LEFT);
            powerOffset = 0;
        }
        else if (poses.isRed && a1wP) {
            follower.setPose(PresetPoses.CLOSE_LOCALIZE_POSE_RIGHT);
            powerOffset = 0;
        }

        // robot gate toggle (lb2wP)
        if (lb2wP) {
            gateIsClosed = !gateIsClosed;
            if(gateIsClosed){{gate.setGateState("close");}}
            else {gate.setGateState("open");}
        }

        // ball cam toggle
        if (b1wP) {ballCamToggle = !ballCamToggle;}

        // hold pos dpr1
        if (!follower.isBusy()) {
            if (lt1wP) {
                holdingPose = follower.getPose();
                follower.holdPoint(holdingPose);
            }
            if (!lt1state && lt1prevState) {
                follower.breakFollowing();
                follower.startTeleOpDrive();
            }
        }

        //Storage and intake control
        robotIntake.update();
        robotStorage.update();

        if (ly2 <= -0.3) {robotIntake.run(-0.8);}
        else if(ly2 >0.3) {robotIntake.run(0.8); robotStorage.setPos(-1);}
        else {
            robotIntake.run(0);
        }

        // outtake preset running
        if (a2wP) {robotOuttake.reset();}
        else if (y2state)
        {
            robotOuttake.run("close");
            gate.setGateState("open");
            if (robotOuttake.upToSpeed) {robotStorage.cycle(true);}
        }
        else if (y2prevState) {gate.setGateState("close");}
        else if (b2state)
        {
            robotOuttake.run("far");
            gate.setGateState("open");
            if(robotOuttake.upToSpeed) {robotStorage.cycle(true);}
        }
        else if (b2prevState) {gate.setGateState("close");}
        else if (x2state) {
            initialPower = poses.getOptimalShooterPowerPercentage(follower.getPose(), true);
            robotOuttake.run(initialPower + powerOffset);
            if(robotOuttake.upToSpeed){robotStorage.cycle(true);}
            gate.setGateState("open");
            if (dpu2wP) {powerOffset += 0.01;}
            else if (dpd2wP) {powerOffset -= 0.01;}
            if(a2state){powerOffset = 0;}
        }
        else if (x2prevState) {gate.setGateState("close");}
        else {
            robotOuttake.run("idle");
            robotStorage.cycle(false);
            }
        if (offToggle) {robotOuttake.run(0);}

        // Fine tune active preset
        if (dpu2wP && !offToggle && !x2state) {robotOuttake.tuneActivePreset(0.05);}
        else if (dpd2wP && !offToggle && !x2state) {robotOuttake.tuneActivePreset(-0.05);}

        if (home1wP) {fcOffset = follower.getHeading();}

        // Reset outtake presets
        if (options1wP) {robotOuttake.reset();}
        if (dpu1wP) {autoDriving = true; follower.followPath(toShootPosePath.get()); }
        if (dpd1wP) {autoDriving = true; follower.followPath(toLaunchLinePath.apply(poses.findClosestLaunchPose(currentPose)));}
        if ((autoDriving) && (y1wP || !follower.isBusy())) {
            follower.startTeleOpDrive(); autoDriving = false;
        }

        if (!x2state && !y2state && !b2state && !(ly2 > 0.3)) {
            robotStorage.setPos(0);
            robotStorage.cycle(false);
        }

        log(initMsg);
        log("Status", "Running");
        log("Field Centric", fieldCentric ? "ON" : "OFF");
        log("Ball Cam", ballCamToggle ? "ON" : "OFF");
        log("Following Path", follower.isBusy() ? "FOLLOWING" : "none");
        //log("Auto-Shooting", isAutoShooting ? "ACTIVE" : "IDLE");
        log("Target Velocity (tps)", robotOuttake.getTargetTps());
        log("Actual Velocity (tps)", robotHardware.outtakeMotor.getVelocity());
        log("IMU Heading (deg)", Math.toDegrees(imuHeading));
        log("Position", String.format("X: %.2f, Y: %.2f", currentPose.getX(), currentPose.getY()));
        log("Goal Pose", poses.goalPose);
        log("Target Angle for Goal", Math.toDegrees(poses.getAngleTowardsGoal(currentPose)));
        panelsTelemetry.update(telemetry);
        follower.update();
        draw();


        home1prevState = home1state;
        options1prevState = options1state;
        dpd1prevState = dpd1state;
        b1prevState = b1state;
        dpu1prevState = dpu1state;
        x1prevState = x1state;
        a1prevState = a1state;
        y1prevState = y1state;
        dpu2prevState = dpu2state; // tune preset increment
        dpd2prevState = dpd2state; // tune preset decrement
        a2prevState = a2state;
        dpr2prevState = dpr2state;
        dpl2prevState = dpl2state;
        lb2prevState = lb2state;
        b2prevState = b2state;
        y2prevState = y2state;
        dpr1prevState = dpr1state;
        x2prevState = x2state;
        lt1prevState = lt1state;

    }

    public abstract void setPresets();
}
class FarSideBlue extends BaseTeleop {
    @Override
    public void setPresets() {
        PresetPoses tempposes = new PresetPoses(new Pose(72,72, Math.toRadians(0)), false);
        poses = new PresetPoses(tempposes.parkPose, false);
        fcOffset = Math.toRadians(180);
        initMsg = "FAR SIDE BLUE";

    }
}

class FarSideRed extends BaseTeleop {
    @Override
    public void setPresets() {
        PresetPoses tempposes = new PresetPoses(new Pose(72,72, Math.toRadians(0)), false);
        poses = new PresetPoses(tempposes.parkPose, true);
        fcOffset = 0;
        initMsg = "FAR SIDE RED";

    }
}

class GoalSideBlue extends BaseTeleop {
    @Override
    public void setPresets() {
        PresetPoses tempposes = new PresetPoses(new Pose(72,72, Math.toRadians(0)), false);
        poses = new PresetPoses(tempposes.closeShootOffLinePose, false);
        fcOffset = Math.toRadians(180);
        initMsg = "GOAL SIDE BLUE";


    }
}

class GoalSideRed extends BaseTeleop {
    @Override
    public void setPresets() {
        PresetPoses tempposes = new PresetPoses(new Pose(72,72, Math.toRadians(0)), false);
        poses = new PresetPoses(tempposes.closeShootOffLinePose, true);
        fcOffset = 0;
        initMsg = "GOAL SIDE RED";


    }
}

class OOWBlue extends BaseTeleop {
    @Override
    public void setPresets() {
        PresetPoses tempposes = new PresetPoses(new Pose(72,72, Math.toRadians(0)), false);
        poses = new PresetPoses(tempposes.closeMoveOutOfWayPose, false);
        fcOffset = Math.toRadians(180);
        initMsg = "OUT OF WAY BLUE";

    }
}

class OOWRed extends BaseTeleop {
    @Override
    public void setPresets() {
        PresetPoses tempposes = new PresetPoses(new Pose(72,72, Math.toRadians(0)), false);
        poses = new PresetPoses(tempposes.closeMoveOutOfWayPose, true);
        fcOffset = 0;
        initMsg = "OUT OF WAY RED";


    }
}

class EndPoseTeleop extends BaseTeleop {
    @Override
    public void setPresets() {
        List<Double> data = FileController.read("Memory.txt");
        double x = data.get(0);
        double y = data.get(1);
        double heading = data.get(2);
        boolean isRed = data.get(3) == 1;
        Pose startPose = new Pose(x,y,heading);

        if (isRed) {
            poses = new PresetPoses(startPose.mirror(), true);
        } else {
            poses = new PresetPoses(startPose, false);
        }
        fcOffset = Math.toRadians(180);
        initMsg = "AUTON POSE TELEOP";


    }
}
