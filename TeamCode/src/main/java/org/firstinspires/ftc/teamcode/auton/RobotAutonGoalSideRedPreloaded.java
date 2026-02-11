package org.firstinspires.ftc.teamcode.auton;



import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.robotcore.util.ElapsedTime;
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


@Autonomous(name="GOAL SIDE RED - Only Preloaded Auton", group="Autonomous", preselectTeleOp = "SELECTABLE - PedroPathingTeleOp")
@Configurable // for Panels
@SuppressWarnings("FieldCanBeLocal") // android studio bugging
public class RobotAutonGoalSideRedPreloaded extends OpMode {

    public Hardware hardware;
    public Outtake outtake;
    public Intake intake;
    public Gate gate;
    public ServoStorage storage;


    private final ElapsedTime timer = new ElapsedTime(); // runtime
    // other vars
    private Pose currentPose;
    public Follower follower;
    private TelemetryManager panelsTelemetry;
    private int pathState;
    private int nextState;

    // shoot sequence
    public boolean rampingUp = false;
    public boolean shooting = false;

    // intake sequence
    public boolean intaking = false;
    public static double storageOuttakePower = 1;

    public static double intakeMaxPower = 1;


    private double previousTime;
    private Paths paths;


    public static class Paths {


        public PathChain ShootPreloaded;
        public PathChain MoveOutOfWay;

        private Pose startPose = new Pose(28.5, 136, Math.toRadians(270));
        private PresetPoses poses = new PresetPoses(startPose, true);


        public Paths(Follower follower) {
            ShootPreloaded = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(poses.startPose, poses.closeShootPose)
                    )
                    .setLinearHeadingInterpolation(startPose.getHeading(), poses.closeShootPose.getHeading())
                    .build();


            MoveOutOfWay = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    poses.closeShootPose,
                                    poses.closeMoveOutOfWayPose
                            )
                    )
                    .setLinearHeadingInterpolation(poses.closeShootPose.getHeading(), poses.gppStartPose.getHeading())
                    .build();

        }
    }

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
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // init pp follower
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        follower.setStartingPose(paths.poses.startPose);
        follower.update();

        // init subsystems
        hardware = new Hardware();
        hardware.initialize(hardwareMap, true, false);
        outtake = new Outtake(hardware, 200d, 0d, 0d, 13.989d);
        intake = new Intake(hardware);
        storage = new ServoStorage(hardware);
        gate = new Gate(hardware);

        Drawing.init();

        drawOnlyCurrent();
        log("Status", "INITIALIZED");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        pathState = 0;
        timer.reset();


        follower.update();
        panelsTelemetry.update();
        currentPose = follower.getPose();

        gate.setGateState("close");
    }

    @Override
    public void loop() {
        currentPose = follower.getPose();
        // update subsystems
        updatePath(outtake.update(), intake.update());
        storage.update();

        // telemetry
        log("Status", "RUNNING");
        log("Path State", pathState);
        log("Current Pose", currentPose);
        panelsTelemetry.update(telemetry);
        follower.update();
        draw();
    }

    @Override
    public void stop() {
        List<Double> data = new ArrayList<>();
        data.add(follower.getPose().getX());
        data.add(follower.getPose().getY());
        data.add(follower.getPose().getHeading());
        if (paths.poses.isRed) {
            data.add(1.0);
        } else {
            data.add(0.0);
        }
        FileController.write("Memory.txt", data);
    }

    // shootpreloaded-->gotoppg-->pickupppg-->shootppg-->park
    public void updatePath(boolean outtakeRFTFinished, boolean intakeRFTFinished) {
        switch (pathState) {
            case 0:
                follower.followPath(paths.ShootPreloaded);
                nextState = 1;
                pathState = 10; // shoot
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.MoveOutOfWay);
                    pathState = -1;
                }
                break;


            case 10: // shoot
                if (!follower.isBusy()) {
                    if (!rampingUp && !shooting) {outtake.run("close"); rampingUp = true;}
                    else if (rampingUp && Math.abs(hardware.outtakeMotor.getVelocity() - outtake.getTargetTps()) < 40) {
                        gate.setGateState("open");
                        intake.runForTime(1, 6500);
                        shooting = true;
                        rampingUp = false;
                    }
                    else if (shooting && (intakeRFTFinished)) {
                        gate.setGateState("close");
                        rampingUp = false; shooting = false;
                        outtake.run("idle");
                        pathState = nextState;
                    }
                }
                break;
            case 11: // shoot
                if (!follower.isBusy()) {
                    if (!rampingUp && !shooting) {outtake.run("close"); rampingUp = true;}
                    else if (rampingUp && Math.abs(hardware.outtakeMotor.getVelocity() - outtake.getTargetTps()) < 40) {
                        gate.setGateState("open");
                        intake.runForTime(1, 100000); storage.cycle(true);
                        shooting = true;
                        rampingUp = false;
                    }
                    else if (shooting && (intakeRFTFinished)) {
                        gate.setGateState("close");
                        rampingUp = false; shooting = false;
                        storage.cycle(false);
                        outtake.run("idle");
                        pathState = nextState;
                    }
                }
                break;

        }
    }
}
