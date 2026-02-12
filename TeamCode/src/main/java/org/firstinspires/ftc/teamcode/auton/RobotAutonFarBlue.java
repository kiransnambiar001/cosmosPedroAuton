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


@Autonomous(name="FAR SIDE BLUE - Auton", group="Autonomous", preselectTeleOp="SELECTABLE - PedroPathingTeleOp")
@Configurable // for Panels
@SuppressWarnings("FieldCanBeLocal") // android studio bugging
public class RobotAutonFarBlue extends OpMode {

    public Hardware hardware;
    public Outtake outtake;
    public Intake intake;
    public Gate gate;
    public ServoStorage storage;
    public FileController fileController;


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
        public PathChain GotoPPG;
        public PathChain PickupPPG;
        public PathChain ShootPPG;
        public PathChain Park;

        private Pose startPose = new Pose(56.000, 8.000, Math.toRadians(270));
        private PresetPoses poses = new PresetPoses(startPose, false);


        public Paths(Follower follower) {
            ShootPreloaded = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(poses.startPose, poses.farShootPose)
                    )
                    .setLinearHeadingInterpolation(poses.startPose.getHeading(), poses.farShootPose.getHeading())
                    .build();



            GotoPPG = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    poses.farShootPose,
                                    poses.farShootPpgStartMidCurvePose,
                                    poses.ppgStartPose
                            )
                    )
                    .setLinearHeadingInterpolation(poses.farShootPose.getHeading(), poses.ppgStartPose.getHeading())
                    .build();


            PickupPPG = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(poses.ppgStartPose, poses.ppgEndPose)
                    )
                    .setTangentHeadingInterpolation()
                    .build();


            ShootPPG = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(poses.ppgEndPose, poses.farShootPose)
                    )
                    .setLinearHeadingInterpolation(poses.ppgEndPose.getHeading(), poses.farShootPose.getHeading())
                    .build();


            Park = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(poses.farShootPose, poses.parkPose)
                    )
                    .setLinearHeadingInterpolation(poses.farShootPose.getHeading(), poses.parkPose.getHeading())
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
        log("Outtake Tps: ", hardware.outtakeMotor.getVelocity());
        log("Outtake Target Tps: ", outtake.getTargetTps());
        if (pathState == 0) {log("Path Name", "Shoot Preloaded");}
        else if (pathState == 1) {log("Path Name", "Go to PPG");}
        else if (pathState == 2) {log("Path Name", "Pickup PPG");}
        else if (pathState == 3) {log("Path Name", "Shoot GPP");}
        else if (pathState == 4) {log("Path Name", "Park");}
        else if (pathState == 10) {log("Path Name", "Shooting (Outtake)");}
        else if (pathState == -1) {log("Path Name", "Autonomous Finished!");}
        log("Current Pose", currentPose);
        panelsTelemetry.update(telemetry);
        follower.update();
        draw();
    }



    // shootpreloaded-->gotoppg-->pickupppg-->shootppg-->park
    public void updatePath(boolean outtakeRFTFinished, boolean intakeRFTFinished) {
        switch (pathState) {
            case 0:
                outtake.run(0.5);
                follower.followPath(paths.ShootPreloaded, true);
                nextState = 1;
                pathState = 10; // shoot
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GotoPPG);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PickupPPG);
                    gate.setGateState("close");
                    intake.run(1);
                    storage.setPos(-1);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    intake.run(0);
                    storage.setPos(0);
                    follower.followPath(paths.ShootPPG);
                    nextState = 4;
                    pathState = 12; // shoot
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Park);
                    outtake.run(0);
                    pathState = -1; // terminate
                }
                break;

            case 10: // shoot
                if (!follower.isBusy()) {
                    if (!rampingUp && !shooting) {rampingUp = true; outtake.run(0.5);}
                    else if (rampingUp && Math.abs(hardware.outtakeMotor.getVelocity() - outtake.getTargetTps()) < 40) {
                        follower.holdPoint(paths.poses.farShootPose);
                        previousTime = hardware.timer.milliseconds();
                        gate.setGateState("open");
                        intake.run(1); storage.cycle(true);
                        shooting = true;
                        rampingUp = false;
                    }
                    else if (shooting) {
                        if ((hardware.timer.milliseconds() >= previousTime + 7250)) {
                            gate.setGateState("close");
                            rampingUp = false; shooting = false;
                            pathState = nextState;
                            intake.run(0); storage.cycle(false);
                            follower.breakFollowing();
                        }
                        else if (Math.abs(hardware.outtakeMotor.getVelocity() - outtake.getTargetTps()) < 40) {
                            intake.run(1); storage.cycle(true);
                        }
                        else {
                            intake.run(0); storage.cycle(false);
                        }
                    }
                }
                break;
            case 11: // shoot
                if (!follower.isBusy()) {
                    if (!rampingUp && !shooting) {rampingUp = true;}
                    else if (rampingUp && Math.abs(hardware.outtakeMotor.getVelocity() - outtake.getTargetTps()) < 40) {
                        previousTime = hardware.timer.milliseconds();
                        gate.setGateState("open");
                        intake.run(1); storage.cycle(true);
                        shooting = true;
                        rampingUp = false;
                    }
                    else if (shooting) {
                        if ((hardware.timer.milliseconds() >= previousTime + 100000 )) {
                            gate.setGateState("close");
                            rampingUp = false; shooting = false;
                            pathState = nextState;
                            intake.run(0); storage.cycle(false);
                        }
                        else if (Math.abs(hardware.outtakeMotor.getVelocity() - outtake.getTargetTps()) < 40) {
                            intake.run(1); storage.cycle(true);
                        }
                        else {
                            intake.run(0); storage.cycle(false);
                        }
                    }
                }
                break;

            case 12: // shoot
                if (!follower.isBusy()) {
                    if (!rampingUp && !shooting) {rampingUp = true; outtake.run(0.53);}
                    else if (rampingUp && Math.abs(hardware.outtakeMotor.getVelocity() - outtake.getTargetTps()) < 40) {
                        follower.holdPoint(paths.poses.farShootPose);
                        previousTime = hardware.timer.milliseconds();
                        gate.setGateState("open");
                        intake.run(1); storage.cycle(true);
                        shooting = true;
                        rampingUp = false;
                    }
                    else if (shooting) {
                        if ((hardware.timer.milliseconds() >= previousTime + 7250)) {
                            gate.setGateState("close");
                            rampingUp = false; shooting = false;
                            pathState = nextState;
                            intake.run(0); storage.cycle(false);
                            follower.breakFollowing();
                        }
                        else if (Math.abs(hardware.outtakeMotor.getVelocity() - outtake.getTargetTps()) < 40) {
                            intake.run(1); storage.cycle(true);
                        }
                        else {
                            intake.run(0); storage.cycle(false);
                        }
                    }
                }
                break;

        }
    }
}
