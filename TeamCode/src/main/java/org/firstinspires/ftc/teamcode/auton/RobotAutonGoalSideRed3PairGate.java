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

@Autonomous(name="GOAL SIDE RED - Gate Auton", group="Autonomous", preselectTeleOp = "SELECTABLE - PedroPathingTeleOp")
@Configurable // for Panels
@SuppressWarnings("FieldCanBeLocal") // android studio bugging
public class RobotAutonGoalSideRed3PairGate extends OpMode {

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
    public double previousTime = 0;

    // intake sequence
    public boolean intaking = false;
    public static double storageOuttakePower = 1;

    public static double intakeMaxPower = 1;


    private Paths paths;


    public static class Paths {


        public PathChain ShootPreloaded;
        public PathChain GotoPGP;
        public PathChain PickupPGP;
        public PathChain ShootPGP;
        public PathChain GotoGPP;
        public PathChain PickupGPP;
        public PathChain ShootGPP;
        public PathChain GotoLever;

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


            GotoGPP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    poses.closeShootPose,
                                    poses.gppStartPose
                            )
                    )
                    .setLinearHeadingInterpolation(poses.closeShootPose.getHeading(), poses.gppStartPose.getHeading())
                    .build();


            PickupGPP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(poses.gppStartPose, poses.gppEndPose)
                    )
                    .setTangentHeadingInterpolation()
                    .build();


            ShootGPP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(poses.gppEndPose, poses.closeShootPose)
                    )
                    .setLinearHeadingInterpolation(poses.gppEndPose.getHeading(), poses.closeShootPose.getHeading())
                    .build();


            GotoPGP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(poses.closeShootPose, poses.pgpStartPose)
                    )
                    .setLinearHeadingInterpolation(poses.closeShootPose.getHeading(), poses.pgpStartPose.getHeading())
                    .build();


            PickupPGP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(poses.pgpStartPose, poses.pgpEndPose)
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierCurve(
                                    poses.pgpEndPose,
                                    poses.leverPoseppgEndPoseCurvePose,
                                    poses.leverPose
                            )
                    )
                    .setLinearHeadingInterpolation(poses.pgpEndPose.getHeading(), poses.leverPose.getHeading())
                    .build();


            ShootPGP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    poses.leverPose,
                                    poses.closeShootOffLinePose
                            )
                    )
                    .setLinearHeadingInterpolation(poses.leverPose.getHeading(), poses.closeShootOffLinePose.getHeading())
                    .build();


            GotoLever = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(poses.closeShootPose, poses.parkLeverPose)
                    )
                    .setLinearHeadingInterpolation(poses.closeShootPose.getHeading(), poses.parkLeverPose.getHeading())
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
        updatePath(outtake.update(), intake.update(), storage.update());

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
    public void updatePath(boolean outtakeRFTFinished, boolean intakeRFTFinished, boolean storageRFTFinished) {
        switch (pathState) {
            case 0:
                outtake.run("close");
                follower.followPath(paths.ShootPreloaded);
                nextState = 1;
                pathState = 10; // shoot
                break;

            case 1:
                if (follower.getCurrentTValue() > 0.97) {
                    follower.followPath(paths.GotoPGP);
                    pathState = 2;
                }
                break;

            case 2:
                if (follower.getCurrentTValue() > 0.97) {
                    follower.followPath(paths.PickupPGP);
                    gate.setGateState("close");
                    intake.run(1); storage.setPos(-1);
                    pathState = 3;
                    previousTime = timer.milliseconds();
                }
                break;
            case 3:
                if (timer.milliseconds() + 3000 <= previousTime) {
                    pathState = 4;
                }
            case 4:
                if (follower.getCurrentTValue() > 0.97) {
                    intake.run(0); storage.setPos(0);
                    follower.followPath(paths.ShootPGP);
                    nextState = 5;
                    pathState = 10; // shoot
                }
                break;

            case 5:
                if (follower.getCurrentTValue() > 0.97) {
                    follower.followPath(paths.GotoGPP);
                    outtake.run(0.445);
                    pathState = 6;
                }
            case 6:
                if (follower.getCurrentTValue() > 0.97) {
                    follower.followPath(paths.PickupGPP);
                    gate.setGateState("close");
                    intake.run(1); storage.setPos(-1);
                    pathState = 7;
                }
                break;
            case 7:
                if (follower.getCurrentTValue() > 0.97) {
                    intake.run(0); storage.setPos(0);
                    follower.followPath(paths.ShootGPP);
                    nextState = 8;
                    pathState = 11; // shoot
                }
                break;
            case 8:
                if(follower.getCurrentTValue() > 0.97) {
                    pathState = -1;
                    outtake.run(0);
                }




//            case 4:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.GotoPGP);
//                    pathState = 5;
//                }
//                break;
//
//            case 5:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.PickupPGP, 0.3, true);
//                    gate.setGateState(true);
//                    intake.run(1);
//                    pathState =6;
//                }
//                break;
//
//            case 6:
//                if (!follower.isBusy()) {
//                    intake.run(0);
//                    follower.followPath(paths.ShootPGP);
//                    nextState = 7;
//                    pathState = 10; // shoot
//                }
//                break;
//
//            case 7:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.GotoLever);
//                    outtake.run(0);
//                    pathState = -1; // terminate
//                }
//                break;

            case 10: // shoot
                if (follower.getCurrentTValue() > 0.97) {
                    if (!rampingUp && !shooting) {rampingUp = true;}
                    else if (rampingUp && Math.abs(hardware.outtakeMotor.getVelocity() - outtake.getTargetTps()) < 40) {
                        previousTime = hardware.timer.milliseconds();
                        gate.setGateState("open");
                        intake.run(1); storage.cycle(3);
                        shooting = true;
                        rampingUp = false;
                    }
                    else if (shooting) {
                        if ((storageRFTFinished)) {
                            gate.setGateState("close");
                            rampingUp = false;
                            shooting = false;
                            pathState = nextState;
                            intake.run(0);
                            storage.cycle(false);
                        }
                    }
                }
                break;
            case 11: // shoot
                if (follower.getCurrentTValue() > 0.97) {
                    if (!rampingUp && !shooting) {rampingUp = true;}
                    else if (rampingUp && Math.abs(hardware.outtakeMotor.getVelocity() - outtake.getTargetTps()) < 40) {
                        previousTime = hardware.timer.milliseconds();
                        gate.setGateState("open");
                        intake.run(1); storage.cycle(true);
                        shooting = true;
                        rampingUp = false;
                    }
                    else if (shooting) {
                        if ((hardware.timer.milliseconds() >= previousTime + 100000)) {
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

        }
    }
}
