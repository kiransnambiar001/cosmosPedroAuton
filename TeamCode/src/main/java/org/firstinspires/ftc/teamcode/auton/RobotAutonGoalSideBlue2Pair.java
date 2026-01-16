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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PresetPoses;
import org.firstinspires.ftc.teamcode.subsystems.CRServoStorage;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;


@Autonomous(name="GOAL SIDE BLUE - Auton", group="Autonomous")
@Configurable // for Panels
@SuppressWarnings("FieldCanBeLocal") // android studio bugging
public class RobotAutonGoalSideBlue2Pair extends LinearOpMode {

    public Hardware hardware;
    public Outtake outtake;
    public Intake intake;
    public Gate gate;
    public CRServoStorage storage;


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
        public PathChain GotoPGP;
        public PathChain PickupPGP;
        public PathChain ShootPGP;
        public PathChain GotoGPP;
        public PathChain PickupGPP;
        public PathChain ShootGPP;
        public PathChain GotoLever;

        private Pose startPose = new Pose(28.5, 136, Math.toRadians(270));
        private PresetPoses poses = new PresetPoses(startPose, false);


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
                    .build();


            ShootPGP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    poses.pgpEndPose,
                                    poses.closeShootPgpStartMidCurvePose,
                                    poses.closeShootPose
                            )
                    )
                    .setLinearHeadingInterpolation(poses.pgpEndPose.getHeading(), poses.closeShootPose.getHeading())
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
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // init pp follower
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        follower.setStartingPose(paths.poses.startPose);
        follower.update();

        // init subsystems
        hardware = new Hardware();
        hardware.initialize(hardwareMap, true);
        outtake = new Outtake(hardware, 200d, 0d, 0d, 13.989d);
        intake = new Intake(hardware);
        storage = new CRServoStorage(hardware);
        gate = new Gate(hardware);

        Drawing.init();

        drawOnlyCurrent();
        log("Status", "INITIALIZED");
        panelsTelemetry.update(telemetry);


        // upon start operations
        waitForStart();
        pathState = 0;
        timer.reset();


        follower.update();
        panelsTelemetry.update();
        currentPose = follower.getPose();

        gate.setGateState(true);


        while (opModeIsActive()) {
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
    }

    // shootpreloaded-->gotoppg-->pickupppg-->shootppg-->park
    public void updatePath(boolean outtakeRFTFinished, boolean intakeRFTFinished, boolean storageRFTFinished) {
        switch (pathState) {
            case 0:
                follower.followPath(paths.ShootPreloaded);
                nextState = 1;
                pathState = 10; // shoot
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GotoGPP);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PickupGPP, 0.3, true);
                    gate.setGateState(true);
                    intake.run(1);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    intake.run(0);
                    follower.followPath(paths.ShootGPP);
                    nextState = 4;
                    pathState = 10; // shoot
                }
                break;

            case 4:
                pathState=10;

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
                if (!follower.isBusy()) {
                    if (!rampingUp && !shooting) {outtake.run("close"); rampingUp = true;}
                    else if (rampingUp && Math.abs(hardware.outtakeMotor.getVelocity() - outtake.getTargetTps()) < 40) {
                        gate.setGateState(false);
                        intake.runForTime(1, 7000); storage.runForTime(storageOuttakePower, 7000);
                        shooting = true;
                        rampingUp = false;
                    }
                    else if (shooting && (intakeRFTFinished || storageRFTFinished)) {
                        gate.setGateState(true);
                        rampingUp = false; shooting = false;
                        outtake.run("idle");
                        pathState = nextState;
                    }
                }
                break;

        }
    }
}
