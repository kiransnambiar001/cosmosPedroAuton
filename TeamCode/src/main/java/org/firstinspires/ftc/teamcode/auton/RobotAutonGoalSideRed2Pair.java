package org.firstinspires.ftc.teamcode.auton;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PresetPoses;
import org.firstinspires.ftc.teamcode.subsystems.CRServoStorage;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;


@Autonomous(name="GOAL SIDE RED - 2Pair Auton", group="Robot")
@Configurable // for Panels
@SuppressWarnings("FieldCanBeLocal") // android studio bugging
public class RobotAutonGoalSideRed2Pair extends LinearOpMode {


    public Hardware hardware;
    public Outtake outtake;
    public Intake intake;
    public CRServoStorage storage;



    private final ElapsedTime timer = new ElapsedTime(); // runtime
    // other vars
    private Pose currentPose;
    public Follower follower;
    private TelemetryManager panelsTelemetry;
    public static int pathState;
    public static int nextState;






    private Paths paths;

    // shoot sequence
    public boolean rampingUp = false;
    public boolean shooting = false;


    public static class Paths {


        public PathChain ShootPreloaded;
        public PathChain GotoGPP;
        public PathChain PickupGPP;
        public PathChain ShootGPP;
        public PathChain GotoPGP;
        public PathChain PickupPGP;
        public PathChain ShootPGP;
        public PathChain GotoLever;

        private Pose startPose = new Pose(20.876, 122.886, Math.toRadians(325));

        public PresetPoses poses = new PresetPoses(startPose, true);



        public Paths(Follower follower) {
            ShootPreloaded = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(startPose, poses.closeShootPose)
                    )
                    .setLinearHeadingInterpolation(startPose.getHeading(), poses.closeShootPose.getHeading())
                    .build();


            GotoGPP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    poses.closeShootPose,
                                    new Pose(61.918, 94.181).mirror(),
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
                            new BezierLine(poses.pgpEndPose, poses.closeShootPose)
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
        // init intake and outtake
        hardware = new Hardware();
        hardware.initialize(hardwareMap, true);
        outtake = new Outtake(hardware, 200d, 0d, 0d, 13.989d);
        intake = new Intake(hardware);
        storage = new CRServoStorage(hardware);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


        // init pp follower
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        follower.setStartingPose(paths.poses.startPose);
        follower.update();

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


        while (opModeIsActive()) {
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose();
            updatePath(outtake.update(), intake.update(), storage.update());


            // telemetry
            log("Status", "RUNNING");
            log("Path State", pathState);
            log("Current Pose", currentPose);
            telemetry.update();
        }
    }

    // shoot preloaded, shoot
    // go to gpp
    // pickup gpp + intake
    // shoot gpp, shoot
    // go to pgp
    // pickup pgp + intake
    // shoot pgp, shoot
    // go to lever
    public void updatePath(boolean outtakeRFTFinished, boolean intakeRFTFinished, boolean storageRFTFinished) {
        switch (pathState) {
            case 0:
                follower.followPath(paths.ShootPreloaded);
                pathState = 10; // shoot
                nextState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GotoGPP);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PickupGPP, 0.5, true);
                    intake.run(1);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    intake.run(0);
                    follower.followPath(paths.ShootGPP);
                    pathState = 10; // shoot
                    nextState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GotoPGP);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PickupPGP, 0.5, true);
                    intake.run(1);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    intake.run(0);
                    follower.followPath(paths.ShootPGP);
                    pathState = 10; // shoot
                    nextState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GotoLever);
                    outtake.run(0);
                    pathState = -1; // terminate
                }
                break;

            case 10: // shoot
                if (!follower.isBusy()) {
                    if (!rampingUp && !shooting) {outtake.run("close"); rampingUp = true;}
                    else if (rampingUp && Math.abs(hardware.outtakeMotor.getVelocity() - outtake.getTargetTps()) < 40) {
                        intake.runForTime(1, 2000); storage.runForTime(1, 2000);
                        shooting = true;
                        rampingUp = false;
                    }
                    else if (shooting && (intakeRFTFinished || storageRFTFinished)) {
                        rampingUp = false; shooting = false;
                        outtake.run("idle");
                        pathState = nextState;
                    }
                }
                break;
        }
    }
}
