package org.firstinspires.ftc.teamcode.auton;



import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PresetPoses;
import org.firstinspires.ftc.teamcode.subsystems.CRServoStorage;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;


@Autonomous(name="FAR SIDE BLUE - Auton", group="Autonomous")
@Configurable // for Panels
@SuppressWarnings("FieldCanBeLocal") // android studio bugging
public class RobotAutonFarBlue extends LinearOpMode {

    public Hardware hardware;
    public Outtake outtake;
    public Intake intake;
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
                                    new Pose(57.5, 34.8),
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
            currentPose = follower.getPose();
            // update subsystems
            updatePath(outtake.update(), intake.update(), storage.update());




            // telemetry
            log("Status", "RUNNING");
            log("Path State", pathState);
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
    }

    // shootpreloaded-->gotoppg-->pickupppg-->shootppg-->park
    public void updatePath(boolean outtakeRFTFinished, boolean intakeRFTFinished, boolean storageRFTFinished) {
        switch (pathState) {
            case 0:
                follower.followPath(paths.ShootPreloaded, 0.5, true);
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
                    follower.followPath(paths.PickupPPG, 0.5, true);
                    intake.run(1);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    intake.run(0);
                    follower.followPath(paths.ShootPPG);
                    nextState = 4;
                    pathState = 10; // shoot
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
                    if (!rampingUp && !shooting) {outtake.run("far"); rampingUp = true;}
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
