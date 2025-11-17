package org.firstinspires.ftc.teamcode;

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

@Autonomous(name="RobotAutonGoalSideBlue2Pair", group="Robot")
@Configurable // for Panels
@SuppressWarnings("FieldCanBeLocal") // android studio bugging
public class RobotAutonGoalSideBlue2Pair extends LinearOpMode {

    public DcMotorEx outtakeMotor;
    public DcMotor intakeMotor;

    private final ElapsedTime timer = new ElapsedTime(); // runtime
    // other vars
    private Pose currentPose;
    public Follower follower;
    private TelemetryManager panelsTelemetry;
    private int pathState;

    private int beforeOuttakeState;
    private double outtakeMaxPower = ((256.2*0.4)/60)*537.7; // 0.7 power percentage
    private int outtakeRunTime = 3000;

    private double intakeMaxPower = 1;

    private double previousTime;
    private Paths paths;

    public static class Paths {

        public PathChain ShootPreloaded;
        public PathChain GotoGPP;
        public PathChain PickupGPP;
        public PathChain ShootGPP;
        public PathChain GotoPGP;
        public PathChain PickupPGP;
        public PathChain ShootPGP;
        public PathChain GotoLever;

        public Paths(Follower follower) {
            ShootPreloaded = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.876, 122.886), new Pose(42.000, 100.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .build();

            GotoGPP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.000, 100.500), new Pose(45.000, 83.750))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            PickupGPP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.000, 83.750), new Pose(19.800, 83.600))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ShootGPP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.800, 83.600), new Pose(42.000, 100.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            GotoPGP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.000, 100.500), new Pose(45.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            PickupPGP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.000, 60.000), new Pose(20.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ShootPGP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 60.000), new Pose(42.000, 100.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            GotoLever = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.000, 100.500), new Pose(28.000, 70.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
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

    @Override
    public void runOpMode() {
        // init intake and outtake
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        outtakeMotor.setDirection(DcMotor.Direction.FORWARD);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // init pp follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        log("Status", "INITIALIZED");
        telemetry.update();

        // upon start operations
        waitForStart();
        pathState = 0;
        timer.reset();

        follower.update();
        panelsTelemetry.update();
        currentPose = follower.getPose();
        paths = new Paths(follower);

        while (opModeIsActive()) {
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose();
            updatePath(timer.milliseconds());

            // telemetry
            log("Status", "RUNNING");
            log("Path State", pathState);
            log("Current Pose", currentPose);
            telemetry.update();
        }
    }

    public void updatePath(double currentTime) {
        switch (pathState) {
            case 0:
                follower.followPath(paths.ShootPreloaded);
                pathState = 10; // shoot
                beforeOuttakeState = 0;
                previousTime = currentTime;
                outtakeMotor.setVelocity(outtakeMaxPower);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GotoGPP);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PickupGPP);
                    pathState = 2;
                    intakeMotor.setPower(intakeMaxPower);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootGPP);
                    pathState = 10; // shoot
                    beforeOuttakeState = 4;
                    previousTime = currentTime;
                    outtakeMotor.setVelocity(outtakeMaxPower);
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
                    follower.followPath(paths.PickupPGP);
                    pathState = 6;
                    intakeMotor.setPower(intakeMaxPower);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootPGP);
                    pathState = 10; // shoot
                    beforeOuttakeState = 6;
                    previousTime = currentTime;
                    outtakeMotor.setVelocity(outtakeMaxPower);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GotoLever);
                    pathState = -1;
                }
            case 10:
                if (currentTime >= outtakeRunTime+previousTime) {
                    outtakeMotor.setVelocity(0);
                    pathState = beforeOuttakeState+1;
                }
        }
    }
}