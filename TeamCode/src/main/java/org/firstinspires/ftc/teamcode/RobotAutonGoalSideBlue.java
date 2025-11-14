package org.firstinspires.ftc.teamcode;

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

@Autonomous(name="RobotAutonGoalSideBlue", group="Robot")
@Configurable // for Panels
@SuppressWarnings("FieldCanBeLocal") // android studio bugging
public class RobotAutonGoalSideBlue extends LinearOpMode {

    public DcMotorEx outtakeMotor;
    public DcMotor intakeMotor;

    private final ElapsedTime timer = new ElapsedTime(); // runtime
    // other vars
    private Pose currentPose;
    public Follower follower;
    private TelemetryManager panelsTelemetry;
    private int pathState;

    private int beforeOuttakeState;
    private double outtakeMaxPower = ((256.2*0.7)/60)*537.7; // 0.7 power percentage
    private int outtakeRunTime;

    private int beforeIntakeState;
    private int intakeRunTime;
    private double intakeMaxPower = 1;

    private double previousTime;
    private Paths paths;

    public static class Paths {
        public PathChain ShootPreloaded;
        public PathChain GotoGPP;
        public PathChain PickupGPP;
        public PathChain HitLever;
        public PathChain ShootGPP;
        public PathChain GoOffLine;

        public Paths(Follower follower) {
            ShootPreloaded = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.183, 128.580), new Pose(56.184, 86.766))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(135))
                    .build();

            GotoGPP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.184, 86.766), new Pose(44.600, 84.217))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            PickupGPP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.600, 84.217), new Pose(15.183, 83.506))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            HitLever = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(15.183, 83.506),
                                    new Pose(28.468, 70.695),
                                    new Pose(13.048, 70.221)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            ShootGPP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(13.048, 70.221), new Pose(56.184, 86.591))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            GoOffLine = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.184, 86.591), new Pose(86.000, 114.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
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
            if (pathState == 0) {log("Path Name", "Shoot Preloaded");}
            else if (pathState == 1) {log("Path Name", "Go to GPP");}
            else if (pathState == 2) {log("Path Name", "Pickup GPP");}
            else if (pathState == 3) {log("Path Name", "Hit Lever");}
            else if (pathState == 4) {log("Path Name", "Shoot GPP");}
            else if (pathState == 5) {log("Path Name", "Go Off Line");}
            else if (pathState == 10) {log("Path Name", "Shooting (Outtake)");}
            else if (pathState == 11) {log("Path Name", "Grabbing (Intake)");}
            else if (pathState == -1) {log("Path Name", "Autonomous Finished!");}
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
                    pathState = 11; // intake
                    beforeIntakeState = 2;
                    previousTime = currentTime;
                    intakeMotor.setPower(intakeMaxPower);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.HitLever);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootGPP);
                    pathState = 10; // shoot
                    beforeOuttakeState = 4;
                    previousTime = currentTime;
                    outtakeMotor.setVelocity(outtakeMaxPower);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GoOffLine);
                    pathState = -1;
                }
            case 10:
                if (currentTime >= outtakeRunTime+previousTime) {
                    outtakeMotor.setVelocity(0);
                    pathState = beforeOuttakeState+1;
                }
            case 11:
                if (currentTime >= intakeRunTime+previousTime) {
                    intakeMotor.setPower(0);
                    pathState = beforeIntakeState+1;
                }
        }
    }
}