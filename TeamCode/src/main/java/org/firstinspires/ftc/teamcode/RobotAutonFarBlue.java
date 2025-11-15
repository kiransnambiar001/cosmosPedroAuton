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

@Autonomous(name="RobotAutonFarBlue", group="Robot")
@Configurable // for Panels
@SuppressWarnings("FieldCanBeLocal") // android studio bugging
public class RobotAutonFarBlue extends LinearOpMode {

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
    private double outtakeRunTime = 3000;

    private double intakeMaxPower = 1;

    private double previousTime;
    private Paths paths;

    public static class Paths {

        public PathChain ShootPreloaded;
        public PathChain GotoPPG;
        public PathChain PickupPPG;
        public PathChain ShootPPG;
        public PathChain Park;

        public Paths(Follower follower) {
            ShootPreloaded = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(55.987, 13.759))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                    .build();

            GotoPPG = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.987, 13.759), new Pose(41.753, 35.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                    .build();

            PickupPPG = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.753, 35.500), new Pose(13.997, 35.500))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ShootPPG = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(13.997, 35.500), new Pose(55.987, 13.522))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            Park = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.987, 13.522), new Pose(38.669, 33.213))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
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
            else if (pathState == 1) {log("Path Name", "Go to PPG");}
            else if (pathState == 2) {log("Path Name", "Pickup PPG");}
            else if (pathState == 3) {log("Path Name", "Shoot GPP");}
            else if (pathState == 4) {log("Path Name", "Park");}
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
                    follower.followPath(paths.GotoPPG);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PickupPPG);
                    pathState = 3;
                    intakeMotor.setPower(intakeMaxPower); // start intake
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootPPG);
                    intakeMotor.setPower(0);
                    pathState = 10; // shoot
                    beforeOuttakeState = 3;
                    previousTime = currentTime;
                    outtakeMotor.setVelocity(outtakeMaxPower);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Park);
                    pathState = -1; // terminate
                }
                break;
            case 10:
                if (currentTime >= outtakeRunTime+previousTime) {
                    outtakeMotor.setVelocity(0);
                    pathState = beforeOuttakeState+1;
                }
        }
    }
}