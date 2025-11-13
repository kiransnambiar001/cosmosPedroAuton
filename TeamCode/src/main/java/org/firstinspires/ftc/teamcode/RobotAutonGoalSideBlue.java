package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="RobotAutonGoalSideBlue", group="Robot")
@Configurable // for Panels
@SuppressWarnings("FieldCanBeLocal") // android studio bugging
public class RobotAutonGoalSideBlue extends LinearOpMode {

    private final ElapsedTime timer = new ElapsedTime(); // runtime

    private final Pose startPose = new Pose(X,Y,heainfd_in_radians); // TODO: ADD COORDINATES
    private final Pose shootPose = new Pose(x,y,Math.toRadians(0));
    private final Pose grabPose = new Pose(x,y,Math.toRadians(0));
    private final Pose pickupPose = new Pose(x,y,Math.toRadians(0));
    private final Pose offlinePose = new Pose(x,y,Math.toRadians(0));

    // variables for paths
    private PathChain shootPreloadedPath;
    private PathChain grabPrepPath;
    private PathChain pickupBallsPath;
    private PathChain finishPickupPath;
    private PathChain shootGrabbedPath;
    private PathChain goOffLaunchLine;


    // other vars
    private Pose currentPose;
    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private int pathState;


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
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // init pp follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        log("Status", "INITIALIZED");
        telemetry.update();

        // upon start operations
        waitForStart();
        pathState = 0;
        timer.reset();

        follower.update();
        panelsTelemetry.update();
        currentPose = follower.getPose();
        buildPaths();

        while (opModeIsActive()) {
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose();
        }
    }

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,pose1))
                .setLinearHeadingInterpolation(startPose.getHeading(),pose1.getHeading())
                .build();
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(pose1,pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(),pose2.getHeading())
                .build();
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(pose2,pose3))
                .setLinearHeadingInterpolation(pose2.getHeading(),pose3.getHeading())
                .build();
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(pose3,pose4))
                .setLinearHeadingInterpolation(pose3.getHeading(),pose4.getHeading())
                .build();
    }

    public void updatePath() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(path2);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(path3);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(path4);
                    pathState = -1;
                }
                break;
        }
    }
}