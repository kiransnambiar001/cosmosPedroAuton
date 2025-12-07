package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathPoint;


// close
//        public Pose startPose = new Pose(20.876, 122.886, Math.toRadians(145));
//        private Pose shootPose = new Pose(42.000, 100.500, Math.toRadians(135));
//        private Pose gppStartPose = new Pose(48.000, 83.750, Math.toRadians(180));
//        private Pose gppEndPose = new Pose(19.800, 83.750, Math.toRadians(180));
//        private Pose pgpStartPose = new Pose(48.000, 60.000, Math.toRadians(180));
//        private Pose pgpEndPose = new Pose(20.000, 60.000, Math.toRadians(180));
//        private Pose leverPose = new Pose(28.000, 70.500, Math.toRadians(180));

// far
//private Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90));
//private Pose shootPose = new Pose(55.987, 13.759, Math.toRadians(110));
//private Pose ppgStartPose = new Pose(41.753, 35.500, Math.toRadians(180));
//private Pose ppgEndPose = new Pose(13.997, 35.500, Math.toRadians(180));
//private Pose parkPose = new Pose(38.669, 33.213, Math.toRadians(180));

@Configurable
public class PresetPoses {

    // robot poses
    public static boolean isRed = true;
    public static Pose startPose;
    public static Pose goalPose = new Pose(12.6, 135.5);
    public static Pose closeShootPose = new Pose(20.876, 122.886, Math.toRadians(145));
    public static Pose farShootPose = new Pose(56.000, 8.000, Math.toRadians(90));;
    public static Pose gppStartPose = new Pose(48.000, 83.750, Math.toRadians(180));
    public static Pose gppEndPose = new Pose(19.800, 83.750, Math.toRadians(180));
    public static Pose pgpStartPose = new Pose(48.000, 60.000, Math.toRadians(180));
    public static Pose pgpEndPose = new Pose(20.000, 60.000, Math.toRadians(180));
    public static Pose parkLeverPose = new Pose(28.000, 70.500, Math.toRadians(180));
    public static Pose ppgStartPose = new Pose(41.753, 35.500, Math.toRadians(180));
    public static Pose ppgEndPose = new Pose(13.997, 35.500, Math.toRadians(180));
    public static Pose parkPose = new Pose(38.669, 33.213, Math.toRadians(180));
    public static Pose[] poses = new Pose[] {
            startPose, closeShootPose, farShootPose, gppStartPose, gppEndPose, pgpStartPose, pgpEndPose, parkLeverPose, ppgStartPose, ppgEndPose, parkPose, goalPose
    };

    // launch line poses
    public static final Pose goalLaunchLineBPose = new Pose(48.4, 94.9);
    public static final Pose goalLaunchLineRPose = new Pose(96, 94.9);
    public static final Pose goalLaunchLineMPose = new Pose(72.1, 71.6);
    public static final Pose farLaunchLineBPose = new Pose(64.5, 15.2);
    public static final Pose farLaunchLineRPose = new Pose(71.9,23);
    public static final Pose farLaunchLineMPose = new Pose(80.4, 15.2);

    public static final BezierLine LAUNCHSEG_FARB = new BezierLine(farLaunchLineBPose, farLaunchLineMPose);
    public static final BezierLine LAUNCHSEG_FARR = new BezierLine(farLaunchLineRPose, farLaunchLineMPose);
    public static final BezierLine LAUNCHSEG_CLOSEB = new BezierLine(goalLaunchLineBPose, goalLaunchLineMPose);
    public static final BezierLine LAUNCHSEG_CLOSER = new BezierLine(goalLaunchLineRPose, goalLaunchLineMPose);
    public static final BezierLine[] GOAL_SEGS = new BezierLine[] {LAUNCHSEG_FARB, LAUNCHSEG_FARR, LAUNCHSEG_CLOSEB, LAUNCHSEG_CLOSER};

    public PresetPoses(Pose startPose, boolean isRed) {
        this.startPose = startPose;
        this.isRed = true;
        if (isRed) {for (int i=0; i<poses.length; i++) {poses[i] = poses[i].mirror();}}
    }

    public Pose findClosestLaunchPose(Pose currentPose) {
        double minDistanceSquared = Double.MAX_VALUE;
        Pose closestPose = null;

        for (BezierLine segment : GOAL_SEGS) {
            Path tempPath = new Path(segment);
            Pose closePose = tempPath.getClosestPoint(currentPose).getPose();

            double distanceSquared = closePose.distSquared(currentPose);
            if (distanceSquared < minDistanceSquared) {
                minDistanceSquared = distanceSquared;
                closestPose = closePose;
            }
        }

        double angle = Math.atan2(
                Math.abs(closestPose.getY() - currentPose.getY()),
                Math.abs(closestPose.getX() - currentPose.getX())
        ); if (!isRed) {angle += Math.toRadians(90);}

        closestPose = closestPose.setHeading(angle - Math.toRadians(180));

        return closestPose;
    }

    public double getAngleTowardsGoal(Pose currentPose) {
        double angle = Math.atan2(
                Math.abs(goalPose.getY() - currentPose.getY()),
                Math.abs(goalPose.getX() - currentPose.getX())
        ); if (!isRed) {angle += Math.toRadians(90);}
        return angle;
    }


}
