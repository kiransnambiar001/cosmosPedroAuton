package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.math.MathFunctions;
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
    public static double shootAngle = 320;
    public boolean isRed = true;
    public Pose startPose;
    public Pose goalPose = new Pose(12.6, 135.5);
    public Pose closeShootPose = new Pose(50, 95, Math.toRadians(shootAngle));
    public Pose farShootPose = new Pose(56.000, 13.0750, Math.toRadians(285));;
    public Pose gppStartPose = new Pose(52.000, 83.750, Math.toRadians(180));
    public Pose closeShootGppStartMidCurvePose = new Pose(61.918, 94.181);
    public Pose gppEndPose = new Pose(19.800, 83.750, Math.toRadians(180));
    public Pose pgpStartPose = new Pose(48.000, 60.000, Math.toRadians(180));
    public Pose closeShootPgpEndMidCurvePose = new Pose(54, 59);
    public Pose pgpEndPose = new Pose(20.000, 60.000, Math.toRadians(180));
    public Pose parkLeverPose = new Pose(28.000, 70.500, Math.toRadians(90));
    public Pose ppgStartPose = new Pose(49.75, 35.500, Math.toRadians(180));
    public Pose farShootPpgStartMidCurvePose = new Pose(57.5, 34.8);
    public Pose ppgEndPose = new Pose(14, 35.500, Math.toRadians(180));
    public Pose parkPose = new Pose(106, 33, Math.toRadians(90));
    public Pose closeMoveOutOfWayPose = new Pose(55, 134.17565, Math.toRadians(270));
    public Pose farStartPose = new Pose(56.000, 8.000, Math.toRadians(270));

    public static Pose LOCALIZE_POSE_LEFT = new Pose(7.9,7.7, Math.toRadians(90));
    public static Pose LOCALIZE_POSE_RIGHT = LOCALIZE_POSE_LEFT.mirror();


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

    // MAKE SURE CONSTRUCTOR GOES IN ORDER OF INCREASING DISTANCES
    public static double[][] powerTable = new double[][] {
            {50, 0.55},
            {60, 0.6},
            {70, 0.65},
            {80, 0.7},
            {100, 0.75},
            {150, 0.8}
    };

    public PresetPoses(Pose startPose, boolean isRed) {
        this.isRed = isRed;
        if (isRed) {
            this.startPose = startPose.mirror();
            this.goalPose = goalPose.mirror();
            this.closeShootPose = closeShootPose.mirror();
            this.farShootPose = farShootPose.mirror();
            this.gppStartPose = gppStartPose.mirror();
            this.gppEndPose = gppEndPose.mirror();
            this.pgpStartPose = pgpStartPose.mirror();
            this.pgpEndPose = pgpEndPose.mirror();
            this.parkLeverPose = parkLeverPose.mirror();
            this.ppgStartPose = ppgStartPose.mirror();
            this.ppgEndPose = ppgEndPose.mirror();
            this.parkPose = parkPose.mirror();
            this.closeShootGppStartMidCurvePose = closeShootGppStartMidCurvePose.mirror();
            this.closeShootPgpEndMidCurvePose = closeShootPgpEndMidCurvePose.mirror();
            this.closeMoveOutOfWayPose = closeMoveOutOfWayPose.mirror();
            this.farShootPpgStartMidCurvePose = farShootPpgStartMidCurvePose.mirror();
            this.farStartPose = farShootPose.mirror();
        } else {this.startPose = startPose;}

            // power table
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

        double angle = this.getAngleTowardsGoal(closestPose);

        closestPose = closestPose.setHeading(angle - Math.toRadians(180));

        return closestPose;
    }

    public double getAngleTowardsGoal(Pose currentPose) {
        // Calculate the absolute angle vector from the robot to the goal
        double angle = Math.atan2(
                goalPose.getY() - currentPose.getY(),
                goalPose.getX() - currentPose.getX()
        );

        // Since goalPose is already mirrored in the constructor based on isRed,
        // we do not need conditional math here. The coordinates handle the mirroring.
        return MathFunctions.normalizeAngle(angle);
    }

    public static double normalizeAngle(double angle) {
        double TWO_PI = 2 * Math.PI;
        double newAngle = angle % TWO_PI;
        if (newAngle <= -Math.PI) newAngle += TWO_PI;
        if (newAngle > Math.PI) newAngle -= TWO_PI;
        return newAngle;
    }

    public double getOptimalShooterPowerPercentage(Pose currentPose, boolean doLinearInterp) {
        double distance = currentPose.distanceFrom(goalPose); // in inches (pedropathing coords are in inches)
        // clamp to least and highest indexes
        if (distance <= powerTable[0][0]) return powerTable[0][1];
        if (distance >= powerTable[powerTable.length - 1][0]) return powerTable[powerTable.length - 1][1];



        if (!doLinearInterp) {
            int optimalCoordinateIndex = 0;
            double minDiff = Double.MAX_VALUE;
            for (int i = 0; i < powerTable.length; i++) {
                double coorDist = powerTable[i][0];
                if (minDiff > Math.abs(coorDist - distance)) {
                    optimalCoordinateIndex = i;
                }
            }
            return powerTable[optimalCoordinateIndex][1];
        }

        else {
            for (int i = 0; i < powerTable.length - 1; i++) {
                if (distance >= powerTable[i][0] && distance <= powerTable[i + 1][0]) {
                    double d0 = powerTable[i][0];
                    double d1 = powerTable[i + 1][0];
                    double p0 = powerTable[i][1];
                    double p1 = powerTable[i + 1][1];

                    // Linear interpolation formula: y = y0 + (x - x0) * ((y1 - y0) / (x1 - x0))
                    return p0 + (distance - d0) * ((p1 - p0) / (d1 - d0));
                }
            }
        }

        return powerTable[0][1]; // default return, should never reach here
    }

    public Pose closestPose(Pose targetPose, Pose[] possiblePoses) {
        double minDist = Double.MAX_VALUE;
        Pose overallPose = null;
        for (Pose pose : possiblePoses) {
            double distSq = targetPose.distSquared(pose);
            if (distSq<minDist) {
                overallPose = pose;
                minDist = distSq;
            }
        }
        return overallPose;
    }

    public double closestAngle(double targetAngle, double[] possibleAngles) {
        double minDiff = Double.MAX_VALUE;
        double overallAngle = 0;
        for (double angle : possibleAngles) {
            double diff = Math.abs(targetAngle-angle);
            if (diff<minDiff) {
                overallAngle = angle;
                minDiff = angle;
            }
        }
        return overallAngle;
    }
}
