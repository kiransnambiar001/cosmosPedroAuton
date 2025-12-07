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
    public boolean isRed = true;
    public Pose startPose;
    public Pose goalPose = new Pose(12.6, 135.5);
    public Pose closeShootPose = new Pose(20.876, 122.886, Math.toRadians(145));
    public Pose farShootPose = new Pose(56.000, 8.000, Math.toRadians(90));;
    public Pose gppStartPose = new Pose(48.000, 83.750, Math.toRadians(180));
    public Pose gppEndPose = new Pose(19.800, 83.750, Math.toRadians(180));
    public Pose pgpStartPose = new Pose(48.000, 60.000, Math.toRadians(180));
    public Pose pgpEndPose = new Pose(20.000, 60.000, Math.toRadians(180));
    public Pose parkLeverPose = new Pose(28.000, 70.500, Math.toRadians(180));
    public Pose ppgStartPose = new Pose(41.753, 35.500, Math.toRadians(180));
    public Pose ppgEndPose = new Pose(13.997, 35.500, Math.toRadians(180));
    public Pose parkPose = new Pose(38.669, 33.213, Math.toRadians(180));


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
    private static final double[] powerTableConstructor = new double[] {50, 0.3, 60, 0.4, 70, 0.5, 80, 0.6, 100, 0.7, 150, 0.8}; // goes in order of {distance, power, distance1, power1, distance2. power2, etc}
    public static double[][] powerTable = new double[powerTableConstructor.length/2][2];

    public PresetPoses(Pose startPose, boolean isRed) {
        this.isRed = true;
        if (isRed) {
            this.startPose = startPose.mirror();
            goalPose = goalPose.mirror();
            closeShootPose = closeShootPose.mirror();
            farShootPose = farShootPose.mirror();
            gppStartPose = gppStartPose.mirror();
            gppEndPose = gppEndPose.mirror();
            pgpStartPose = pgpStartPose.mirror();
            pgpEndPose = pgpEndPose.mirror();
            parkLeverPose = parkLeverPose.mirror();
            ppgStartPose = pgpStartPose.mirror();
            ppgEndPose = pgpEndPose.mirror();
            parkPose = parkPose.mirror();
        } else {this.startPose = startPose;}

            // power table
        double currentDistance = 0; double currentPower = 0;
        for (int i = 0; i<powerTableConstructor.length; i++) {
            if ((i%2) == 0) {currentDistance=powerTableConstructor[i];}
            else {
                currentPower=powerTableConstructor[i];
                powerTable[((i+1)/2)-1] = new double[] {currentDistance, currentPower}; // add to powerTable
            }
        }   
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

    public double getOptimalShooterPowerPercentage(Pose currentPose) {
        double distance = currentPose.distanceFrom(goalPose); // in inches (pedropathing coords are in inches)
        int optimalCoordinateIndex = 0;
        double minDiff = Double.MAX_VALUE;
        for (int i=0; i<powerTable.length; i++) {
            double coorDist = powerTable[i][1];
            if (minDiff > Math.abs(coorDist-distance)) {optimalCoordinateIndex = i;}
        }
        return powerTable[optimalCoordinateIndex][2];
    }
}
