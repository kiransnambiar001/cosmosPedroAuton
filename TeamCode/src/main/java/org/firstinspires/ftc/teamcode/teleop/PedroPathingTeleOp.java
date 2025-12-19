package org.firstinspires.ftc.teamcode.teleop;

import java.util.function.Function;
import java.util.function.Supplier;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PresetPoses;
import org.firstinspires.ftc.teamcode.subsystems.CRServoStorage;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@Configurable
@TeleOp(name="PedroPathing TeleOp", group="LinearOpMode")
public class PedroPathingTeleOp extends OpMode {

    // Create hardware object
    Hardware robotHardware = new Hardware();
    Intake robotIntake;
    CRServoStorage robotStorage;
    Outtake robotOuttake;

    // pp vars
    public static Pose startPose = new Pose(72, 72, Math.toRadians(90));

    public PresetPoses poses = new PresetPoses(startPose, false);
    public static Pose currentPose;
    private Follower follower;
    public boolean autoDriving = false;
    private Supplier<PathChain> toShootPosePath;
    private Function<Pose, PathChain> toLaunchLinePath;
    private TelemetryManager panelsTelemetry;

    // input vars
    boolean fieldCentric = false;

    // Auto shoot sequence tracking
    boolean isIntakeRunning = false;
    boolean isAutoShooting = false;
    double spoolUpEndTime = 0;
    double currentOuttakePower = 0;
    public static double slowModeMultiplier = 0.3;

    // toggles
    boolean ballCamToggle = false;
    boolean slowMode = false;
    boolean offToggle = false;


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
    public void init() {
        // Initialize hardware
        robotHardware.initialize(hardwareMap,true);
        robotIntake = new Intake(robotHardware);
        robotStorage = new CRServoStorage(robotHardware);
        robotOuttake = new Outtake(robotHardware, 200d, 0d, 0d, 13.989d);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        toShootPosePath = () -> follower.pathBuilder()
            .addPath(new Path(new BezierLine(follower::getPose, poses.closeShootPose)))
            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, poses.closeShootPose.getHeading(), 0.8))
            .build();

        toLaunchLinePath = (Pose targetPose) -> follower.pathBuilder()
            .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
            .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, targetPose::getHeading, 0.8))
            .build();

        telemetry.addData("Status", "INITIALIZED");
        telemetry.addData("Drive Mode", "Ready for TeleOp");
        telemetry.update();
    }

    @Override
    public void start() {
        robotHardware.imu.resetYaw();
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        currentPose = follower.getPose();

        //      Gamepad 1 inputs
        double ly1 = -gamepad1.left_stick_y; // forward/backward driving
        double lx1 = gamepad1.left_stick_x; // strafing
        double rx1 = gamepad1.right_stick_x / 2; // turning (decrease by factor of 2)
        slowMode = (gamepad1.right_trigger >= 0.5) ? true : false;
        boolean home1wP = gamepad1.guideWasPressed(); // reset yaw value on gyro
        boolean options1wP = gamepad1.optionsWasPressed(); // field centric toggle
        boolean dpd1wP = gamepad1.dpadDownWasPressed(); // go to closest shoot pose
        boolean b1wP = gamepad1.bWasPressed(); // ball cam toggle
        boolean dpu1wP = gamepad1.dpadUpWasPressed(); // go to closest launch line pose
        boolean x1wP = gamepad1.xWasPressed();
        boolean y1wP = gamepad1.yWasPressed(); // abort autonomous drive

        //      Gamepad 2 inputs
        double ly2 = gamepad2.left_stick_y; // robot intake run
        double rt2state = gamepad2.right_trigger; // storage forward
        boolean rb2state = gamepad2.right_bumper; // storage reverse
        boolean a2wP = gamepad2.aWasPressed(); // outtake idle on/off
        boolean b2state = gamepad2.b; // outtake preset for close shoot
        boolean y2state = gamepad2.y; // outtake preset for far shoot
        boolean x2state = gamepad2.x; // set outtake power based on power tables
        boolean dpu2wP = gamepad2.dpadUpWasPressed(); // tune preset increment
        boolean dpd2wP = gamepad2.dpadDownWasPressed(); // tune preset decrement


        double imuHeading = robotHardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //      Drivetrain Control
        //Field centric toggle
        if (home1wP && fieldCentric) {robotHardware.imu.resetYaw();}
        if (options1wP) {fieldCentric = !fieldCentric;}

        if (!autoDriving) {
            if (slowMode) {ly1*=slowModeMultiplier; lx1*=slowModeMultiplier; rx1*=slowModeMultiplier;}

            if (fieldCentric) { // field centric
                if (ballCamToggle) {
                    double turnPower = 0;

                    double targetHeading = poses.getAngleTowardsGoal(currentPose) - Math.toRadians(180);

                    // B. Get where we are currently looking
                    double currentHeading = follower.getPose().getHeading();

                    // C. Calculate the error (shortest path to target)
                    // NormalizeRadians ensures we turn -10 degrees instead of +350 degrees
                    double headingError = AngleUnit.normalizeRadians(targetHeading - currentHeading);

                    // D. Calculate Turn Power (P-Controller)
                    // 'kP' is the stiffness of the lock.
                    // Start with 1.5. If it's too sluggish, go up. If it shakes, go down.
                    double kP = 1.5;
                    turnPower = headingError * kP;

                    // Clamp the turn power so it doesn't go too crazy (optional safety)
                    turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

                    // E. Drive Field Centric with calculated turn power
                    // false = Field Centric
                    follower.setTeleOpDrive(ly1, lx1, turnPower, false);

                }
                else {follower.setTeleOpDrive(ly1, lx1, -rx1, false);} // normal field centric
            }
            else {follower.setTeleOpDrive(ly1, lx1, -rx1, true);} // rbt centric
        }

        //      Intake Control
        robotIntake.update();

        // ball cam toggle
        if (b1wP) {ballCamToggle = !ballCamToggle;}

        if (ly2 >= 0.3) {robotIntake.run(1.0);}
        else if (ly2 <= -0.3) {robotIntake.run(-1.0);}
        else {robotIntake.run(0);}

        //      Storage Control
        robotStorage.update();
        if (rt2state >= 0.3 && !rb2state) {robotStorage.run(1.0);}
        else if (rb2state && rt2state < 0.3) {robotStorage.run(-1.0);}
        else {robotStorage.run(0.0);}

        // outtake preset running
        if (a2wP) {offToggle = !offToggle;}
        else if (b2state) {robotOuttake.run("close");}
        else if (y2state) {robotOuttake.run("far");}
        else if (x2state) {currentOuttakePower = poses.getOptimalShooterPowerPercentage(currentPose);}

        else {robotOuttake.run("idle");}
        if (offToggle) {robotOuttake.run(0);}

        // Fine tune active preset
        if (dpu2wP && !offToggle && !b2state) {robotOuttake.tuneActivePreset(0.05);}
        else if (dpd2wP && !offToggle) {robotOuttake.tuneActivePreset(-0.05);}


        // Reset outtake presets
        if (options1wP) {robotOuttake.reset();}
        if (dpu1wP) {autoDriving = true; follower.followPath(toShootPosePath.get());}
        if (dpd1wP) {autoDriving = true; follower.followPath(toLaunchLinePath.apply(poses.findClosestLaunchPose(currentPose)));}
        if (autoDriving && (y1wP || !follower.isBusy())) {follower.startTeleOpDrive(); autoDriving = false;}

        log("Status", "Running");
        log("Field Centric", fieldCentric ? "ON" : "OFF");
        log("Ball Cam", ballCamToggle ? "ON" : "OFF");
        log("Following Path", follower.isBusy() ? "FOLLOWING" : "none");
        log("Auto-Shooting", isAutoShooting ? "ACTIVE" : "IDLE");
        log("Outtake Motor Power", currentOuttakePower);
        log("Target Velocity (tps)", robotOuttake.getTargetTps());
        log("Actual Velocity (tps)", robotHardware.outtakeMotor.getVelocity());
        log("IMU Heading (deg)", Math.toDegrees(imuHeading));
        panelsTelemetry.update(telemetry);
        follower.update();

    }
}
