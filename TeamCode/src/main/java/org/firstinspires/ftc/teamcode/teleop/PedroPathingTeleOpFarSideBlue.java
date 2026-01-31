package org.firstinspires.ftc.teamcode.teleop;



import android.annotation.SuppressLint;

import java.util.function.Function;
import java.util.function.Supplier;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.GamepadManager;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PresetPoses;
import org.firstinspires.ftc.teamcode.subsystems.CRServoStorage;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;




@Configurable
@TeleOp(name="FAR SIDE BLUE - PedroPathingTeleOp", group="LinearOpMode")
public class PedroPathingTeleOpFarSideBlue extends OpMode {

    // Create hardware object
    Hardware robotHardware = new Hardware();
    Intake robotIntake;
    CRServoStorage robotStorage;
    Outtake robotOuttake;
    Gate gate;
    boolean gateIsClosed = false;
    public GamepadManager g1Manager, g2Manager;

    // pp vars
    public Pose startPose = new Pose(56.000, 8.000, Math.toRadians(270));

    public PresetPoses poses;
    public static Pose currentPose;
    private Follower follower;
    public boolean autoDriving = false;
    public boolean autoAligning = false;
    public boolean launchPoseAutoDriving = false;
    private Supplier<PathChain> toShootPosePath;
    private Function<Pose, PathChain> toLaunchLinePath;
    private TelemetryManager panelsTelemetry;


    // selection
    public int selection_index = 0;
    public String[] selectables = new String[] {"FarSideBlue", "FarSideRed", "GoalSideBlue", "GoalSideRed", "Default"};
    public int selected_index = 0;
    public int max_index = selectables.length;

    // input vars
    boolean fieldCentric = false;
    double fcOffset = Math.toRadians(180);

    public Pose holdingPose = new Pose(0,0,0);

    // Auto shoot sequence tracking
    boolean isAutoShooting = false;
    public static double slowModeMultiplier = 0.3;

    // toggles
    boolean ballCamToggle = false;
    PIDFController headingPIDFController = new PIDFController(new PIDFCoefficients(0,0,0,0));
    boolean slowMode = false;
    boolean offToggle = false;

    // for button wasPressed detection
    boolean home1prevState = false;
    boolean options1prevState = false;
    boolean dpd1prevState = false;
    boolean dpr1prevState = false;
    boolean lt1prevState = false;
    boolean rb1prevState = false;
    boolean lb1prevState = false;
    boolean b1prevState = false;
    boolean dpu1prevState = false;
    boolean x1prevState = false;
    boolean y1prevState = false;
    boolean dpu2prevState = false; // tune preset increment
    boolean dpd2prevState = false; // tune preset decrement
    boolean a2prevState = false;
    boolean dpr2prevState = false;
    boolean dpl2prevState = false;
    boolean lb2prevState = false;
    boolean lt2prevstate = false;
    boolean a1prevState = false;
    boolean b2prevState = false;
    boolean y2prevState = false;
    boolean x2prevState = false;

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
    public void init() {
        // Initialize hardware
        robotHardware.initialize(hardwareMap,true);
        robotIntake = new Intake(robotHardware);
        robotStorage = new CRServoStorage(robotHardware);
        robotOuttake = new Outtake(robotHardware, 200d, 0d, 0d, 13.989d);
        gate = new Gate(robotHardware);

        poses = new PresetPoses(startPose, false);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(poses.ppgStartPose);
        follower.update();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        Drawing.init();


        toShootPosePath = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, poses.closeShootPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, poses.closeShootPose.getHeading(), 0.8))
                .build();

        toLaunchLinePath = (Pose targetPose) -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, targetPose::getHeading, 0.8))
                .build();


        g1Manager = PanelsGamepad.INSTANCE.getFirstManager();
        g2Manager = PanelsGamepad.INSTANCE.getSecondManager();

        telemetry.addData("Status", "INITIALIZED");
        telemetry.addData("Drive Mode", "Ready for TeleOp");
        telemetry.update();
        drawOnlyCurrent();

    }

    @Override
    public void start() {


        follower.update();
        robotHardware.imu.resetYaw();
        follower.startTeleOpDrive();


    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        headingPIDFController.setCoefficients(follower.constants.coefficientsHeadingPIDF);
        currentPose = follower.getPose();
        Gamepad g1 = g1Manager.asCombinedFTCGamepad(gamepad1);
        Gamepad g2 = g2Manager.asCombinedFTCGamepad(gamepad2);


        //      Gamepad 1 inputs
        double ly1 = -g1.left_stick_y; // forward/backward driving
        double lx1 = -g1.left_stick_x; // strafing
        double rx1 = g1.right_stick_x / 2; // turning (decrease by factor of 2)
        slowMode = g1.right_trigger >= 0.3;
        boolean home1state = g1.guide; // reset yaw value on gyro
        boolean options1state = g1.options; // field centric toggle
        boolean dpd1state = g1.dpad_down; // go to closest shoot pose
        boolean dpr1state = g1.dpad_right;
        boolean b1state = g1.b; // ball cam toggle
        boolean dpu1state = g1.dpad_up; // go to closest launch line pose
        boolean x1state = g1.x; // right localize
        boolean a1state = g1.a; // left localize
        boolean y1state = g1.y; // abort autonomous drive
        boolean lt1state = g1.left_trigger > 0.3; // hold pose

        boolean home1wP = home1state && !home1prevState;
        boolean options1wP = options1state && !options1prevState;
        boolean dpd1wP = dpd1state && !dpd1prevState;
        boolean b1wP = b1state && !b1prevState;
        boolean dpu1wP = dpu1state && !dpu1prevState;
        boolean x1wP = x1state && !x1prevState;
        boolean y1wP = y1state && !y1prevState;
        boolean a1wP = a1state && !a1prevState;
        boolean dpr1wP = dpr1state && !dpr1prevState;

        //      Gamepad 2 inputs
        double ly2 = g2.left_stick_y; // robot intake run
        double rt2state = g2.right_trigger; // storage forward
        boolean rb2state = g2.right_bumper; // storage reverse
        boolean a2state = g2.a; // outtake idle on/off
        boolean b2state = g2.b; // outtake preset for close shoot
        boolean y2state = g2.y; // outtake preset for far shoot
        boolean x2state = g2.x; // set outtake power based on power tables
        boolean dpu2state = g2.dpad_up; // tune preset increment
        boolean dpd2state = g2.dpad_down; // tune preset decrement
        boolean dpr2state = g2.dpad_right;
        boolean dpl2state = g2.dpad_left;
        boolean lb2state = g2.left_bumper; // gate toggle
        boolean lt2state = g2.left_trigger > 0.3;

        boolean a2wP = a2state && !a2prevState;
        boolean dpu2wP = dpu2state && !dpu2prevState;
        boolean dpd2wP = dpd2state && !dpd2prevState;
        boolean dpr2wP = dpr2state && !dpr2prevState;
        boolean dpl2wP = dpl2state && !dpl2prevState;
        boolean lb2wP = lb2state && !lb2prevState;
        boolean lt1wP = lt1state && !lt1prevState;
        boolean lt2wp = lt2state && !lt2prevstate;


        double imuHeading = robotHardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // rumble outtake feedback
        double outtakeTargetTps = robotOuttake.getTargetTps();
        double outtakeActualTps = robotHardware.outtakeMotor.getVelocity();
        if (robotOuttake.getCurrentPreset().equals("idle")) {gamepad2.stopRumble();}
        else if (outtakeTargetTps - 50 < robotHardware.outtakeMotor.getVelocity() && robotHardware.outtakeMotor.getVelocity() < outtakeTargetTps + 50) {
            gamepad2.rumble(-1); // infinite
        }
        else {gamepad2.stopRumble();}

        //      Drivetrain Control
        //Field centric toggle
        if (home1wP && fieldCentric) {robotHardware.imu.resetYaw();}
        if (options1wP) {fieldCentric = !fieldCentric;}

        if (!autoDriving) {
            if (slowMode) {ly1*=slowModeMultiplier; lx1*=slowModeMultiplier; rx1*=slowModeMultiplier;}

            if (fieldCentric) { // field centric
                if (ballCamToggle) {

                    double targetHeading = poses.getAngleTowardsGoal(currentPose) - Math.toRadians(180);

                    double error = MathFunctions.getTurnDirection(follower.getPose().getHeading(), targetHeading)
                            * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), targetHeading);

                    headingPIDFController.updateError(error);

                    follower.setTeleOpDrive(ly1, lx1, headingPIDFController.run(), false, fcOffset);

                }
                else {follower.setTeleOpDrive(ly1, lx1, -rx1, false, fcOffset);} // normal field centric
            }
            else {follower.setTeleOpDrive(ly1, lx1, -rx1, true);} // rbt centric
        }



        //      Intake Control
        robotIntake.update();

        if (!poses.isRed && x1wP) {
            follower.setPose(PresetPoses.LOCALIZE_POSE_RIGHT);
        }
        else if (poses.isRed && x1wP) {
            follower.setPose(PresetPoses.LOCALIZE_POSE_LEFT);
        }

        // robot gate toggle (lb2wP)
        if (lb2wP) {
            gateIsClosed = !gateIsClosed;
            gate.setGateState(gateIsClosed);
        }

        // ball cam toggle
        if (b1wP) {ballCamToggle = !ballCamToggle;}

        if (ly2 >= 0.3) {robotIntake.run(1.0);}
        else if (ly2 <= -0.3) {robotIntake.run(-1.0);}
        else {robotIntake.run(0);}

        // hold pos dpr1
        if (!follower.isBusy()) {
            if (lt1wP) {
                holdingPose = follower.getPose();
                follower.holdPoint(holdingPose);
            }
            if (!lt1state && lt1prevState) {
                follower.breakFollowing();
                follower.startTeleOpDrive();
            }
        }
        //      Storage Control
        robotStorage.update();
        if (rt2state >= 0.3 && !rb2state) {robotStorage.run(1.0);}
        else if (rb2state && rt2state < 0.3) {robotStorage.run(-1.0);}
        else {robotStorage.run(0.0);}

        // outtake preset running
        if (a2wP) {robotOuttake.reset();}
        else if (y2state) {robotOuttake.run("close"); gate.setGateState(false);} else if (y2prevState) {gate.setGateState(true);}
        else if (b2state) {robotOuttake.run("far"); gate.setGateState(false);} else if (b2prevState) {gate.setGateState(true);}
        else if (x2state) {robotOuttake.run(poses.getOptimalShooterPowerPercentage(follower.getPose(), true)); gate.setGateState(false);} else if (x2prevState) {gate.setGateState(true);}


        else {robotOuttake.run("idle");}
        if (offToggle) {robotOuttake.run(0);}

        // Fine tune active preset
        if (dpu2wP && !offToggle && !b2state) {robotOuttake.tuneActivePreset(0.05);}
        else if (dpd2wP && !offToggle) {robotOuttake.tuneActivePreset(-0.05);}

        if (home1wP) {fcOffset = follower.getHeading();}


        // Reset outtake presets
        if (options1wP) {robotOuttake.reset();}
        if (dpu1wP) {autoDriving = true; follower.followPath(toShootPosePath.get()); }
        if (dpd1wP) {autoDriving = true; follower.followPath(toLaunchLinePath.apply(poses.findClosestLaunchPose(currentPose)));}
        if ((autoDriving) && (y1wP || !follower.isBusy())) {
            follower.startTeleOpDrive(); autoDriving = false;
        }

        log("Status", "Running");
        log("Field Centric", fieldCentric ? "ON" : "OFF");
        log("Ball Cam", ballCamToggle ? "ON" : "OFF");
        log("Following Path", follower.isBusy() ? "FOLLOWING" : "none");
        log("Auto-Shooting", isAutoShooting ? "ACTIVE" : "IDLE");
        log("Target Velocity (tps)", robotOuttake.getTargetTps());
        log("Actual Velocity (tps)", robotHardware.outtakeMotor.getVelocity());
        log("IMU Heading (deg)", Math.toDegrees(imuHeading));
        log("Position", String.format("X: %.2f, Y: %.2f", currentPose.getX(), currentPose.getY()));
        log("Goal Pose", poses.goalPose);
        log("Target Angle for Goal", Math.toDegrees(poses.getAngleTowardsGoal(currentPose)));
        panelsTelemetry.update(telemetry);
        follower.update();
        draw();


        home1prevState = home1state;
        options1prevState = options1state;
        dpd1prevState = dpd1state;
        b1prevState = b1state;
        dpu1prevState = dpu1state;
        x1prevState = x1state;
        a1prevState = a1state;
        y1prevState = y1state;
        dpu2prevState = dpu2state; // tune preset increment
        dpd2prevState = dpd2state; // tune preset decrement
        a2prevState = a2state;
        dpr2prevState = dpr2state;
        dpl2prevState = dpl2state;
        lb2prevState = lb2state;
        b2prevState = b2state;
        y2prevState = y2state;
        dpr1prevState = dpr1state;
        x2prevState = x2state;

    }
}
