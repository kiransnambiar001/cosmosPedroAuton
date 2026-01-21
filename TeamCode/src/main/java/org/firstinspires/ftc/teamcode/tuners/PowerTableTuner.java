package org.firstinspires.ftc.teamcode.tuners;



import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDFController;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PresetPoses;
import org.firstinspires.ftc.teamcode.subsystems.CRServoStorage;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@Configurable
@TeleOp(name="Power Table Tuner", group="LinearOpMode")
public class PowerTableTuner extends OpMode {

    // Create hardware object
    Hardware robotHardware = new Hardware();
    Intake robotIntake;
    CRServoStorage robotStorage;
    Outtake robotOuttake;

    // pp vars
    public static Pose startPose = new Pose(56, 8, Math.toRadians(90));

    public PresetPoses poses = new PresetPoses(startPose, false);
    public static Pose currentPose;
    private Follower follower;
    private TelemetryManager panelsTelemetry;

    // input vars
    boolean fieldCentric = false;

    public static PIDFCoefficients headingPIDFCoefficients = new PIDFCoefficients(1,0,0.075,0.1);
    PIDFController headingPIDFController = new PIDFController(headingPIDFCoefficients);


    public static double outtakeOnPower = 0.3;
    double currentOuttakePower = 0;
    public static double slowModeMultiplier = 0.3;

    // toggles
    boolean ballCamToggle = false;
    boolean slowMode = false;

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
        boolean b1wP = gamepad1.bWasPressed(); // ball cam toggle

        //      Gamepad 2 inputs
        double ly2 = gamepad2.left_stick_y; // robot intake run
        double rt2state = gamepad2.right_trigger; // storage forward
        boolean rb2state = gamepad2.right_bumper; // storage reverse
        boolean b2state = gamepad2.b; // outtake preset for close shoot
        boolean dpu2wP = gamepad2.dpadUpWasPressed(); // tune preset increment
        boolean dpd2wP = gamepad2.dpadDownWasPressed(); // tune preset decrement


        double imuHeading = robotHardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //      Drivetrain Control
        //Field centric toggle
        if (home1wP && fieldCentric) {robotHardware.imu.resetYaw();}
        if (options1wP) {fieldCentric = !fieldCentric;}


        if (slowMode) {ly1*=slowModeMultiplier; lx1*=slowModeMultiplier; rx1*=slowModeMultiplier;}

        if (fieldCentric) { // field centric
            if (ballCamToggle) {
                double targetHeading = poses.getAngleTowardsGoal(currentPose) - Math.toRadians(180);
                double currentHeading = follower.getPose().getHeading();

                double output = headingPIDFController.calculate(currentHeading, targetHeading);

                output = Math.max(-1.0, Math.min(1.0, output));
                follower.setTeleOpDrive(ly1, lx1, output, false);
            }
            else {follower.setTeleOpDrive(ly1, lx1, rx1, false);} // normal field centric
        }
        else {follower.setTeleOpDrive(ly1, lx1, rx1, true);} // rbt centric


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
        if (b2state) {robotOuttake.run(outtakeOnPower);}
        else {robotOuttake.run("idle");}

        // Fine tune outtakeOnPower
        if (dpu2wP) {outtakeOnPower += 0.01;} else if (dpd2wP) {outtakeOnPower -= 0.01;}



        log("Status", "Running");
        log("Field Centric", fieldCentric ? "ON" : "OFF");
        log("Ball Cam", ballCamToggle ? "ON" : "OFF");
        log("Outtake Motor Current Power", currentOuttakePower);
        log("Outtake Motor Shoot Power", outtakeOnPower);
        log("Target Velocity (tps)", robotOuttake.getTargetTps());
        log("Actual Velocity (tps)", robotHardware.outtakeMotor.getVelocity());
        log("Distance from Goal", currentPose.distanceFrom(poses.goalPose));
        log("IMU Heading (deg)", Math.toDegrees(imuHeading));
        panelsTelemetry.update(telemetry);

    }
}
