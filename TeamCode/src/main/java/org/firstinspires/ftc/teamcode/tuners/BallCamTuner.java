package org.firstinspires.ftc.teamcode.tuners;

import java.util.function.Function;
import java.util.function.Supplier;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.gamepad.GamepadManager;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Vector;
import com.pedropathing.util.PoseHistory;
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDFController;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PresetPoses;
import org.firstinspires.ftc.teamcode.subsystems.CRServoStorage;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;




@Configurable
@TeleOp(name="BallCam Tuner", group="LinearOpMode")
public class BallCamTuner extends OpMode {

    // Create hardware object
    Hardware robotHardware = new Hardware();

    // pp vars
    public static Pose startPose = new Pose(72, 72, Math.toRadians(0));
    public static Pose currentPose;
    private Follower follower;
    private Supplier<PathChain> toShootPosePath;
    private Function<Pose, PathChain> toLaunchLinePath;
    private TelemetryManager panelsTelemetry;


    // toggles
    boolean ballCamToggle = false;
    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 0.075;
    public static double kF = 0.1;
    public PIDFCoefficients ballCamCoefficients = new PIDFCoefficients(kP, kI, kD, kF);
    PIDFController ballCamController = new PIDFController(ballCamCoefficients);



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


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        Drawing.init();

        telemetry.addData("Status", "INITIALIZED");
        telemetry.addData("Drive Mode", "Ready for TeleOp");
        telemetry.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        robotHardware.imu.resetYaw();
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        currentPose = follower.getPose();


        double imuHeading = robotHardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //      Drivetrain Control
        //Field centric toggle

        double targetHeading = 0;
        double currentHeading = follower.getPose().getHeading();

        double output = ballCamController.calculate(currentHeading, targetHeading);
        output = Math.max(-1.0, Math.min(1.0, output));
        follower.setTeleOpDrive(0, 0, output, false);

        ballCamCoefficients = new PIDFCoefficients(kP, kI, kD, kF);
        ballCamController.setCoefficients(ballCamCoefficients);

        log("Status", "Running");
        log("IMU Heading (deg)", Math.toDegrees(imuHeading));
        log("Position", String.format("X: %.2f, Y: %.2f", currentPose.getX(), currentPose.getY()));
        panelsTelemetry.update(telemetry);
        follower.update();
        draw();

    }
}
