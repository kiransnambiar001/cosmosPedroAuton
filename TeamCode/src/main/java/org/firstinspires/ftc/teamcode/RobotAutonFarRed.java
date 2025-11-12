package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RobotAutonFarRed", group="Robot")
public class RobotAutonFarRed extends LinearOpMode {

    // Power constants
    private static final double MOVE_POWER = 0.6;
    private static final double TURN_POWER = 0.5;
    private static final int COMMAND_DELAY_MS = 100; // Small delay between commands

    @Override
    public void runOpMode() {

        // Initialize all hardware and subsystems
        Hardware robotHardware = new Hardware();
        robotHardware.initialize(hardwareMap, true);
        Drivetrain drivetrain = new Drivetrain(robotHardware);
        Intake intake = new Intake(robotHardware);
        Storage storage = new Storage(robotHardware);
        Outtake outtake = new Outtake(robotHardware);

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.addData("Path", "Blue Alliance, Far Side");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            try {
                telemetry.addData("Status", "Running Autonomous");
                telemetry.update();

                // Step 1: Move forward 30 inches (76.2 cm)
                logStep("Moving forward 76.2 cm");
                drivetrain.move(30, MOVE_POWER);
                sleep(COMMAND_DELAY_MS);

//                // Step 2: Turn 25 degrees left
//                logStep("Turning 25 degrees left");
//                drivetrain.turn(25, TURN_POWER);
//                sleep(COMMAND_DELAY_MS);
//
//                // Step 3: Power Outtake
//                logStep("Powering Outtake Power w/ 'Far' Preset");
//                double outtakePower = outtake.setPreset("far");
//                outtake.run(outtakePower);
//                sleep(1000); // longer sleep to allow flywheel to speed up
//
//                // Step 4: Power Storage and intake while outtake is running
//                logStep("Turning on storage and intake motors"); // feeding balls into outtake
//                intake.run(1.0);
//                storage.run(1.0);
//                sleep(5000); // extra sleep to ensure launching
//
//                // Step 5: Turn off outtake, intake, and storage
//                logStep("Turning off outtake, intake, and storage motors");
//                intake.run(0);
//                storage.run(0);
//                outtake.run(0);
//                sleep(COMMAND_DELAY_MS);
//
//                // Step 5: Move forward 16 inches (40.64 cm)
//                logStep("Moving forward 40.64 cm");
//                drivetrain.move(40.64, MOVE_POWER);
//                sleep(COMMAND_DELAY_MS);


//                // Step 1: Move forward 36 inches (91.44 cm)
//                logStep("Moving forward 91.44 cm");
//                drivetrain.move(91.44, MOVE_POWER);
//                sleep(COMMAND_DELAY_MS);
//
//                // Step 2: Turn 90 degrees left
//                logStep("Turning 90 degrees");
//                drivetrain.turn(90, TURN_POWER);
//                sleep(COMMAND_DELAY_MS);
//
//                // Step 3: Move forward 24 inches (60.96 cm)
//                logStep("Moving forward 60.96 cm");
//                drivetrain.move(60.96, MOVE_POWER);
//                sleep(COMMAND_DELAY_MS);
//
//                // Step 4: Turn 90 degrees right
//                logStep("Turning -90 degrees");
//                drivetrain.turn(-90, TURN_POWER);
//                sleep(COMMAND_DELAY_MS);
//
//                // Step 5: Run intake and move forward slowly to collect
//                logStep("Collecting artifact");
//                intake.run(1.0);
//                sleep(COMMAND_DELAY_MS);
//                drivetrain.move(25.4, MOVE_POWER * 0.5);
//                sleep(500);
//                intake.run(0.0);
//                sleep(COMMAND_DELAY_MS);
//
//                // Step 6: Move backward 24 inches (60.96 cm)
//                logStep("Moving backward 60.96 cm");
//                drivetrain.move(-60.96, MOVE_POWER);
//                sleep(COMMAND_DELAY_MS);
//
//                // Step 7: Turn 90 degrees right
//                logStep("Turning -90 degrees");
//                drivetrain.turn(-90, TURN_POWER);
//                sleep(COMMAND_DELAY_MS);
//
//                // Step 8: Move forward 24 inches (60.96 cm)
//                logStep("Moving forward 60.96 cm");
//                drivetrain.move(60.96, MOVE_POWER);
//                sleep(COMMAND_DELAY_MS);
//
//                // Step 9: Turn 90 degrees left
//                logStep("Turning 90 degrees");
//                drivetrain.turn(90, TURN_POWER);
//                sleep(COMMAND_DELAY_MS);
//
//                // Step 10: Launch 2 artifacts
//                logStep("Preparing to launch artifacts");
//                double launchPower = outtake.setPreset("close");
//                outtake.run(launchPower);
//                sleep(1500); // Spool up time
//
//                logStep("Launching first artifact");
//                storage.run(1.0);
//                sleep(750);
//                storage.run(0.0);
//                sleep(500);
//
//                logStep("Launching second artifact");
//                storage.run(1.0);
//                sleep(750);
//                storage.run(0.0);
//                sleep(COMMAND_DELAY_MS);
//
//                outtake.run(0.0);
//                sleep(COMMAND_DELAY_MS);
//
//                // Step 11: Final positioning
//                logStep("Final positioning");
//                drivetrain.move(33.02, MOVE_POWER);
//                sleep(COMMAND_DELAY_MS);

                // Ensure all motors are stopped
                drivetrain.stopMotors();
                intake.run(0.0);
                storage.run(0.0);
                outtake.run(0.0);

                // End of Sequence
                telemetry.addData("Status", "Autonomous Complete");
                telemetry.update();

            } catch (Exception e) {
                telemetry.addData("ERROR", e.getMessage());
                telemetry.update();

                // Emergency stop all motors
                drivetrain.stopMotors();
                intake.run(0.0);
                storage.run(0.0);
                outtake.run(0.0);
            }
        }
    }

    private void logStep(String step) {
        telemetry.addData("Current Step", step);
        telemetry.update();
    }
}