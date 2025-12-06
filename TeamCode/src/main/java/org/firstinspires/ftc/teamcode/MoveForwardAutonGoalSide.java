package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@Autonomous(name="GOAL SIDE - MoveForward Auton", group="Robot")

public class MoveForwardAutonGoalSide extends LinearOpMode {

    Hardware robotHardware = new Hardware();
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        robotHardware.initialize(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        timer.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            sleep(3500); // 3.5 seconds
            drive(-0.5,-0.5,-0.5,-0.5,1300); // backwards for 1 second TODO: FUNCTION IS BLOCKING

        }
    }

    private void drive(double flp, double frp, double blp, double brp, long millis) {
        robotHardware.frontLeft.setPower(flp);
        robotHardware.frontRight.setPower(frp);
        robotHardware.backLeft.setPower(blp);
        robotHardware.backRight.setPower(brp);
        timer.reset();
        while (timer.milliseconds() <= millis) {sleep(10);}
        robotHardware.frontLeft.setPower(0);
        robotHardware.frontRight.setPower(0);
        robotHardware.backLeft.setPower(0);
        robotHardware.backRight.setPower(0);
    }
}