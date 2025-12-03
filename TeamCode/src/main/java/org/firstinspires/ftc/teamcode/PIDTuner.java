package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="PID Tuner", group="LinearOpMode")
public class PIDTuner extends LinearOpMode {
    Hardware robotHardware = new Hardware();
    Outtake robotOuttake;

    private double pVal = 0;
    private double iVal = 0;
    private double dVal = 0;
    private double fVal = 0.013;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware.initialize(hardwareMap);
        robotOuttake = new Outtake(robotHardware);

        waitForStart();

        boolean rb2prevState = false;
        boolean lb2prevState = false;
        boolean dpu2prevState = false;
        boolean dpd2prevState = false;

        while (opModeIsActive()) {
            boolean rb2 = gamepad2.right_bumper;
            boolean lb2 = gamepad2.left_bumper;
            boolean dpu2 = gamepad2.dpad_up;
            boolean dpd2 = gamepad2.dpad_down;
            double rt2 = gamepad2.right_trigger;

            if (rb2 && !rb2prevState) {
                pVal += 0.001;
            } rb2prevState = rb2;

            if (lb2 && !lb2prevState) {
                pVal -= 0.001;
            } lb2prevState = lb2;

            if (dpu2 && !dpu2prevState) {
                dVal += 0.001;
            } dpu2prevState = dpu2;

            if (dpd2 && !dpd2prevState) {
                dVal -= 0.001;
            } dpd2prevState = dpd2;
            robotHardware.outtakeMotor.setVelocityPIDFCoefficients(pVal,iVal,dVal,fVal);

            if (rt2 >0.5) {
                robotOuttake.run(0.7);
            } else {
                robotOuttake.run(0);
            }

            telemetry.addData("P Value", pVal);
            telemetry.addData("I Value", iVal);
            telemetry.addData("D Value", dVal);
            telemetry.addData("F Value", fVal);
            telemetry.addData("Target Velocity (tps)", robotOuttake.getTargetTps());
            telemetry.addData("Current Velocity (tps)", robotHardware.outtakeMotor.getVelocity());
            telemetry.update();
        }

    }
}
