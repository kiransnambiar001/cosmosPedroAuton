package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@TeleOp(name="PID Tuner", group="LinearOpMode")
public class PIDTuner extends LinearOpMode {
    Hardware robotHardware = new Hardware();
    Outtake robotOuttake;

    private double pVal, fVal, dVal, iVal;




    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware.initialize(hardwareMap);
        robotOuttake = new Outtake(robotHardware);

        PIDFCoefficients coefficients = robotHardware.outtakeMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fVal = coefficients.f;
        dVal = coefficients.d;
        iVal = coefficients.i;
        pVal = coefficients.p;

        waitForStart();

        boolean rb2prevState = false;
        boolean lb2prevState = false;
        boolean dpu2prevState = false;
        boolean dpd2prevState = false;
        boolean dpr2prevState = false;
        boolean dpl2prevState = false;


        while (opModeIsActive()) {
            boolean rb2 = gamepad2.right_bumper; // pval+
            boolean lb2 = gamepad2.left_bumper; // pval-
            boolean dpu2 = gamepad2.dpad_up; // dval+
            boolean dpd2 = gamepad2.dpad_down; // dval-
            boolean dpr2 = gamepad2.dpad_right; // ival+
            boolean dpl2 = gamepad2.dpad_left; // ival-
            double rt2 = gamepad2.right_trigger; // turn on outtake to 0.7
            double lt2 = gamepad2.left_trigger;


            if (rb2 && !rb2prevState) {pVal += 0.01;} rb2prevState = rb2;
            if (lb2 && !lb2prevState) {pVal -= 0.01;} lb2prevState = lb2;

            if (dpu2 && !dpu2prevState) {dVal += 0.01;} dpu2prevState = dpu2;
            if (dpd2 && !dpd2prevState) {dVal -= 0.01;} dpd2prevState = dpd2;

            if (dpr2 && !dpr2prevState) {iVal += 0.01;} dpr2prevState = dpr2;
            if (dpl2 && !dpl2prevState) {iVal -= 0.01;} dpl2prevState = dpl2;


//            robotHardware.outtakeMotor.setVelocityPIDFCoefficients(pVal,iVal,dVal,fVal);

            if (rt2 >0.5) {robotOuttake.run(0.1);}
            else if (lt2 > 0.5) {robotOuttake.run(0.05);}
            else {robotOuttake.run(0);}

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
