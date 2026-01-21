package org.firstinspires.ftc.teamcode.tuners;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.subsystems.CRServoStorage;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.SolversOuttake;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.controller.PIDFController;
@Configurable
@TeleOp(name="PID Solvers Tuner", group="LinearOpMode")
public class PIDFSolversTuner extends OpMode {
    Hardware robotHardware = new Hardware();
    SolversOuttake robotOuttake;
    CRServoStorage robotStorage;
    public com.seattlesolvers.solverslib.controller.PIDFController pidf;

    public static double pVal, fVal, dVal, iVal;
    public static double idle;
    public static double increment = 0.01;
    public static double powerPercentage = 0.1;
    public TelemetryManager panels;


    private void log(String caption, Object... text) {
        if (text.length == 1) {
            telemetry.addData(caption, text[0]);
            panels.debug(caption + ": " + text[0]);
        } else if (text.length >= 2) {
            StringBuilder message = new StringBuilder();
            for (int i = 0; i < text.length; i++) {
                message.append(text[i]);
                if (i < text.length - 1) message.append(" ");
            }
            telemetry.addData(caption, message.toString());
            panels.debug(caption + ": " + message);
        }
    }

    @Override
    public void init() {
        robotHardware.initialize(hardwareMap, false);
        robotStorage = new CRServoStorage(robotHardware);

        fVal = 0.00052d;
        dVal = 0d;
        iVal = 0d;
        pVal = 0.01d;
        idle = 0.2;

        robotOuttake = new SolversOuttake(robotHardware, pVal, iVal, dVal, fVal);


        pidf = new com.seattlesolvers.solverslib.controller.PIDFController(pVal,iVal,dVal,fVal);

        panels = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        double output = pidf.calculate(robotHardware.outtakeMotor.getCurrentPosition(), robotOuttake.getTargetTps());
        boolean rb2wP = gamepad2.rightBumperWasPressed(); // pval+
        boolean lb2wP = gamepad2.leftBumperWasPressed(); // pval-
        boolean dpu2wP = gamepad2.dpadUpWasPressed(); // dval+
        boolean dpd2wP = gamepad2.dpadDownWasPressed(); // dval-
        boolean dpr2wP = gamepad2.dpadRightWasPressed(); // ival+
        boolean dpl2wP = gamepad2.dpadLeftWasPressed(); // ival-
        boolean x2wP = gamepad2.xWasPressed(); //f+
        boolean b2wP = gamepad2.bWasPressed();// f-
        double rt2 = gamepad2.right_trigger;
        double lt2 = gamepad2.left_trigger;
        double ry2 = gamepad2.right_stick_y;

        if (ry2 > 0.5) {
            robotStorage.run(1);
        } else if (ry2 < -0.5) {
            robotStorage.run(-1);
        } else {
            robotStorage.run(0);
        }


        if (rb2wP) {pVal += increment;}
        if (lb2wP) {pVal -= increment;}

        if (dpu2wP) {dVal += increment;}
        if (dpd2wP) {dVal -= increment;}

        if (dpr2wP) {iVal += increment;}
        if (dpl2wP) {iVal -= increment;}

        if (x2wP) {fVal += increment;}
        if (b2wP) {fVal -= increment;}


        if (rt2 >0.5) {robotOuttake.setTargetTps(powerPercentage);}
        else if (lt2 > 0.5) {robotOuttake.setTargetTps(powerPercentage);}
        else {robotOuttake.setTargetTps(idle);}




        // telemetry
        log("Increment Value", increment);

        log("P Value", pVal); log("I Value", iVal); log("D Value", dVal); log("F Value", fVal);

        log("Target Velocity (tps)", robotOuttake.getTargetTps());
        log("Current Velocity (tps)", robotHardware.outtakeMotor.getVelocity());

        log("Error (tps)", Math.abs(robotHardware.outtakeMotor.getVelocity() - robotOuttake.getTargetTps()));
        panels.update(telemetry);
    }
}
