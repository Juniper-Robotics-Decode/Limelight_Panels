package org.firstinspires.ftc.teamcode.integration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class PIDTest extends LinearOpMode {

    MotorEx motor;
    public static double vP=0.005, vI=0, vD=0, vF = 0;

    public static double ks=0, kv=1.2235, ka=0;

    public static double defaultVelocity = 0;  // RPM

    public static double targetVelocityRPM = defaultVelocity;

    public static double targetVelocityTicks;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = new MotorEx(hardwareMap,"Motor", Motor.GoBILDA.BARE);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor.setRunMode(Motor.RunMode.VelocityControl);

        waitForStart();
        while (opModeIsActive()) {
            updatePID();
            telemetry.update();
        }
    }



    public void updatePID() { // This method is used to update position every loop.

        /*pidfController.setPIDF(vP,vI,vD,vF);


        measuredVelocityTicks = motor.getCorrectedVelocity();
        //   measuredVelocityRPM = convertTicksToRPM(measuredVelocityTicks);

        // The error - sign (which finds velocity)
        double error = targetVelocity - measuredVelocityTicks;

        // We use zero because we already calculate for error
        double additionalPower = pidfController.calculate(error, 0);

        */


        motor.setVeloCoefficients(vP,vI,vD);
        motor.setFeedforwardCoefficients(ks,kv,ka);
        targetVelocityTicks = convertRPMToTicks(targetVelocityRPM);
        motor.setVelocity(targetVelocityTicks);
        telemetry.addData("Target Velocity RPM", targetVelocityRPM);
        telemetry.addData("Target Velocity Ticks", targetVelocityTicks);
        telemetry.addData("Current Velocity Corrected", motor.getCorrectedVelocity());
        telemetry.addData("Current Velocity Get", motor.getVelocity());

        //motor.setVelocity(targetVelocity,RADIANS);
    }
    private static double convertRPMToTicks(double RPMVelocity) {
        return (RPMVelocity*28)/60;
    }



}
