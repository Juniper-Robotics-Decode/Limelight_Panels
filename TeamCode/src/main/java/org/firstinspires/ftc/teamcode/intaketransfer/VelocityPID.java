package org.firstinspires.ftc.teamcode.intaketransfer;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class VelocityPID extends LinearOpMode{
    DcMotorEx motor;
    public static double vP=0.001, vI=0, vD=0, vF=0;

    public static double velocity = 12000;

    public static double targetVelocity = Math.abs((double) velocity /60*2*Math.PI);

    private PIDFController pidfController;

    double measuredVelocity;

    @Override
    public void runOpMode(){
        motor = hardwareMap.get(DcMotorEx.class, "Motor");
        //motor.setDirection(DcMotorSimple.Direction.REVERSE);
        pidfController = new PIDFController(vP,vI,vD,vF);
        this.telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()){
            pidfController.setPIDF(vP,vI,vD,vF);
            measuredVelocity = motor.getVelocity();
            updatePID();
            telemetry.addData("p = ", pidfController.getP());
            telemetry.addData("currentVelocity", measuredVelocity*-1);
            telemetry.addData("targetVelocity", targetVelocity);
            telemetry.update();
        }
    }

    public void updatePID() { // This method is used to update position every loop.

            // The error - sign (which finds velocity)
            double error = targetVelocity - measuredVelocity;

            // We use zero because we already calculate for error
            double power = pidfController.calculate(error, 0);
            motor.setVelocity(power,RADIANS);
        }
}
