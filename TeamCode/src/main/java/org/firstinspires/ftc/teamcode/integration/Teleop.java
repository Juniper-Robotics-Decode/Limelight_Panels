package org.firstinspires.ftc.teamcode.integration;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Teleop extends LinearOpMode {
    DcMotorEx motor;
    Limelight3A limelight;

    LimelightCamera limelightCamera;
    Flywheel flywheel;

    @Override
    public void runOpMode() throws InterruptedException {
        motor =  hardwareMap.get(DcMotorEx.class, "Motor");
        limelight = hardwareMap.get(Limelight3A .class, "limelight");

        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(1);


        limelightCamera = new LimelightCamera(limelight);
        flywheel = new Flywheel(motor,telemetry);

        waitForStart();

        while (opModeIsActive()) {

            limelightCamera.update();
            flywheel.updatePID(limelightCamera.getFlatDistance());

            telemetry.addData("X", limelightCamera.getX());

            telemetry.addData("Y", limelightCamera.getY());

            telemetry.addData("Z", limelightCamera.getZ());

            telemetry.addData("Flat Distance", limelightCamera.getFlatDistance());

            telemetry.addData("Current Velocity Radians", motor.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("Current Velocity Ticks", motor.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("Target Velocity Radians", flywheel.getTargetVelocity());

            telemetry.update();
        }


    }


}
