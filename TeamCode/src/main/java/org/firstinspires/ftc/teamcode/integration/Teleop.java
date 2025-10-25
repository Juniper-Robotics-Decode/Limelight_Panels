package org.firstinspires.ftc.teamcode.integration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.FtcDashboard;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Teleop extends LinearOpMode {
    MotorEx motor;
    Limelight3A limelight;
    LimelightCamera limelightCamera;
    Flywheel flywheel;


    @Override
    public void runOpMode() throws InterruptedException {

        motor = new MotorEx(hardwareMap,"Motor", Motor.GoBILDA.BARE);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(1);


        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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

            telemetry.update();
        }


    }


}
