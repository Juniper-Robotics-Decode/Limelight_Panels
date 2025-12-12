package org.firstinspires.ftc.teamcode.shooter.testClasses.flywheelCameraTest;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;

public class FlywheelCameraTest extends LinearOpMode {
    MotorEx motor;
    Limelight3A limelight;
    LimelightCamera limelightCamera;
    Flywheel flywheel;


    @Override
    public void runOpMode() throws InterruptedException {

        motor = new MotorEx(hardwareMap,"Motor", Motor.GoBILDA.BARE);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        /*DcMotorEx internalMotor = (DcMotorEx) motor;
        LynxModule module = (LynxModule) internalMotor.getController();*/



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


      //      double current = module.getCurrent(CurrentUnit.AMPS);

            telemetry.addData("X", limelightCamera.getX());

            telemetry.addData("Y", limelightCamera.getY());

            telemetry.addData("Z", limelightCamera.getZ());

            telemetry.addData("Flat Distance", limelightCamera.getFlatDistance());

            telemetry.addData("tx",limelightCamera.getTx());

            telemetry.addData("Has target", limelightCamera.hasTarget());

            telemetry.addData("Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());

          //  telemetry.addData("Current", current);

            telemetry.addData("accel",motor.getAcceleration());



            telemetry.update();
        }


    }


}
