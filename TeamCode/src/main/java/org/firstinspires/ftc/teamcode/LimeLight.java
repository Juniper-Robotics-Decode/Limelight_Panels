package org.firstinspires.ftc.teamcode;

import com.bylazar.utils.LoopTimer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

@TeleOp
public class LimeLight extends LinearOpMode {
    Limelight3A limelight;
     ElapsedTime timer;

    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // 100 updates per sec
        limelight.start();
        limelight.pipelineSwitch(0);
        timer = new ElapsedTime(0);


        waitForStart();

        while (opModeIsActive()) {
            timer.reset();
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());

            LLResult result = limelight.getLatestResult();


            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);

            telemetry.addData("result there", result != null);
            telemetry.addData("result valid", result.isValid());


            if (result != null && result.isValid()) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)



                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            telemetry.addData("pipeline index", result.getPipelineIndex());
            telemetry.addData("PipeLine Type", status.getPipelineType());
            telemetry.addData("Loop Time", timer.milliseconds());
            telemetry.update();

        }
    }


}
