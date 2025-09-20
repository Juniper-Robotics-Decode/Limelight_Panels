package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LimeLight extends LinearOpMode {
    Limelight3A limelight;

    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // 100 updates per sec
        limelight.start();
        limelight.pipelineSwitch(0);

        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("PipeLine Type", status.getPipelineType());
            //  telemetry.addData("LL Streaming", status.isStreaming());

            LLResult result = limelight.getLatestResult();

            telemetry.addData("result there", result != null);
            telemetry.addData("result valid", result.isValid());
            if (result != null) { // should be results.isValid() too but removed
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
            telemetry.update();

        }
    }


}
