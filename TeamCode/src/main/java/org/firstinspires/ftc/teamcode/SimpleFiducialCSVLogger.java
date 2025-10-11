package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.opencsv.CSVWriter;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@TeleOp
public class SimpleFiducialCSVLogger extends LinearOpMode {

    private Limelight3A limelight;
    private File file;
    private FileWriter fileWriter;
    private CSVWriter csvWriter;
    private final ArrayList<String[]> dataArray = new ArrayList<>();

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(1);

        // CSV file setup
        file = new File(String.format("%s/FIRST/fiducialData.csv",
                Environment.getExternalStorageDirectory().getAbsolutePath()));
        try {
            fileWriter = new FileWriter(file);
            csvWriter = new CSVWriter(fileWriter);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        // Header row
        dataArray.add(new String[]{"X (m)", "Y (m)", "FlatDistance (m)"});

        waitForStart();

        for (int i = 0; i < 3000 && opModeIsActive(); i++) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                // Get position data from first detected fiducial
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {

                    LLResultTypes.FiducialResult fiducial = fiducials.get(0); // just takes first fiducial it sees - before had a for loop to find ID

                    double x = fiducial.getCameraPoseTargetSpace().getPosition().x;
                    double y = fiducial.getCameraPoseTargetSpace().getPosition().y;

                    double flatDistance = Math.sqrt(x * x + y * y);

                    telemetry.addData("Fiducial ID", fiducial.getFiducialId());
                    telemetry.addData("X", x);
                    telemetry.addData("Y", y);
                    telemetry.addData("Flat Dist", flatDistance);

                    dataArray.add(new String[]{
                            String.valueOf(x),
                            String.valueOf(y),
                            String.valueOf(flatDistance)
                    });
                } else {
                    telemetry.addData("Fiducials", "None detected");
                }
            } else {
                telemetry.addData("Limelight", "No valid result");
            }
            telemetry.update();
        }

        exportData();
    }

    private void exportData() {
        try {
            csvWriter.writeAll(dataArray);
            csvWriter.close();
        } catch (Exception ignored) {}
    }
}
