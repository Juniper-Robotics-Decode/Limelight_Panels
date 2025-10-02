package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.opencsv.CSVWriter;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

@TeleOp
public class LimelightCSVTest extends LinearOpMode {

    private static final Logger log = LoggerFactory.getLogger(LimelightCSVTest.class);
    private Limelight3A limelight;
    private ElapsedTime loopTimer;
    //Objects
    private File file;
    private FileWriter fileWriter;
    private CSVWriter csvWriter;

    //Booleans
    private boolean dpadLeftWasJustPressed;
    private boolean dpadLeftIsDown;
    private boolean addData;
    private boolean dataStopped;

    private final ArrayList<String[]> dataArray = new ArrayList<String[]>();

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100); // 100 updates/sec
        limelight.start();
        limelight.pipelineSwitch(1);

        loopTimer = new ElapsedTime();

        dpadLeftWasJustPressed = false;
        dpadLeftIsDown = false;
        addData = true;
        dataStopped = false;

        file = new File(String.format("%s/FIRST/limelightData.csv", Environment.getExternalStorageDirectory().getAbsolutePath()));
        try {
            fileWriter = new FileWriter(file);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        csvWriter = new CSVWriter(fileWriter);

        dataArray.add(new String[]{"X(meters)", "Y(meters)"});

        waitForStart();

        for (int i = 0; i < 5000; i++) {
            loopTimer.reset();
            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();

          //  updateTelemetry(status, result);

            telemetry.addData("Loop Time (ms)", loopTimer.milliseconds());
            telemetry.update();


            dataCollection(result);
            telemetry.addData("-", i);
            log();
            telemetry.update();
        }

        exportData();
        telemetry.addData("-", "EXPORTED DATA");
        telemetry.update();
    }

    public void dataCollection(LLResult result) {
        if (result != null && result.isValid()) {
            Pose3D mt1 = getMegaTag1Pose(result);
            if (mt1 != null) {
                telemetry.addData("MT1 XY Distance", getXYDistanceToGoal(mt1));
                double m1X = getXCoordinate(mt1);
                telemetry.addData("MTI x", m1X);
                double m1Y = getYCoordinate(mt1);
                telemetry.addData("MTI y", m1Y);

                String x = m1X + "";
                String y = m1Y + "";

                if (addData)
                    dataArray.add(new String[]{x, y});
            }
        }
        else {
            telemetry.addData("No targets","");
        }
    }

    public void exportData() {
        dataStopped = true;
        try {
            csvWriter.writeAll(dataArray);
            csvWriter.close();
        } catch (Exception ignored) {

        }
    }

    public void log() {
        if (!addData)
            telemetry.addData("-", "DATA IS NOT BEING ADDED");
        if (dataStopped)
            telemetry.addData("-", "DATA HAS BEEN EXPORTED");

        telemetry.addData("", "");
        telemetry.addData("-", "DpadLeft to toggle readings");
        telemetry.addData("-", "DpadDown to save and close readings");
    }


    //tx,ty,ta stuff
    public double[] getTargetData(LLResult result) {
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();
            return new double[]{tx, ty, ta};
        }
        return new double[]{0, 0, 0};
    }

    // MegaTag poses returned - kind of used to create objects of both version 1 & 2

    public Pose3D getMegaTag1Pose(LLResult result) {
        if (result != null && result.isValid()) {
            return result.getBotpose();
        }
        return null;
    }

    public Pose3D getMegaTag2Pose(LLResult result) {
        if (result != null && result.isValid()) {
            return result.getBotpose_MT2();
        }
        return null;
    }

    // Next two methods find distances & orientation w/ mega tag 1 or 2

    public double getXYDistanceToGoal(Pose3D pose) {
        if (pose != null) {
            // Robot's position
            double robotX = pose.getPosition().x;
            double robotY = pose.getPosition().y;

            // Goal's position (Red Goal center)
            double goalX = 0; // TODO: Change
            double goalY = 0;

            double distanceX = goalX - robotX;
            double distanceY = goalY - robotY;
            return Math.sqrt(distanceX * distanceX + distanceY * distanceY);
        }
        return 0;
    }

    public double getXCoordinate(Pose3D pose) {
        return pose.getPosition().x;

    }

    public double getYCoordinate(Pose3D pose) {
        return pose.getPosition().y;

    }



    public double[] getOrientation3D(Pose3D pose) {
        if (pose != null) {
            double yaw = pose.getOrientation().getYaw(AngleUnit.DEGREES);
            double pitch = pose.getOrientation().getPitch(AngleUnit.DEGREES);
            double roll = pose.getOrientation().getRoll(AngleUnit.DEGREES);

            return new double[]{yaw, pitch, roll};
        }
        return new double[]{0, 0, 0};
    }


    /* --------------------  Own Algorithim -------------------- */

    private static final double CAMERA_HEIGHT_M = 0.2794;  // 11 in
    private static final double TARGET_HEIGHT_M = 0; // TODO: Change
    private static final double CAMERA_PITCH_DEG = 24.0; // may not be best data

    public double getXDistance_ty(LLResult result) {
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleToTarget = Math.toRadians(CAMERA_PITCH_DEG + ty);
            return (TARGET_HEIGHT_M - CAMERA_HEIGHT_M) / Math.tan(angleToTarget);
        }
        return 0;
    }

    public double getXYDistance_ty(LLResult result) {
        double x = getXDistance_ty(result);
        double y = TARGET_HEIGHT_M - CAMERA_HEIGHT_M;
        return Math.sqrt(x * x + y * y);
    }



    /* --------------------  Telemetry -------------------- */

    public void updateTelemetry(LLStatus status, LLResult result) {
        telemetry.addData("Name", status.getName());
        telemetry.addData("Pipeline", status.getPipelineIndex());
        telemetry.addData("FPS", status.getFps());

        if (result != null && result.isValid()) {
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("ta", result.getTa());

         /*   // MegaTag1
            Pose3D mt1 = getMegaTag1Pose(result);
            if (mt1 != null) {
                telemetry.addData("MT1 XY Distance", getXYDistanceToGoal(mt1));
            }

            // MegaTag2
            Pose3D mt2 = getMegaTag2Pose(result);
            if (mt2 != null) {
                telemetry.addData("MT2 XY Distance", getXYDistanceToGoal(mt2));
                double[] ypr = getOrientation3D(mt2);
                telemetry.addData("Yaw", ypr[0]);
                telemetry.addData("Pitch", ypr[1]);
                telemetry.addData("Roll", ypr[2]);
         */
           // telemetry.addData("Distance (ty)", getXYDistance_ty(result));

        }

        // ty-based
        else

        {
            telemetry.addData("Limelight", "No Targets");
        }
    }

}


