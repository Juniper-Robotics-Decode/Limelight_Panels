package org.firstinspires.ftc.teamcode.finitestatemachine.wrappers;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class LimelightCamera {

    private Limelight3A limelight;

    private boolean hasValidTarget = false;
    private double x_m = 0;
    private double y_m = 0;
    private double z_m = 0;
    private int targetID = 0;
    private double flatDistance_m = 0;
    private double tx_degrees = 0;
    private double ty_degrees = 0;
    private Telemetry telemetry;

    public LimelightCamera(Limelight3A limelight3A, Telemetry telemetry) {
        limelight = limelight3A;
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(1);
        this.telemetry = telemetry;
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        telemetry.addData("is there a limelight result", !(result == null));
        telemetry.addData("is there a valid limelight result", result.isValid());
        if (result == null) { // || !result.isValid()
            hasValidTarget = false;
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        telemetry.addData("is there a fiducial result", !(fiducials == null));
        telemetry.addData("is there a non-empty fiducial", !fiducials.isEmpty());
        if (fiducials == null || fiducials.isEmpty()) {
            hasValidTarget = false;
            return;
        }

        LLResultTypes.FiducialResult fiducial = null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == 24) {
                fiducial = f;
                break;
            }
        }

        telemetry.addData("was the target ID found", !(fiducial==null));
        if (fiducial == null) {
            hasValidTarget = false;
            return;
        }

        hasValidTarget = true;
        targetID = fiducial.getFiducialId();


        x_m = fiducial.getCameraPoseTargetSpace().getPosition().x;
        y_m = fiducial.getCameraPoseTargetSpace().getPosition().y;
        z_m = fiducial.getCameraPoseTargetSpace().getPosition().z;
        tx_degrees = result.getTx();
        ty_degrees = result.getTy();

        flatDistance_m = Math.sqrt(x_m * x_m + z_m * z_m);
    }


    public boolean hasTarget() {
        return hasValidTarget;
    }

    public double getX() {
        return x_m;
    }

    public double getY() {
        return y_m;
    }

    public double getZ() {
        return z_m;
    }

    public double getTx() {
        return tx_degrees;
    }

    public double getTy() {return ty_degrees;}

    public double getFlatDistance() {
        return flatDistance_m;
    }

    public int getTargetID() {
        return targetID;
    }

}