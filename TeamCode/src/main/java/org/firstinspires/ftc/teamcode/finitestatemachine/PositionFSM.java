package org.firstinspires.ftc.teamcode.finitestatemachine;

import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.finitestatemachine.wrappers.HWMap;
import org.firstinspires.ftc.teamcode.finitestatemachine.wrappers.LimelightCamera;

public class PositionFSM {

    public enum States {
        ZONE_1(10),
        ZONE_2(25),
        ZONE_3(30),
        NO_VALID_TARGET;

        private double targetAngle;

        States(double angle) {
            this.targetAngle = angle;
        }

        States() {
            this.targetAngle = 0; // doesn't matter if 0 - never used
        }

        public double getTargetAngle() {
            return targetAngle;
        }
    }

    private States state;
    private LimelightCamera limelightCamera;
    private InterpLUT velocityMap;

    private double defaultFlywheelVelocity;
    private double flywheelTargetVelocityRPM;
    private double pitchTargetAngle;
    private double turretError;

    private double LIMELIGHT_FORWARD_OFFSET = 0.33;
    private double threshold1 = 2, threshold2 = 3;

    private Telemetry telemetry;

    public PositionFSM(HWMap hwMap, Telemetry telemetry) {
        limelightCamera = new LimelightCamera(hwMap.getLimelight(), telemetry);
        state = States.NO_VALID_TARGET;
        createVelocityMap();
        this.telemetry = telemetry;
    }

    public void updateState() {
        limelightCamera.update();

        if(limelightCamera.hasTarget()) {
            if (limelightCamera.getFlatDistance() >= threshold2) {
                state = States.ZONE_3;
            } else if (limelightCamera.getFlatDistance() >= threshold1) {
                state = States.ZONE_2;
            } else {
                state = States.ZONE_1;
            }
        }
        else {
            state = States.NO_VALID_TARGET;
        }


        findFlywheelTargetVelocity(limelightCamera.getFlatDistance());
        findPitchTargetAngle();
        findTurretError(limelightCamera.getTx());
    }

    private void createVelocityMap() {
        velocityMap = new InterpLUT();

        // distance (m) , velocity (rpm)

        velocityMap.add(1.2, 3450);
        velocityMap.add(2.048, 3642);
        velocityMap.add(2.896, 3814);
        velocityMap.add(3.745, 3942.85);

        velocityMap.createLUT();

    }

    public void findFlywheelTargetVelocity(double distance_m) {
        if(distance_m < 0.9 || distance_m > 3.415) {
            flywheelTargetVelocityRPM = defaultFlywheelVelocity;
        }
        else {
            flywheelTargetVelocityRPM = velocityMap.get(distance_m+LIMELIGHT_FORWARD_OFFSET);
        }

    }

    public void findPitchTargetAngle() {
        if(state == States.NO_VALID_TARGET) {
            return;
        }
        pitchTargetAngle = state.getTargetAngle();
    }

    public void findTurretError(double tx) {
        if(state == States.NO_VALID_TARGET) {
            turretError = 0;
        }
        else {
            turretError = tx;
        }
    }

    public double getFlywheelTargetVelocityRPM() {
        return flywheelTargetVelocityRPM;
    }

    public double getPitchTargetAngle() {
        return pitchTargetAngle;
    }

    public double getTurretError() {
        return turretError;
    }

    public void log() {
        telemetry.addData("position FSM state", state);
        telemetry.addData("Flywheel Target", flywheelTargetVelocityRPM);
        telemetry.addData("Pitch Target", pitchTargetAngle);
        telemetry.addData("Turret Error", turretError);

        telemetry.addData("X", limelightCamera.getX());
        telemetry.addData("Y", limelightCamera.getY());
        telemetry.addData("Z", limelightCamera.getZ());
        telemetry.addData("Flat Distance", limelightCamera.getFlatDistance());
        telemetry.addData("tx",limelightCamera.getTx());
        telemetry.addData("Has target", limelightCamera.hasTarget());
    }

}
