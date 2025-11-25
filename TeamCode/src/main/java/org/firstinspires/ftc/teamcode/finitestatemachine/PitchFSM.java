package org.firstinspires.ftc.teamcode.finitestatemachine;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.finitestatemachine.wrappers.AxonCRServoWrapper;
import org.firstinspires.ftc.teamcode.finitestatemachine.wrappers.HWMap;
import org.firstinspires.ftc.teamcode.finitestatemachine.wrappers.MotorWrapper;

public class PitchFSM {
    public enum States{
        ALIGNING,
        ALIGNED
    }

    private AxonCRServoWrapper pitchServo;
    private States state;
    private double targetAngle;

    private PIDFController pidfController;
    public static double TOLERANCE = 3;
    public static double P=0, I=0, D=0, F=0;

    Telemetry telemetry;

    public PitchFSM(HWMap hwMap, Telemetry telemetry) {
       // pitchServo = new AxonCRServoWrapper(hwMap.ge) // TODO: gear ratio +
        state = States.ALIGNING;
        pidfController = new PIDFController(P,I,D,F);
        pidfController.setTolerance(TOLERANCE);
        this.telemetry = telemetry;
    }

    public void updateState(){
        updatePID();
        if(pidfController.atSetPoint()) {
            state = TurretFSM.States.ALIGNED;
        }
        else {
            state = TurretFSM.States.ALIGNING;
        }
    }

    public void updatePID() {
        pidfController.setPIDF(P,I,D,F);
        pidfController.setTolerance(TOLERANCE);
        turretMotor.readPosition();

        double delta = angleDelta(turretMotor.getScaledPos(), targetAngle);
        double sign = angleDeltaSign(turretMotor.getScaledPos(), targetAngle);
        double error = delta * sign;

        double power = pidfController.calculate(error,0);
        turretMotor.set(power);
    }

    private double angleDelta(double measuredAngle, double targetAngle) {
        return Math.min(normalizeDegrees(measuredAngle - targetAngle), 360 - normalizeDegrees(measuredAngle - targetAngle));
    }

    private double angleDeltaSign(double measuredAngle, double targetAngle) {
        return -(Math.signum(normalizeDegrees(targetAngle - measuredAngle) - (360 - normalizeDegrees(targetAngle - measuredAngle))));
    }

    public void setTargetAngle(double turretError) {
        targetAngle = turretMotor.getScaledPos() + turretError;
    }

    public boolean ALIGNED() {
        return state == TurretFSM.States.ALIGNED;
    }

    public void log() {
        telemetry.addData("turret state", state);
        telemetry.addData("turret target angle", targetAngle);
        telemetry.addData("turret current angle", turretMotor.getScaledPos());
    }

}
