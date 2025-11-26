package org.firstinspires.ftc.teamcode.finitestatemachine;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.finitestatemachine.wrappers.AxonCRServoWrapper;
import org.firstinspires.ftc.teamcode.finitestatemachine.wrappers.HWMap;
import org.firstinspires.ftc.teamcode.finitestatemachine.wrappers.MotorWrapper;

@Config
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
    public static double UPPER_HARD_STOP = 30;
    public static double LOWER_HARD_STOP = 0;
    public static double gearRatio = 1.0/12.0;


    Telemetry telemetry;

    public PitchFSM(HWMap hwMap, Telemetry telemetry) {
        pitchServo = new AxonCRServoWrapper(hwMap.getPitchServo(),hwMap.getPitchEncoder(),false,false,0,gearRatio); // TODO: Change ratio
        state = States.ALIGNING;
        pidfController = new PIDFController(P,I,D,F);
        pidfController.setTolerance(TOLERANCE);
        this.telemetry = telemetry;
    }

    public void updateState(){
        updatePID();
        if(pidfController.atSetPoint()) {
            state = States.ALIGNED;
        }
        else {
            state = States.ALIGNING;
        }
    }

    public void updatePID() {
        if(targetAngle > UPPER_HARD_STOP) {
            targetAngle = UPPER_HARD_STOP;
        }
        else if (targetAngle < LOWER_HARD_STOP) {
            targetAngle = LOWER_HARD_STOP;
        }
        pidfController.setPIDF(P,I,D,F);
        pidfController.setTolerance(TOLERANCE);
        pitchServo.readPos();

        double delta = angleDelta(pitchServo.getScaledPos(), targetAngle);
        double sign = angleDeltaSign(pitchServo.getScaledPos(), targetAngle);
        double error = delta * sign;

        double power = pidfController.calculate(error,0);
        pitchServo.set(power);
    }

    private double angleDelta(double measuredAngle, double targetAngle) {
        return Math.min(normalizeDegrees(measuredAngle - targetAngle), 360 - normalizeDegrees(measuredAngle - targetAngle));
    }

    private double angleDeltaSign(double measuredAngle, double targetAngle) {
        return -(Math.signum(normalizeDegrees(targetAngle - measuredAngle) - (360 - normalizeDegrees(targetAngle - measuredAngle))));
    }

    public void setTargetAngle(double pitchTargetAngle) {
        targetAngle = pitchTargetAngle;
    }

    public boolean ALIGNED() {
        return state == States.ALIGNED;
    }

    public void log() {
        telemetry.addData("pitch state", state);
        telemetry.addData("pitch target angle", targetAngle);
        telemetry.addData("pitch current angle", pitchServo.getScaledPos());
    }

}
