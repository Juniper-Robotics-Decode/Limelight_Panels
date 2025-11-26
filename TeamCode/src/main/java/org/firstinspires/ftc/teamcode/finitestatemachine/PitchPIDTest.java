package org.firstinspires.ftc.teamcode.finitestatemachine;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.finitestatemachine.wrappers.AxonCRServoWrapper;
import org.firstinspires.ftc.teamcode.finitestatemachine.wrappers.HWMap;
import org.firstinspires.ftc.teamcode.finitestatemachine.wrappers.MotorWrapper;

@Config
@TeleOp
public class PitchPIDTest extends LinearOpMode {

    HWMap hwMap;
    private AxonCRServoWrapper pitchServo;
    private double targetAngle;

    private PIDFController pidfController;
    public static double TOLERANCE = 3;
    public static double P=0, I=0, D=0, F=0;
    public static double gearRatio = 1.0/12.0;

    public static double UPPER_HARD_STOP = 90;
    public static double LOWER_HARD_STOP = -90;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HWMap(hardwareMap);
        pitchServo = new AxonCRServoWrapper(hwMap.getPitchServo(),hwMap.getPitchEncoder(),false,false,0,gearRatio); // TODO: Change ratio
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pidfController.setTolerance(TOLERANCE);
        pidfController = new PIDFController(P,I,D,F);

        waitForStart();
        while (opModeIsActive()) {
            updatePID();
            telemetry.addData("pitch target angle", targetAngle);
            telemetry.addData("pitch servo current angle", pitchServo.getServoAngle());
            telemetry.addData("pitch current angle", pitchServo.getScaledPos());
            telemetry.update();
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



}

