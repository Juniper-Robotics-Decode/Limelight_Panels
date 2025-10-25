package org.firstinspires.ftc.teamcode.integration;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.annotation.Target;

@Config
public class Flywheel{
    MotorEx motor;
    public static double vP=0.005, vI=0, vD=0, vF = 0;

    public static double ks=0, kv=1.2235, ka=0;

    public static double defaultVelocity = 0;  // RPM

    public static double targetVelocityRPM = defaultVelocity;

    public static double targetVelocityTicks;

    InterpLUT velocityMap;

    Telemetry telemetry;



    public Flywheel (MotorEx flywheel, Telemetry telemetry) {
        motor = flywheel;
        motor.setRunMode(Motor.RunMode.VelocityControl);
        //motor.setDirection(DcMotorSimple.Direction.REVERSE);
        createVelocityMap();
        this.telemetry = telemetry;
    }

    public void updatePID(double distance_m) { // This method is used to update position every loop.

        if(distance_m < 1.1684 || distance_m > 3.0226) {
            targetVelocityRPM = defaultVelocity;
        }
        else {
             setVelocityByDistance(distance_m); // set target velocity in RPM
        }

        motor.setVeloCoefficients(vP,vI,vD);
        motor.setFeedforwardCoefficients(ks,kv,ka);
        targetVelocityTicks = convertRPMToTicks(targetVelocityRPM);
        motor.setVelocity(targetVelocityTicks);
        telemetry.addData("Target Velocity RPM", targetVelocityRPM);
        telemetry.addData("Target Velocity Ticks", targetVelocityTicks);
        telemetry.addData("Current Velocity Corrected", motor.getCorrectedVelocity());
    }

    private void createVelocityMap() {
        velocityMap = new InterpLUT();

        // distance (m) , velocity (rpm)

        velocityMap.add(1.1684, 1600);
        velocityMap.add(2.032, 1725);
        velocityMap.add(2.6416, 1775);
        velocityMap.add(3.0226, 1795);

        velocityMap.createLUT();

    }

    public void setVelocityByDistance(double distance_m) {

        targetVelocityRPM = velocityMap.get(distance_m+0.3);
    }

    private static double convertTicksToRPM(double ticksVelocity) {
        return (ticksVelocity*60)/28;
    }


    private static double RPMToRadians(double RPMVelocity) {
        return (RPMVelocity/60)*2*Math.PI;
    }

    private static double convertRPMToTicks(double RPMVelocity) {
        return (RPMVelocity*28)/60;
    }




}
