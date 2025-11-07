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
    public static double vP=3, vI=0, vD=0, vF = 0; // 0.3 p for just the first half

    public static double ks=0, kv=1.7, ka=0;  // 1.37 for just the first half

    public static double defaultVelocity = 0;  // RPM

    public static double targetVelocityRPM = defaultVelocity;

    public static double targetVelocityTicks;

    InterpLUT velocityMap;

    Telemetry telemetry;



    public Flywheel (MotorEx flywheel, Telemetry telemetry) {
        motor = flywheel;
        motor.setRunMode(Motor.RunMode.VelocityControl);

        createVelocityMap();
        this.telemetry = telemetry;
    }

    public void updatePID(double distance_m) { // This method is used to update position every loop.

        if(distance_m < 0.9 || distance_m > 3.745) {
            targetVelocityRPM = defaultVelocity;
        }
        else {
             setVelocityByDistance(distance_m); // set target velocity in RPM
        }

        motor.setVeloCoefficients(vP,vI,vD);
        motor.setFeedforwardCoefficients(ks,kv,ka);
        targetVelocityTicks = convertRPMToTicks(targetVelocityRPM);
        targetVelocityTicks = -targetVelocityTicks;
        motor.setVelocity(targetVelocityTicks);
        telemetry.addData("Target Velocity RPM", targetVelocityRPM);
        telemetry.addData("Target Velocity Ticks", targetVelocityTicks);
        telemetry.addData("Current Velocity Corrected", motor.getCorrectedVelocity());
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

    public void setVelocityByDistance(double distance_m) {

        targetVelocityRPM = velocityMap.get(distance_m+0.33);
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
