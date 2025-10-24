package org.firstinspires.ftc.teamcode.integration;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Flywheel{
    Motor motor;
    public static double vP=0.001, vI=0, vD=0, vF=0;

    public static double defaultVelocity = 0;  // RPM

    public static double targetVelocity = defaultVelocity; // convert to rpm

    private PIDFController pidfController;

    public static double measuredVelocityTicks;

    public static double measuredVelocityRPM;

    InterpLUT velocityMap;

    Telemetry telemetry;

    double power;


    public Flywheel (Motor flywheel, Telemetry telemetry) {
        motor = flywheel;
     //   motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //motor.setDirection(DcMotorSimple.Direction.REVERSE);
        pidfController = new PIDFController(vP,vI,vD,vF);
        createVelocityMap();
        this.telemetry = telemetry;

    }

    public void updatePID(double distance_m) { // This method is used to update position every loop.


        if(distance_m < 1.1684 || distance_m > 3.0226) {
            targetVelocity = defaultVelocity;
        }
        else {
             setVelocityByDistance(distance_m); // set target velocity in RPM
        }

       // targetVelocity = convertRPMToRadiansPerSecond(targetVelocity); // target velocity now in Radians/sec

        pidfController.setPIDF(vP,vI,vD,vF);
        measuredVelocityTicks = motor.getCorrectedVelocity();
        measuredVelocityRPM = convertTicksToRPM(measuredVelocityTicks);

        // The error - sign (which finds velocity)
        double error = targetVelocity - measuredVelocityRPM;

        // We use zero because we already calculate for error
        double additionalPower = pidfController.calculate(measuredVelocityTicks, 500);
        power = power + additionalPower;
        telemetry.addData("Power", power);
        motor.set(power);
        //motor.setVelocity(targetVelocity,RADIANS);
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

    public double setVelocityByDistance(double distance_m) {

        targetVelocity = velocityMap.get(distance_m+0.3) *0.1;
        return targetVelocity;
    }

    public double getTargetVelocity () {
        return targetVelocity;
    }

    public double getMeasuredVelocityRPM () {
        return measuredVelocityRPM;
    }

    private static double convertTicksToRPM(double ticksVelocity) {
        return (ticksVelocity*60)/28;
    }


}
