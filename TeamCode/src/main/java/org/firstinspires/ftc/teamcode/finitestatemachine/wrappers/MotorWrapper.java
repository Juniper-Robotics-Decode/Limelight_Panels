/*
package org.firstinspires.ftc.teamcode.finitestatemachine.wrappers;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class MotorWrapper {
    private final MotorEx motorEx;
    private double lastReadVelocity;
    private double lastReadPosition;

    public MotorWrapper(MotorEx motorEx, boolean velocityControl) {
        this.motorEx = motorEx;
        if (velocityControl) {
            motorEx.setRunMode(Motor.RunMode.VelocityControl);
        }
    }

    */
/**
     Description: The set method is a wrapper of the motor set method
     @param : the power to set the motor at
     *//*

    public void set(double power) {
        motorEx.set(power);
    }

    public double get() {
        return motorEx.get();
    }
    */
/**
     Description: The following method reads the encoder to get velocity of the motor
     @return: the velocity that is just read
     *//*

    public double readVelocity() {
        lastReadVelocity = motorEx.getCorrectedVelocity();
        return lastReadVelocity;
    }

    */
/**
     Description: The following method gets the velocity of the roller that was last read
     @return: the velocity that was read before
     *//*

    public double getVelocity() {
        return lastReadVelocity;
    }

    public void setVelocity(double targetVelocity) {
        motorEx.setVelocity(targetVelocity);
    }

    public void setVelocityConstants(double vP, double vI, double vD, double ks, double kv, double ka) {
        motorEx.setVeloCoefficients(vP,vI,vD);
        motorEx.setFeedforwardCoefficients(ks,kv,ka);

    }

    // TODO: add position

    public double readPosition() {
        re
    }

}
*/
