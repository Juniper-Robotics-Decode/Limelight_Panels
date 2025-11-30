package org.firstinspires.ftc.teamcode.finitestatemachine.wrappers;


import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class NewAxonServo {
    private CRServo axon;
    private AnalogInput encoder;
    private double sign = 1;
    private double ratio;

    // New variables for continuous angle tracking
    private double continuousServoAngle = 0;
    private double lastRawServoAngle = 0;
    private double encoderOffset = 0; // We'll handle offset differently
    private boolean isFirstRead = true;

    private boolean inverseEncoder = false;


    public NewAxonServo(CRServo axon, AnalogInput encoder, boolean inversePower, boolean inverseEncoder, double encoderOffset, double ratio) {
        this.axon = axon;
        this.encoder = encoder;
        this.ratio = ratio;

        if (inversePower) {
            sign = -1;
        }
        this.inverseEncoder = inverseEncoder;

        // Use the offset on the very first read
        this.encoderOffset = encoderOffset;
    }

    public void set(double power) {
        axon.set(power * sign);
    }

    public double get() {
        return axon.get() * sign;
    }

    /**
     * Reads the encoder and calculates the continuous angle.
     * This method MUST be called once per loop.
     */
    public void readPos() {
        // 1. Get the raw 0-360 angle from the encoder
        double currentRawAngle = (encoder.getVoltage() / 3.3) * 360.0;

        if (inverseEncoder) {
            currentRawAngle = 360.0 - currentRawAngle;
        }

        // 2. Handle the very first read to set our starting point
        if (isFirstRead) {
            lastRawServoAngle = currentRawAngle;
            // Apply offset *only* to the starting continuous angle
            continuousServoAngle = currentRawAngle + encoderOffset;
            isFirstRead = false;
            return;
        }

        // 3. Calculate the change (delta) from the last angle
        double delta = currentRawAngle - lastRawServoAngle;

        // 4. Handle the "wrap-around" (the core logic)
        // If the jump is more than 180 degrees, it must have wrapped
        if (delta > 180.0) {
            delta -= 360.0; // e.g., jump from 10 to 350 -> delta = 340 -> delta = -20
        } else if (delta < -180.0) {
            delta += 360.0; // e.g., jump from 350 to 10 -> delta = -340 -> delta = +20
        }

        // 5. Add the corrected delta to our continuous angle
        continuousServoAngle += delta;

        // 6. Save the current raw angle for the next loop's comparison
        lastRawServoAngle = currentRawAngle;
    }

    /**
     * @return The raw 0-360 angle from the encoder (use for debugging)
     */
    public double getServoAngle() {
        return lastRawServoAngle;
    }

    /**
     * @return The continuous, "unwrapped" angle of the mechanism
     * This is what you feed into your PID.
     */
    public double getScaledPos() {
        // We apply the ratio to the continuous angle
        return (continuousServoAngle * ratio);
    }

    // --- Other methods ---
    public double readAndGetVoltage() {
        return encoder.getVoltage();
    }

    // You must reset the continuous angle if you reset the encoder position
    public void resetEncoder(double offset) {
        this.encoderOffset = offset;
        this.isFirstRead = true; // Force a re-initialization on the next readPos()
    }
}
