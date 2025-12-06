package org.firstinspires.ftc.teamcode.finitestatemachine.wrappers;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HWMap {
    private final MotorEx flywheelMotor;
    private final MotorEx turretMotor;
    private final CRServo pitchServo;
    private final AnalogInput pitchEncoder;
    private final Limelight3A limelight;

    public HWMap (HardwareMap hardwareMap) {
        flywheelMotor = new MotorEx(hardwareMap,"FM", Motor.GoBILDA.BARE);
        flywheelMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        turretMotor = new MotorEx(hardwareMap,"TM", Motor.GoBILDA.RPM_1150); // TODO: get right RPM
        pitchServo = new CRServo(hardwareMap, "PS");
        pitchEncoder = hardwareMap.get(AnalogInput.class, "PE");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }


    public Limelight3A getLimelight() {
        return limelight;
    }

    public MotorEx getFlywheelMotor() {
        return flywheelMotor;
    }

    public MotorEx getTurretMotor() {
        return turretMotor;
    }

    public CRServo getPitchServo() {
        return pitchServo;
    }

    public AnalogInput getPitchEncoder() {
        return pitchEncoder;
    }
}
