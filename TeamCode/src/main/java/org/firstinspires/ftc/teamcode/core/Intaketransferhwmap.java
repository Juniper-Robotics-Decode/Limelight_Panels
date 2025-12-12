
package org.firstinspires.ftc.teamcode.core;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intaketransferhwmap {
    private final MotorEx intakeMotor;
    private final MotorEx transferMotor;
    private final Servo transferServo;

    public Intaketransferhwmap(HardwareMap hardwareMap) {
        intakeMotor = new MotorEx(hardwareMap, "IM", Motor.GoBILDA.RPM_1150);
        transferMotor = new MotorEx(hardwareMap, "TRM", Motor.GoBILDA.RPM_312);
        transferServo = hardwareMap.get(Servo.class, "TS");
        transferServo.setDirection(Servo.Direction.REVERSE);
    }

    public MotorEx getIntakeMotor() {
        return intakeMotor;
    }

    public MotorEx getTransferMotor() {
        return transferMotor;
    }

    public Servo getTransferServo() {
        return transferServo;
    }
}




