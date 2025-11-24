package org.firstinspires.ftc.teamcode.finitestatemachine;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.finitestatemachine.wrappers.HWMap;

public class ShooterFSM {

    public enum States{
        PREPARING_TO_SHOOT,
        PREPARED_TO_SHOOT,
        TOGGLING_FLYWHEEL,
        TOGGLED_FLYWHEEL
    }

    private FlywheelFSM flywheelFSM;
    private PositionFSM positionFSM;
    private States state;

    private Telemetry telemetry;
    public ShooterFSM (HWMap hardwareMap, Telemetry telemetry) {
        flywheelFSM = new FlywheelFSM(hardwareMap,telemetry);
        positionFSM = new PositionFSM(hardwareMap,telemetry);
        state = States.PREPARING_TO_SHOOT;
    }

    public void updateState(boolean aPress) {
        flywheelFSM.updateState();
        positionFSM.updateState();
        findTargetState(aPress);

        switch (state) {
            case PREPARING_TO_SHOOT:
                flywheelFSM.setTargetVelocityRPM(positionFSM.getFlywheelTargetVelocityRPM());
                if(flywheelFSM.AT_TARGET_VELOCITY()) {
                    state = States.PREPARED_TO_SHOOT;
                }
                break;
            case TOGGLING_FLYWHEEL:
                if(flywheelFSM.STOPPED()) {
                    flywheelFSM.setTargetVelocityRPM(positionFSM.getFlywheelTargetVelocityRPM());
                }
                else {
                    flywheelFSM.setTargetVelocityRPM(0);
                }

                if(flywheelFSM.AT_TARGET_VELOCITY()) {
                    state = States.TOGGLED_FLYWHEEL;
                }
            break;
        }
    }

    public void findTargetState(boolean aPress) {
        if(aPress) {
            state = States.TOGGLING_FLYWHEEL;
        }
        else if(!(state == States.TOGGLING_FLYWHEEL)) {
            state = States.PREPARING_TO_SHOOT;
        }
    }

    public void log() {
        telemetry.addData("shooter state", state);
        flywheelFSM.log();
        positionFSM.log();
    }
}
