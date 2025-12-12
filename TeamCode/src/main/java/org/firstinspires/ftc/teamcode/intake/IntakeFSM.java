package org.firstinspires.ftc.teamcode.intake;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.Intaketransferhwmap;


public class IntakeFSM {


    enum State {
        RAMPING_UP_TO_INTAKE,
        RAMPING_UP_TO_EJECT,
        STOPPING,
        READY_TO_INTAKE,
        EJECTING,
        STOPPED
    }

    private RollerFSM Roller;
    private State currentState = State.RAMPING_UP_TO_INTAKE;
    private Telemetry telemetry;

    public IntakeFSM(Intaketransferhwmap hardwareMap, Telemetry telemetry) {
        Roller = new RollerFSM(hardwareMap, telemetry);
        this.telemetry = telemetry;
    }

    public void updateState(boolean YPress, boolean D_Pad_Left_Press) {
        Roller.updateState();
        findTargetState(YPress, D_Pad_Left_Press);
        switch (currentState) {
            case RAMPING_UP_TO_EJECT:
                Roller.eject();
                if (Roller.EJECTING()) {
                    currentState = State.EJECTING;
                }
                break;

            case RAMPING_UP_TO_INTAKE:
                Roller.intake();
                if (Roller.INTAKING()) {
                    currentState = State.READY_TO_INTAKE;
                }
                break;

            case STOPPING:
                Roller.stop();
                if (Roller.STOPPED()) {
                    currentState = State.STOPPED;
                }
                break;
        }
        telemetry.addData("Current State ", currentState);

    }

    public void findTargetState(boolean YPress, boolean D_Pad_Left_Press) {

        if (YPress && (currentState == State.READY_TO_INTAKE || currentState == State.STOPPED)) {
            currentState = State.RAMPING_UP_TO_EJECT;
        } else if (YPress && currentState == State.EJECTING) {
            currentState = State.RAMPING_UP_TO_INTAKE;
        }

        if (D_Pad_Left_Press && (currentState == State.READY_TO_INTAKE || currentState == State.EJECTING)) {
            currentState = State.STOPPING;
        } else if (D_Pad_Left_Press && currentState == State.STOPPED) {
            currentState = State.RAMPING_UP_TO_INTAKE;
        }
    }
}