package org.firstinspires.ftc.teamcode.intaketransfer;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.intaketransfer.BeltFSM;
import org.firstinspires.ftc.teamcode.intaketransfer.TransferServoFSM;

import java.util.concurrent.TimeUnit;

public class TransferFSM {


    public enum State {
        MOVING_DOWN,
        MOVING_UP,
        START_TO_MOVE,
        STOPPING,
        AT_DOWN,
        AT_UP,
        MOVING,
        STOPPED,
    }

    private State currentState = State.START_TO_MOVE;
    private BeltFSM Belt;
    private Telemetry telemetry;
    private TransferServoFSM transferServoFSM;
    Timing.Timer autoMoveTimer;


    public TransferFSM(Intaketransferhwmap hardwareMap, Telemetry telemetry) {
        Belt = new BeltFSM(hardwareMap, telemetry);
        this.telemetry = telemetry;
        transferServoFSM = new TransferServoFSM(hardwareMap, telemetry);
        autoMoveTimer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
    }

    public void updateState(boolean D_Pad_Right_Press, boolean Right_Bumper) {
        Belt.updateState();
        transferServoFSM.updateState();
        findTargetState(D_Pad_Right_Press, Right_Bumper);

        switch (currentState) {

            case START_TO_MOVE:
                Belt.Move();
                if (Belt.MOVING()) {
                    currentState = State.MOVING;
                }
                break;

            case MOVING_DOWN:
                transferServoFSM.MoveDown();
                if (transferServoFSM.AT_DOWN()) {
                    currentState = State.AT_DOWN;
                }
                break;

            case MOVING_UP:
                if (!autoMoveTimer.isTimerOn()) {
                    autoMoveTimer.start();
                }
                transferServoFSM.MoveUp();
                if (transferServoFSM.AT_UP()) {
                    currentState = State.AT_UP;
                }
                break;

            case STOPPING:
                Belt.Stop();
                if (Belt.STOPPED()) {
                    currentState = State.STOPPED;
                }
                break;
        }
        telemetry.addData("Transfer Current State ", currentState);
    }

    public void findTargetState(boolean D_Pad_Right_Press, boolean Right_Bumper) {
        if (D_Pad_Right_Press && Belt.MOVING()) {
            currentState = State.STOPPING;
        }
        if (D_Pad_Right_Press && Belt.STOPPED()) {
            currentState = State.START_TO_MOVE;
        }
        if (Right_Bumper && transferServoFSM.AT_UP()) {
            currentState = State.MOVING_DOWN;

        }
        if (Right_Bumper && transferServoFSM.AT_DOWN()) {
            currentState = State.MOVING_UP;
        }
        if (autoMoveTimer.done() && transferServoFSM.AT_UP()) {
            autoMoveTimer.pause();
            currentState = State.MOVING_DOWN;
        }
    }
}
