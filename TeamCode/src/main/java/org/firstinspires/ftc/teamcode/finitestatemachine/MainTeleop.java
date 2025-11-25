package org.firstinspires.ftc.teamcode.finitestatemachine;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.finitestatemachine.wrappers.HWMap;

@TeleOp
public class MainTeleop extends LinearOpMode {

    HWMap hwMap;
    GamepadEx gamepad;
    ShooterFSM shooterFSM;
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            hwMap = new HWMap(hardwareMap);
            gamepad = new GamepadEx(gamepad1);
            shooterFSM = new ShooterFSM(hwMap,telemetry);
        }catch (Exception e) {
            telemetry.addData("Exception", e.getMessage());
            telemetry.update();
        }
        waitForStart();
        while(opModeIsActive()) {
            gamepad.readButtons();
            shooterFSM.updateState(gamepad.wasJustPressed(GamepadKeys.Button.B));
            log();
        }
    }

    private void log() {
        shooterFSM.log();
        telemetry.update();
    }
}
