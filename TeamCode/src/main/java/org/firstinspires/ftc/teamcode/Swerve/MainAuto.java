package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class MainAuto extends LinearOpMode {

    private static String red = "RED";
    private static String blue = "BLUE";

    public static String ALLIANCE = red;

    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeInInit()) {
        if (gamepad1.a && ALLIANCE.equals(red)) {
            ALLIANCE = blue;
        } else if (gamepad1.a && ALLIANCE.equals(blue)) {
            ALLIANCE = red;
        }
        telemetry.addData("ALLIANCE", ALLIANCE);
        telemetry.update();
        }
        waitForStart();
    }

}

