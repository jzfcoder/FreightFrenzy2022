package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutonomousTest", group = "tests")
public class directionTest extends LinearOpMode {
    HardwareController robot = new HardwareController(this, 0, 0 ,0);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.moveStraightOnYforInches(1000, 10, 0.25, 0.5);
        robot.moveStraightOnYforInches(1000, 10, -0.25, 0.5);
        telemetry.addData("> current action", "moving straight on Y");

        robot.turnToAngleIMU(90, 0.25);
        robot.turnToAngleIMU(0, 0.25);
        telemetry.addData("> current action", "turning to angle");

        robot.strafeOnAuxforInches(1000, 10, HardwareController.strafeDirection.RIGHT, 0.25, 0.5);
        robot.strafeOnAuxforInches(1000, 10, HardwareController.strafeDirection.LEFT, 0.25, 0.5);
        telemetry.addData("> current action", "strafing on Auxiliary");

        robot.setAllWheelsPowerforTime(1000, 0.25);
        robot.setAllWheelsPowerforTime(1000, -0.25);
        telemetry.addData("> current action", "set all wheels power for time");
    }
}
