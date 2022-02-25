package org.firstinspires.ftc.teamcode.NewBluePaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareController;

@Autonomous(name = "NewMovementAutonTest")
public class AutonTest2 extends LinearOpMode {
    HardwareController robot;

    @Override
    public void runOpMode()
    {
        robot = new HardwareController(this, 0, 0, 0);

        waitForStart();

        if (isStopRequested()) return;

        robot.moveStraightOnYforInches(1000, 15, 0.5, 0.5);
        robot.strafeOnXforInches(1000, 10, HardwareController.strafeDirection.RIGHT, 0.5, 0.5);
        robot.strafeOnAuxforInches(1000, 10, HardwareController.strafeDirection.LEFT, 0.5, 0.5);
        robot.turnToAngleAstro(90, 0.5);
        robot.turnToAngleAstro(0, 0.5);
        robot.moveStraightOnYforInches(1000, 15, -0.5, 0.5);
    }
}
