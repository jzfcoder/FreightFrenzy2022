package org.firstinspires.ftc.teamcode.NewBluePaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareController;
import org.firstinspires.ftc.teamcode.TeamMarkerDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "newBlueWGettingCarried")
public class newBlueWGettingCarried extends LinearOpMode {

    @Override
    public void runOpMode()
    {
        HardwareController robot = new HardwareController(this, 9, 63, 180);

        telemetry.addData("init", "finished");
        telemetry.update();

        waitForStart();

        if (isStopRequested())
        {
            return;
        }

        Trajectory inWarehouse = robot.drive.trajectoryBuilder(robot.localizer.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(35.0, 63.0, Math.toRadians(180.0)))
                .splineToConstantHeading(new Vector2d(40, 60), 0)
                .build();

        Trajectory toSide = robot.drive.trajectoryBuilder(inWarehouse.end())
                .lineToLinearHeading(new Pose2d(40.0, 40.0, Math.toRadians(180.0)))
                .build();

        robot.drive.followTrajectory(inWarehouse);
        robot.drive.followTrajectory(toSide);
    }

}
