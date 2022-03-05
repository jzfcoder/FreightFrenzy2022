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
        HardwareController robot = new HardwareController(this, 9, 63, -90);

        telemetry.addData("init", "finished");
        telemetry.update();

        waitForStart();

        if (isStopRequested())
        {
            return;
        }

        Trajectory toWarehouse = robot.drive.trajectoryBuilder(robot.localizer.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(40.0, 63.0, Math.toRadians(-90.0)), Math.toRadians(0.0))
                .splineToConstantHeading(new Vector2d(40.0, 40.0), Math.toRadians(-90.0))
                .build();

        robot.drive.followTrajectory(toWarehouse);
    }

}
