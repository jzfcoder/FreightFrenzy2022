package org.firstinspires.ftc.teamcode.NewRedPaths;

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

@Autonomous(name = "newRedWGettingCarried")
public class newRedWGettingCarried extends LinearOpMode {

    @Override
    public void runOpMode()
    {
        HardwareController robot = new HardwareController(this, 9, -63, 0);

        telemetry.addData("init", "finished");
        telemetry.update();

        waitForStart();

        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(runtime.milliseconds() <= 750)
        {
            robot.Cap.setPosition(-1);
        }
        robot.Cap.setPosition(0.5);

        if (isStopRequested())
        {
            return;
        }

        Trajectory inWarehouse = robot.drive.trajectoryBuilder(robot.localizer.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(35.0, -63.0, Math.toRadians(0.0)))
                .splineToConstantHeading(new Vector2d(40, -60), 0)
                .build();

        Trajectory toSide = robot.drive.trajectoryBuilder(inWarehouse.end())
                .lineToConstantHeading(new Vector2d(40.0, -40.0))
                .build();

        robot.drive.followTrajectory(inWarehouse);
        robot.drive.followTrajectory(toSide);
    }

}
