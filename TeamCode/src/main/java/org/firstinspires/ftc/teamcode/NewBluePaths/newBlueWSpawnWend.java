package org.firstinspires.ftc.teamcode.NewBluePaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareController;
import org.firstinspires.ftc.teamcode.TeamMarkerDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "newBlueWSpawnWend")
public class newBlueWSpawnWend extends LinearOpMode {

    TeamMarkerDetector.TeamMarkerPosition teamMarkerPosition;
    TeamMarkerDetector teamMarkerDetector;

    @Override
    public void runOpMode()
    {
        HardwareController robot = new HardwareController(this, 9, 63, -90);
        teamMarkerDetector = new TeamMarkerDetector(this);
        teamMarkerDetector.init();
        runCV();

        telemetry.addData("init", "finished");
        telemetry.update();

        waitForStart();

        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        if (isStopRequested())
        {
            robot.gcp.stop();
            return;
        }

        // deliver preload box
        Trajectory aHub = robot.drive.trajectoryBuilder(robot.localizer.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-15.0, 63.0))
                .build();
        robot.extendLinears(teamMarkerPosition, 0.5);
        robot.drive.followTrajectory(aHub);

        // drive to warehouse entrance
        Trajectory toWarehouse = robot.drive.trajectoryBuilder(aHub.end())
                .splineToLinearHeading(new Pose2d(40.0, 63.0, Math.toRadians(-90.0)), Math.toRadians(0.0))
                .build();
        robot.drive.followTrajectory(toWarehouse);

        // Cycle
        do {
            // drive until box is intaken
            robot.driveUntilIntake(2500, 0.5, 2.0);

            // drive to entrance of warehouse
            Trajectory outWarehouse = robot.drive.trajectoryBuilder(robot.localizer.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(25.0, 63.0, Math.toRadians(-90.0)), Math.toRadians(0.0))
                    .build();
            robot.drive.followTrajectory(outWarehouse);

            // drive to hub and drop off cargo
            aHub = robot.drive.trajectoryBuilder(robot.localizer.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(-15.0, 63.0))
                    .build();
            robot.extendLinears(TeamMarkerDetector.TeamMarkerPosition.RIGHT, 0.5);
            robot.drive.followTrajectory(aHub);

        } while (runtime.milliseconds() < 25000);
    }

    void runCV()
    {
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (runtime.milliseconds() < 2000)
        {
            if (teamMarkerDetector.getTeamMarkerPosition() != teamMarkerPosition.NOT_DETECTED)
            {
                teamMarkerPosition = teamMarkerDetector.getTeamMarkerPosition();
            }
            sleep(50);
        }
        telemetry.addData("cvPosition", teamMarkerDetector.PosToString(teamMarkerPosition));
        telemetry.update();
        teamMarkerDetector.clean();
    }
}
