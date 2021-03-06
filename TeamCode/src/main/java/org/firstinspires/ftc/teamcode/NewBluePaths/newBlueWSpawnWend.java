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
        HardwareController robot = new HardwareController(this, 9, 63, 180);
        teamMarkerDetector = new TeamMarkerDetector(this);
        teamMarkerDetector.init();

        telemetry.addData("init", "finished");
        telemetry.update();

        waitForStart();

        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(runtime.milliseconds() <= 750)
        {
            robot.Cap.setPosition(-1);
        }
        robot.Cap.setPosition(0.5);

        runCV();

        if (isStopRequested())
        {
            robot.gcp.stop();
            robot.rr.stop();
            return;
        }

        // deliver preload box
        Trajectory aHub = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(-12.0, 62.8))
                    .build();
        robot.drive.followTrajectory(aHub);
        robot.extendLinears(teamMarkerPosition, 0.5);
        robot.openServo();
        robot.closeServo();
        robot.retractLinears(0.5);

        // drive into warehouse
        Trajectory toWarehouse = robot.drive.trajectoryBuilder(aHub.end())
                .lineToLinearHeading(new Pose2d(30.0, 63.0, Math.toRadians(180.0)))
                .build();
        robot.drive.followTrajectory(toWarehouse);

        robot.driveUntilIntake(2500, -0.25, 2.0);

        Trajectory outWarehouse = robot.drive.trajectoryBuilder(robot.localizer.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(25.0, 63.0, Math.toRadians(180.0)))
                .build();

        robot.intakeOut(2.0);
        robot.drive.followTrajectory(outWarehouse);
        robot.stopIntake();

        aHub = robot.drive.trajectoryBuilder(outWarehouse.end())
                .lineToLinearHeading(new Pose2d(-12.0, 62.8, Math.toRadians(180.0)))
                .build();

        robot.drive.followTrajectory(aHub);
        robot.extendLinears(TeamMarkerDetector.TeamMarkerPosition.RIGHT, 0.5);
        robot.openServo();
        robot.closeServo();
        robot.retractLinears(0.5);

        /*
        // Cycle
        do {
            // drive until box is intaken
            robot.driveUntilIntake(2500, -0.5, 2.0);

            if (runtime.milliseconds() < 25000)
            {
                // drive to entrance of warehouse
                Trajectory outWarehouse = robot.drive.trajectoryBuilder(robot.localizer.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(25.0, 63.0, Math.toRadians(180.0)), Math.toRadians(0.0))
                        .build();
                robot.drive.followTrajectory(outWarehouse);

                // drive to hub and drop off cargo
                aHub = robot.drive.trajectoryBuilder(outWarehouse.end())
                        .lineToConstantHeading(new Vector2d(-12.0, 62.8))
                        .build();
                robot.drive.followTrajectory(aHub);
                robot.extendLinears(teamMarkerPosition, 0.5);
                robot.openServo();
                robot.closeServo();
                robot.retractLinears(0.5);

                toWarehouse = robot.drive.trajectoryBuilder(aHub.end())
                        .splineToLinearHeading(new Pose2d(30.0, 63.0, Math.toRadians(180.0)), Math.toRadians(0.0))
                        .build();
                robot.drive.followTrajectory(toWarehouse);
            }
            else
            {
                break;
            }
        } while (runtime.milliseconds() < 28000); */

        robot.rr.stop();
        robot.gcp.stop();
    }

    void runCV()
    {
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (runtime.milliseconds() < 2000)
        {
            TeamMarkerDetector.TeamMarkerPosition temp;
            temp = teamMarkerDetector.getTeamMarkerPosition();
            if (temp != TeamMarkerDetector.TeamMarkerPosition.NOT_DETECTED)
            {
                teamMarkerPosition = temp;
            }
            sleep(50);
        }
        if (teamMarkerPosition == null) { teamMarkerPosition = TeamMarkerDetector.TeamMarkerPosition.NOT_DETECTED; }
        sleep(2000);
        telemetry.addData("cvPosition", teamMarkerDetector.PosToString(teamMarkerPosition));
        telemetry.update();
        //teamMarkerDetector.clean();
    }
}
