package org.firstinspires.ftc.teamcode.NewBluePaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareController;
import org.firstinspires.ftc.teamcode.TeamMarkerDetector;

@Autonomous(name = "BlueWSpawnWEndNoCycle")
public class BlueWSpawnWEndNoCycle extends LinearOpMode {

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

        runCV();

        if (isStopRequested())
        {
            robot.gcp.stop();
            return;
        }

        // deliver preload box
        Trajectory aHub = robot.drive.trajectoryBuilder(robot.localizer.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(-12.0, 62.8))
                    .build();
        robot.drive.followTrajectory(aHub);
        robot.extendLinears(teamMarkerPosition, 0.5);
        robot.openServo();
        robot.closeServo();
        robot.retractLinears(0.5);

        // drive into warehouse
        Trajectory toWarehouse = robot.drive.trajectoryBuilder(aHub.end())
                .lineToConstantHeading(new Vector2d(40.0, 63.0))
                .build();
        robot.drive.followTrajectory(toWarehouse);
        Trajectory finish = robot.drive.trajectoryBuilder(toWarehouse.end())
                .lineToConstantHeading(new Vector2d(40.0, 40.0))
                .build();
        robot.drive.followTrajectory(finish);
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
