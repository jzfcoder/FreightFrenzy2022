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

@Autonomous(name = "newBlueSUSpawnSUEnd")
public class newBlueSUSpawnSUEnd extends LinearOpMode {

    TeamMarkerDetector.TeamMarkerPosition teamMarkerPosition;
    TeamMarkerDetector teamMarkerDetector;

    @Override
    public void runOpMode()
    {
        HardwareController robot = new HardwareController(this, -33, 63, 180);
        teamMarkerDetector = new TeamMarkerDetector(this);
        teamMarkerDetector.init();

        //telemetry.addData("init", "finished");
        //telemetry.update();

        waitForStart();
        runCV();
        teamMarkerPosition = TeamMarkerDetector.TeamMarkerPosition.MIDDLE;

        if (isStopRequested())
        {
            robot.gcp.stop();
            return;
        }

        Trajectory aHub = robot.drive.trajectoryBuilder(new Pose2d(-33, 63, Math.toRadians(180.0)))
                    .lineToConstantHeading(new Vector2d(-14.0, 62.8))
                    .build();
        robot.drive.followTrajectory(aHub);
        robot.extendLinears(teamMarkerPosition, 0.5);
        robot.openServo();
        robot.closeServo();
        robot.retractLinears(0.5);

        aHub = robot.drive.trajectoryBuilder(robot.localizer.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(-14.0, 58.0))
                    .build();
        robot.drive.followTrajectory(aHub);

        Trajectory toCarousel = robot.drive.trajectoryBuilder(aHub.end())
                .splineToLinearHeading(new Pose2d(-58.0, 56.5, Math.toRadians(-90.0)), Math.toRadians(180.0))
                .build();

        robot.drive.followTrajectory(toCarousel);

        robot.carouselSpin(-0.5);

        Trajectory toSU = robot.drive.trajectoryBuilder(toCarousel.end())
                .lineToConstantHeading(new Vector2d(-60.0, 37.5))
                .build();

        robot.drive.followTrajectory(toSU);
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
