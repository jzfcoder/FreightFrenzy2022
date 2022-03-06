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
        //TODO: fix cv
        //runCV();

        if (isStopRequested())
        {
            robot.gcp.stop();
            return;
        }

        Trajectory aHub = robot.drive.trajectoryBuilder(new Pose2d(-33, 63, Math.toRadians(180.0)))
                .lineToConstantHeading(new Vector2d(-15.0, 60.0))
                .build();
        robot.drive.followTrajectory(aHub);
        robot.extendLinears(TeamMarkerDetector.TeamMarkerPosition.MIDDLE, 0.5);
        robot.openServo();
        robot.closeServo();
        robot.retractLinears(0.5);

        Trajectory toCarousel = robot.drive.trajectoryBuilder(aHub.end())
                .splineToLinearHeading(new Pose2d(-58.0, 55.0, Math.toRadians(-90.0)), Math.toRadians(180.0))
                .build();

        robot.drive.followTrajectory(toCarousel);

        robot.carouselSpin(-0.5);

        Trajectory toSU = robot.drive.trajectoryBuilder(toCarousel.end())
                .lineToConstantHeading(new Vector2d(-60.0, 35.5))
                .build();

        robot.drive.followTrajectory(toSU);
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
        //teamMarkerDetector.clean();
    }
}
