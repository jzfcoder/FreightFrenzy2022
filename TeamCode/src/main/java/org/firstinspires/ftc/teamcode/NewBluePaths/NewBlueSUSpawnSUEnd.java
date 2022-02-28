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

@Autonomous(name = "NewBlueSUSpawnSUEnd", group = "Blue")
public class NewBlueSUSpawnSUEnd extends LinearOpMode {
    TeamMarkerDetector.TeamMarkerPosition teamMarkerPosition;
    TeamMarkerDetector teamMarkerDetector;

    @Override
    public void runOpMode()
    {
        HardwareController robot = new HardwareController(this, 0, 0, 0);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        teamMarkerDetector = new TeamMarkerDetector(this);
        teamMarkerDetector.init();

        waitForStart();

        // Run CV to get position of team element
        runCV();

        Trajectory aHub = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-24, 0), Math.toRadians(180))
                .build();

        drive.followTrajectory(aHub);
        robot.extendLinears(teamMarkerPosition, 0.5);
        robot.openServo();
        sleep(1000);
        robot.closeServo();
        robot.retractLinears(0.5);

        Trajectory carousel = drive.trajectoryBuilder(aHub.end())
                .splineTo(new Vector2d(10, -10), Math.toRadians(180))
                .splineTo(new Vector2d(5, -5), Math.toRadians(180))
                .build();
        drive.followTrajectory(carousel);
    }

    void runCV()
    {
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (runtime.milliseconds() < 2000)
        {
            teamMarkerPosition = teamMarkerDetector.getTeamMarkerPosition();
            sleep(50);
        }
        sleep(2000);
        //teamMarkerDetector.clean();
        telemetry.addData("cvPosition", teamMarkerDetector.PosToString(teamMarkerPosition));
        telemetry.update();
    }
}
