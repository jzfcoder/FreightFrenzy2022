package org.firstinspires.ftc.teamcode.NewBluePaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

@Autonomous(name = "NewMovementAutonTest")
public class AutonTest2 extends LinearOpMode {

    @Override
    public void runOpMode()
    {
        HardwareController robot = new HardwareController(this, 0, 0, 0);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        localizer.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        localizer.update();
        Pose2d pose = localizer.getPoseEstimate();

        waitForStart();


        if (isStopRequested()) return;
        localizer.update();
        pose = localizer.getPoseEstimate();
        robot.newTurnToAngle(90, pose.getHeading());
        localizer.update();
        pose = localizer.getPoseEstimate();
        robot.newTurnToAngle(-90, pose.getHeading());
        localizer.update();
        pose = localizer.getPoseEstimate();
        robot.newTurnToAngle(45, pose.getHeading());
        localizer.update();
        pose = localizer.getPoseEstimate();
        robot.newTurnToAngle(-10, pose.getHeading());
    }
}
