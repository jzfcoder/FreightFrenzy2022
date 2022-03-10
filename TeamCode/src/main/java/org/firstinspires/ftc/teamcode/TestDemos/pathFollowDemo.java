package org.firstinspires.ftc.teamcode.TestDemos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareController;

@Autonomous(name = "pathFollowDemo")
public class pathFollowDemo extends LinearOpMode {

    @Override
    public void runOpMode()
    {
        HardwareController robot = new HardwareController(this, -33, 63, 180);

        waitForStart();

        robot.Cap.setPosition(0.5);

        robot.carouselSpin(-0.5);
    }
}
