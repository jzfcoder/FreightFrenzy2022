package org.firstinspires.ftc.teamcode.BluePaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareController;
import org.firstinspires.ftc.teamcode.TeamMarkerDetector;

@Autonomous(name = "BlueWSpawnGettingCarried", group = "Blue")
public class BlueWSpawnGettingCarried extends LinearOpMode {
    TeamMarkerDetector.TeamMarkerPosition teamMarkerPosition;

    @Override
    public void runOpMode()
    {
        HardwareController robot = new HardwareController(this, 0, 0, 0);

        waitForStart();

        // BlueWspawnGettingCarried

        // Orient for Warehouse and enter warehouse
        robot.moveStraightOnYforInches(10000, 30, 0.5, 0.5);
        robot.moveStraightOnYforInches(10000, 7, -0.5, 0.5);
        sleep(1000);
        robot.turnToAngleIMU(90, 0.5);
        sleep(1000);
        robot.setAllWheelsPower(0.15);
        sleep(2000);
        robot.setAllWheelsPower(2);
        sleep(1000);
        robot.stopAllWheels();
    }
}
