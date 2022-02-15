package org.firstinspires.ftc.teamcode.BluePaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareController;
import org.firstinspires.ftc.teamcode.TeamMarkerDetector;

@Autonomous(name = "BlueWSpawnWEnd", group = "Blue")
public class BlueWSpawnWEnd extends LinearOpMode {
    TeamMarkerDetector.TeamMarkerPosition teamMarkerPosition;
    TeamMarkerDetector teamMarkerDetector;

    @Override
    public void runOpMode()
    {
        HardwareController robot = new HardwareController(this, 0, 0, 0);
        teamMarkerDetector = new TeamMarkerDetector(this);
        teamMarkerDetector.init();

        waitForStart();

        // Run CV to get position of team element
        runCV();

        // Push team element out of the way and orient for shipping hub placement
        if (teamMarkerPosition.equals(TeamMarkerDetector.TeamMarkerPosition.LEFT))
        {
            robot.strafeOnXforInches(1000, 2, HardwareController.strafeDirection.LEFT, 0.5, 0.5);
        }
        else
        {
            //robot.strafeOnXforInches(1000, 5, HardwareController.strafeDirection.LEFT, 0.5, 0.5);
        }
        robot.moveStraightOnYforInches(10000, 49, 0.5, 0.5);
        robot.moveStraightOnYforInches(10000, 5, -0.5, 0.5);
        robot.turnToAngle(90, 0.5);

        // Extend linears to position determined by CV, then open servo
        //robot.moveStraightOnXforIN(1000, 1, 0.5, 0.5);
        robot.extendLinears(teamMarkerPosition, 0.25);
        sleep(1000);
        robot.openServo();
        robot.closeServo();
        robot.retractLinears(0.25);

        // Orient for warehouse crossing and cross
        if (teamMarkerPosition.equals(TeamMarkerDetector.TeamMarkerPosition.LEFT))
        {
            robot.strafeOnAuxforInches(100000, 43, HardwareController.strafeDirection.LEFT, 0.5, 0.5);
        }
        else
        {
            robot.strafeOnAuxforInches(100000, 40, HardwareController.strafeDirection.LEFT, 0.5, 0.5);
        }
        robot.setAllWheelsPower(0.15);
        sleep(1500);
        robot.setAllWheelsPower(2);
        sleep(900);
        robot.stopAllWheels();
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
