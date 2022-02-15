package org.firstinspires.ftc.teamcode.BluePaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareController;
import org.firstinspires.ftc.teamcode.TeamMarkerDetector;

@Autonomous(name = "BlueSUSpawnSUEnd", group = "Blue")
public class BlueSUSpawnSUEnd extends LinearOpMode {
    TeamMarkerDetector.TeamMarkerPosition teamMarkerPosition;
    TeamMarkerDetector teamMarkerDetector;

    @Override
    public void runOpMode()
    {
        HardwareController robot = new HardwareController(this, 0, 0, 0);
        teamMarkerDetector = new TeamMarkerDetector(this);
        teamMarkerDetector.init();

        waitForStart();

        //robot.LinearL.setPower(-0.25);
        //robot.LinearR.setPower(0.25);

        // Run CV to get position of team element
        runCV();

        // Push team element out of the way and orient for shipping hub placement
        //robot.strafeOnXforInches(1000, 5, HardwareController.strafeDirection.RIGHT, 0.5, 0.5);
        robot.moveStraightOnYforInches(10000, 50, 0.5, 0.5);
        robot.moveStraightOnYforInches(10000, 6, -0.5, 0.5);
        robot.strafeOnXforInches(10000, 9, HardwareController.strafeDirection.LEFT, 0.5, 0.5);
        robot.turnToAngle(-85, 0.5);

        // Extend linears to position determined by CV, then open servo
        robot.extendLinears(teamMarkerPosition, 0.5);

        sleep(300);
        robot.openServo();
        robot.closeServo();
        robot.retractLinears(0.5);

        // Orient for storage unit parking
        //robot.strafeOnAuxforInches(100000, 25, HardwareController.strafeDirection.RIGHT, 0.5, 0.5);
        robot.turnToAngle(0, 0.5);
        robot.strafeOnXforInches(5000, 46, HardwareController.strafeDirection.RIGHT, 0.5, 0.5);
        robot.moveStraightOnYforInches(10000, 30, -0.5, 0.5);
        robot.carouselSpin(0.65);
        robot.turnToAngle(0, 0.5);
        robot.moveStraightOnYforInches(10000, 14.75, 0.5, 0.5);
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
