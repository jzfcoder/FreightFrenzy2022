package org.firstinspires.ftc.teamcode.RedPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareController;
import org.firstinspires.ftc.teamcode.TeamMarkerDetector;

@Disabled
@Autonomous(name = "RedSUSpawnSUEnd", group = "Red")
public class RedSUSpawnSUEnd extends LinearOpMode {
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
        robot.strafeOnXforInches(1000, 15, HardwareController.strafeDirection.LEFT, 0.5, 0.5);
        robot.moveStraightOnYforInches(10000, 53, 0.5, 0.5);

        robot.moveStraightOnYforInches(10000, 10, -0.5, 0.2);
        robot.strafeOnXforInches(10000, 7, HardwareController.strafeDirection.RIGHT, 0.5, 0.5);
        robot.turnToAngleIMU(90, 0.5);

        // Extend linears to position determined by CV, then open servo
        robot.extendLinears(teamMarkerPosition, 0.5);

        sleep(500);
        robot.openServo();
        robot.closeServo();
        robot.retractLinears(0.5);

        // Orient for storage unit parking
        robot.strafeOnAuxforInches(100000, 35, HardwareController.strafeDirection.LEFT, 0.5, 0.5);
        //robot.strafeOnAuxforInches(10000, 15, HardwareController.strafeDirection.RIGHT, 0.5, 0.5);
        robot.turnToAngleIMU(-55, 0.5);
        robot.setAllWheelsPowerforTime(3000, 0.25);
        robot.carouselSpin(-0.65);
        robot.setAllWheelsPowerforTime(1000, -0.5);
        robot.turnToAngleIMU(0, 0.5);
        sleep(500);
        robot.moveStraightOnYforInches(5000, 13, 0.5, 0.5);
        robot.strafeOnAuxforInches(6000, 53, HardwareController.strafeDirection.LEFT, 0.5, 0.5);
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
