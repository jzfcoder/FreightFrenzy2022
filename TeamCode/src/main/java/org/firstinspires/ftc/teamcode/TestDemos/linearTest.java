package org.firstinspires.ftc.teamcode.TestDemos;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.HardwareController;
import org.firstinspires.ftc.teamcode.TeamMarkerDetector;

@TeleOp(name="linearsTesting")
public class linearTest extends LinearOpMode
{
    HardwareController robot;

    @Override
    public void runOpMode()
    {
        robot = new HardwareController(this, 0, 0, 0);

        waitForStart();

        while(opModeIsActive())
        {

            RobotLog.vv("linears", robot.Linear.getCurrentPosition() + "");

            if (gamepad2.a)
            {
                robot.extendLinears(TeamMarkerDetector.TeamMarkerPosition.LEFT, 0.5);
            }
            if (gamepad2.b)
            {
                robot.extendLinears(TeamMarkerDetector.TeamMarkerPosition.MIDDLE, 0.5);
            }
            if (gamepad2.x)
            {
                robot.extendLinears(TeamMarkerDetector.TeamMarkerPosition.RIGHT, 0.5);
            }
            if (gamepad2.y)
            {
                robot.retractLinears(0.5);
            }
            telemetry.addData("linears:", robot.Linear.getCurrentPosition());
            telemetry.update();
        }
    }

}
