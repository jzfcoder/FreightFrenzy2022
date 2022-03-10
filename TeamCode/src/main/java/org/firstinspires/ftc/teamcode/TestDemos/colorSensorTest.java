package org.firstinspires.ftc.teamcode.TestDemos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareController;

@TeleOp(name = "colorSensorTest")
public class colorSensorTest extends LinearOpMode {
    HardwareController robot;

    @Override
    public void runOpMode()
    {
        robot = new HardwareController(this, 0, 0, 0);

        waitForStart();


        while(opModeIsActive())
        {
            if (robot.colorSensor.alpha() > 70)
            {
                telemetry.addData("intaken?", "true");
            }
            else
            {
                telemetry.addData("intaken?", "false");
            }

            telemetry.addData("red", robot.colorSensor.red());
            telemetry.addData("green", robot.colorSensor.green());
            telemetry.addData("blue", robot.colorSensor.blue());
            telemetry.addData("alpha", robot.colorSensor.alpha());

            telemetry.update();
        }
    }
}
