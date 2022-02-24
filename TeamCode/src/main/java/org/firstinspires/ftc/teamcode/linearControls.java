package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "linearControls")
public class linearControls extends LinearOpMode {
    // 192.168.43.1

    HardwareController robot;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        robot = new HardwareController(this, 0, 0, 0);

        // wait for start
        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {
                if (gamepad1.right_stick_y > 0)
                {
                    robot.Linear.setPower(gamepad1.right_stick_y);
                }
                else if (gamepad1.left_stick_y > 0)
                {
                    robot.Linear.setPower(-gamepad1.left_stick_y);
                }
                robot.Linear.setPower(0);
            }
        }
    }
}
