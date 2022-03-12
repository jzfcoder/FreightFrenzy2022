package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

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

        float slide;
        float slidePower;

        telemetry.addData("init", "finished");
        telemetry.update();

        // wait for start
        waitForStart();

        if (isStopRequested())
        {
            return;
        }

            while (opModeIsActive()) {
                if (gamepad2.right_trigger > 0)
                {
                    slidePower = gamepad2.right_trigger;
                    slide = slidePower * 2;

                    robot.Linear.setPower(slide);
                }
                else if (gamepad2.left_trigger > 0)
                {
                    slidePower = -gamepad2.left_trigger;
                    slide = slidePower * 2;

                    robot.Linear.setPower(slide);
                }
                else
                {
                    slide = 0;
                    robot.Linear.setPower(slide);
                }
                RobotLog.vv("linears", robot.Linear.getCurrentPosition() + "");

                telemetry.addData("linear: ", robot.Linear.getCurrentPosition());

                telemetry.update();
            }
        }
    }
