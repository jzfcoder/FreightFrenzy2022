package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "AstroBot1")
public class testImplementation extends LinearOpMode {
    // 192.168.43.1

    HardwareController robot;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        robot = new HardwareController(this, 0, 0, 0);

        /// ------------------------------------------------+
        /// Variable Declarations                           |
        /// ------------------------------------------------+

        // declare volume variables
        float vertical;
        float horizontal;
        float pivot;
        float slide;
        float intake;
        float carousel;
        float a;
        float slidePower;
        float intakePower;
        float sens;

        // wait for start
        waitForStart();
            while (opModeIsActive()) {

                if (gamepad2.right_trigger > 0)
                {
                    slidePower = gamepad2.right_trigger;
                    slide = slidePower * 2;

                    robot.LinearL.setPower(slide);
                    robot.LinearR.setPower(slide);
                }
                else if (gamepad2.left_trigger > 0)
                {
                    if (robot.LinearL.getCurrentPosition() < 0)
                    {
                        slidePower = -gamepad2.left_trigger;
                        slide = slidePower * 2;

                        robot.LinearL.setPower(slide);
                        robot.LinearR.setPower(slide);
                    }
                    else
                    {
                        slide = 0;
                        robot.LinearL.setPower(slide);
                        robot.LinearR.setPower(slide);
                    }
                }
                else
                {
                    slide = 0;
                    robot.LinearL.setPower(slide);
                    robot.LinearR.setPower(slide);
                }
                RobotLog.vv("linears", robot.LinearL.getCurrentPosition() + ", " + robot.LinearR.getCurrentPosition());

                if (gamepad2.right_bumper)
                {
                    intakePower = 2;
                }
                else if (gamepad2.left_bumper)
                {
                    intakePower = -2;
                }
                else
                {
                    intakePower = 0;
                }

                if(gamepad1.right_bumper)
                {
                    sens = 0.5f;
                }
                else
                {
                    sens = 2.0f;
                }

                vertical = -gamepad1.left_stick_y * sens;
                horizontal = gamepad1.left_stick_x * sens;
                pivot = gamepad1.right_stick_x * sens;
                robot.powerRF.setPower(-pivot + (vertical - horizontal));
                robot.powerRB.setPower(-pivot + vertical + horizontal);
                robot.powerLF.setPower(pivot + vertical + horizontal);
                robot.powerLB.setPower(pivot + (vertical - horizontal));

                if (gamepad1.right_trigger > 0)
                {
                    carousel = gamepad1.right_trigger * 0.39f;
                }
                else if (gamepad1.left_trigger > 0)
                {
                    carousel = -gamepad1.left_trigger * 0.39f;
                }
                else
                {
                    carousel = 0;
                }

                robot.Carousel.setPower(carousel * 0.75f);

                intake = intakePower;
                robot.Intake.setPower(intake);

                if (gamepad2.a) {
                    a = 1;
                }
                else {
                    a = 0;
                }
                if (gamepad2.b)
                {
                    RobotLog.vv("linearHeight", "" + robot.LinearL.getCurrentPosition());
                }
                robot.Drop.setPosition(a);

                robot.odometry();
                RobotLog.vv("position", robot.position + "");

                telemetry.addData("x",robot.position[0]);
                telemetry.addData("y",robot.position[1]);
                telemetry.addData("rotation",robot.position[2]);
                telemetry.update();
            }
        }
    }
