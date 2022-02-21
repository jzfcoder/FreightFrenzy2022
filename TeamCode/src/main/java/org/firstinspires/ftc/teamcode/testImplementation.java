package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

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
        StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(hardwareMap);

        localizer.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

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

        telemetry.addData("init", "finished");
        telemetry.update();

        // wait for start
        waitForStart();

            while (opModeIsActive()) {
                localizer.update();
                Pose2d myPose = localizer.getPoseEstimate();

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

                telemetry.addData("left Encoder", robot.getOdometryLEPosition());
                telemetry.addData("right Encoder", robot.getOdometryREPosition());
                telemetry.addData("auxiliary Encoder", robot.getOdometryHEPosition());

                telemetry.addData("old x", robot.ancientposition[0]);
                telemetry.addData("old y", robot.ancientposition[1]);
                telemetry.addData("old rotation", robot.ancientposition[2]);

                telemetry.addData("gcp x", robot.getOdometryX());
                telemetry.addData("gcp y", robot.getOdometryY());
                telemetry.addData("gcp rotation", robot.getOdometryOrientationDegrees());

                telemetry.addData("rr x", myPose.getX());
                telemetry.addData("rr y", myPose.getY());
                telemetry.addData("rr rotation", Math.toDegrees(myPose.getHeading()));

                telemetry.update();
            }
            robot.gcp.stop();
        }
    }
