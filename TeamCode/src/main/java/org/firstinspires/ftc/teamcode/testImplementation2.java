package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

@TeleOp(name = "AstroBotRed")
public class testImplementation2 extends LinearOpMode {
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
        float cap = 0;

        telemetry.addData("init", "finished");
        telemetry.update();

        // wait for start
        waitForStart();

        if (isStopRequested())
        {
            return;
        }

            while (opModeIsActive()) {
                localizer.update();
                Pose2d myPose = localizer.getPoseEstimate();

                if(gamepad1.right_bumper)
                {
                    sens = 0.5f;
                }
                else
                {
                    sens = 2.0f;
                }

                if (gamepad2.right_trigger > 0)
                {
                    slidePower = gamepad2.right_trigger;
                    slide = slidePower * 2 * sens;

                    robot.Linear.setPower(slide);
                }
                else if (gamepad2.left_trigger > 0)
                {
                    if (robot.Linear.getCurrentPosition() < 0)
                    {
                        slidePower = -gamepad2.left_trigger;
                        slide = slidePower * 2 * sens;

                        robot.Linear.setPower(slide);
                    }
                    else
                    {
                        slide = 0;
                        robot.Linear.setPower(slide);
                    }
                }
                else
                {
                    slide = 0;
                    robot.Linear.setPower(slide);
                }
                RobotLog.vv("linears", robot.Linear.getCurrentPosition() + "");

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
                else if (robot.colorSensor.alpha() > 120)
                {
                    carousel = 2.0f;
                }

                else
                {
                    carousel = 0;
                }
                robot.Carousel.setPower(carousel * 0.75f);

                intake = intakePower;
                robot.IntakeF.setPower(intake);
                robot.IntakeB.setPower(intake);

                if (gamepad2.a) {
                    a = 0.4f;
                }
                else {
                    a = 0.0f;
                }
                if (gamepad2.b)
                {
                    RobotLog.vv("linearHeight", "" + robot.Linear.getCurrentPosition());
                    telemetry.update();
                }
                robot.Drop.setPosition(a);

                robot.Cap.setPosition((gamepad2.left_stick_y / 8) + 0.5);

                telemetry.addData("left Encoder", localizer.getLeftEncoder());
                telemetry.addData("right Encoder", localizer.getRightEncoder());
                telemetry.addData("auxiliary Encoder", localizer.getFrontEncoder()   );

                telemetry.addData("astro x", robot.getAstroXPosition());
                telemetry.addData("astro y", robot.getAstroXPosition());
                telemetry.addData("astro orientation", robot.getAstroOrientation());

                telemetry.addData("rr x", myPose.getX());
                telemetry.addData("rr y", myPose.getY());
                telemetry.addData("rr rotation", Math.toDegrees(myPose.getHeading()));

                telemetry.addData("linear: ", robot.Linear.getCurrentPosition());

                telemetry.update();
            }
        }
    }
