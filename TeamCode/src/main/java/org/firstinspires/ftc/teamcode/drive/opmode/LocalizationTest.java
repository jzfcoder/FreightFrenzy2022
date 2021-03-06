package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.HardwareController.MOTOR_RF;
import static org.firstinspires.ftc.teamcode.HardwareController.MOTOR_RB;
import static org.firstinspires.ftc.teamcode.HardwareController.MOTOR_LF;
import static org.firstinspires.ftc.teamcode.HardwareController.MOTOR_LB;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Disabled
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    Encoder leftEncoder;
    Encoder rightEncoder;
    Encoder frontEncoder;

    DcMotor powerRF;
    DcMotor powerRB;
    DcMotor powerLF;
    DcMotor powerLB;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "powerLB"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "powerRB"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "powerLF"));

        drive.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        powerRF = hardwareMap.get(DcMotor.class, MOTOR_RF);
        powerRB = hardwareMap.get(DcMotor.class, MOTOR_RB);
        powerLF = hardwareMap.get(DcMotor.class, MOTOR_LF);
        powerLB = hardwareMap.get(DcMotor.class, MOTOR_LB);

        waitForStart();

        while (opModeIsActive()) {

            double vertical = -gamepad1.left_stick_y * 2.0;
            double horizontal = gamepad1.left_stick_x * 2.0;
            double pivot = gamepad1.right_stick_x * 2.0;
            powerRF.setPower(-pivot + (vertical - horizontal));
            powerRB.setPower(-pivot + vertical + horizontal);
            powerLF.setPower(pivot + vertical + horizontal);
            powerLB.setPower(pivot + (vertical - horizontal));

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Left Y pos", leftEncoder.getCurrentPosition() + "");
            telemetry.addData("Right Y pos", rightEncoder.getCurrentPosition() + "");
            telemetry.addData("front x pos", frontEncoder.getCurrentPosition() + "");
            telemetry.update();
        }
    }
}
