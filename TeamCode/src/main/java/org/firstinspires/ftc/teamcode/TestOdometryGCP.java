package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by Sarthak on 6/1/2019.
 * Example OpMode that runs the GlobalCoordinatePosition thread and accesses the (x, y, theta) coordinate values
 */
@TeleOp(name = "GCP Test", group = "Test")
public class TestOdometryGCP extends LinearOpMode {

    HardwareController robot;

    @Override
    public void runOpMode() throws InterruptedException {

        float vertical;
        float horizontal;
        float pivot;
        float sens = 2.0f;
        double factor = 4000;
        boolean touched = false;

        robot = new HardwareController(this, 0, 0, 0);

        //Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        waitForStart();

        /**
         * *****************
         * OpMode Begins Here
         * *****************
         */


        while(opModeIsActive())
        {
            vertical = -gamepad1.left_stick_y * sens;
            horizontal =  gamepad1.left_stick_x * sens;
            pivot = -gamepad1.right_stick_x * sens;

            robot.powerRF.setPower(-pivot + (vertical - horizontal));
            robot.powerRB.setPower(-pivot + vertical + horizontal);
            robot.powerLF.setPower(pivot + vertical + horizontal);
            robot.powerLB.setPower(pivot + (vertical - horizontal));

            robot.odometry();
            updateTelemetry(factor);
        }

        //Stop the thread
        robot.gcp.stop();
    }

    void updateTelemetry(double factor) {
        //Display Global (x, y, theta) coordinates
        telemetry.addData("L Encoder", robot.getOdometryLEPosition());
        telemetry.addData("R Encoder", robot.getOdometryREPosition());
        telemetry.addData("H Encoder", robot.getOdometryHEPosition());

        telemetry.addData("imu angle", (robot.imu.getAngularOrientation().firstAngle * 180 ) / Math.PI);

        telemetry.addData("> x", robot.ancientposition[0]);
        telemetry.addData("> y", robot.ancientposition[1]);
        telemetry.addData("> h", (robot.ancientposition[2] * 180) / Math.PI);

        telemetry.update();
    }
}