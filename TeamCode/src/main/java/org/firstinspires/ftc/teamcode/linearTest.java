package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name="linearsTesting")
public class linearTest extends LinearOpMode
{
    DcMotor LinearL = null;
    DcMotor LinearR = null;

    @Override
    public void runOpMode()
    {
        LinearL = hardwareMap.get(DcMotor.class, "LinearL");
        LinearR = hardwareMap.get(DcMotor.class, "LinearR");

        waitForStart();

        while(opModeIsActive())
        {
            LinearL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LinearR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            RobotLog.vv("linears", LinearL.getCurrentPosition() + "");

            if (gamepad2.left_trigger > 1)
            {
                moveLinear(gamepad2.left_trigger, 10);
            }
        }
    }

    public void moveLinear(double power, int distance)
    {
        LinearL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LinearL.setTargetPosition(distance);
        LinearR.setTargetPosition(distance);

        LinearL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LinearR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        powerLinear(power);

        while (LinearL.isBusy() && LinearR.isBusy()) {}

        stopLinear();
        LinearL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void powerLinear(double power)
    {
        LinearL.setPower(power);
        LinearR.setPower(power);
    }

    public void stopLinear()
    {
        LinearL.setPower(0);
        LinearR.setPower(0);
    }
}
