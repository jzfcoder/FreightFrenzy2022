package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.util.RobotLog;

public class rrLocalizerThread implements Runnable{
    boolean isRunning = true;

    HardwareController robot;
    public rrLocalizerThread(HardwareController controller)
    {
        robot = controller;
    }

    public void stop()
    {
        isRunning = false;
    }

    @Override
    public void run() {
        RobotLog.vv("rrLocalizerThread", "starting");
        while(isRunning) {
            RobotLog.vv("rrLocalizerPos", robot.localizer.getPoseEstimate().toString());
            robot.localizer.update();
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
