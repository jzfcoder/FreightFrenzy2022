package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AstroGCP implements Runnable{
    // Odo wheels
    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    public DcMotor encoderAux;

    //Thead run condition
    private boolean isRunning = true;

    final static int ODOMETRY_THREAD_SLEEP_INTERVAL = 50;
    final static int REV_ENCODER_COUNTS_PER_REVOLUTION = 8192;

    final static double xFactor = 1;
    final static double yFactor = 1;
    final static double L = 16; // TODO: Calibrate (dist between encoder 1 & 2 in inches)
    final static double B = 2.5; // TODO: Calibrate (dist between encoder 3 and midpoint of 1 & 2 in inches)
    final static double D = 2.0; // wheel diameter in inches
    final static double ODOMETRY_COUNTS_PER_INCH = (D * Math.PI) / REV_ENCODER_COUNTS_PER_REVOLUTION;

    double robotGlobalXCoordinatePosition = 0;
    double robotGlobalYCoordinatePosition = 0;
    double robotOrientationDegrees = 0;

    int curRightPos = 0;
    int curLeftPos = 0;
    int curAuxPos = 0;
    double curHeading = 0;

    int oldRightPos = 0;
    int oldLeftPos = 0;
    int oldAuxPos = 0;

    public AstroGCP(DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder, double startingX,
                       double startingY, double startingOrientationDegrees){
        this.encoderLeft = verticalEncoderLeft;
        this.encoderRight = verticalEncoderRight;
        this.encoderAux = horizontalEncoder;

        this.robotGlobalXCoordinatePosition = startingX;
        this.robotGlobalYCoordinatePosition = startingY;
        this.robotOrientationDegrees = Math.toRadians(startingOrientationDegrees);
    }

    private void globalCoordinatePositionUpdate()
    {
        oldRightPos = curRightPos;
        oldLeftPos = curLeftPos;
        oldAuxPos = curAuxPos;

        curRightPos = -encoderRight.getCurrentPosition();
        curLeftPos = encoderLeft.getCurrentPosition();
        curAuxPos = encoderAux.getCurrentPosition();

        //curHeading = imu.getAngularOrientation().firstAngle;
        //RobotLog.vv("encoder", "" + curRightPos + ", " + curLeftPos + ", " + curAuxPos);

        int dn1 = curLeftPos - oldLeftPos;
        int dn2 = curRightPos - oldRightPos;
        int dn3 = curAuxPos - oldAuxPos;

        double dtheta = ODOMETRY_COUNTS_PER_INCH * (dn2-dn1) / L;
        double dx = xFactor * ODOMETRY_COUNTS_PER_INCH * ((dn1+dn2) / 2.0);
        double dy = yFactor * ODOMETRY_COUNTS_PER_INCH * (dn3 - (dn2-dn1) * B / L);

        double theta = robotOrientationDegrees + (dtheta / 2.0);
        robotGlobalXCoordinatePosition += dx * Math.cos(theta) - dy * Math.sin(theta);
        robotGlobalYCoordinatePosition += dx * Math.sin(theta) + dy * Math.cos(theta);
        robotOrientationDegrees = theta;
    }

    public void run() {
        while(isRunning) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(ODOMETRY_THREAD_SLEEP_INTERVAL);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void stop() { isRunning = false; }

    public double getLEncoderPosition() { return encoderLeft.getCurrentPosition(); }
    public double getREncoderPosition() { return encoderRight.getCurrentPosition(); }
    public double getHEncoderPosition() { return encoderAux.getCurrentPosition(); }

    public double getXinInches() { return robotGlobalXCoordinatePosition; }
    public double getYinInches() { return robotGlobalYCoordinatePosition; }
    public double getOrientationDegrees() { return robotOrientationDegrees; }
}
