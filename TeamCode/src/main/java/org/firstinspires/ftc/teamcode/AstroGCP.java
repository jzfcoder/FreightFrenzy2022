package org.firstinspires.ftc.teamcode;

public class AstroGCP implements Runnable{
    final static double REV_ENCODER_COUNTS_PER_REVOLUTION = 8192;
    final static double L = 10.5; // dist between encoder 1 & 2 in inches
    final static double B = 2.5; // dist between encoder 3 and midpoint of 1 & 2 in inches
    final static double D = 1.378; // wheel diameter in inches
    final static double ODOMETRY_INCHES_PER_COUNT = (D * Math.PI) / REV_ENCODER_COUNTS_PER_REVOLUTION;

    public final static double xFactor = 0.94;
    final static double yFactor = 0.94;
    final static double hFactor = 0.94;

    double[] startPos = new double[3];
    double[] pos = new double[3];

    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentAuxPosition = 0;

    private int oldRightPosition = 0;
    private int oldLeftPosition = 0;
    private int oldAuxPosition = 0;

    private boolean isRunning = false;
    private int sleepTime = 50;

    HardwareController hardwareController;

    public AstroGCP(HardwareController hw, double x, double y, double h)
    {
        hardwareController = hw;
        startPos[0] = x;
        startPos[1] = y;
        startPos[2] = h;
    }

    public void update()
    {
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldAuxPosition = currentAuxPosition;

        currentRightPosition = hardwareController.encoderRight.getCurrentPosition();
        currentLeftPosition = hardwareController.encoderLeft.getCurrentPosition();
        currentAuxPosition = hardwareController.encoderAux.getCurrentPosition();

        int dn1 = currentLeftPosition - oldLeftPosition;
        int dn2 = currentRightPosition - oldRightPosition;
        int dn3 = currentAuxPosition - currentAuxPosition;

        double dtheta = ODOMETRY_INCHES_PER_COUNT * (dn2 - dn1) / L;
        double dx = ODOMETRY_INCHES_PER_COUNT * (dn1 + dn2) / 2.0;
        double dy = ODOMETRY_INCHES_PER_COUNT * (dn3 - (dn2 - dn1) * B / L);

        double theta = pos[2] + (dtheta / 2.0);
        pos[0] += (dx * Math.cos(theta) - dy * Math.sin(theta)) * xFactor;
        pos[1] += (dx * Math.sin(theta) + dy * Math.cos(theta)) * yFactor;
        pos[2] += (dtheta) * hFactor;
    }

    public double getXAstroGCP()
    {
        return pos[0];
    }

    public double getYAstroGCP()
    {
        return pos[1];
    }

    public double getOrientationAstroGCP()
    {
        return pos[2];
    }

    public void stop() { isRunning = false; }

    @Override
    public void run() {
        while(isRunning) {
            update();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
