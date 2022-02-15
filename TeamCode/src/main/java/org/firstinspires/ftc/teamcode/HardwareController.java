package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HardwareController
{
    public static final String IMU_CALIBRATION_FILENAME = "IMUCalibration.json";
    public static final String MOTOR_RF = "powerRF";
    public static final String MOTOR_RB = "powerRB";
    public static final String MOTOR_LF = "powerLF";
    public static final String MOTOR_LB = "powerLB";

    public static double topDist = -1900;
    public static double middleDist = -1500;
    public static double bottomDist = -1100;

    // ODOMETRY VARIABLES
    public static final int ODOMETRY_THREAD_SLEEP_INTERVAL = 50;
    public static final int REV_ENCODER_COUNTS_PER_REVOLUTION = 8192;
    final static double xFactor = 0.94;
    final static double yFactor = 0.94;
    final static double R = 3.0; // wheel radius in cm
    final static double ODOMETRY_COUNTS_PER_INCH = (R * Math.PI) / REV_ENCODER_COUNTS_PER_REVOLUTION;


    public double ROBOT_INITIAL_ANGLE;

    public DcMotor powerRF;
    public DcMotor powerRB;
    public DcMotor powerLF;
    public DcMotor powerLB;

    public DcMotor LinearL;
    public DcMotor LinearR;

    public DcMotor Intake;
    public DcMotor Carousel;

    public Servo Drop;

    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    public DcMotor encoderAux;

    public BNO055IMU imu;


    protected LinearOpMode opMode;
    protected OdometryGCP gcp;
    Thread gcpThread;

    public ElapsedTime runtime = new ElapsedTime();

    public HardwareController(LinearOpMode opmode, double x, double y, double theta)
    {
        opMode = opmode;
        HardwareMap hw = opMode.hardwareMap;
        initialize(hw, x, y, theta);
    }

    // -------------------

    private void initialize(HardwareMap hardwareMap, double x, double y, double theta)
    {
        // connect to stuff i guess
        defineObjects(hardwareMap);

        // set motor dir
        setMotorDir();

        // set zero power behaviour
        setZPBehavior();

        // set Modes
        setModes();

        // shadow the motors with the odo encoders
        encoderLeft = powerLB;
        encoderRight = powerRB;
        encoderAux = powerLF;

        // init IMU
        initIMU();

        // init GCP
        //initGCP(x, y, theta);

        resetEncoders();
    }

    private void defineObjects(HardwareMap hardwareMap)
    {
        powerRF = hardwareMap.get(DcMotor.class, MOTOR_RF);
        powerRB = hardwareMap.get(DcMotor.class, MOTOR_RB);
        powerLF = hardwareMap.get(DcMotor.class, MOTOR_LF);
        powerLB = hardwareMap.get(DcMotor.class, MOTOR_LB);

        LinearL = hardwareMap.get(DcMotor.class, "LinearL");
        LinearR = hardwareMap.get(DcMotor.class, "LinearR");

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Carousel = hardwareMap.get(DcMotor.class, "Carousel");
        Drop = hardwareMap.get(Servo.class, "Drop");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
    }

    private void setMotorDir()
    {
        powerLF.setDirection(DcMotor.Direction.REVERSE);
        powerLB.setDirection(DcMotor .Direction.REVERSE);

        LinearL.setDirection(DcMotor.Direction.FORWARD);
        LinearR.setDirection(DcMotor.Direction.REVERSE);

        Intake.setDirection(DcMotor.Direction.REVERSE);
    }

    private void setModes()
    {
        powerRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        powerRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        powerLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        powerLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LinearL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setZPBehavior()
    {
        LinearL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LinearR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        powerRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        powerRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        powerLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        powerLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void resetEncoders()
    {
        powerRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        powerRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        powerRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        powerRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        powerLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        powerLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        powerLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        powerLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LinearL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LinearR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initIMU()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = IMU_CALIBRATION_FILENAME; // see the calibration sample opmode
        imu.initialize(parameters);

        // If your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        // As long as Robot is powered on and imu initialized with Hub stationary, we do not need
        // to remap axis

        while (!imu.isGyroCalibrated()) {
            opMode.sleep(50);
            opMode.idle();
        }

        ROBOT_INITIAL_ANGLE = imu.getAngularOrientation().firstAngle;
    }

    /*
    private void initGCP(double xPos, double yPos, double orientation)
    {
        gcp = new OdometryGCP(encoderLeft, encoderRight, encoderAux, ODOMETRY_COUNTS_PER_INCH, xPos * ODOMETRY_COUNTS_PER_INCH, yPos * ODOMETRY_COUNTS_PER_INCH, orientation, ODOMETRY_THREAD_SLEEP_INTERVAL);

        //gcp.reverseLeftEncoder();
        //gcp.reverseRightEncoder();

        gcpThread = new Thread(gcp);
        gcpThread.start();
    }
    */

    // ODOMETRY

    private int curRightPos = 0;
    private int curLeftPos = 0;
    private int curAuxPos = 0;
    private double curHeading = 0;

    private int oldRightPos = 0;
    private int oldLeftPos = 0;
    private int oldAuxPos = 0;

    public double[] position = new double[3];

    public void odometry()
    {
        oldRightPos = curRightPos;
        oldLeftPos = curLeftPos;
        oldAuxPos = curAuxPos;

        curRightPos = -encoderRight.getCurrentPosition();
        curLeftPos = encoderLeft.getCurrentPosition();
        curAuxPos = encoderAux.getCurrentPosition();
        curHeading = imu.getAngularOrientation().firstAngle;
        //RobotLog.vv("encoder", "" + curRightPos + ", " + curLeftPos + ", " + curAuxPos);

        int dn1 = curLeftPos - oldLeftPos;
        int dn2 = curRightPos - oldRightPos;
        int dn3 = curAuxPos - oldAuxPos;

        double dx = xFactor * ODOMETRY_COUNTS_PER_INCH * dn3;
        double dy = yFactor * ODOMETRY_COUNTS_PER_INCH * ((dn1 + dn2) / 2.0);

        double theta = curHeading;
        position[0] += dx * Math.cos(theta) - dy * Math.sin(theta);
        position[1] += dx * Math.sin(theta) + dy * Math.cos(theta);
        position[2] = theta;
    }

    //
    // ODOMETRY ROUNTINES
    //

    public double getOdometryLEPosition() {
        return encoderLeft.getCurrentPosition();
    }

    public double getOdometryREPosition() {
        return encoderRight.getCurrentPosition();
    }

    public double getOdometryHEPosition() {
        return encoderAux.getCurrentPosition();
    }

    public double getXPosition() { return position[0]; }
    public double getYPosition() { return position[1]; }
    public double getOrientation() { return curHeading; }

    public void turnToAngle(double robotAngle, double power) {
        Orientation angles;

        // Add the initial angle at which the robot is initialized.
        robotAngle -= ROBOT_INITIAL_ANGLE;

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startAngle = angles.firstAngle;
        double deltaAngle = robotAngle - angles.firstAngle;
        double minDelta = 2.0;
        double LeftPower, RightPower;
        double endAngle = 0;

        RobotLog.vv("AstroBot", "turnToAngle: Enter: DestAngle = %.2f, StartAngle = %.2f", robotAngle, startAngle);
        RobotLog.vv("AstroBot", "turnToAngle: DeltaAngle = %.2f", deltaAngle);
        deltaAngle = normalizeAngle(deltaAngle);
        RobotLog.vv("AstroBot", "turnToAngle: Normalized DeltaAngle = %.2f", deltaAngle);

        if (Math.abs(deltaAngle) < minDelta) {
            RobotLog.vv("AstroBot", "turnToAngle: Exit: DestAngle = %.2f, StartAngle = %.2f : deltaAngle(%.2f) is less than minAngle(%.2f)",
                    robotAngle, startAngle, deltaAngle, minDelta);
            return;
        }

        if (deltaAngle > 0 ) {
            RobotLog.vv("AstroBot", "turnToAngle: Turning anti clockwise");
            LeftPower = -Math.abs(power);
            RightPower = Math.abs(power);
        } else {
            RobotLog.vv("AstroBot", "turnToAngle: Turning clockwise");
            LeftPower = Math.abs(power);
            RightPower = -Math.abs(power);
        }

        RobotLog.vv("AstroBot", "turnToAngle: Setting RightPower(%.2f), LeftPower(%.2f)", RightPower, LeftPower);
        setWheelsPower(RightPower, LeftPower);

        while (opMode.opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            endAngle = angles.firstAngle;
            double diff = robotAngle - angles.firstAngle;
            RobotLog.vv("AstroBot", "turnToAngle: DestAngle = %.2f, StartAngle = %.2f, CurrAngle = %.2f, Delta = %.2f",
                    robotAngle, startAngle, endAngle, diff);

            diff = normalizeAngle(diff);
            RobotLog.vv("AstroBot", "turnToAngle: Normalized DeltaAngle = %.2f", diff);
            if (Math.abs(diff) < minDelta) {
                RobotLog.vv("AstroBot", "turnToAngle: DeltaAngle(%.2f) is less than minDelta (%.2f)", diff, minDelta);
                break;
            }

            if (deltaAngle > 0) {
                // Robot is turning anti clockwise direction
                // robotAngle is greater than angles.firstAngle
                if (diff < 0) {
                    RobotLog.vv("AstroBot", "turnToAngle: Clockwise turn - DeltaAngle(%.2f) is less than zero", diff);
                    break;
                }
            } else {
                // Robot is turning clockwise direction
                // robotAngle is less than angles.firstAngle
                if (diff > 0) {
                    RobotLog.vv("AstroBot", "turnToAngle: Anti Clockwise turn - DeltaAngle(%.2f) is greater than zero", diff);
                    break;
                }
            }
        }

        RobotLog.vv("AstroBot", "turnToAngle: Setting power to zero");
        stopAllWheels();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        RobotLog.vv("AstroBot", "turnToAngle: Exit: DestAngle = %.2f, StartAngle = %.2f, EndAngle = %.2f, FinalAngle = %.2f",
                robotAngle, startAngle, endAngle, angles.firstAngle);
    }

    public void moveStraightOnXforInches(double maxRuntime, double dist, double power, double allowedDeviation) {
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        double startPos = getXPosition();
        PIDController pidAngle = new PIDController(0.05, 0.005, 0.05);

        double angle = getOrientation();
        if ((angle > 45) && (angle < 135))
            pidAngle.setSetPoint(90);
        else
            pidAngle.setSetPoint(-90);

        pidAngle.setOutputBounds(-0.2, 0.2);
        pidAngle.setInputBounds(true, -180, 180);

        while (runtime.milliseconds() < maxRuntime && opMode.opModeIsActive()) {
            double leftPower = power;
            double rightPower = power;

            double currAngle = getOrientation();
            double correction = pidAngle.calculateCorrection(currAngle);

            rightPower += correction;
            leftPower -= correction;

            double xCurr = getXPosition();
            double deviation = dist - Math.abs(xCurr - startPos);
            if (rightPower < 0 && leftPower < 0) {
                setWheelsPower(Math.min(rightPower, -0.25), Math.min(leftPower, -0.25));
            }
            setWheelsPower(Math.max(rightPower, 0.25), Math.max(leftPower, 0.25));

            RobotLog.vv("AstroBot", "moveStraightOnXforInches: Runtime = %.2f, rPower = %.2f, lPower = %.2f, power = %.2f, correction = %.2f",
                    runtime.milliseconds(), rightPower, leftPower, power, correction);

            RobotLog.vv("AstroBot", "moveStraightOnXforInches: Runtime = %.2f, startX = %.2f, currX = %.2f, dist = %.2f, deviation(%.2f), allowedDeviation(%.2f)",
                    runtime.milliseconds(), startPos, xCurr, dist, deviation, allowedDeviation);

            if (deviation < allowedDeviation)
            {
                RobotLog.vv("AstroBot", "moveStraightOnXforInches: Runtime = %.2f, startX = %.2f, currX = %.2f, dist = %.2f, deviation(%.2f) < allowedDeviation(%.2f)",
                        runtime.milliseconds(), startPos, xCurr, dist, deviation, allowedDeviation);
                break;
            }
            //opMode.sleep(10);
        }

        stopAllWheels();
    }

    public void moveStraightOnYforInches(double maxRuntime, double dist, double power, double allowedDeviation) {
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        double startPos = getYPosition();
        double yCurr = getYPosition();
        PIDController pidAngle = new PIDController(0.05, 0.005, 0.05);

        double angle = getOrientation();
        if ((angle > -45) && (angle < 45))
            pidAngle.setSetPoint(0.0);
        else
            pidAngle.setSetPoint(180.0);

        pidAngle.setInputBounds(true, -180, 180);
        pidAngle.setOutputBounds(-0.10, 0.10);

        while (runtime.milliseconds() < maxRuntime && opMode.opModeIsActive()) {
            odometry();
            double leftPower = power;
            double rightPower = power;

            double currAngle = getOrientation();
            double correction = pidAngle.calculateCorrection(currAngle);

            rightPower += correction;
            leftPower -= correction;
            RobotLog.vv("AstroBot", "moveStraightOnYforInches: power = %.2f, currAngle = %.2f, correction = %.2f", power, currAngle, correction);

            yCurr = getYPosition();
            double deviation = dist - Math.abs(yCurr - startPos);

            setWheelsPower(rightPower, leftPower);

            RobotLog.vv("AstroBot", "move: Runtime = %.2f, rPower = %.2f, lPower = %.2f",
                    runtime.milliseconds(), rightPower, leftPower);

            RobotLog.vv("AstroBot", "move: Runtime = %.2f, startY = %.2f, currY = %.2f, dist = %.2f, deviation(%.2f), allowedDeviation(%.2f)",
                    runtime.milliseconds(), startPos, yCurr, dist, deviation, allowedDeviation);

            RobotLog.vv("readings", getYPosition() + "");

            if ( deviation < allowedDeviation) {
                RobotLog.vv("AstroBot", "move: Runtime = %.2f, startY = %.2f, currY = %.2f, dist = %.2f, deviation(%.2f) < allowedDeviation(%.2f)",
                        runtime.milliseconds(), startPos, yCurr, dist, deviation, allowedDeviation);
                break;
            }
            opMode.sleep(10);
        }
        stopAllWheels();
    }
    /*
    void moveStraightOnVerticalforInches(double maxRuntime, double dist, double power, double allowedDeviation) {
        double vCurr = (encoderLeft.getCurrentPosition() + encoderRight.getCurrentPosition()) / 2 * yFactor * cm_per_tick;
        double startPos = (encoderLeft.getCurrentPosition() - encoderRight.getCurrentPosition()) / 2 * yFactor * cm_per_tick;
        while (runtime.milliseconds() < maxRuntime && opMode.opModeIsActive()) {
            odometry();
            double leftPower = power;
            double rightPower = power;

            vCurr = (encoderLeft.getCurrentPosition() + encoderRight.getCurrentPosition()) / 2 * yFactor * cm_per_tick;
            double deviation = dist - Math.abs(vCurr - startPos);

            setWheelsPower(rightPower, leftPower);

            if ( deviation < allowedDeviation) {
                break;
            }
            opMode.sleep(10);
        }
        stopAllWheels();
    }*/

    public void moveStraightOnYforCMUsingPID(double maxRuntime, double dist, double maxPower, double allowedDeviation) {
        final double MIN_CORRECTION_ANGLE = -0.15;
        final double MAX_CORRECTION_ANGLE = 0.15;
        final double CORRECTION_DISTANCE = 4;
        dist -= CORRECTION_DISTANCE;

        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        boolean powerSign = (maxPower == Math.abs(maxPower));

        double startPos = getYPosition();
        PIDController pidAngle = new PIDController(0.05, 0.005, 0.05);
        // pidAngle output bounds are set dynamically based on the power ratio.
//        pidAngle.setOutputBounds(-0.10, 0.10);

        double angle = getOrientation();
        if ((angle > -45) && (angle < 45))
            pidAngle.setSetPoint(0.0);
        else
            pidAngle.setSetPoint(180.0);

        pidAngle.setInputBounds(true, -180, 180);

        PIDController pidPower = new PIDController(0.05, 0, 0.08);
        pidPower.setSetPoint(startPos + dist);
        pidPower.setOutputBounds(0.40, Math.abs(maxPower));

        while (runtime.milliseconds() < maxRuntime && opMode.opModeIsActive()) {
            odometry();
            RobotLog.vv("odo", getXPosition() + ", " + getYPosition());

            double leftPower = 0;
            double rightPower = 0;

            double power = Math.abs(pidPower.calculateCorrection(getYPosition()));
//            double powerRatio = power/Math.abs(maxPower);
            double powerRatio = 1.0;

            double currAngle = getOrientation();
            pidAngle.setOutputBounds(MIN_CORRECTION_ANGLE * powerRatio, MAX_CORRECTION_ANGLE * powerRatio);
            double correction = pidAngle.calculateCorrection(currAngle);

            if (!powerSign) {
                power = -power;
            }

            rightPower = power + correction;
            leftPower = power - correction;

            double yCurr = getYPosition();
            double deviation = dist - Math.abs(yCurr - startPos);

            setWheelsPower(rightPower, leftPower);

            if (deviation < allowedDeviation) {
                break;
            }

            opMode.sleep(10);
        }

        stopAllWheels();
    }

    public void strafeOnXforInches(double maxRuntime, double dist, strafeDirection direction, double power, double allowedDeviation) {
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        strafeWheels(direction, power);

        double startPosX = getXPosition();
        double startPosY = getYPosition();

        while (runtime.milliseconds() < maxRuntime) {
            odometry();
            double xCurr = getXPosition();
            double yCurr = getYPosition();
            double deviation = dist - Math.abs(xCurr - startPosX);

            RobotLog.vv("AstroBot", "strafeOnXforInches: Runtime = %.2f, startX = %.2f, startY = %.2f, currX = %.2f, currY = %.2f",
                    runtime.milliseconds(), startPosX, startPosY, xCurr, yCurr);
            RobotLog.vv("AstroBot", "strafeOnXforInches: dist = %.2f, deviation(%.2f), allowedDeviation(%.2f)",
                    dist, deviation, allowedDeviation);

            if ( deviation < allowedDeviation) {
                RobotLog.vv("AstroBot", "strafeOnXforInches: Runtime = %.2f, startX = %.2f, currX = %.2f, dist = %.2f, deviation(%.2f) < allowedDeviation(%.2f)",
                        runtime.milliseconds(), startPosX, xCurr, dist, deviation, allowedDeviation);
                break;
            }

            opMode.sleep(10);
        }

        RobotLog.vv("AstroBot", "strafeOnXforInches: Stopping");
        stopAllWheels();
    }

    public void strafeOnYforInches(double maxRuntime, double dist, strafeDirection direction, double power, double allowedDeviation) {
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        strafeWheels(direction, power);

        double startPosX = getXPosition();
        double startPosY = getYPosition();

        while (runtime.milliseconds() < maxRuntime) {
            odometry();
            double xCurr = getXPosition();
            double yCurr = getYPosition();
            double deviation = dist - Math.abs(startPosY - yCurr);

            RobotLog.vv("AstroBot", "strafeOnYforInches: Runtime = %.2f, startX = %.2f, startY = %.2f, currX = %.2f, currY = %.2f",
                    runtime.milliseconds(), startPosX, startPosY, xCurr, yCurr);
            RobotLog.vv("AstroBot", "strafeOnYforInches: dist = %.2f, deviation(%.2f), allowedDeviation(%.2f)",
                    dist, deviation, allowedDeviation);

            if ( deviation < allowedDeviation) {
                RobotLog.vv("AstroBot", "strafeOnYforInches: Runtime = %.2f, startX = %.2f, currX = %.2f, dist = %.2f, deviation(%.2f) < allowedDeviation(%.2f)",
                        runtime.milliseconds(), startPosY, yCurr, dist, deviation, allowedDeviation);
                break;
            }

            opMode.sleep(10);
        }

        RobotLog.vv("AstroBot", "strafeOnYforInches: Stopping");
        stopAllWheels();
    }

    public void strafeOnAuxforInches(double maxRuntime, double dist, strafeDirection direction, double power, double allowedDeviation) {
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        strafeWheels(direction, power);

        double startPosX = encoderAux.getCurrentPosition() * xFactor * ODOMETRY_COUNTS_PER_INCH;

        while (runtime.milliseconds() < maxRuntime) {
            odometry();
            double xCurr = encoderAux.getCurrentPosition() * xFactor * ODOMETRY_COUNTS_PER_INCH;
            double deviation = dist - Math.abs(startPosX - xCurr);

            if ( deviation < allowedDeviation) {
                break;
            }

            opMode.sleep(10);
        }

        RobotLog.vv("AstroBot", "strafeOnYforInches: Stopping");
        stopAllWheels();
    }

    // Simple Motor Controls
    public void stopAllWheels()
    {
        setAllWheelsPower(0.0);
    }

    public void setWheelsPower(double RP, double LP)
    {
        powerRF.setPower(RP);
        powerRB.setPower(RP);
        powerLF.setPower(LP);
        powerLB.setPower(LP);
    }

    public void setAllWheelsPower(double p)
    {
        powerRF.setPower(p);
        powerLF.setPower(p);
        powerRB.setPower(p);
        powerLB.setPower(p);
    }

    public void setAllWheelsPowerforTime(double t, double p) {
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (runtime.milliseconds() < t)
        {
            powerRF.setPower(p);
            powerLF.setPower(p);
            powerRB.setPower(p);
            powerLB.setPower(p);
        }
        stopAllWheels();
    }

    public void strafeWheels(strafeDirection direction, double power)
    {
        if (direction == strafeDirection.LEFT)
        {
            power = Math.abs(power);
        }
        else
        {
            power = -Math.abs(power);
        }

        powerRF.setPower(power);
        powerRB.setPower(-power);
        powerLF.setPower(-power);
        powerLB.setPower(power);
    }

    private double normalizeAngle(double inputAngle)
    {
        double outputAngle = inputAngle;

        if (inputAngle <= -180)
        {
            outputAngle = inputAngle + 360;
        }
        else if (inputAngle > 180)
        {
            outputAngle = inputAngle - 360;
        }

        return outputAngle;
    }


    public void extendLinears(TeamMarkerDetector.TeamMarkerPosition pos, double power)
    {
        if(pos.equals(TeamMarkerDetector.TeamMarkerPosition.LEFT))
        {
            while(opMode.opModeIsActive() && LinearL.getCurrentPosition() > bottomDist)
            {
                LinearL.setPower(power);
                LinearR.setPower(power);
            }
            LinearL.setPower(0);
            LinearR.setPower(0);
            RobotLog.vv("linearStatus", "reached Bottom");
        }
        else if (pos.equals(TeamMarkerDetector.TeamMarkerPosition.MIDDLE))
        {
            while(opMode.opModeIsActive() && LinearL.getCurrentPosition() > middleDist)
            {
                LinearL.setPower(power);
                LinearR.setPower(power);
            }
            LinearL.setPower(0);
            LinearR.setPower(0);
            RobotLog.vv("linearStatus", "reached Middle");
        }
        else
        {
            while(opMode.opModeIsActive() && LinearL.getCurrentPosition() > topDist)
            {
                LinearL.setPower(power);
                LinearR.setPower(power);
            }
            LinearL.setPower(0);
            LinearR.setPower(0);
            RobotLog.vv("linearStatus", "reached Top");
        }
    }

    public void retractLinears(double power)
    {
        while(opMode.opModeIsActive() && LinearL.getCurrentPosition() < 0)
        {
            LinearL.setPower(-power);
            LinearR.setPower(-power);
            RobotLog.vv("LinearPos", LinearL.getCurrentPosition() + ", " + LinearR.getCurrentPosition());
        }
        LinearL.setPower(0);
        LinearR.setPower(0);
    }

    public void openServo()
    {
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (runtime.milliseconds() < 1000)
        {
            Drop.setPosition(1);
        }
    }

    public void carouselSpin (double power) {
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (runtime.milliseconds() < 3000) {
            Carousel.setPower(0.39 * power);
        }
        Carousel.setPower(0);
    }

    public void closeServo()
    {
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (runtime.milliseconds() < 1000)
        {
            Drop.setPosition(0);
        }
    }

    public enum strafeDirection
    {
        LEFT,
        RIGHT
    }
}
