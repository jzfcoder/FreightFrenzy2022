package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

public class PIDController {

    // Kp, Ki, and Kd constants controlling the P, I and D factors of PID Controller.
    private double Kp;
    private double Ki;
    private double Kd;

    private double setPoint;
    private double prevError = 0.0;

    private double maxOutputValue;
    private double minOutputValue;

    private double totalError;

    // continious, minInput and maxInput are used for continuous input values such as angles
    // finding delta between -170 170 should give you 20.
    // For angles use minInput as -180 and maxInput as 180
    private boolean continious;
    private double minInput;
    private double maxInput;

    // PIDController can be used to keep the Robot on a straight line using IMU
    // Kp for going straight is 0.05 for 1 degree angle.
    // minOutputValue set to -0.2 and maxOutputValue set to 0.2
    // Kd and Ki are set to 0.0
    // We should use a base power of 0.8 and add a correction of max 0.2 to keep on a straight line.

    // PIDController can be used to move the robot to a setpoint at a distance.
    // Kp for ramping up and slowing down is set to 0.1 for 1 inch. so Kp would be 0.1
    // Kd should be set to 0.01
    // minOutputValue set to -0.8 and maxOutputValue set to +0.8

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        continious = false;
        minInput = 0;
        maxInput = 0;
    }

    public void setInputBounds(boolean continious, double minInput, double maxInput) {
        this.continious = continious;
        this.minInput = minInput;
        this.maxInput = maxInput;
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    public double getSetPoint() {
        return setPoint;
    }

    public void setOutputBounds(double minValue, double maxValue) {
        this.minOutputValue = minValue;
        this.maxOutputValue = maxValue;
    }

    public double calculateCorrection(double input) {
        double error = setPoint - input;

        if (continious) {
            if (Math.abs(error) > (maxInput - minInput) / 2)
            {
                if (error > 0)
                    error = error - maxInput + minInput;
                else
                    error = error + maxInput - minInput;
            }
        }

        double correction = 0.0;

        // Integrate the error only if Integral values does not cross max or min thresholds.


        // First implementation let's do only Proportional based.
        double kPcorrection = error*Kp;
        double kDcorrection = (error - prevError)*Kd;

        double kIcorrection = (totalError + error) * Ki;

        if ((kPcorrection <= minOutputValue) || (kPcorrection >= maxOutputValue)) {
            // if kP correction is already resulting in max correction, do not integrate the error.
            // Integral is only to offset the state where we are in a loop of specific erorr results in a specific
            // correction and that correction results in the same error.
            kIcorrection = 0;
            RobotLog.vv("AstroBot", "As kPcorrection(%.2f) is out of correction range[min:%.2f, max:%.2f] of not calculating integral component",
                    kPcorrection, minOutputValue, maxOutputValue);
        }

        if ((kIcorrection > minOutputValue) && (kIcorrection < maxOutputValue)) {
            totalError += error;
        }

        correction = kPcorrection + kIcorrection + kDcorrection;

        RobotLog.vv("AstroBot", "kPcorrection = %.2f, kIcorrection = %.2f, kDcorrection = %.2f",
                kPcorrection, kIcorrection, kDcorrection);
        RobotLog.vv("AstroBot", "setPoint = %.2f, input = %.2f, error = %.2f, correction = %.2f",
                setPoint, input, error, correction);
        RobotLog.vv("AstroBot", "min %.2f, max %.2f", minOutputValue, maxOutputValue);
        if (correction < minOutputValue) {
            correction = minOutputValue;
        } else if (correction > maxOutputValue) {
            correction = maxOutputValue;
        }

        prevError = error;

        return correction;
    }

}