package org.firstinspires.ftc.teamcode.util;

public class SimplePID {
    private double kP, kI, kD;
    private double integralSum = 0;
    private double lastError = 0;

    public SimplePID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double target, double current) {
        double error = target - current;
        integralSum += error;
        double derivative = error - lastError;
        lastError = error;
        return kP*error + kI*integralSum + kD*derivative;
    }
}
