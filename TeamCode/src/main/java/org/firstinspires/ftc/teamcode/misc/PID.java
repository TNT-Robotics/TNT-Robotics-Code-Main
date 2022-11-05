package org.firstinspires.ftc.teamcode.misc;

public class PID {

    double kP = 0;
    double kI = 0;
    double kD = 0;
    double kF = 0;

    double integralCounter = 0;

    long lastUpdateTime = -1L;
    double lastUpdateError = -1D;

    public PID(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }


    private double[] getNumericalValues(double currentState, double targetState) {

        double currentUpdateError = targetState - currentState;

        // Safe Division
        long deltaTime = this.lastUpdateTime == -1 ? 0 : System.currentTimeMillis() - lastUpdateTime;
        double changeInError = this.lastUpdateError == -1 ? 0 : currentUpdateError - this.lastUpdateError;;

        // Calculate PID Terms
        double proportion = currentUpdateError * kP;
        double derivative = changeInError / deltaTime;
        this.integralCounter += currentUpdateError * deltaTime;

        // Store values
        this.lastUpdateTime = System.currentTimeMillis();
        this.lastUpdateError = currentUpdateError;

        return new double [] {
                proportion,
                this.integralCounter * this.kI,
                derivative * this.kD,
                this.kF
        };
    }

    public double getOutputFromError(double targetState, double currentState) {
        double[] numericalValues = getNumericalValues(currentState, targetState);

        double sum = 0;

        for (double numericalValue : numericalValues) {
            sum += numericalValue;
        }
        if (sum > 1.2) {
            sum = 1.2;
        } else if (sum < -1.2) {
            sum = -1.2;
        }
        return sum;
    }
}
