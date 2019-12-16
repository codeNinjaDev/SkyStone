package org.firstinspires.ftc.teamcode.libs;

/**
 * Created by peter on 4/11/18.
 */

public class PIDController {
    double P, I, D, maxRange, absoluteTolerance, setpoint, summation, lastError, currentError, feedback;
    double deltaError;
    double output;
    double aff;

    double lastTime;

    public PIDController(double P, double I, double D, double maxRange, double absoluteTolerance) {
        this.P = P;
        this.I = I;
        this.D = D;
        summation = 0;
        feedback = 0;
        output = 0;
        this.maxRange = maxRange;
        this.absoluteTolerance = absoluteTolerance;
        aff = 0;
        lastTime = 0;
    }

    public void setSetpoint(double setpoint, boolean startTimer) {
        this.setpoint = setpoint;

        if(startTimer) {
            setTimestamp();
        }
    }


    public void setTimestamp() {
        lastTime = System.nanoTime();
    }

    public double run(double feedback) {
        this.feedback = feedback;

        // Convert to ms
        double deltaTime = (System.nanoTime() - lastTime) / 1000000.0;

        currentError = setpoint - feedback;
        deltaError = currentError - lastError;
        summation += (currentError * deltaTime);

        output = P * currentError + I * summation + D * (deltaError / deltaTime);
        lastError = currentError;
        output += aff;
        if(output > maxRange)
            output = maxRange;
        else if(output < -maxRange)
            output = -maxRange;
        
        setTimestamp();
        return output;
    }
    public void setArbitraryFeedForward(double ff) {
        aff = ff;
    }
    public boolean onTarget() {
        currentError = setpoint - feedback;
        if(Math.abs(currentError) < absoluteTolerance)
            return true;
        else
            return false;

    }






}
