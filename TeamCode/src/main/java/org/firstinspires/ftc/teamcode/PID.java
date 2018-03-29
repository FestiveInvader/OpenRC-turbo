package org.firstinspires.ftc.teamcode;

/**
 * Created by Valentin on 5/29/2017.
 */

public class   PID {
    private double sum;
    private double KP;
    private double KI;
    private double KD;
    private double time;
    private double previousError = 0;

    public PID (double p, double i, double d, double time) {
        KP = p;
        KI = i;
        KD = d;
        this.time = time;
        sum = 0;
    }

    public double pidLoop (double currentPosition, double target) {
        double error = (target-currentPosition);
        //Calculates the P term
        double returnP = KP * error;
        sum += error*time;
        //Calculates the I term
        double returnI = KI*sum;
        //Calculates the D term
        double returnD = KD * ((error-previousError)/time);
        previousError = error;
        //If the PID is greater than 1, (motor power limit), return 1
        if (returnP + returnI + returnD > 1) {
            return 1;
        }
        //If the PID is less than -1, (motor power limit), return -1
        if (returnP + returnI + returnD < -1) {
            return -1;
        }
    /*Return the sum of P,I,D. Knowing the if the values were too large for motor powers
     it would have returned otherwise, this is OK.*/
    return returnP + returnI + returnD;
    }

}


