package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {

    private DcMotorEx shooter;

    //    private double power = .33;
    private boolean isRunning;

    public Shooter (DcMotorEx shooter1)
    {
        this.shooter = shooter1;

        //shooter.setVelocityPIDFCoefficients(7,1.5,0,0.0);

    }

    public void shoot(double rpm)
    {

        shooter.setVelocity(-rpm);
        isRunning = true;

    };
    public void stopShooting()
    {
        shooter.setPower(0);
        isRunning = false;

    };

    public String getCoeff() {
        PIDFCoefficients coeff = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        return "" + coeff.p + ", " + coeff.i + ", " + coeff.d;
    }

    public double getRPM() {
        return shooter.getVelocity();
    }

    public boolean running(){
        return isRunning;
    };

}