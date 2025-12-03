package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {

    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
//    private double power = .33;
    private boolean isRunning;

    public Shooter (DcMotorEx shooterLeft, DcMotorEx shooterRight)
    {
        this.shooterLeft = shooterLeft;
        this.shooterRight = shooterRight;

        shooterLeft.setVelocityPIDFCoefficients(7,1.5,0,0.0);
        shooterLeft.setVelocityPIDFCoefficients(7,1.5,0,0.0);


    }

    public void shoot(double rpm)
    {
//        shooterLeft.setPower(power);
//        shooterRight.setPower(-power);
        shooterLeft.setVelocity(rpm);
        shooterRight.setVelocity(-rpm);
        isRunning = true;

    };
    public void stopShooting()
    {
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        isRunning = false;

    };

    public String getCoeff() {
        PIDFCoefficients coeff = shooterLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        return "" + coeff.p + ", " + coeff.i + ", " + coeff.d;
    }

    public double getRPM() {
        return shooterLeft.getVelocity();
    }

    public boolean running(){
        return isRunning;
    };

}