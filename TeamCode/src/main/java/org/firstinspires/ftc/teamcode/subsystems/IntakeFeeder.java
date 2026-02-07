package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeFeeder {

    private DcMotor intake1;
    private DcMotor intake2;
    private DcMotor feeder;

    private double power = 1.0;

    public IntakeFeeder(DcMotor intake1, DcMotor intake2, DcMotor feeder)
    {
        this.intake1 = intake1;
        this.intake2 = intake2;

        this.feeder = feeder;


    }

    public void runIntake1()
    {
        intake1.setPower(power);
    };

    public void runIntake2()
    {
        intake2.setPower(power);
    };


    public void runFeeder()
    {
        feeder.setPower(power);

    };


    public void reverse()
    {
        feeder.setPower(-power * .5);
        intake1.setPower(-power  * .5);
        intake2.setPower(-power  * .5);
    }
    public void stopIntake1()
    {
        intake1.setPower(0);
    };
    public void stopIntake2()
    {
        intake2.setPower(0);
    };
    public void stopFeeder()
    {
        feeder.setPower(0);
    };



}