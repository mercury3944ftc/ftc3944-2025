package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeFeeder {

    private DcMotor intake;
    private DcMotor feeder;
    private double power = 1.0;

    public IntakeFeeder(DcMotor intake1, DcMotor intake2)
    {
        this.intake = intake1;
        this.feeder = intake2;

    }

    public void runIntake()
    {
        intake.setPower(-power);
    };

    public void runFeeder()
    {
        feeder.setPower(power);

    };

    public void reverse()
    {
        feeder.setPower(-power * .5);
        intake.setPower(power  * .5);
    }
    public void stopIntake()
    {
        intake.setPower(0);
    };

    public void stopFeeder()
    {
        feeder.setPower(0);

    };


}