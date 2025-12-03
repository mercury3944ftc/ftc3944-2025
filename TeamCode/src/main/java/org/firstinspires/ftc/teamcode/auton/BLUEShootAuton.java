package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.IntakeFeeder;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Timer;
import java.util.TimerTask;

//@Disabled
@Autonomous(name="BLUEShootAuton")
public class BLUEShootAuton extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the DC motors for each wheel
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor feeder = hardwareMap.get(DcMotor.class, "feeder");
        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");


        // Reverse right side and arm motors
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoder values
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        feeder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        // Tell motors to run from power level
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tell wheel and climber motors to resist external force
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Shooter shooter = new Shooter(shooter1, shooter2);
        IntakeFeeder intakeFeeder = new IntakeFeeder(intake, feeder);
        DriveTrain driveTrain = new DriveTrain(leftFront, rightFront, leftRear, rightRear);

        waitForStart();
        Timer timer = new Timer();
        TimerTask moveBack = new TimerTask() {
            @Override
            public void run() {
                driveTrain.drive(0.0, 0.3, 0.0);
            }
        };

        TimerTask runShooters1 = new TimerTask() {
            @Override
            public void run() {
                driveTrain.drive(0,0,0);
                shooter.shoot(780);
            }
        };

        TimerTask runShooters2 = new TimerTask() {
            @Override
            public void run() {
                driveTrain.drive(0,0,0);
                shooter.shoot(760);
            }
        };
        TimerTask shoot1 = new TimerTask() {
            @Override
            public void run() {
                intakeFeeder.runFeeder();
            }
        };
        TimerTask shoot2 = new TimerTask() {
            @Override
            public void run() {
                intakeFeeder.runFeeder();
                intakeFeeder.runIntake();
            }
        };
        TimerTask moveRight = new TimerTask() {
            @Override
            public void run() {
                shooter.stopShooting();
                intakeFeeder.stopFeeder();
                intakeFeeder.stopIntake();
                driveTrain.drive(-0.3, 0.0, 0.0);
            }
        };
        TimerTask stopDrivingTask = new TimerTask() {
            @Override
            public void run() {
                driveTrain.drive(0.0, 0.0, 0.0);

            }
        };

        timer.schedule(moveBack, (long) 0);
        timer.schedule(runShooters1, (long) 2500);
        timer.schedule(shoot1, (long) 4500);
        timer.schedule(runShooters2, (long) 4750);
        timer.schedule(shoot2, (long) 6000);
        timer.schedule(moveRight, (long) 12000);
        timer.schedule(stopDrivingTask, (long) 14000);
        sleep(16000);

        telemetry.addData("leftFront", leftFront.getCurrentPosition());
        telemetry.addData("leftRear", leftRear.getCurrentPosition());
        telemetry.addData("rightFront", rightFront.getCurrentPosition());
        telemetry.addData("rightRear", rightRear.getCurrentPosition());
        telemetry.addData("Intake", intake.getCurrentPosition());
        telemetry.addData("Feeder", feeder.getCurrentPosition());
        telemetry.addData("Shooter Left", shooter1.getCurrentPosition());
        telemetry.addData("Shooter Right", shooter2.getCurrentPosition());
        telemetry.update();
    }
}

