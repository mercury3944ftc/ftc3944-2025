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
@Autonomous(name="REDShootAuton")
public class REDShootAuton extends LinearOpMode
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
        DcMotor intake1 = hardwareMap.get(DcMotor.class, "intake1");
        DcMotor intake2 = hardwareMap.get(DcMotor.class, "intake2");
        DcMotor feeder = hardwareMap.get(DcMotor.class, "feeder1");
        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");


        // Reverse right side and arm motors
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoder values
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        feeder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        // Tell motors to run from power level
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tell wheel and climber motors to resist external force
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Shooter shooter = new Shooter(shooter1);
        IntakeFeeder intakeFeeder = new IntakeFeeder(intake1, intake2, feeder);
        DriveTrain driveTrain = new DriveTrain(leftFront, rightFront, leftRear, rightRear);

        waitForStart();
        Timer timer = new Timer();
        TimerTask moveBack = new TimerTask() {
            @Override
            public void run() {
                driveTrain.drive(0.0, 0.4, 0.0);
            }
        };

        TimerTask runShooters1 = new TimerTask() {
            @Override
            public void run() {
                driveTrain.drive(0,0,0);
                shooter.shoot(1500);
            }
        };


        TimerTask runShooters2 = new TimerTask() {
            @Override
            public void run() {
                driveTrain.drive(0,0,0);
                shooter.shoot(1500);
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
                intakeFeeder.runIntake1();
                intakeFeeder.runIntake2();

            }
        };
        TimerTask moveLeft = new TimerTask() {
            @Override
            public void run() {
                shooter.stopShooting();
                intakeFeeder.stopFeeder();
                intakeFeeder.stopIntake1();
                intakeFeeder.stopIntake2();
                driveTrain.drive(0.3, 0.0, 0.0);
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
        timer.schedule(runShooters2, (long) 5000);
        timer.schedule(shoot2, (long) 6000);
        timer.schedule(moveLeft, (long) 12000);
        timer.schedule(stopDrivingTask, (long) 14000);
        sleep(16000);

        telemetry.addData("leftFront", leftFront.getCurrentPosition());
        telemetry.addData("leftRear", leftRear.getCurrentPosition());
        telemetry.addData("rightFront", rightFront.getCurrentPosition());
        telemetry.addData("rightRear", rightRear.getCurrentPosition());
        telemetry.addData("Intake1", intake1.getCurrentPosition());
        telemetry.addData("Intake2", intake2.getCurrentPosition());
        telemetry.addData("Feeder", feeder.getCurrentPosition());
        telemetry.addData("Shooter Left", shooter1.getCurrentPosition());
        telemetry.addData("Shooter Left RPM", shooter.getRPM());
        telemetry.addData("Shooter Coeff", shooter.getCoeff());
//            telemetry.addData("Distance To Goal", distanceToGoal);
        telemetry.update();
    }
}

