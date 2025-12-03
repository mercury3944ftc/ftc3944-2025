package org.firstinspires.ftc.teamcode.auton;
import static java.lang.Math.pow;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.IntakeFeeder;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Timer;
import java.util.TimerTask;

//@Disabled
@Autonomous(name="FarShootAuton")
public class FarShootAuton extends LinearOpMode
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
        TimerTask move = new TimerTask() {
            @Override
            public void run() {
                driveTrain.drive(0.0, -0.2, 0.0);
            }
        };
        TimerTask stopDrivingTask = new TimerTask() {
            @Override
            public void run() {
                driveTrain.drive(0.0, 0.0, 0.0);
            }
        };
        timer.schedule(move, (long) 0);
        timer.schedule(stopDrivingTask, (long) 2000);
        sleep(28000);

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

