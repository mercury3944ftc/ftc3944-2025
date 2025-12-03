package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.pow;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.IntakeFeeder;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

//@Disabled
@TeleOp(name="TeleopTest")
public class TeleopTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

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
        shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        // Tell motors to run from power level
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

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


        // Wait for driver to press play
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            //Send power to drive motors
            driveTrain.drive((double) -gamepad1.left_stick_x, (double) -gamepad1.left_stick_y, (double) -gamepad1.right_stick_x);

            //Run intake
            if (gamepad1.x) {
                intakeFeeder.runIntake();
            } else if (gamepad1.b) {
                intakeFeeder.reverse();
            } else {
                intakeFeeder.stopIntake();
            }

            //Run feeder if shooter is already running
            if (gamepad1.y /*&& shooter.running()*/) {
                intakeFeeder.runFeeder();
            } else if (gamepad1.b) {
                intakeFeeder.reverse();
            } else {
                intakeFeeder.stopFeeder();
            }

            if (gamepad1.right_bumper) {
                shooter.shoot(740); //Default velocity needed for 1 1/2 diagonal tiles way from apriltag!!!
                driveTrain.target(limelight);

            } else if (gamepad1.left_bumper) {
                shooter.stopShooting();
            }

            //Using dpad to raise and lower rpm for LR Model setup/testing
            //Tell driver that he can raise and lower VELOCITY of the motors by using up and down on the d pad!!!
            if (gamepad1.dpad_up){
                shooter.shoot(shooter.getRPM() + 10);
            }

            if (gamepad1.dpad_down){
                shooter.shoot(shooter.getRPM() - 10);
            }

            telemetry.addData("leftFront", leftFront.getCurrentPosition());
            telemetry.addData("leftRear", leftRear.getCurrentPosition());
            telemetry.addData("rightFront", rightFront.getCurrentPosition());
            telemetry.addData("rightRear", rightRear.getCurrentPosition());
            telemetry.addData("Intake", intake.getCurrentPosition());
            telemetry.addData("Feeder", feeder.getCurrentPosition());
            telemetry.addData("Shooter Left", shooter1.getCurrentPosition());
            telemetry.addData("Shooter Right", shooter2.getCurrentPosition());
            telemetry.addData("Shooter Left RPM", shooter.getRPM());
            telemetry.addData("Shooter Coeff", shooter.getCoeff());

            telemetry.update();
        }
    }
}

