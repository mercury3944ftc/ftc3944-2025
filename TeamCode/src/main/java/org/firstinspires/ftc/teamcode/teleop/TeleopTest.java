package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.pow;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebCam;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

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

//        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");
//        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
//        limelight.start(); // This tells Limelight to start looking! //A reference to see if the camera detects anything right now
        AprilTagWebCam aprilTagWebCam = new AprilTagWebCam();
        double distanceToGoal = 0;
        String currTeam = "blue";



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

        aprilTagWebCam.init(hardwareMap, telemetry);
        // Wait for driver to press play
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            //Send power to drive motors
            driveTrain.drive((double) -gamepad1.left_stick_x, (double) -gamepad1.left_stick_y, (double) -gamepad1.right_stick_x);
            aprilTagWebCam.update();

//            LLResult result = limelight.getLatestResult();
//            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty())// If it does see something and it is valid
//            {
//                LLResultTypes.FiducialResult tag = result.getFiducialResults().get(0); // gets the tag of the first April Tag it detects (will change later because multiple objects can be detected)
//                targetPose = tag.getTargetPoseRobotSpace(); // Gets position of camera relative to field
//                distanceToGoal = distanceToPoint(targetPose);
//
//            }
            AprilTagDetection blueTag = aprilTagWebCam.getTagBySpecificID(20);
            AprilTagDetection redTag = aprilTagWebCam.getTagBySpecificID(24);
            if (currTeam.equals("blue"))
            {
                if (blueTag != null && blueTag.ftcPose != null)
                {
                    distanceToGoal = blueTag.ftcPose.range;
                    distanceToGoal *= 1.1569;
                    distanceToGoal -= 0.5116;
                    aprilTagWebCam.displayDetectionTelemetry(blueTag);
                }
                else
                {
                    telemetry.addLine("There is no april tag data");
                }
            }
            else
            {
                if (blueTag != null && blueTag.ftcPose != null)
                {
                    distanceToGoal = blueTag.ftcPose.range;
                    aprilTagWebCam.displayDetectionTelemetry(blueTag);
                }
                else
                {
                    telemetry.addLine("There is no april tag data");
                }
            }


            //Run intake
            if (gamepad1.x) {
                intakeFeeder.runIntake1();
                intakeFeeder.runIntake2();

            } else if (gamepad1.b) {
                intakeFeeder.reverse();
            } else {
                intakeFeeder.stopIntake1();
                intakeFeeder.stopIntake2();
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
                double rpm = 1500;
//                if (distanceToGoal != 0)
//                {
//                    rpm = RPMFunction(distanceToGoal);
//                }
                shooter.shoot(rpm); //Default velocity needed for 1 1/2 diagonal tiles way from apriltag!!!
//                driveTrain.target(limelight);

            } else if (gamepad1.left_bumper) {
                shooter.stopShooting();
            }

            //Using dpad to raise and lower rpm for LR Model setup/testing
            //Tell driver that he can raise and lower VELOCITY of the motors by using up and down on the d pad!!!
            if (gamepad1.dpad_up){
                shooter.shoot(shooter.getRPM() - 10);
            }

            if (gamepad1.dpad_down){
                shooter.shoot(shooter.getRPM() + 10);
            }

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
            telemetry.addData("Distance To Goal", distanceToGoal);

            telemetry.update();
        }
        aprilTagWebCam.stop();
    }

    public double RPMFunction(double distance)
    {
        double rpm = distance;
        return rpm;
    }

    double distanceToPoint(Pose3D botPose) {
        double dx = botPose.getPosition().x;
        double dy = botPose.getPosition().y;
        double dz = botPose.getPosition().z;

        return Math.sqrt((dx*dx) + (dz*dz));
    }

//use limelight.getbotpose to get position

}

