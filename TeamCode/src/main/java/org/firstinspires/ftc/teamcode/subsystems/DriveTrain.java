package org.firstinspires.ftc.teamcode.subsystems;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.SimplePID;

import java.util.List;
import java.util.function.Supplier;

public class DriveTrain {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    public DriveTrain(DcMotor LF, DcMotor RF, DcMotor LR, DcMotor RR)
    {
        leftFront = LF;
        rightFront = RF;
        leftRear = LR;
        rightRear = RR;
    }

    public void drive(double stickLX, double stickLY, double stickRX) {
        // Movement - left stick
        double y = -stickLY; // negated because intake is forward
        double x = stickLX;
        // Rotation - right stick
        double rx = stickRX * .75;
        // Apply mecanum formulae to wheels, with a common denominator so no
        // aliasing appears around nyquist or -nyquist
        //𝓯𝓻𝓮𝓪𝓴 𝓫𝓸𝓽  ❤️
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator; //TODO THIS DOES NOT WORK RN, I SUSPECT NOT UPDATING

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    SimplePID turnPID = new SimplePID(0.05, 0.0, 0.005);
    public void target(Limelight3A limelight) {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (!fiducials.isEmpty()) {
                int tagID = fiducials.get(0).getFiducialId();
                double tx = result.getTx(); // horizontal error (degrees)

                // PID calculation: we want tx → 0
                double turnPower = turnPID.calculate(0, tx);

                // limit power to avoid spinning too fast
                turnPower = Math.max(Math.min(turnPower, 0.5), -0.5);

                drive(0.0, 0.0, turnPower); // z-axis rotation only

                // telemetry.addData("Tag ID", tagID);
                // telemetry.addData("tx", tx);
                // telemetry.addData("Turn Power", turnPower);
                // telemetry.update();
            }
        }
    }


//    public void target(Limelight3A limelight) {
//        LLResult result = limelight.getLatestResult();
//
//        if (result != null && result.isValid()) {
//            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//
//            if (!fiducials.isEmpty()) {
//                int tagID = fiducials.get(0).getFiducialId(); // get the first detected tag
//                double tx = result.getTx();
//
//                double power = 0.0;
//
//                if (tx > 2.0) {
//                    power = -0.25;
//                } else if (tx < -2){
//                    power = 0.25;
//                }
//
//                drive(0.0, 0.0, power);
//    }

}
