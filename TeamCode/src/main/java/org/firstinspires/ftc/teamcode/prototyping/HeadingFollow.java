package org.firstinspires.ftc.teamcode.prototyping;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.MecanumDrive;

//@TeleOp(name = "HeadingFollowTeleOp", group = "Drive")
@Config
public class HeadingFollow extends LinearOpMode {

    // Configurable PIDF constants (tuned from dashboard)
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.001;
    public static double kF = 0.0;

    // Target location and heading
    public static double targetX = 66;
    public static double targetY = 66;
    public static double targetHeading = Math.toRadians(90);

    MecanumDrive drive;
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    // FTC PIDF controller
    private PIDFController headingController;

    private boolean BFlag=false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, startPose);

        // Use MultipleTelemetry for DS + Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Create PIDF controller for heading
        headingController = new PIDFController(kP, kI, kD, kF);


        BFlag=false;
        waitForStart();

        while (opModeIsActive()) {
            // Compute correction
            double correction = computeHeadingCorrection(drive.localizer.getPose());

            // Gamepad translational input
            double driveY = -gamepad1.left_stick_y;
            double driveX = gamepad1.left_stick_x;

            // Drive robot
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(driveX, driveY), correction));
            drive.updatePoseEstimate();

            Vector2d currentPos = drive.localizer.getPose().position;
            double currentHeading = drive.localizer.getPose().heading.toDouble();
            double angleToTarget=Math.atan2(targetY - currentPos.y, targetX - currentPos.x);

            telemetry.addData("Current Heading (deg)", Math.toDegrees(currentHeading));
            telemetry.addData("Correction", correction);
            telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
            telemetry.addData("Current Pos", currentPos);
            telemetry.addData("Error ", Math.toDegrees(headingController.getPositionError()));
            telemetry.addData("error (C-AT)",angleWrap(currentHeading-angleToTarget));
            telemetry.addData("Angle To Target", Math.toDegrees(Math.atan2(targetY - currentPos.y, targetX - currentPos.x)));

            telemetry.update();
        }
    }


    private double computeHeadingCorrection(Pose2d pose) {
        Vector2d currentPos = pose.position;
        double currentHeading = pose.heading.toDouble();

        // Desired angle
        double angleToTarget = Math.atan2(targetY - currentPos.y, targetX - currentPos.x);

        // Wrap error to [-π, π]
        double error = angleWrap(angleToTarget - currentHeading);

        // Feed error into PIDF controller
        return headingController.calculate(0, error);
    }

    // Keep angle in range [-π, π]
    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
