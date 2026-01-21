package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.teleop.SB5.map;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp
public class cool extends LinearOpMode {

    /* ================= TUNABLE PARAMETERS ================= */

    // PID (units = inches)
    public static double kP = 0.035;
    public static double kI = 0.0;
    public static double kD = 0.12;

    // Feedforward (SMOOTHED)
    public static double kF = 0.06;

    // Static friction breaker
    public static double minTurnPower = 0.06;

    // Safety limit
    public static double maxTurnPower = 0.6;

    // Dynamic deadband (inches)
    public static double deadbandMin = 0.12;
    public static double deadbandMax = 0.6;

    // Slew rate (power per second)
    public static double slewRate = 3.0;

    // Error low-pass filter
    public static double errorAlpha = 0.15; // lower = smoother

    /* ================= HARDWARE ================= */

    private MecanumDrive drive;
    private Intake intake;
    private Outtake outtake;
    private Feeder feeder;
    private static final RobotHardware robot = RobotHardware.getInstance();

    /* ================= APRILTAG ================= */

    private AprilTagDetection tag;
    private boolean aprilVisible = false;

    // ⚠️ CHANGE THIS IF NEEDED:
    // If z moves left/right → use z
    // If y moves left/right → use y
    private double tagTurnError = 0;

    /* ================= CONTROLLER STATE ================= */

    private double integral = 0;
    private double lastError = 0;
    private double lastOutput = 0;
    private double filteredError = 0;
    private long lastTime = System.nanoTime();

    /* ================= ACTIONS ================= */

    public static List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);
        intake = new Intake(robot);
        outtake = new Outtake(robot);
        feeder = new Feeder(robot);

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setCameraPose(
                        new Position(DistanceUnit.INCH, -2.36, 2.44, 15, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 90, 0)
                )
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {

            /* ================= APRILTAG UPDATE ================= */

            if (!tagProcessor.getDetections().isEmpty()) {
                tag = tagProcessor.getDetections().get(0);
                aprilVisible = true;

                // 🔴 IMPORTANT: choose correct axis
                tagTurnError = tag.ftcPose.z; // ← LEFT / RIGHT for YOUR camera
            } else {
                aprilVisible = false;
            }

            /* ================= DRIVER DRIVE ================= */

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            drive.updatePoseEstimate();

            /* ================= APRILTAG ALIGN ================= */

            boolean alignPressed = gamepad1.right_trigger > 0.1;

            if (alignPressed && aprilVisible && tag.id == 20) {

                double turn = aprilTurnUpdate(tagTurnError);

                drive.leftFront.setPower(turn);
                drive.leftBack.setPower(turn);
                drive.rightFront.setPower(-turn);
                drive.rightBack.setPower(-turn);

            } else {
                resetController();
            }

            /* ================= TELEMETRY ================= */

            telemetry.addData("April Visible", aprilVisible);
            telemetry.addData("Raw Error", tagTurnError);
            telemetry.addData("Filtered Error", filteredError);
            telemetry.addData("Turn Output", lastOutput);

            telemetry.addData("LF Current", drive.leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("LB Current", drive.leftBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("RF Current", drive.rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("RB Current", drive.rightBack.getCurrent(CurrentUnit.AMPS));

            telemetry.update();
        }
    }

    /* ================= TURN CONTROLLER ================= */

    private double aprilTurnUpdate(double error) {

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;
        if (dt <= 0) return lastOutput;

        // -------- LOW PASS FILTER (kills AprilTag jitter) --------
        filteredError = filteredError * (1 - errorAlpha) + error * errorAlpha;
        error = filteredError;

        double absError = Math.abs(error);

        // -------- DYNAMIC DEADBAND --------
        double deadband = Range.clip(
                absError * 0.25,
                deadbandMin,
                deadbandMax
        );

        if (absError < deadband) {
            integral = 0;
            lastOutput *= 0.5; // soft decay → NO oscillation
            return lastOutput;
        }

        // -------- PID --------
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double pid = kP * error + kI * integral + kD * derivative;

        // -------- SMOOTH FEEDFORWARD --------
        double ff = kF * Math.tanh(error / 1.0);

        double target = pid + ff;

        // -------- MINIMUM TURN POWER --------
        if (Math.abs(target) < minTurnPower) {
            target = Math.signum(target) * minTurnPower;
        }

        // -------- SLEW RATE LIMIT --------
        double maxDelta = slewRate * dt;
        target = Range.clip(target, lastOutput - maxDelta, lastOutput + maxDelta);

        target = Range.clip(target, -maxTurnPower, maxTurnPower);
        lastOutput = target;
        return target;
    }

    private void resetController() {
        integral = 0;
        lastError = 0;
        lastOutput = 0;
        filteredError = 0;
        lastTime = System.nanoTime();
    }
}
