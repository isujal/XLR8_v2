package org.firstinspires.ftc.teamcode.extras;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

import java.util.List;

//@TeleOp(name = "DynamicTurret_NoTime", group = "ProtoTypes")
//@Config
@Disabled
@Deprecated
public class turret_tracking_dynamic extends LinearOpMode {
    public static String MOTOR_NAME = "turretMotor";
    public static String WEBCAM_NAME = "Webcam 1";

    // PID
    public static double kP = 0.007;
    public static double kI = 0.000001;
    public static double kD = 0.0005;

    public static double kFF = 0.006;
    public static int MOTOR_QUAD_TICKS = 112;
    public static int GEAR_RATIO = 40;
    public static final int TICKS_PER_REV = MOTOR_QUAD_TICKS * GEAR_RATIO;
    public static boolean wrapEnabled = true;
    public static double minAngleDeg = -720;
    public static double maxAngleDeg = 720;
    public static double maxIntegral = 1e6;
    public static double maxPower = 0.8;

    private DcMotorEx turretMotor;
    private IMU imu;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private double integral = 0;
    private double lastError = 0;
    private double fieldTargetHeading = 0;

    private final double ticksPerDegree = ((double) TICKS_PER_REV) / 360.0;
    private final double degreesPerTick = 360.0 / ((double) TICKS_PER_REV);

    @Override
    public void runOpMode() throws InterruptedException {

        turretMotor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        imu = hardwareMap.get(IMU.class, "imu");

        turretMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        initAprilTag();

        telemetry.addLine("Ready. Press start.");
        telemetry.update();
        waitForStart();

        double robotHeading = normalizeDeg(getRobotHeading());
        double currentTicks = turretMotor.getCurrentPosition();
        double currentAngleDeg = ticksToAngle(currentTicks);
        fieldTargetHeading = normalizeDeg(robotHeading + (currentAngleDeg));

        while (opModeIsActive()) {

            robotHeading = normalizeDeg(getRobotHeading());
            AngularVelocity yawRate = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            double chassisYawRateDegPerSec = yawRate != null ? yawRate.zRotationRate : 0.0;

            List<AprilTagDetection> detections = aprilTag.getDetections();
            boolean tagVisible = !detections.isEmpty();
            if (tagVisible) {
                double tagYawDeg = detections.get(0).ftcPose.yaw;
                fieldTargetHeading = normalizeFull(robotHeading + tagYawDeg);
            }

            double desiredTurretRelDeg = normalizeDeg(fieldTargetHeading - robotHeading);
            double targetTicks = angleToTicksContinuous(desiredTurretRelDeg);

            currentTicks = turretMotor.getCurrentPosition();
            double error = targetTicks - currentTicks;

            if (!wrapEnabled) {
                double clampedDeg = clamp(desiredTurretRelDeg, minAngleDeg, maxAngleDeg);
                targetTicks = angleToTicksContinuous(clampedDeg);
                error = targetTicks - currentTicks;
            }

            integral += error;
            if (Math.abs(integral) > maxIntegral) integral = Math.signum(integral) * maxIntegral;

            double derivative = (error - lastError);
            double pidPower = kP * error + kI * integral + kD * derivative;

            double ffPower = -kFF * chassisYawRateDegPerSec;
            double power = pidPower + ffPower;
            power = clamp(power, -maxPower, maxPower);

            turretMotor.setPower(power);
            lastError = error;

            telemetry.addData("TagVisible", tagVisible);
            telemetry.addData("RobotHeading", "%.2f", robotHeading);
            telemetry.addData("FieldTargetHeading", "%.2f", fieldTargetHeading);
            telemetry.addData("DesiredRelDeg", "%.2f", desiredTurretRelDeg);
            telemetry.addData("Error", "%.1f", error);
            telemetry.addData("PIDPower", "%.3f", pidPower);
            telemetry.addData("FFPower", "%.3f", ffPower);
            telemetry.addData("OutPower", "%.3f", power);
            telemetry.update();
        }

        visionPortal.close();
    }

    // ----------------------- helpers ------------------------
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawTagOutline(false)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private double getRobotHeading() {
        double y = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (Double.isNaN(y)) return 0;
        if (y < 0) y += 360.0;
        return y;
    }

    private double ticksToAngle(double ticks) {
        return (ticks / (double) TICKS_PER_REV) * 360.0;
    }

    private double angleToTicksContinuous(double desiredRelDeg) {
        double desiredTicksRaw = (desiredRelDeg / 360.0) * TICKS_PER_REV;
        double currentTicks = turretMotor.getCurrentPosition();
        double revolutionsDifference = Math.round((currentTicks - desiredTicksRaw) / (double) TICKS_PER_REV);
        return desiredTicksRaw + revolutionsDifference * TICKS_PER_REV;
    }

    private double normalizeDeg(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }

    private double normalizeFull(double angle) {
        return ((angle % 360) + 360) % 360;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
