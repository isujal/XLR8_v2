package org.firstinspires.ftc.teamcode.extras;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
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

/**
 * Dynamic turret lock:
 * - Camera mounted on turret (so tagYaw is relative to turret/camera).
 * - Field-relative lock: fieldTargetHeading = robotHeading + tagYaw.
 * - If tag lost -> hold last fieldTargetHeading; after timeout -> search mode pans slowly.
 * - Supports continuous wrap-around (>360) by mapping encoder ticks <-> continuous angle.
 * - Feedforward uses IMU yaw rate to improve tracking while the chassis rotates.
 *
 * Tune the public static fields via FTC Dashboard (they are annotated with @Config).
 */
//@TeleOp(name = "DynamicTurret_Advanced", group = "ProtoTypes")
//@Config
@Deprecated
public class timer_turret_tracking extends LinearOpMode {
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

    // Search behavior
    public static double searchTimeoutSec = 1.0;
    public static double searchSpeedDegPerSec = 10;
    public static int searchDirection = 1;

    // Integral anti-windup + output clamp
    public static double maxIntegral = 1e6;
    public static double maxPower = 0.8;

    // =======================================================================

    private DcMotorEx turretMotor;
    private IMU imu;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Controller state
    private double integral = 0;
    private double lastError = 0;

    private double fieldTargetHeading = 0;

    private long lastTagSeenTimeNs = 0;

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

        lastTagSeenTimeNs = System.nanoTime();

        // main loop: event-driven (no fixed sleep). Updates happen every loop iteration.
        while (opModeIsActive()) {

            // --- sensor reads ---
            robotHeading = normalizeDeg(getRobotHeading());
            AngularVelocity yawRate = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            double chassisYawRateDegPerSec = yawRate != null ? yawRate.zRotationRate : 0.0; // deg/s, positive CCW typically

            // --- vision ---
            List<AprilTagDetection> detections = aprilTag.getDetections();
            boolean tagVisible = !detections.isEmpty();
            double tagYawDeg = 0.0; // relative to camera/turret
            if (tagVisible) {
                tagYawDeg = detections.get(0).ftcPose.yaw; // in degrees
                // update fieldTargetHeading using fresh detection
                // tagYawDeg is turret-camera relative yaw to tag: fieldHeading = robotHeading + tagYawDeg
                fieldTargetHeading = normalizeFull(robotHeading + tagYawDeg);
                lastTagSeenTimeNs = System.nanoTime();
            } else {
                // tag lost: check timeout for entering search mode (use elapsed time)
                long now = System.nanoTime();
                double secSinceSeen = (now - lastTagSeenTimeNs) / 1e9;
                if (secSinceSeen >= searchTimeoutSec) {
                    // SEARCH MODE: slowly pan fieldTargetHeading so the turret scans the field.
                    // Advance by searchSpeedDegPerSec * dt
                    // Use dt from last loop (approx): compute using lastTagSeenTimeNs->now so panning speed is time-driven but only for search
                    double dt = secSinceSeen - searchTimeoutSec; // approximate time spent in search
                    // We don't want to jump huge amounts if loop hiccups: scale by small fraction
                    double pan = searchDirection * searchSpeedDegPerSec * (dt > 0.02 ? 0.02 : dt); // cap dt stepping
                    // More robust: just nudge by small fixed per-loop step computed from loop time estimate (safe)
                    fieldTargetHeading = normalizeFull(fieldTargetHeading + pan);
                    // note: lastTagSeenTimeNs is not updated here so secSinceSeen keeps growing; small increments avoid big jumps
                }
            }

            // --- compute desired turret relative angle & target ticks ---
            // desired turret angle relative to robot body (where turret should be pointing, in degrees)
            double desiredTurretRelDeg = normalizeDeg(fieldTargetHeading - robotHeading);
            // convert to continuous ticks target (we allow target to be any continuous angle)
            double targetTicks = angleToTicksContinuous(desiredTurretRelDeg);

            // --- read current turret position in ticks & compute error ---
            currentTicks = turretMotor.getCurrentPosition();
            double error = targetTicks - currentTicks;

            // If wrapEnabled==false then clamp desiredTurretRelDeg into min/max and recompute targetTicks
            if (!wrapEnabled) {
                double clampedDeg = clamp(desiredTurretRelDeg, minAngleDeg, maxAngleDeg);
                targetTicks = angleToTicksContinuous(clampedDeg);
                error = targetTicks - currentTicks;
            }

            // --- PID (measurement-driven: no explicit dt used for derivative; derivative = deltaError) ---
            // Integral accumulation with simple anti-windup
            integral += error;
            if (Math.abs(integral) > maxIntegral) integral = Math.signum(integral) * maxIntegral;

            double derivative = (error - lastError);

            // PID output (in power units)
            double pidPower = kP * error + kI * integral + kD * derivative;

            // --- feedforward from chassis yaw rate (deg/s) ---
            // If robot rotates CCW (positive deg/s) and tag is field-fixed, turret must rotate CW relative to robot to keep pointing.
            // So we add feedforward opposite to yawRate: power contribution = -kFF * yawRateDegPerSec
            double ffPower = -kFF * chassisYawRateDegPerSec;

            double power = pidPower + ffPower;

            // safety clamp
            power = clamp(power, -maxPower, maxPower);

            // apply to motor
            turretMotor.setPower(power);

            lastError = error;

            // telemetry (compact)
            telemetry.addData("TagVisible", tagVisible);
            telemetry.addData("RobotHeading", "%.2f", robotHeading);
            telemetry.addData("FieldTargetHeading", "%.2f", fieldTargetHeading);
            telemetry.addData("DesiredRelDeg", "%.2f", desiredTurretRelDeg);
            telemetry.addData("CurrentTicks", "%.1f", currentTicks);
            telemetry.addData("TargetTicks", "%.1f", targetTicks);
            telemetry.addData("Error(ticks)", "%.1f", error);
            telemetry.addData("PIDPower", "%.3f", pidPower);
            telemetry.addData("FFPower", "%.3f", ffPower);
            telemetry.addData("OutPower", "%.3f", power);
            telemetry.update();
        }

        visionPortal.close();
    }

    // ----------------------- helpers ------------------------

    // initialize a very simple AprilTag processor + VisionPortal
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

    // read IMU yaw (0..360)
    private double getRobotHeading() {
        double y = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (Double.isNaN(y)) return 0;
        if (y < 0) y += 360.0;
        return y;
    }

    // convert encoder ticks -> continuous angle relative to turretZero (deg)
    // uses current raw ticks (which are unbounded), so result is continuous (can be >360 or <0)
    private double ticksToAngle(double ticks) {
        // turretZero is assumed 0 ticks == 0 deg, adjust if you have different zero
        return (ticks / (double) TICKS_PER_REV) * 360.0;
    }

    // convert desired relative angle (in -180..180) to continuous target ticks close to current encoder position
    private double angleToTicksContinuous(double desiredRelDeg) {
        // Convert the desired absolute relative angle (in -180..180) to a ticks value nearest the current encoder ticks.
        double desiredTicksRaw = (desiredRelDeg / 360.0) * TICKS_PER_REV;

        // Current encoder ticks:
        double currentTicks = turretMotor.getCurrentPosition();

        // We need to find an integer N such that desiredTicks = desiredTicksRaw + N * TICKS_PER_REV
        // is closest to currentTicks (so turret will take shortest rotation to desired angle)
        double revolutionsDifference = Math.round((currentTicks - desiredTicksRaw) / (double) TICKS_PER_REV);
        double desiredTicksContinuous = desiredTicksRaw + revolutionsDifference * TICKS_PER_REV;

        return desiredTicksContinuous;
    }

    // convenience: convert angleDeg to ticks ignoring continuity (useful for absolute setups)
    private double angleToTicks(double angleDeg) {
        return (angleDeg / 360.0) * TICKS_PER_REV;
    }

    // normalize to (-180..180]
    private double normalizeDeg(double angle) {
        angle = ((angle + 180) % 360 + 360) % 360 - 180;
        return angle;
    }

    // normalize to [0..360)
    private double normalizeFull(double angle) {
        angle = ((angle % 360) + 360) % 360;
        return angle;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
