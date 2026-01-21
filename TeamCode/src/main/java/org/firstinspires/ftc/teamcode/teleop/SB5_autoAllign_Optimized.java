package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

@Config
//@TeleOp(name = "SB5_AUTO_ALIGN_Optimized", group = "ATELEOP")
public class SB5_autoAllign_Optimized extends LinearOpMode {

    // hardware / subsystems (reuse your implementations)
    private RobotHardware robot;
    private MecanumDrive drive;
    private Intake intake;
    private Feeder feeder;
    private Outtake outtake;

    // dashboard / telemetry
    private FtcDashboard dashboard;
    private TelemetryPacket packet = new TelemetryPacket();
    private ElapsedTime loopTimer = new ElapsedTime();

    // Vision
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    // Shared state flags (thread-safe primitives)
    private final AtomicBoolean alignHold = new AtomicBoolean(false); // Y: hold-to-align
    private final AtomicBoolean running = new AtomicBoolean(false);
    private volatile double alignOutput = 0.0; // signed power for alignment (from vision thread)
    private volatile boolean tagVisible = false;
    private volatile int detectedTagId = -1;
    private volatile double detectedY = 0.0;
    private volatile double detectedZ = 0.0;

    // Turret PID shared vars
    private volatile double turretPidOutput = 0.0; // power to apply to turret motors
    private volatile int turretEncoderPos = 0;

    // Feed FSM
    private enum FeedState { IDLE, INTAKING, FIRST_DETECTED, TRANSFER, FULL }
    private FeedState feedState = FeedState.IDLE;

    // Cached previous gamepad copies (to detect edges)
    private final com.qualcomm.robotcore.hardware.Gamepad cachedGamepad1 = new com.qualcomm.robotcore.hardware.Gamepad();
    private final com.qualcomm.robotcore.hardware.Gamepad prevGamepad1 = new com.qualcomm.robotcore.hardware.Gamepad();

    // Tunables (expose via @Config)
    public static double driveScaleSlow = 0.5;
    public static double driveScaleDefault = 1.0;
    public static double alignMaxPower = 0.45;
    public static double turretKp = 0.02;
    public static double turretKi = 0.0;
    public static double turretKd = 0.0;
    public static int turretEncMin = 0;
    public static int turretEncMax = 27845;

    // Telemetry throttle
    private int telemetryCounter = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // init hardware and subsystems (reuse your implementations)
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap, telemetry);

        intake = new Intake(robot);
        feeder = new Feeder(robot);
        outtake = new Outtake(robot);

        drive = new MecanumDrive(hardwareMap, new com.acmerobotics.roadrunner.Pose2d(0, 0, 0));

        dashboard = FtcDashboard.getInstance();

        // set shooter PIDF (cheap setter, not heavy)
        Globals.shooterMode = true;
        robot.shooter.setVelocityPIDFCoefficients(350, 0, 0, 13.2);

        // Setup AprilTag (but do not enable processing until alignHold is true)
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                // camera pose - keep as in your original config (adjust if needed)
                .setCameraPose(
                        new org.firstinspires.ftc.robotcore.external.navigation.Position(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH, -2.36, 2.44, 15, 0),
                        new org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES, 0, -90, 90, 0)
                ).build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(640, 480))
                .build();

        // shared running flag
        running.set(true);

        // Start background threads BEFORE waitForStart so they remain responsive in init loop
        Thread turretThread = new Thread(this::turretLoop, "TurretThread");
        Thread visionThread = new Thread(this::visionLoop, "VisionThread");
        turretThread.setDaemon(true);
        visionThread.setDaemon(true);
        turretThread.start();
        visionThread.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for start but keep minimal work in init (we still call small methods)
        while (opModeInInit() && !isStopRequested()) {
            // reset flags and states cheaply
            feedState = FeedState.IDLE;
            intakeTimerResetIfNeeded();
            // run a cheap turret update once: get encoder and feed to PID thread via volatile
            turretEncoderPos = robot.turretEncoder.getCurrentPosition();
            // small sleep to avoid busy-wait
            sleep(10);
        }

        waitForStart();
        loopTimer.reset();

        // MAIN TELEOP LOOP: minimal work, avoid allocations
        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();

            // --- 1) Gamepad caching (cheap copy) ---
            prevGamepad1.copy(cachedGamepad1);
            cachedGamepad1.copy(gamepad1);

            // --- 2) Drive input (cached values) ---
            final double leftY = -cachedGamepad1.left_stick_y;
            final double leftX = -cachedGamepad1.left_stick_x;
            final double rightX = -cachedGamepad1.right_stick_x;

            final double driveScale = (cachedGamepad1.left_trigger > 0.1) ? driveScaleSlow : driveScaleDefault;

            // direct set to MecanumDrive -- this is lightweight (no allocations)
            drive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                    new com.acmerobotics.roadrunner.Vector2d(leftY * driveScale, leftX * driveScale),
                    rightX * driveScale
            ));

            // --- 3) Align button (hold behavior) ---
            // Use Y as hold-to-align. If held, visionThread will enable processing and compute alignOutput.
            final boolean yHeld = cachedGamepad1.y;
            alignHold.set(yHeld);

            // apply align output to drive (if active)
            if (alignHold.get() && tagVisible) {
                // alignOutput is signed rotation power from vision thread; clamp
                double rotPower = Math.max(-alignMaxPower, Math.min(alignMaxPower, alignOutput));
                // quick apply: rotate in place proportionally (scale down translational movement)
                drive.leftFront.setPower(-rotPower);
                drive.leftBack.setPower(-rotPower);
                drive.rightBack.setPower(rotPower);
                drive.rightFront.setPower(rotPower);
            }

            // --- 4) Intake toggle (left bumper edge) ---
            if (cachedGamepad1.left_bumper && !prevGamepad1.left_bumper) {
                intakeToggleFlip();
            }

            // Apply intake/roller state based on intakeFlag
            if (intakeFlagState()) {
                // call your subsystem method that starts intake (assumed method names)
                intake.setRoller(Intake.IntakeRollerState.ON); // replace with your method if different
            } else {
                intake.setRoller(Intake.IntakeRollerState.OFF);
            }

            // --- 5) Feed FSM (very small, deterministic checks) ---
            runFeedFSM(); // uses beam sensors from robot (no allocations)

            // --- 6) Shooter controls (x / y quick set) ---
            if (cachedGamepad1.x && !prevGamepad1.x) {
                Globals.shooterMode = true;
                outtake.setHood(Outtake.HoodState.NEAR_END);  // adapt to your method name
                outtake.setShooter(Outtake.ShooterState.NEAR);
            }
            if (cachedGamepad1.y && !prevGamepad1.y) {
                // y is hold-to-align; but single press also sets shooter far
                Globals.shooterMode = true;
                outtake.setHood(Outtake.HoodState.FAR);
                outtake.setShooter(Outtake.ShooterState.TELE_FAR);
            }

            // --- 7) Manual turret override (right stick when pressed) ---
            if (Math.abs(cachedGamepad1.right_stick_x) > 0.05) {
                // small manual turret command - write power directly, but keep pid thread aware of manual override
                double manual = -cachedGamepad1.right_stick_x * 0.4;
                robot.turret1.setPower(manual);
                robot.turret2.setPower(-manual);
                // also inform PID thread (optionally) - write encoder pos
                turretEncoderPos = robot.turretEncoder.getCurrentPosition();
            }

            // --- 8) Telemetry (throttled) ---
            telemetryCounter++;
            if (telemetryCounter >= 6) { // update telemetry ~ every 6 loops
                telemetryCounter = 0;
                packet.put("loop_ms", loopTimer.milliseconds());
                packet.put("tagVisible", tagVisible);
                packet.put("alignOut", alignOutput);
                packet.put("turretPid", turretPidOutput);
                packet.put("counterFeedState", feedState.toString());
                dashboard.sendTelemetryPacket(packet);
                telemetry.addData("loop (ms)", "%.2f", loopTimer.milliseconds());
                telemetry.update();
            }

            // short sleep to yield (tune as needed; small to keep loop ~3-6ms)
            sleep(3);
        }

        // STOP: signal threads and stop everything safely
        running.set(false);
        if (visionPortal != null) {
            visionPortal.close();
        }
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    // -----------------------
    // Background: turret PID + IMU unwrap loop
    // Runs at a fixed modest frequency and writes turretPidOutput and applies motors
    // -----------------------
    private void turretLoop() {
        // simple PID variables
        double integral = 0.0;
        double lastError = 0.0;
        final int sleepMs = 10; // 100 Hz-ish
        while (running.get() && opModeIsActive()) {
            try {
                // read latest turret encoder once
                turretEncoderPos = robot.turretEncoder.getCurrentPosition();
                // compute desired target from IMU unwrap (Globals.currentTurretState expected to be degrees)
                double imuAngle = Globals.currentTurretState; // degrees, continuous tracked elsewhere if you already have unwrap
                // convert encoder pose to degrees (map)
                double c = map(turretEncoderPos, turretEncMin, turretEncMax, 0, 360);
                if (imuAngle < 0) c = -c;
                double error = imuAngle - c;

                // PID
                integral += error * (sleepMs / 1000.0);
                double derivative = (error - lastError) / (sleepMs / 1000.0);
                lastError = error;
                double pid = turretKp * error + turretKi * integral + turretKd * derivative;

                // publish to shared var
                turretPidOutput = pid;
                turretPidOutput = Math.max(-1.0, Math.min(1.0, turretPidOutput));

                // SAFETY: small deadband and clamp
                if (Math.abs(pid) < 0.01) {
                    robot.turret1.setPower(0.0);
                    robot.turret2.setPower(0.0);
                } else {
                    // apply -pid to turret1 and pid to turret2 (same as your original)
                    robot.turret1.setPower(-turretPidOutput);
                    robot.turret2.setPower(turretPidOutput);
                }

                // sleep to maintain rate
                Thread.sleep(sleepMs);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            } catch (Exception e) {
                // swallow exceptions so thread doesn't die unexpectedly; log minimally
                telemetry.addData("TurretThreadErr", e.getMessage());
                telemetry.update();
            }
        }
    }

    // -----------------------
    // Background: vision loop
    // When alignHold is true, enable tag processing and compute a cheap alignment output at low rate.
    // When alignHold is false, disable tag processing to save CPU.
    // -----------------------
    private void visionLoop() {
        final int idleSleepMs = 80;   // when not aligning, sample slowly
        final int activeSleepMs = 25; // when aligning, run faster but still limited
        boolean lastEnabled = false;

        while (running.get() && opModeIsActive()) {
            try {


                if (alignHold.get()) {

                    // enable tag processing (cheap call)
                    if (!lastEnabled) {
                        visionPortal.setProcessorEnabled(tagProcessor, true);
                        lastEnabled = true;
                    }

                    // get detections (this is a cheap reference read in many pipelines; adjust if yours copies)
                    List<AprilTagDetection> dets = tagProcessor.getDetections();
                    if (dets != null && !dets.isEmpty()) {
                        AprilTagDetection d = dets.get(0);
                        tagVisible = true;
                        detectedTagId = d.id;
                        detectedY = d.ftcPose.y; // distance-like value
                        detectedZ = d.ftcPose.z;
                        // compute simple proportional align output based on tag X offset (ftcPose.x)
                        double xOffset = d.ftcPose.x; // positive = some direction, adjust sign per your camera mounting
                        double alignP = 0.02; // small P gain
                        double raw = -alignP * xOffset;
                        // clamp
                        alignOutput = Math.max(-alignMaxPower, Math.min(alignMaxPower, raw));
                    } else {
                        tagVisible = false;
                        alignOutput = 0.0;
                    }

                    Thread.sleep(activeSleepMs);
                } else {
                    // disable processing to save CPU/GPU
                    if (lastEnabled) {
                        visionPortal.setProcessorEnabled(tagProcessor, false);
                        lastEnabled = false;
                        tagVisible = false;
                        alignOutput = 0.0;
                    }
                    Thread.sleep(idleSleepMs);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            } catch (Exception e) {
                // minimal logging
                telemetry.addData("VisionThreadErr", e.getMessage());
                telemetry.update();
            }
        }
    }

    // -----------------------
    // FEED FSM: small, deterministic, no allocations
    // Uses your beam sensors robot.intakeBeam / robot.feederBeam / robot.outtakeBeam
    // -----------------------
    private void runFeedFSM() {
        boolean intakeBeam = robot.intakeBeam.getState();
        boolean feederBeam = robot.feederBeam.getState();
        boolean outtakeBeam = robot.outtakeBeam.getState();

        switch (feedState) {
            case IDLE:
                if (intakeFlagState()) {
                    feeder.setLF(Feeder.LowerFeederState.ON); // adapt method names
                    intake.setRoller(Intake.IntakeRollerState.ON);
                    feedState = FeedState.INTAKING;
                } else {
                    feeder.setUF(Feeder.UpperFeederState.OFF);
                    feeder.setLF(Feeder.LowerFeederState.OFF);
                    intake.setRoller(Intake.IntakeRollerState.OFF);
                }
                break;

            case INTAKING:
                // detect ball entering intake (beam false = ball present depending on wiring)
                if (!intakeBeam) {
                    feedState = FeedState.FIRST_DETECTED;
                }
                break;

            case FIRST_DETECTED:
                // move it to feeder; wait for feeder beam change
                if (!feederBeam) {
                    feedState = FeedState.TRANSFER;
                }
                break;

            case TRANSFER:
                // if outtake beam sees ball, we are moved up
                if (!outtakeBeam) {
                    feedState = FeedState.FULL;
                }
                break;

            case FULL:
                // we are full: stop rollers
                feeder.setLF(Feeder.LowerFeederState.OFF);
                intake.setRoller(Intake.IntakeRollerState.OFF);
                // clear condition: if operator toggles intake off, go to IDLE
                if (!intakeFlagState()) {
                    feedState = FeedState.IDLE;
                }
                break;
        }
    }

    // -----------------------
    // Small helpers - adapt to your actual subsystem method names
    // -----------------------
    private volatile boolean intakeFlag = false;
    private void intakeToggleFlip() {
        intakeFlag = !intakeFlag;
    }
    private boolean intakeFlagState() {
        return intakeFlag;
    }
    private void intakeTimerResetIfNeeded() {
        // placeholder for any pre-init resets you used
    }

    // map helper (copy of your original)
    private static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        if (inMax - inMin == 0) return outMin;
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
}
