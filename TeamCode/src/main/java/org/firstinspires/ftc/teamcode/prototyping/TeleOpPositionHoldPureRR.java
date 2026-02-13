package org.firstinspires.ftc.teamcode.prototyping;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

//@TeleOp(name = "Position Hold (Pure RR)")
//@Config
public class TeleOpPositionHoldPureRR extends LinearOpMode {


    // ========== TRAJECTORY TUNING PARAMETERS ==========
    public static double AUTO_MAX_VEL = 80.0;
    public static double AUTO_MAX_ACCEL = 40.0;

    // Position hold uses slower, more precise movements
    public static double HOLD_MAX_VEL = 60.0;
    public static double HOLD_MAX_ACCEL = 20.0;

    // ========== POSITION HOLD TUNING ==========
    // How close is "close enough" to stop correcting
    public static double HOLD_POSITION_TOLERANCE = 2; // inches
    public static double HOLD_HEADING_TOLERANCE = 5; // degrees

    // ========== SHOOTER CONFIGURATION ==========
    public static int SHOOTER_RPM_BUFFER = 50;

    // ========== DRIVE CONFIGURATION ==========
    public static double PRECISION_MODE_SCALE = 0.3;
    public static double NORMAL_POWER_SCALE = 1.0;
    public static double JOYSTICK_CANCEL_THRESHOLD = 0.3;

    // ========== HARDWARE AND SUBSYSTEMS ==========
    private static RobotHardware robot=RobotHardware.getInstance();

    private MecanumDrive drive;

    // ========== GAMEPAD STATE ==========
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();

    // ========== CONTROL STATE ==========
    private boolean shooterEnabled = false;
    private double powerScale = NORMAL_POWER_SCALE;

    // ========== DRIVE STATE MACHINE ==========
    /**
     * Three drive modes, all using Road Runner's trajectory system:
     *
     * MANUAL: Driver controls via joysticks
     * AUTO_RETURN: Smooth one-time trajectory to saved position
     * HOLD_POSITION: Continuously regenerates micro-trajectories to maintain position
     *                This mode leverages Road Runner's PID by repeatedly commanding
     *                it to go to the target position, creating active position holding
     */
    private enum DriveState {
        MANUAL,         // Driver has control
        AUTO_RETURN,    // One-time trajectory navigation
        HOLD_POSITION   // Continuous trajectory regeneration for holding
    }

    private DriveState driveState = DriveState.MANUAL;
    private Pose2d savedPose = null;
    private Pose2d targetHoldPose = null;

    // Hold mode uses continuous trajectory updates
    private long lastHoldUpdateTime = 0;
    private Action currentHoldTrajectory = null;

    // ========== ACTION QUEUE ==========
    private final List<Action> runningActions = new ArrayList<>();
    private Action currentTrajectory = null;

    // ========== TELEMETRY VARIABLES ==========
    private double currentPositionError = 0;
    private double currentHeadingError = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // ========== INITIALIZATION ==========
        telemetry.addLine("Initializing hardware...");
        telemetry.update();

        robot.init(hardwareMap, telemetry);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("✓ Hardware initialized");
        telemetry.update();

        // ========== PRE-START LOOP ==========
        while (opModeInInit()) {
            updateGamepadState();
            runQueuedActions();


            telemetry.addLine("✓ Robot Ready");
            telemetry.addLine("");
            telemetry.addLine("Controls:");
            telemetry.addLine("  Dpad-Up: Save position");
            telemetry.addLine("  Dpad-Right: Navigate to saved position");
            telemetry.addLine("  Dpad-Down: HOLD current position");
            telemetry.addLine("  Dpad-Left: HOLD saved position");
            telemetry.addLine("  Move joystick to cancel hold/nav");
            telemetry.addLine("");
            telemetry.addLine("Note: Uses Road Runner PID for ALL movement");
            telemetry.update();
        }

        waitForStart();

        // ========== MAIN CONTROL LOOP ==========
        while (opModeIsActive()) {

            updateGamepadState();
            drive.updatePoseEstimate();

            handleNavigationControls();

            runQueuedActions();
            handleDriveControls();

//            handleResetControl();
            handleShooterControls();
//            handleLaunchControls();

            updateTelemetry();
        }
    }

    private void updateGamepadState() {
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }

    /**
     * Handles robot driving with three distinct modes, ALL using Road Runner:
     *
     * MANUAL: Driver controls via joysticks (standard field-centric)
     * AUTO_RETURN: Smooth trajectory navigation to saved position
     * HOLD_POSITION: Continuous micro-trajectory regeneration for active holding
     *
     * The HOLD_POSITION mode is the clever part:
     * Instead of writing our own PID loop, we continuously ask Road Runner
     * to navigate to the target position. Road Runner's built-in PID naturally
     * resists disturbances by constantly correcting the trajectory.
     */
    private void handleDriveControls() {
        if (driveState == DriveState.MANUAL) {
            // Standard manual control
            if (gamepad2.left_trigger > 0.2) {
                powerScale = PRECISION_MODE_SCALE;
            } else {
                powerScale = NORMAL_POWER_SCALE;
            }






            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad2.left_stick_y*powerScale,
                            -gamepad2.left_stick_x*powerScale
                    ),
                    -gamepad2.right_stick_x*powerScale
            ));

            if (gamepad2.right_stick_button) {
                Pose2d currentPose = drive.localizer.getPose();
                drive.localizer.setPose(
                        new Pose2d(currentPose.position.x, currentPose.position.y, 0)
                );
            }

        } else if (driveState == DriveState.AUTO_RETURN) {
            // One-time trajectory navigation mode
            double joystickMagnitude = Math.hypot(gamepad2.left_stick_x, gamepad2.left_stick_y);
            boolean driverWantsControl = joystickMagnitude > JOYSTICK_CANCEL_THRESHOLD
                    || Math.abs(gamepad2.right_stick_x) > JOYSTICK_CANCEL_THRESHOLD;

            if (driverWantsControl) {
                cancelTrajectory();
                telemetry.addLine("✗ Navigation cancelled");
            }

            if (currentTrajectory != null && !runningActions.contains(currentTrajectory)) {
                // Trajectory completed - automatically switch to holding that position
                currentTrajectory = null;
                targetHoldPose = savedPose;
                driveState = DriveState.HOLD_POSITION;
                telemetry.addLine("✓ Arrived - Now holding position");
            }

        } else if (driveState == DriveState.HOLD_POSITION) {
            // Active position holding using Road Runner's trajectory system

            // Check if driver wants to cancel and take manual control
            double joystickMagnitude = Math.hypot(gamepad2.left_stick_x, gamepad2.left_stick_y);
            boolean driverWantsControl = joystickMagnitude > JOYSTICK_CANCEL_THRESHOLD
                    || Math.abs(gamepad2.right_stick_x) > JOYSTICK_CANCEL_THRESHOLD;

            if (driverWantsControl) {
                // Driver moved joystick - release hold
                cancelHoldMode();
                telemetry.addLine("✗ Position hold released");
            } else {
                // Execute position hold using Road Runner
                executeRoadRunnerHold();
            }
        }
    }

    /**
     * Executes position holding using Road Runner's trajectory system.
     *
     * The key insight: Instead of writing PID control ourselves, we leverage
     * Road Runner's existing trajectory follower by continuously commanding it
     * to navigate to the target position.
     *
     * How it works:
     * 1. Check if we're far enough from target to need correction
     * 2. If yes, generate a new micro-trajectory to the target position
     * 3. Road Runner's PID controllers handle all the actual motor control
     * 4. Repeat periodically (every HOLD_UPDATE_INTERVAL_MS)
     *
     * This creates "active compliance" - Road Runner naturally resists being
     * pushed away from the target because it's always trying to follow a
     * trajectory to that target.
     *
     * Benefits over manual PID:
     * - Uses Road Runner's already-tuned PID coefficients
     * - Respects velocity and acceleration limits automatically
     * - Handles coordinate transformations correctly
     * - More maintainable - no separate control code to tune
     */
    private void executeRoadRunnerHold() {
        if (targetHoldPose == null) {
            return;
        }

        Pose2d currentPose = drive.localizer.getPose();

        // Calculate error for telemetry and tolerance checking
        double dx = targetHoldPose.position.x - currentPose.position.x;
        double dy = targetHoldPose.position.y - currentPose.position.y;
        double distanceError = Math.hypot(dx, dy);

        double headingError = normalizeRadians(
                targetHoldPose.heading.toDouble() - currentPose.heading.toDouble()
        );
        double headingErrorDeg = Math.toDegrees(headingError);

        currentPositionError = distanceError;
        currentHeadingError = headingErrorDeg;

        // Check if we need to update the hold trajectory
        boolean needsUpdate = false;

        // Update if we've drifted beyond tolerance (external disturbance)
        if (distanceError > HOLD_POSITION_TOLERANCE ||
                Math.abs(headingErrorDeg) > HOLD_HEADING_TOLERANCE) {
            needsUpdate = true;
        }

        // Update if previous hold trajectory completed
        if (currentHoldTrajectory != null && !runningActions.contains(currentHoldTrajectory)) {
            needsUpdate = true;
        }

        if (needsUpdate) {
            // Remove old hold trajectory if it exists
            if (currentHoldTrajectory != null) {
                runningActions.remove(currentHoldTrajectory);
            }

            // Generate new micro-trajectory to target position
            // Road Runner will use its PID to follow this trajectory
            VelConstraint holdVelConstraint = new MinVelConstraint(Arrays.asList(
                    new TranslationalVelConstraint(HOLD_MAX_VEL)
            ));

            currentHoldTrajectory = drive.actionBuilder(currentPose)
                    .strafeToLinearHeading(
                            new Vector2d(targetHoldPose.position.x, targetHoldPose.position.y),
                            targetHoldPose.heading.toDouble(),
                            holdVelConstraint
                    )
                    .build();

            // Add to action queue - Road Runner takes over from here
            runningActions.add(currentHoldTrajectory);
        }
    }

    /**
     * Enhanced navigation controls with position hold functionality.
     */
    private void handleNavigationControls() {
        // Save current position
        if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
            savedPose = drive.localizer.getPose();
            telemetry.addLine("✓ Position saved!");
        }

        // Navigate to saved position with smooth trajectory
        if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
            if (savedPose == null) {
                telemetry.addLine("⚠ No saved position!");
                return;
            }

            if (driveState != DriveState.MANUAL) {
                telemetry.addLine("⚠ Already in auto mode!");
                return;
            }

            startTrajectoryToSavedPose();
        }

        // Hold current position - enter continuous hold mode at current location
        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
            if (driveState != DriveState.MANUAL) {
                telemetry.addLine("⚠ Already in auto mode!");
                return;
            }

            targetHoldPose = drive.localizer.getPose();
            driveState = DriveState.HOLD_POSITION;
            lastHoldUpdateTime = 0; // Force immediate trajectory generation
            currentHoldTrajectory = null;

            telemetry.addLine("🔒 Holding current position");
            telemetry.addLine("   (Using Road Runner PID)");
            telemetry.addLine("   Try pushing the robot!");
        }

        // Hold saved position - enter hold mode at saved location
        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
            if (savedPose == null) {
                telemetry.addLine("⚠ No saved position!");
                return;
            }

            if (driveState != DriveState.MANUAL) {
                telemetry.addLine("⚠ Already in auto mode!");
                return;
            }

            targetHoldPose = savedPose;
            driveState = DriveState.HOLD_POSITION;
            lastHoldUpdateTime = 0; // Force immediate trajectory generation
            currentHoldTrajectory = null;

            telemetry.addLine("🔒 Holding saved position");
            telemetry.addLine("   (Using Road Runner PID)");
            telemetry.addLine("   Robot will navigate and hold!");
        }
    }

    /**
     * Starts smooth trajectory navigation to saved position.
     * Upon completion, automatically transitions to holding that position.
     */
    private void startTrajectoryToSavedPose() {
        VelConstraint velConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(AUTO_MAX_VEL)
        ));

        currentTrajectory = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(
                        new Vector2d(savedPose.position.x, savedPose.position.y),
                        savedPose.heading.toDouble(),
                        velConstraint
                )
                .build();

        runningActions.add(currentTrajectory);
        driveState = DriveState.AUTO_RETURN;

        telemetry.addLine("→ Navigating to saved position...");
        telemetry.addLine("   (Road Runner trajectory)");
    }

    private void cancelTrajectory() {
        if (currentTrajectory != null) {
            runningActions.remove(currentTrajectory);
            currentTrajectory = null;
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        driveState = DriveState.MANUAL;
        targetHoldPose = null;
        currentHoldTrajectory = null;
    }

    private void cancelHoldMode() {
        if (currentHoldTrajectory != null) {
            runningActions.remove(currentHoldTrajectory);
            currentHoldTrajectory = null;
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        driveState = DriveState.MANUAL;
        targetHoldPose = null;
    }



    private void handleShooterControls() {

        //TODO ------------------------------ INTAKE & STOP ------------------------------




        //TODO ---------------------------- INTAKE REVERSE-----------------------------------------------


    }




    private void runQueuedActions() {
        if (runningActions.isEmpty()) {
            return;
        }

        TelemetryPacket packet = new TelemetryPacket();
        List<Action> remainingActions = new ArrayList<>();

        for (Action action : runningActions) {
            if (action.run(packet)) {
                remainingActions.add(action);
            }
        }

        runningActions.clear();
        runningActions.addAll(remainingActions);
    }

    /**
     * Enhanced telemetry showing Road Runner-based position hold information.
     */
    private void updateTelemetry() {
        // Drive mode with visual indicators
        String modeSymbol;
        String modeText;

        switch (driveState) {
            case MANUAL:
                modeSymbol = "🎮";
                modeText = "MANUAL";
                break;
            case AUTO_RETURN:
                modeSymbol = "🤖";
                modeText = "AUTO-RETURN (RR Trajectory)";
                break;
            case HOLD_POSITION:
                modeSymbol = "🔒";
                modeText = "HOLDING (RR Micro-Trajectories)";
                break;
            default:
                modeSymbol = "❓";
                modeText = "UNKNOWN";
        }

        telemetry.addData(modeSymbol + " Drive Mode", modeText);

        if (powerScale == PRECISION_MODE_SCALE && driveState == DriveState.MANUAL) {
            telemetry.addData("⚡ Power", "PRECISION");
        }

        // Position hold feedback
        if (driveState == DriveState.HOLD_POSITION && targetHoldPose != null) {
            telemetry.addLine();
            telemetry.addData("📍 Position Error", "%.2f inches", currentPositionError);
            telemetry.addData("🧭 Heading Error", "%.1f°", currentHeadingError);

            if (currentPositionError < HOLD_POSITION_TOLERANCE
                    && Math.abs(currentHeadingError) < HOLD_HEADING_TOLERANCE) {
                telemetry.addLine("✓ Within tolerance");
            } else {
                telemetry.addLine("⚡ Road Runner correcting...");
            }

            telemetry.addData("Hold Traj Active", currentHoldTrajectory != null &&
                    runningActions.contains(currentHoldTrajectory));
        }

        telemetry.addLine();

        // Shooter status
        telemetry.addData("🎯 Shooter", shooterEnabled ? "ACTIVE" : "OFF");
        if (shooterEnabled) {
//            telemetry.addData("  Left", "%.0f RPM", robot.leftShooter.getVelocity());
//            telemetry.addData("  Mid", "%.0f RPM", robot.midShooter.getVelocity());
//            telemetry.addData("  Right", "%.0f RPM", robot.rightShooter.getVelocity());
        }

        telemetry.addLine();

        // Position information
        Pose2d currentPose = drive.localizer.getPose();
        telemetry.addData("📍 Current",
                "(%.1f, %.1f) @ %.0f°",
                currentPose.position.x,
                currentPose.position.y,
                Math.toDegrees(currentPose.heading.toDouble())
        );

        if (savedPose != null) {
            telemetry.addData("💾 Saved",
                    "(%.1f, %.1f) @ %.0f°",
                    savedPose.position.x,
                    savedPose.position.y,
                    Math.toDegrees(savedPose.heading.toDouble())
            );
        } else {
            telemetry.addData("💾 Saved", "None");
        }

        if (targetHoldPose != null && driveState == DriveState.HOLD_POSITION) {
            telemetry.addData("🎯 Holding",
                    "(%.1f, %.1f) @ %.0f°",
                    targetHoldPose.position.x,
                    targetHoldPose.position.y,
                    Math.toDegrees(targetHoldPose.heading.toDouble())
            );
        }

        telemetry.update();
    }

    // ========== UTILITY METHODS ==========

    private static double normalizeRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}