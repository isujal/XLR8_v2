//package org.firstinspires.ftc.teamcode.prototyping;
//
//import static org.firstinspires.ftc.teamcode.RobotHardware.Global.*;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.MinVelConstraint;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.VelConstraint;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.RobotHardware.Global;
//import org.firstinspires.ftc.teamcode.RobotHardware.RobotHarware;
//import org.firstinspires.ftc.teamcode.Sequences.INITSeq;
//import org.firstinspires.ftc.teamcode.Sequences.ServoSeq;
//import org.firstinspires.ftc.teamcode.Sequences.ShooterSeq;
//import org.firstinspires.ftc.teamcode.SubSystem.Hood;
//import org.firstinspires.ftc.teamcode.SubSystem.INTAKE;
//import org.firstinspires.ftc.teamcode.SubSystem.PTO;
//import org.firstinspires.ftc.teamcode.SubSystem.SHOOTER;
//
//import java.util.ArrayList;
//import java.util.Arrays;
//import java.util.List;
//
//@TeleOp(name = "Non-Blocking Smooth TeleOp")
//@Config
//public class TeleOpNonBlockingTrajectory extends LinearOpMode {
//
//    // ========== TRAJECTORY TUNING PARAMETERS ==========
//    // These control how smoothly and quickly the robot moves during auto-return
//    // Adjust these from FTC Dashboard to tune performance without redeploying
//    public static double AUTO_MAX_VEL = 80; // inches per second - lower = smoother but slower
//    public static double AUTO_MAX_ACCEL = 40.0; // inches per second^2 - lower = gentler starts/stops
//    public static double AUTO_MAX_ANG_VEL = Math.PI; // radians per second - turn speed
//
//    // ========== SHOOTER CONFIGURATION ==========
//    public static int SHOOTER_RPM_BUFFER = 50; // RPM tolerance below target
//
//    // ========== DRIVE CONFIGURATION ==========
//    public static double PRECISION_MODE_SCALE = 0.3; // Power scale when trigger held
//    public static double NORMAL_POWER_SCALE = 1.0; // Full power for normal driving
//    public static double JOYSTICK_CANCEL_THRESHOLD = 0.3; // How much joystick movement cancels auto
//
//    // ========== HARDWARE AND SUBSYSTEMS ==========
//    private final RobotHarware robot = RobotHarware.getInstance();
//    private INTAKE intake;
//    private PTO pto;
//    private SHOOTER shooter;
//    private Hood hood;
//    private MecanumDrive mecanumDrive;
//
//    // ========== GAMEPAD STATE FOR EDGE DETECTION ==========
//    private final Gamepad currentGamepad1 = new Gamepad();
//    private final Gamepad previousGamepad1 = new Gamepad();
//
//    // ========== CONTROL STATE ==========
//    private boolean shooterEnabled = false;
//    private double powerScale = NORMAL_POWER_SCALE;
//
//    // ========== DRIVE STATE MACHINE ==========
//    // This enum tracks whether the driver or an autonomous action controls the drive
//    private enum DriveState {
//        MANUAL,      // Driver has control - joysticks command the motors
//        AUTO_RETURN  // Trajectory has control - joysticks are disabled except to cancel
//    }
//
//    private DriveState driveState = DriveState.MANUAL;
//    private Pose2d savedPose = null; // The position we'll return to when dpad-right is pressed
//
//    // ========== ACTION QUEUE ==========
//    // All concurrent actions (trajectories, intake, shooter, etc.) run through this queue
//    // This allows multiple subsystems to operate simultaneously without blocking
//    private final List<Action> runningActions = new ArrayList<>();
//
//    // ========== TRAJECTORY TRACKING ==========
//    // We need to track the specific trajectory action so we can detect when it completes
//    // This is separate from other actions like intake or shooter commands
//    private Action currentTrajectory = null;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        // ========== INITIALIZATION PHASE ==========
//        telemetry.addLine("Initializing hardware...");
//        telemetry.update();
//
//        robot.init(hardwareMap, telemetry);
//        intake = new INTAKE(robot);
//        pto = new PTO(robot);
//        shooter = new SHOOTER(robot);
//        hood = new Hood(robot);
//
//        // Configure PID for each shooter motor
//        robot.leftShooter.setVelocityPIDFCoefficients(near_left_P, near_left_I, near_left_D, near_left_F);
//        robot.midShooter.setVelocityPIDFCoefficients(near_mid_P, near_mid_I, near_mid_D, near_mid_F);
//        robot.rightShooter.setVelocityPIDFCoefficients(near_right_P, near_right_I, near_right_D, near_right_F);
//
//        // Create mecanum drive ONCE with origin at starting position
//        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//
//        telemetry.addLine("✓ Hardware initialized");
//        telemetry.update();
//
//        // ========== PRE-START LOOP ==========
//        while (opModeInInit()) {
//            updateGamepadState();
//            runQueuedActions();
//
//            if (runningActions.isEmpty()) {
//                runningActions.add(INITSeq.INITaction(intake, pto, shooter, hood));
//            }
//
//            telemetry.addLine("✓ Robot Ready");
//            telemetry.addLine("");
//            telemetry.addLine("Controls:");
//            telemetry.addLine("  Dpad-Up: Save current position");
//            telemetry.addLine("  Dpad-Right: Return to saved position");
//            telemetry.addLine("  Move joystick during auto to cancel");
//            telemetry.update();
//        }
//
//        waitForStart();
//
//        // ========== MAIN CONTROL LOOP ==========
//        // CRITICAL: The order of operations in this loop is very important!
//        // We must handle state changes BEFORE processing actions to prevent conflicts
//        while (opModeIsActive()) {
//
//            // STEP 1: Update inputs and odometry
//            // This must happen first so all subsequent code uses current data
//            updateGamepadState();
//            mecanumDrive.updatePoseEstimate();
//
//            // STEP 2: Handle navigation button presses BEFORE processing actions
//            // This ensures that when we queue a trajectory and change driveState,
//            // the state change takes effect before handleDriveControls() runs
//            handleNavigationControls();
//
//            // STEP 3: Process all queued actions
//            // At this point, if we just queued a trajectory, driveState is already AUTO_RETURN
//            // The trajectory action will command the motors during this step
//            runQueuedActions();
//
//            // STEP 4: Handle drive controls
//            // This method checks driveState and only commands motors if we're in MANUAL mode
//            // If we're in AUTO_RETURN, this method does nothing to the motors,
//            // leaving them under the trajectory's control from step 3
//            handleDriveControls();
//
//            // STEP 5: Handle all other controls
//            // These don't interfere with driving so order doesn't matter
//            handleResetControl();
//            handleIntakeControls();
//            handleShooterControls();
//            handleLaunchControls();
//
//            // STEP 6: Update telemetry last
//            updateTelemetry();
//        }
//    }
//
//    /**
//     * Updates gamepad state for edge detection.
//     * By keeping both current and previous state, we can detect the exact moment
//     * a button transitions from unpressed to pressed, preventing repeated triggers.
//     */
//    private void updateGamepadState() {
//        previousGamepad1.copy(currentGamepad1);
//        currentGamepad1.copy(gamepad1);
//    }
//
//    /**
//     * Handles robot driving based on current drive state.
//     *
//     * CRITICAL IMPLEMENTATION DETAIL:
//     * When in MANUAL mode, this method calls driveFieldCentric() to command motors.
//     * When in AUTO_RETURN mode, this method does NOT touch the motors at all.
//     *
//     * This separation is what allows non-blocking trajectories to work. The trajectory
//     * commands the motors during runQueuedActions(), and then this method deliberately
//     * leaves those commands alone by not calling driveFieldCentric().
//     *
//     * If this method called driveFieldCentric() in AUTO_RETURN mode (even with zero
//     * joystick values), it would overwrite the trajectory's motor commands and the
//     * robot wouldn't move properly.
//     */
//    private void handleDriveControls() {
//        if (driveState == DriveState.MANUAL) {
//            // Driver has full control - read joysticks and command motors
//
//            // Precision mode for fine control during scoring
//            if (gamepad1.left_trigger > 0.2) {
//                powerScale = PRECISION_MODE_SCALE;
//            } else {
//                powerScale = NORMAL_POWER_SCALE;
//            }
//
//            mecanumDrive.driveFieldCentric(
//                    -gamepad1.left_stick_x * powerScale,
//                    -gamepad1.left_stick_y * powerScale,
//                    gamepad1.right_stick_x * powerScale,
//                    mecanumDrive.localizer.getPose().heading.toDouble()
//            );
//
//            if (gamepad1.right_stick_button) {
//                Pose2d currentPose = mecanumDrive.localizer.getPose();
//                mecanumDrive.localizer.setPose(
//                        new Pose2d(currentPose.position.x, currentPose.position.y, 0)
//                );
//            }
//
//        } else if (driveState == DriveState.AUTO_RETURN) {
//            // Trajectory has control - we do NOT call driveFieldCentric() here
//            // The trajectory is commanding the motors through runQueuedActions()
//            // We only check if the driver wants to cancel or if trajectory completed
//
//            // Check for cancellation via joystick movement
//            double joystickMagnitude = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
//            boolean driverWantsControl = joystickMagnitude > JOYSTICK_CANCEL_THRESHOLD
//                    || Math.abs(gamepad1.right_stick_x) > JOYSTICK_CANCEL_THRESHOLD;
//
//            if (driverWantsControl) {
//                // Driver moved joystick - cancel trajectory and return control
//                cancelTrajectory();
//                telemetry.addLine("✗ Auto-return cancelled by driver");
//            }
//
//            // Check if trajectory completed naturally
//            // If the trajectory action is no longer in our running actions list,
//            // it has finished executing and returned false from its run() method
//            if (currentTrajectory != null && !runningActions.contains(currentTrajectory)) {
//                // Trajectory completed successfully
//                currentTrajectory = null;
//                driveState = DriveState.MANUAL;
//                telemetry.addLine("✓ Arrived at saved position");
//            }
//        }
//    }
//
//    /**
//     * Handles position saving and autonomous return triggering.
//     *
//     * This method is called BEFORE runQueuedActions() in the main loop,
//     * which is critical. When we press dpad-right, we queue the trajectory
//     * and immediately set driveState = AUTO_RETURN. Then when the main loop
//     * continues to runQueuedActions(), the trajectory runs and commands the motors.
//     * Then when we reach handleDriveControls(), it sees AUTO_RETURN and doesn't
//     * interfere with the motors. This ordering prevents conflicts.
//     */
//    private void handleNavigationControls() {
//        // Save current position when dpad-up is pressed
//        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
//            savedPose = mecanumDrive.localizer.getPose();
//            telemetry.addLine("✓ Position saved!");
//            telemetry.addLine(String.format("  (%.1f, %.1f) @ %.0f°",
//                    savedPose.position.x,
//                    savedPose.position.y,
//                    Math.toDegrees(savedPose.heading.toDouble())));
//        }
//
//        // Return to saved position when dpad-right is pressed
//        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
//            if (savedPose == null) {
//                telemetry.addLine("⚠ No saved position! Press dpad-up first.");
//                return;
//            }
//
//            if (driveState != DriveState.MANUAL) {
//                telemetry.addLine("⚠ Already navigating!");
//                return;
//            }
//
//            // Build a smooth RoadRunner trajectory using motion profiling
//            // This creates a path with controlled acceleration and velocity
//            startTrajectoryToSavedPose();
//        }
//    }
//
//    /**
//     * Creates and queues a smooth RoadRunner trajectory to the saved position.
//     *
//     * This is the key method that makes non-blocking trajectories work. We create
//     * the trajectory action using RoadRunner's builder, add it to our action queue,
//     * and set the drive state to AUTO_RETURN. The trajectory will execute incrementally
//     * through runQueuedActions() while the main loop continues running.
//     *
//     * The velocity and acceleration constraints control how smooth the motion is.
//     * Lower values create gentler motion that's less likely to slip on the field.
//     */
//    private void startTrajectoryToSavedPose() {
//        // Create velocity constraint for smooth motion
//        VelConstraint velConstraint = new MinVelConstraint(Arrays.asList(
//                new TranslationalVelConstraint(AUTO_MAX_VEL)
//        ));
//
//        // Build the trajectory from current position to saved position
//        // strafeToLinearHeading creates a smooth path that simultaneously
//        // moves to the target position and rotates to the target heading
//        currentTrajectory = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
//                .strafeToLinearHeading(
//                        new Vector2d(savedPose.position.x, savedPose.position.y),
//                        savedPose.heading.toDouble(),
//                        velConstraint
//                )
//                .build();
//
//        // Add trajectory to action queue (non-blocking!)
//        // This is the critical step - we're NOT using Actions.runBlocking()
//        // Instead, we add it to our queue where it will execute incrementally
//        runningActions.add(currentTrajectory);
//
//        // Switch drive state IMMEDIATELY before the next loop iteration
//        // This ensures handleDriveControls() sees AUTO_RETURN and doesn't interfere
//        driveState = DriveState.AUTO_RETURN;
//
//        telemetry.addLine("→ Navigating to saved position...");
//        telemetry.addLine("  Move joystick to cancel");
//    }
//
//    /**
//     * Cancels the current trajectory and returns control to driver.
//     * This immediately stops the robot and clears the trajectory from the queue.
//     */
//    private void cancelTrajectory() {
//        // Remove trajectory from action queue
//        if (currentTrajectory != null) {
//            runningActions.remove(currentTrajectory);
//            currentTrajectory = null;
//        }
//
//        // Explicitly stop all motors
//        mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
//
//        // Return to manual control
//        driveState = DriveState.MANUAL;
//    }
//
//    /**
//     * Handles reset to initialization state.
//     */
//    private void handleResetControl() {
//        if (currentGamepad1.back) {
//            runningActions.add(INITSeq.INITaction(intake, pto, shooter, hood));
//        }
//    }
//
//    /**
//     * Handles intake motor controls using non-blocking queued actions.
//     */
//    private void handleIntakeControls() {
//        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
//            runningActions.add(intake.IntakeMotorCommand(INTAKE.IntakeMotor.IN));
//        }
//
//        if (currentGamepad1.left_bumper) {
//            runningActions.add(intake.IntakeMotorCommand(INTAKE.IntakeMotor.INIT));
//        }
//    }
//
//    /**
//     * Handles shooter motor controls using non-blocking queued actions.
//     */
//    private void handleShooterControls() {
//        if (currentGamepad1.start && !previousGamepad1.start) {
//            shooterEnabled = !shooterEnabled;
//        }
//
//        if (shooterEnabled) {
//            runningActions.add(ShooterSeq.ShooterNear_Same_RPM(shooter));
//        } else {
//            runningActions.add(ShooterSeq.Shooterstop(shooter));
//        }
//    }
//
//    /**
//     * Handles launching mechanisms.
//     * Only allows firing when shooter motors are at target speed.
//     */
//    private void handleLaunchControls() {
//        if (currentGamepad1.x && isShooterReady(robot.leftShooter, Global.leftShooter_near)) {
//            runningActions.add(ServoSeq.leftServo(intake));
//        }
//
//        if (currentGamepad1.y && isShooterReady(robot.midShooter, Global.midShooter_near)) {
//            runningActions.add(ServoSeq.midServo(intake));
//        }
//
//        if (currentGamepad1.b && isShooterReady(robot.rightShooter, Global.rightShooter_near)) {
//            runningActions.add(ServoSeq.rightServo(intake));
//        }
//
//        if (currentGamepad1.a && areAllShootersReady()) {
//            runningActions.add(new SequentialAction(
//                    ServoSeq.ServoAction(intake),
//                    new SleepAction(1.0),
//                    ServoSeq.ServoINIT(intake)
//            ));
//        }
//    }
//
//    /**
//     * Checks if a shooter motor is at target speed within tolerance.
//     */
//    private boolean isShooterReady(com.qualcomm.robotcore.hardware.DcMotorEx motor,
//                                   double targetRPM) {
//        return motor.getVelocity() >= (targetRPM - SHOOTER_RPM_BUFFER);
//    }
//
//    /**
//     * Checks if all shooter motors are ready to launch.
//     */
//    private boolean areAllShootersReady() {
//        return robot.leftShooter.getVelocity() >= (Global.common_Near_RPM - SHOOTER_RPM_BUFFER)
//                && robot.midShooter.getVelocity() >= (Global.common_Near_RPM - SHOOTER_RPM_BUFFER)
//                && robot.rightShooter.getVelocity() >= (Global.common_Near_RPM - SHOOTER_RPM_BUFFER);
//    }
//
//    /**
//     * Processes all queued actions incrementally.
//     *
//     * This is where the magic happens for non-blocking trajectories. Each action
//     * (including the trajectory) gets to run one small step per loop iteration.
//     * The trajectory calculates the next point on its path and commands the motors.
//     * Actions that return true stay in the queue for next iteration.
//     * Actions that return false are removed (they've completed).
//     */
//    private void runQueuedActions() {
//        if (runningActions.isEmpty()) {
//            return;
//        }
//
//        TelemetryPacket packet = new TelemetryPacket();
//        List<Action> remainingActions = new ArrayList<>();
//
//        for (Action action : runningActions) {
//            // Run the action for one iteration
//            // For a trajectory, this calculates the next position and commands motors
//            // The action returns true if it needs to continue, false if complete
//            boolean stillRunning = action.run(packet);
//
//            if (stillRunning) {
//                remainingActions.add(action);
//            }
//        }
//
//        runningActions.clear();
//        runningActions.addAll(remainingActions);
//    }
//
//    /**
//     * Updates driver station telemetry with useful real-time information.
//     */
//    private void updateTelemetry() {
//        // Drive mode indicator
//        String modeSymbol = driveState == DriveState.AUTO_RETURN ? "🤖" : "🎮";
//        String modeText = driveState == DriveState.AUTO_RETURN ? "AUTO-RETURN" : "MANUAL";
//        telemetry.addData(modeSymbol + " Drive Mode", modeText);
//
//        if (powerScale == PRECISION_MODE_SCALE && driveState == DriveState.MANUAL) {
//            telemetry.addData("⚡ Power", "PRECISION MODE");
//        }
//
//        telemetry.addLine();
//
//        // Shooter status
//        telemetry.addData("🎯 Shooter", shooterEnabled ? "ACTIVE" : "OFF");
//        if (shooterEnabled) {
//            telemetry.addData("  Left RPM", "%.0f / %.0f",
//                    robot.leftShooter.getVelocity(), Global.leftShooter_near);
//            telemetry.addData("  Mid RPM", "%.0f / %.0f",
//                    robot.midShooter.getVelocity(), Global.midShooter_near);
//            telemetry.addData("  Right RPM", "%.0f / %.0f",
//                    robot.rightShooter.getVelocity(), Global.rightShooter_near);
//        }
//
//        telemetry.addLine();
//
//        // Current position
//        Pose2d currentPose = mecanumDrive.localizer.getPose();
//        telemetry.addData("📍 Current Position",
//                "(%.1f, %.1f) @ %.0f°",
//                currentPose.position.x,
//                currentPose.position.y,
//                Math.toDegrees(currentPose.heading.toDouble())
//        );
//
//
//
//        // Saved position
//        if (savedPose != null) {
//            telemetry.addData(" Saved Position",
//                    "(%.1f, %.1f) @ %.0f°",
//                    savedPose.position.x,
//                    savedPose.position.y,
//                    Math.toDegrees(savedPose.heading.toDouble())
//            );
//
//            if (driveState == DriveState.AUTO_RETURN) {
//                double dx = savedPose.position.x - currentPose.position.x;
//                double dy = savedPose.position.y - currentPose.position.y;
//                double distance = Math.hypot(dx, dy);
//                telemetry.addData("  Distance", "%.1f inches", distance);
//            }
//        } else {
//            telemetry.addData("💾 Saved Position", "None - Press dpad-up");
//        }
//
//        telemetry.update();
//    }
//}