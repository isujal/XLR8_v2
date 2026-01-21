package org.firstinspires.ftc.teamcode.prototyping;


import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Utils.utilities.ServoMapper;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.sequences.teleOp.FrontIntakeSeq;
//import org.firstinspires.ftc.teamcode.sequences.teleOp.FrontShootSeq;
import org.firstinspires.ftc.teamcode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;
import java.util.List;

@Config
//@TeleOp(name ="TeleOpTurretTest")

public class TeleopTurretTest extends LinearOpMode {
    private static RobotHardware robot=RobotHardware.getInstance();
    public List<LynxModule> allHubs;

    private MecanumDrive drive;
    public double botHeading;

    private FtcDashboard dashboard;

    //TODO ---------------------------HOOD Stages

    //TODO ---------------------------COLOR Sensor

    public NormalizedRGBA clrfrontIntakergba;
    public float[] clrfrontIntakehsv;
    public double clrfrontIntakedistance;

    public NormalizedRGBA clrbackIntakergba;
    public float[] clrbackIntakehsv;
    public double clrbackIntakedistance;

    //TODO ---------------------------Flags

    public static boolean intakeIN =false;
    public static int storetoBack =0;
    public static int storetoFront =0;
    public static int stored3rdtoFront =0;
    public static int stored3rdtoBack =0;

    public static boolean launchFar=false;
    public static boolean launchNear=false;

    public static boolean shootfromFront=false;
    public static boolean shootfromBack=false;

    public static boolean readytoShootFar=false;
    public static boolean readytoShootNear=false;
    public static boolean shooterON=false;
    public static boolean NOballstored = false;

    public static int buffer=80;

    //TODO ---------------------------SHOOTER PID
    public static double P=100;//1.8 //300
    public static double I=0;//0.18
    public static double D=0;//0
    public static double F=12;//18 //13

    //TODO -------------------Turret Constants--------

    public static boolean turretActuate =false;
    public static double c;
    public static double target = 0;
    public static double error ;
    public static double integral;
    public static double previous_error;
    public double derivative;
    public static double pid;
    public double a;

    public static double kp = 0.012/*0.01*/, ki = 0, kd = 0.015; /*0.02;*/

    public static double yaw = 0;
//    public static double Yaw = 0;
    private double totalAngle;
    private double angle;
    private double prevAngle;
    private double actualTurretPose;
    private double currentTurretDeg;
    private double desiredTurretDeg;


    /// //////////////////////////////////////////////
    /// TODO TURRET CAMERA TRACKING
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // true = use webcam, false = use RC phone camera
    private static final boolean USE_WEBCAM = true;

    // --- state variables ---
    private AprilTagDetection tag = null;
    private int ID = -1;
    private double x = 0.0, y = 0.0, z = 0.0;
    private double Pitch = 0.0, Yaw = 0.0, Roll = 0.0;
    private boolean AprilTagDetected = false;
    private int tagID = -1;
    private int patternID = -1;

    // --- turret limits / smoothing config ---
    private static final double TURRET_LIMIT_DEG = 100.0;
    private static final boolean ENABLE_SMOOTHING = true;
    private static final double SMOOTH_ALPHA = 0.25; // 0..1, lower = smoother (slower)
    private double smoothedAngleDeg = 0.0;
    private boolean hasSmoothed = false;

    // --- last known turret angle (persisted when tag lost) ---
    // Start at 0.0 per your request
//    private double lastKnownAngleDeg = 0.0;
    private double lastKnownRelativeDeg = 0.0;   // last camera-relative offset (rawAngleDeg)
    private double lastDesiredTurretDeg = 0.0;
    private Intake intake;
    private Outtake outtake;
    private Feeder feeder;


    ///  //////////////////////////////////////////////
    // encoder->angle calibration (use the same min/max you used in run_turret)
    private static final double MIN_IN_POS = 0.0;      // replace with your real min encoder ticks (e.g. 0)
    private static final double MAX_IN_POS = 27845.0;  // from your example

    // store encoder value at opmode start so start pose becomes 0°
    private int encoderZeroTicks = 0;
    private double encoderZeroDeg = 0.0; // mapped & normalized value at init
    private boolean encoderZeroInitialized = false;

    // sign convention: set to +1 if your mapping already gives left positive,
// set to -1 if mapping currently gives left negative and you want to flip
    private static final int LEFT_POSITIVE_SIGN = +1; // change to -1 only if needed

     private int detectionCount = 0; // add this field at class level
 private static final int DETECTION_CONFIRM_FRAMES = 2; // require 2 consecutive frames before accepting new target
 private static final double MAX_DEG_RATE_PER_FRAME = 8.0;



    public static List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
//        RobotHardware robot = RobotHardware.getInstance();
        robot.init(hardwareMap,telemetry);
        intake = new Intake(robot);
        outtake = new Outtake(robot);
        feeder = new Feeder(robot);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();



        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        //TODO ---------------------------HOOD Stages

        initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeInInit()) {
            Globals.currentTurretState=Globals.turretInit;

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }


            runningActions = updateAction();


            /// ////////////////////////////////////
            // do this once during init
            encoderZeroTicks = robot.turretEncoder.getCurrentPosition();
            double mapped = map(encoderZeroTicks, MIN_IN_POS, MAX_IN_POS, 0.0, 360.0);
            encoderZeroDeg = LEFT_POSITIVE_SIGN * normalizeAngleDegrees(mapped);
            encoderZeroInitialized = true;
            telemetry.addData("encoderZeroTicks", encoderZeroTicks);
            telemetry.addData("encoderZeroDeg (signed)", encoderZeroDeg);
//            telemetry.update();

            actualTurretPose = robot.turretEncoder.getCurrentPosition();

//            angle = getContinuousIMU(Globals.currentTurret);
//            run_turret(angle, 0, 30584, actualTurretPose, telemetry);

            //TODO ----------------BOOLEAN----------------



            //TODO ----------------Flags----------------

            launchFar=false;
            launchNear=false;

            shootfromFront=true;
            shootfromBack=false;

            stored3rdtoFront =0;
            stored3rdtoBack =0;

            readytoShootFar=false;
            readytoShootNear=false;

            shooterON=false;

            NOballstored=false;

            turretActuate =false;

            telemetry.addLine();
            printTelemetry();
//            printFlags();
//            printGlobalFlags();
//            printDriveTelemetry();
//            printCurrentTelemetry();
//            printVelocityTelemetry();
//            printBeamTelemetry();
//            printColorSensor();
            telemetry.addLine();
            telemetry.update();
        }


        robot.resetEncoder();

        waitForStart();

        while (opModeIsActive()){

//            YawPitchRollAngles orientation = drive.lazyImu.get().getRobotYawPitchRollAngles();
//
//            Yaw = orientation.getYaw(AngleUnit.DEGREES);
//            Globals.turretFARLEFT_IMU=Globals.turretFARLEFT-Yaw;
//            Globals.turretNEARLEFT_IMU=Globals.turretNEARLEFT-Yaw;


            // Cap both values between -100 and +100
//            Globals.turretFARLEFT_IMU = clamp(Globals.turretFARLEFT_IMU, -100, 100);
//            Globals.turretNEARLEFT_IMU = clamp(Globals.turretNEARLEFT_IMU, -100, 100);

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            runningActions = updateAction();

            botHeading = drive.localizer.getPose().heading.toDouble();


            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            //TODO ---------------------------DRIVE



            drive.updatePoseEstimate();

            if (gamepad1.left_stick_button){
                drive.lazyImu.get().resetYaw();
            }


            //TODO ----------------------TURRET---------------------
            actualTurretPose = robot.turretEncoder.getCurrentPosition();

            currentTurretDeg = getTurretAngleFromEncoder(actualTurretPose);

            aprilTagDetection();

            run_turret(desiredTurretDeg, MIN_IN_POS, MAX_IN_POS, actualTurretPose, telemetry);



            if (gamepad1.dpad_down){
                turretActuate=true;
            }
            if(turretActuate){
                runningActions.add(
                        new SequentialAction(
                                Outtake.TurretCommand(Outtake.TurretState.IMU_TRACK))
                );
            }


//            printTelemetry();
//            printFlags();
//            printGlobalFlags();
//            printDriveTelemetry();
//            printCurrentTelemetry();
//            printVelocityTelemetry();
//            printBeamTelemetry();
//            printColorSensor();
            telemetry.addLine();
            telemetry.update();
        }
    }

    public void printTelemetry(){
        YawPitchRollAngles orientation = drive.lazyImu.get().getRobotYawPitchRollAngles();

        telemetry.addData("Yaw / Pitch / Roll", "%.1f / %.1f / %.1f",
                orientation.getYaw(AngleUnit.DEGREES),
                orientation.getPitch(AngleUnit.DEGREES),
                orientation.getRoll(AngleUnit.DEGREES));

        telemetry.addData("BotHeading",botHeading);

        telemetry.addData("NOballstored",NOballstored);
        telemetry.addData("shooterON",shooterON);
        telemetry.addData("PID",pid);
        telemetry.addData("Error",error);
        telemetry.addData("Previous err",previous_error);
        telemetry.addData("Turret Enc",actualTurretPose);

        telemetry.addData("Hood Pose",robot.hood.getPosition());

        telemetry.addData("LEFT BACK POSE",drive.leftBack.getCurrentPosition());
        telemetry.addData("RIGHT BACK POSE",drive.rightBack.getCurrentPosition());
        telemetry.addLine();

    }
    public void printFlags(){
        telemetry.addData("IntakeIN",intakeIN);
        telemetry.addData("storetoFront",storetoFront);
        telemetry.addData("storetoBack",storetoBack);
        telemetry.addData("stored3rdtoFront",stored3rdtoFront);
        telemetry.addData("stored3rdtoBack",stored3rdtoBack);
        telemetry.addLine();
    }

    public void printDriveTelemetry(){
        telemetry.addData("Right Back Current : ", drive.rightBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Left Back Current : ", drive.leftBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Front Current : ", drive.rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Left Front Current : ", drive.leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addLine();

    }



    public void printColorSensor(){
        telemetry.addData("Front CLR Distance",clrfrontIntakedistance );

        telemetry.addLine()
                .addData("HUE","%.3f",clrfrontIntakehsv[0])
                .addData("Saturation","%.3f", clrfrontIntakehsv[1])
                .addData("Value","%.3f", clrfrontIntakehsv[2]);
        telemetry.addLine()
                .addData("RED","%.3f",clrfrontIntakergba.red)
                .addData("GREEN","%.3f", clrfrontIntakergba.green)
                .addData("BLUE","%.3f", clrfrontIntakergba.blue);

        telemetry.addData("Back CLR Distance",clrbackIntakedistance );

        telemetry.addLine()
                .addData("HUE","%.3f",clrbackIntakehsv[0])
                .addData("Saturation","%.3f", clrbackIntakehsv[1])
                .addData("Value","%.3f", clrbackIntakehsv[2]);
        telemetry.addLine()
                .addData("RED","%.3f",clrbackIntakergba.red)
                .addData("GREEN","%.3f", clrbackIntakergba.green)
                .addData("BLUE","%.3f", clrbackIntakergba.blue);
        telemetry.addLine();

    }
    public float[] rgbToHsv(float rNorm, float gNorm, float bNorm) {
        float[] hsv = new float[3];

        float max = Math.max(rNorm, Math.max(gNorm, bNorm));
        float min = Math.min(rNorm, Math.min(gNorm, bNorm));
        float delta = max - min;
        // Value
        hsv[2] = max;

        // Saturation
        hsv[1] = max == 0 ? 0 : delta / max;

        // Hue
        if (delta == 0) {
            hsv[0] = 0;
        } else {
            if (max == rNorm) {
                hsv[0] = (60 * ((gNorm - bNorm) / delta) + 360) % 360;
            } else if (max == gNorm) {
                hsv[0] = (60 * ((bNorm - rNorm) / delta) + 120) % 360;
            } else if (max == bNorm) {
                hsv[0] = (60 * ((rNorm - gNorm) / delta) + 240) % 360;
            }
        }
        return hsv;
    }

    public static List<Action> updateAction(){
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        List<Action> RemovableActions = new ArrayList<>();

        for (Action action : runningActions) {

            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        return newActions;
    }
    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    public double  getContinuousIMU(double currentAngle) {
        double delta = currentAngle - prevAngle;

        // Handle wrap-around
        if (delta > 180) {
            delta -= 360;
        } else if (delta < -180) {
            delta += 360;
        }

        totalAngle += delta;
        prevAngle = currentAngle;

        return totalAngle;
    }

    /**
     * PID turret controller: accepts desired absolute turret angle in degrees.
     *
     * @param desiredTurretDeg  absolute target heading (degrees). Convention: 0 = start pose.
     *                          Use LEFT positive, RIGHT negative (same as currentTurretDeg).
     * @param min_in_pos        encoder ticks that map to 0 degrees
     * @param max_in_pos        encoder ticks that map to 360 degrees
     * @param pose              current encoder ticks (robot.turretEncoder.getCurrentPosition())
     * @param telemetry
     * @return pid output (power applied to servos)
     */
    public static double run_turret(double desiredTurretDeg,
                                    double min_in_pos, double max_in_pos,
                                    double pose,
                                    Telemetry telemetry) {

        // 1) Convert encoder ticks -> 0..360, then -> -180..+180 signed degrees
        double mapped = map(pose, min_in_pos, max_in_pos, 0.0, 360.0);
        double currentDeg = normalizeAngleDegrees(mapped);

        // If your encoder needs flipping so left is positive, apply:
        // currentDeg = LEFT_POSITIVE_SIGN * currentDeg; // LEFT_POSITIVE_SIGN is +1 or -1

        // 2) Compute shortest angular error (desired - current) in [-180,180]
        double errorDeg = normalizeAngleDegrees(desiredTurretDeg - currentDeg);

        // 3) PID integration (anti-windup)
        integral += errorDeg;
        // clamp integral to avoid windup (tune limit as needed)
        final double INTEGRAL_LIMIT = 1000.0;
        if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
        if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;

        double derivative = errorDeg - previous_error;
        previous_error = errorDeg;

        // 4) PID output (units = power for continuous servo)
        double out = kp * errorDeg + ki * integral + kd * derivative;

        // 5) small static bias to help overcome stiction (your earlier ±0.045)
        final double BIAS = 0.045;
        if (errorDeg > 0) out += BIAS;
        else if (errorDeg < 0) out -= BIAS;

        // 6) clamp power to [-1, 1] (continuous servo safe limits)
        out = Math.max(-1.0, Math.min(1.0, out));

        // 7) apply power to turret servos
        robot.turret1.setPower(-out);
        robot.turret2.setPower(out);

        // 8) bookkeeping + telemetry
        pid = out;
        error = errorDeg;

        telemetry.addData("turret.currentDeg", String.format("%.3f", currentDeg));
        telemetry.addData("turret.desiredDeg", String.format("%.3f", desiredTurretDeg));
        telemetry.addData("turret.errorDeg", String.format("%.3f", errorDeg));
        telemetry.addData("turret.pidOut", String.format("%.3f", out));

        return out;
    }

    public static double run_turret1(double imu, double min_in_pos, double max_in_pos, double pose, Telemetry telemetry){
        pid = 0;

        c = map(pose, min_in_pos, max_in_pos, 0, 360);

        if (imu < 0){
            c = -c;
        }

        double target = imu;

        double derivative = 0;
        if(imu < 0){
            telemetry.addLine("negative heading");
            error = c + target;
//            error = -error;
            integral = integral + error;
            derivative = error - previous_error;

            previous_error = error;
            if (error<0){
                pid = kp*error + kd*derivative + ki*integral-0.045;
            }
            else {
                pid = kp*error + kd*derivative + ki*integral+0.045;

            }
            robot.turret1.setPower(-pid);
            robot.turret2.setPower(pid);
            return pid;

        }
        else {
            telemetry.addLine("positive heading");

            error = target - c;
//            error = -error;
            integral = integral + error;
            derivative = error - previous_error;

            previous_error = error;
            if (error<0){
                pid = kp*error + kd*derivative + ki*integral-0.045;
            }
            else {
                pid = kp*error + kd*derivative + ki*integral+0.045;

            }
            robot.turret1.setPower(-pid);
            robot.turret2.setPower(pid);
            return pid;
        }
    }

    public double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    // normalize angle to [-180, 180)
    public static double normalizeAngleDegrees(double a) {
        a = ((a % 360.0) + 360.0) % 360.0; // 0..360
        if (a >= 180.0) a -= 360.0;        // -180..180
        return a;
    }

    public double getTurretAngleFromEncoder(double poseTicks) {
        // 1) raw mapped angle 0..360
        double mapped = map(poseTicks, MIN_IN_POS, MAX_IN_POS, 0.0, 360.0);

        // 2) normalize to -180..180
        double signed = normalizeAngleDegrees(mapped);

        // 3) apply sign convention: ensure left is positive
        signed = LEFT_POSITIVE_SIGN * signed;

        // 4) subtract the initial-zero offset (so start pose -> 0)
        if (!encoderZeroInitialized) {
            // if not initialized, don't subtract yet — caller should set zero at init
            return signed;
        }
        double currentDeg = signed - encoderZeroDeg;

        // 5) keep currentDeg in -180..180 (after subtraction it could drift)
        currentDeg = normalizeAngleDegrees(currentDeg);

        return currentDeg;
    }


    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                // Example lens intrinsics (adjust if you calibrate your own camera):
                .setLensIntrinsics(445.085, 445.085, 326.262, 235.802)
                // Optional: specify camera position/orientation on the robot
                .setCameraPose(
                        new Position(DistanceUnit.INCH, -2.36, 2.44, 15, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 90, 0)
                )
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }
    // class-level fields you already have (ensure these exist):
// private double smoothedAngleDeg = 0.0; private boolean hasSmoothed = false;
// private double lastKnownRelativeDeg = 0.0; private double lastDesiredTurretDeg = 0.0;
// private int detectionCount = 0; // add this field at class level
// private static final int DETECTION_CONFIRM_FRAMES = 2; // require 2 consecutive frames before accepting new target
// private static final double MAX_DEG_RATE_PER_FRAME = 8.0; // max change of desired per loop (deg)

//    public void aprilTagDetection1() {
//        List<AprilTagDetection> dets = aprilTag.getDetections();
//
//        if (dets != null && dets.size() > 0) {
//            tag = dets.get(0);
//            ID = tag.id;
//            x = tag.ftcPose.x; // inches (right +, left -)
//            y = tag.ftcPose.y; // inches (forward +)
//            z = tag.ftcPose.z; // inches (up)
//            Pitch = tag.ftcPose.pitch;
//            Yaw = tag.ftcPose.yaw;
//            Roll = tag.ftcPose.roll;
//
//            AprilTagDetected = true;
//        } else {
//            AprilTagDetected = false;
//            ID = -1;
//            x = 0.0; y = 0.0; z = 0.0;
//            Pitch = 0.0; Yaw = 0.0; Roll = 0.0;
//        }
//
//        // classify IDs
//        if (ID == 20 || ID == 24) tagID = ID; else tagID = -1;
//        if (ID == 21 || ID == 22 || ID == 23) patternID = ID; else patternID = -1;
//
//        // --- camera relative angle (camera frame) ---
//        double rawAngleDeg = 0.0;
//        boolean haveNewReading = false;
//        if (AprilTagDetected) {
//            rawAngleDeg = calculateTurretAngleDeg(x, y); // camera frame: + = RIGHT (per your comment)
//            haveNewReading = true;
//        }
//
//        // --- convert to turret frame: LEFT = + (flip camera sign) ---
//        double relativeToUse = -rawAngleDeg;
//
//        // deadband small noise
//        final double DEADBAND_DEG = 0.15;
//        if (Math.abs(relativeToUse) < DEADBAND_DEG) relativeToUse = 0.0;
//
//        // --- smoothing (EMA) on turret-framed value ---
//        if (haveNewReading && ENABLE_SMOOTHING) {
//            if (!hasSmoothed) { smoothedAngleDeg = relativeToUse; hasSmoothed = true; }
//            else { smoothedAngleDeg = SMOOTH_ALPHA * relativeToUse + (1.0 - SMOOTH_ALPHA) * smoothedAngleDeg; }
//            relativeToUse = smoothedAngleDeg;
//        }
//
//        // --- detection confirmation (avoid single-frame spikes) ---
//        if (haveNewReading) detectionCount++; else detectionCount = 0;
//        boolean confirmed = detectionCount >= DETECTION_CONFIRM_FRAMES;
//
//        // read current turret absolute angle BEFORE computing candidate
//        // NOTE: ensure currentTurretDeg is updated in the loop before calling aprilTagDetection()
//        // (your loop already sets currentTurretDeg = getTurretAngleFromEncoder(actualTurretPose))
//        double desiredCandidate = lastDesiredTurretDeg; // fallback
//
//        if (confirmed) {
//            // compute absolute candidate from current turret angle + relative offset
//            desiredCandidate = normalizeAngleDegrees(currentTurretDeg + relativeToUse);
//
//            // clamp to turret mechanical limits
//            desiredCandidate = Math.max(-TURRET_LIMIT_DEG, Math.min(TURRET_LIMIT_DEG, desiredCandidate));
//
//            // rate-limit change (prevents large jumps)
//            double delta = desiredCandidate - lastDesiredTurretDeg;
//            // normalize delta to shortest path (wrap-safe)
//            delta = normalizeAngleDegrees(delta);
//
//            // clamp per-frame change
//            if (Math.abs(delta) > MAX_DEG_RATE_PER_FRAME) {
//                delta = Math.signum(delta) * MAX_DEG_RATE_PER_FRAME;
//            }
//
//            double limitedDesired = normalizeAngleDegrees(lastDesiredTurretDeg + delta);
//
//            // accept limitedDesired as the new commanded heading and update stored values
//            desiredTurretDeg = limitedDesired;
//            lastDesiredTurretDeg = desiredTurretDeg;
//            lastKnownRelativeDeg = relativeToUse;
//        } else {
//            // not confirmed yet: hold previous absolute heading (do not recompute from last relative)
//            desiredTurretDeg = lastDesiredTurretDeg;
//        }
//
//        // Telemetry
//        telemetry.addData("AprilTagSeen", AprilTagDetected);
//        telemetry.addData("rawAngleDeg(camera)", haveNewReading ? String.format("%.3f", rawAngleDeg) : "N/A");
//        telemetry.addData("relativeToUse(turretFrame)", String.format("%.3f", relativeToUse));
//        telemetry.addData("confirmedFrames", detectionCount);
//        telemetry.addData("currentTurretDeg", String.format("%.3f", currentTurretDeg));
//        telemetry.addData("desiredCandidate", String.format("%.3f", desiredCandidate));
//        telemetry.addData("desiredTurretDeg(commanded)", String.format("%.3f", desiredTurretDeg));
//        telemetry.addData("lastKnownRelativeDeg", String.format("%.3f", lastKnownRelativeDeg));
//        telemetry.addData("lastDesiredTurretDeg", String.format("%.3f", lastDesiredTurretDeg));
//    }


    public void aprilTagDetection() {
        List<AprilTagDetection> dets = aprilTag.getDetections();

        double rawAngleDeg = 0.0;
        boolean haveNewReading = false;

        if (dets != null && dets.size() > 0) {
            tag = dets.get(0);
            ID = tag.id;
            x = tag.ftcPose.x; // inches (right +, left -)  z
            y = tag.ftcPose.y; // inches (forward +)
            z = tag.ftcPose.z; // inches (up)      x
            Pitch = tag.ftcPose.pitch;
            Yaw = tag.ftcPose.yaw;
            Roll = tag.ftcPose.roll;
            rawAngleDeg = calculateTurretAngleDeg(z, y); // camera frame: + = RIGHT (per earlier comment)
            haveNewReading = true;
            AprilTagDetected = true;
        } else {
            AprilTagDetected = false;
            ID = -1;
            // reset telemetry values for clarity
            x = 0.0;
            y = 0.0;
            z = 0.0;
            Pitch = 0.0;
            Yaw = 0.0;
            Roll = 0.0;
        }

        // classify IDs (your rules)
        if (ID == 20 || ID == 24) tagID = ID;
        if (ID == 21 || ID == 22 || ID == 23) patternID = ID;
        // if we have a reading, compute desired; otherwise hold lastDesiredTurretDeg
        if (haveNewReading) {
            // convert to turret frame: LEFT = + (flip camera sign)
            double relativeToUse = -rawAngleDeg;

            // compute absolute desired heading (degrees)
            double desired = normalizeAngleDegrees(currentTurretDeg + relativeToUse);

            // clamp to physical turret limits
            desired = Math.max(-TURRET_LIMIT_DEG, Math.min(TURRET_LIMIT_DEG, desired));

            lastKnownRelativeDeg = relativeToUse;

            // store and command
            lastDesiredTurretDeg = desired;
            desiredTurretDeg = desired;
        } else {
            // no detection: keep last commanded absolute heading
            desiredTurretDeg = lastDesiredTurretDeg;
        }

        // Telemetry for debugging & testing
        telemetry.addData("AprilTagSeen", AprilTagDetected);
        telemetry.addData("ID", ID);
        telemetry.addData("tagID", tagID);
        telemetry.addData("patternID", patternID);
        telemetry.addData("ftcPose.x (in) [lat]", "%.2f", x);
        telemetry.addData("ftcPose.y (in) [fwd]", "%.2f", y);

        if (haveNewReading) {
            telemetry.addData("rawAngleDeg (camera)", String.format("%.3f", rawAngleDeg));
        } else {
            telemetry.addData("rawAngleDeg (camera)", "No detection");
        }

        telemetry.addData("lastKnownRelativeDeg", String.format("%.3f", lastKnownRelativeDeg));
        telemetry.addData("lastDesiredTurretDeg", String.format("%.3f", lastDesiredTurretDeg));
        telemetry.addData("currentTurretDeg", String.format("%.3f", currentTurretDeg));
        telemetry.addData("desiredTurretDeg", String.format("%.3f", desiredTurretDeg));
    }
    public void aprilTagDetection1() {
        List<AprilTagDetection> dets = aprilTag.getDetections();

        if (dets != null && dets.size() > 0) {
            tag = dets.get(0);
            ID = tag.id;
            x = tag.ftcPose.x; // inches (right +, left -)
            y = tag.ftcPose.y; // inches (forward +)
            z = tag.ftcPose.z; // inches (up)
            Pitch = tag.ftcPose.pitch;
            Yaw = tag.ftcPose.yaw;
            Roll = tag.ftcPose.roll;

            AprilTagDetected = true;
        } else {
            AprilTagDetected = false;
            ID = -1;
            // reset telemetry values for clarity
            x = 0.0;
            y = 0.0;
            z = 0.0;
            Pitch = 0.0;
            Yaw = 0.0;
            Roll = 0.0;
        }

        // classify IDs (your rules)
        if (ID == 20 || ID == 24) tagID = ID;
        if (ID == 21 || ID == 22 || ID == 23) patternID = ID;

        // compute camera-relative angle (degrees)
        double rawAngleDeg = 0.0;
        boolean haveNewReading = false;
        if (AprilTagDetected) {
            rawAngleDeg = calculateTurretAngleDeg(x, y); // camera frame: + = RIGHT (per earlier comment)
            haveNewReading = true;
        }

        // --- convert to turret frame (LEFT = +). Flip BEFORE smoothing. ---
        double relativeToUse = -rawAngleDeg;

        // small deadband to ignore tiny jitter
        final double DEADBAND_DEG = 0.01;
        if (Math.abs(relativeToUse) < DEADBAND_DEG) relativeToUse = 0.0;

        // optional smoothing (EMA) applied to turret-framed value
        if (haveNewReading && ENABLE_SMOOTHING) {
            if (!hasSmoothed) {
                smoothedAngleDeg = relativeToUse;
                hasSmoothed = true;
            } else {
                smoothedAngleDeg = SMOOTH_ALPHA * relativeToUse + (1.0 - SMOOTH_ALPHA) * smoothedAngleDeg;
            }
            relativeToUse = smoothedAngleDeg;
        }

        // compute absolute desired turret heading (use currentTurretDeg from loop)
        if (haveNewReading) {
            double desired = normalizeAngleDegrees(currentTurretDeg + relativeToUse);
            desired = Math.max(-TURRET_LIMIT_DEG, Math.min(TURRET_LIMIT_DEG, desired));

            // store last-known values for hold-on-loss behavior
            lastKnownRelativeDeg = relativeToUse;
            lastDesiredTurretDeg = desired;

            // final commanded heading this loop
            desiredTurretDeg = desired;
        } else {
            // no detection -> hold last commanded absolute heading
            desiredTurretDeg = lastDesiredTurretDeg;
        }

        // Telemetry for debugging & testing
        telemetry.addData("AprilTagSeen", AprilTagDetected);
        telemetry.addData("ID", ID);
        telemetry.addData("tagID", tagID);
        telemetry.addData("patternID", patternID);
        telemetry.addData("ftcPose.x (in) [lat]", "%.2f", x);
        telemetry.addData("ftcPose.y (in) [fwd]", "%.2f", y);

        if (haveNewReading) {
            telemetry.addData("rawAngleDeg (camera)", String.format("%.3f", rawAngleDeg));
            if (ENABLE_SMOOTHING) telemetry.addData("smoothedRelDeg", String.format("%.3f", relativeToUse));
        } else {
            telemetry.addData("rawAngleDeg (camera)", "No detection");
            if (ENABLE_SMOOTHING) telemetry.addData("smoothedRelDeg", "No detection");
        }

        telemetry.addData("lastKnownRelativeDeg", String.format("%.3f", lastKnownRelativeDeg));
        telemetry.addData("lastDesiredTurretDeg", String.format("%.3f", lastDesiredTurretDeg));
        telemetry.addData("currentTurretDeg", String.format("%.3f", currentTurretDeg));
        telemetry.addData("desiredTurretDeg", String.format("%.3f", desiredTurretDeg));
    }


    public double calculateTurretAngleDeg(double ftcPoseX, double ftcPoseY) {
        // safety: if both are tiny, return 0.0 (we treat as no meaningful measurement)
        final double EPS = 1e-6;
        if (Math.abs(ftcPoseX) < EPS && Math.abs(ftcPoseY) < EPS) return 0.0;

        // Use atan2(opposite, adjacent) = atan2(x, y)
        double angleRad = Math.atan2(ftcPoseX, ftcPoseY); // note order: x then y
        double angleDeg = Math.toDegrees(angleRad);

        // optional: small deadband (kept commented out; you can enable if desired)
        // if (Math.abs(angleDeg) < 0.05) angleDeg = 0.0;

        return angleDeg;
    }
}
