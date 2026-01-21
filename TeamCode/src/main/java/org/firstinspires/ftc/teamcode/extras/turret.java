//package org.firstinspires.ftc.teamcode;
//
//import static org.firstinspires.ftc.teamcode.LimeUtils.LimeGlobals.*;
//import org.firstinspires.ftc.teamcode.LimeUtils.LimeGlobals;
//import org.firstinspires.ftc.teamcode.LimeUtils.LimeGlobals.*;
//
//import static java.lang.Math.abs;
//
//import androidx.core.math.MathUtils;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.LLStatus;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServoImplEx;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.opencv.dnn.Image2BlobParams;
//
//import java.util.List;
//
//@Config
//@TeleOp(name = "Sensor: Limelight3A Turret Adjust", group = "Sensor")
//public class turret extends LinearOpMode {
//
//    private static double errCpr;
//    private static double previous_error = 0;
//    private static double integral = 0;
//    private static double error;
//    private static double pid = 0;
//    private static double c = 0;
//    private Limelight3A limelight;
//    private static Servo s1;
//    public static DcMotorEx m1;
//    public static CRServoImplEx cr;
//    public static DcMotorEx enc;
//    public static double kp = 0.004, ki = 0, kd = 0.01, setPoint = 45;
//    public static MecanumDrive drive;
//    public static double currentPose = 0;
//    public static double prevAngle = 0;
//    public static double totalAngle = 0;
//    private static double angle;
//    private boolean targetFound;
//    private Pose3D botpose;
//
//    @Override
//    public void runOpMode() throws InterruptedException
//    {
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        s1 = hardwareMap.get(Servo.class, "s1");
//        cr = hardwareMap.get(CRServoImplEx.class, "cr");
//        enc = hardwareMap.get(DcMotorEx.class,"enc");
////        m1 = hardwareMap.get(DcMotorEx.class, "m1");
////        enc =  hardwareMap.get(DcMotorEx.class,"m1");
//        drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
////        m1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
////        m1.setZeroPowerBehavior(BRAKE);
//        enc.setDirection(DcMotorEx.Direction.REVERSE);
//        telemetry.setMsTransmissionInterval(11);
//
//        limelight.pipelineSwitch(5);
//
//        /*
//         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
//         */
//        limelight.start();
//
//        telemetry.addData(">", "Robot Ready.  Press Play.");
//        telemetry.update();
//        previous_error = 0;
//        integral = 0;
//        angle = 0;
//        prevAngle = 0;
//        totalAngle = 0;
//        while (opModeInInit()) {
//            s1.setPosition(0.5);
//            enc.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        }
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            targetFound = false;
//            currentPose = enc.getCurrentPosition();
//            drive.updatePoseEstimate();
//
//            LLStatus status = limelight.getStatus();
//            telemetry.addData("Name", "%s",
//                    status.getName());
//            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                    status.getTemp(), status.getCpu(),(int)status.getFps());
//            telemetry.addData("Pipeline", "Index: %d, Type: %s",
//                    status.getPipelineIndex(), status.getPipelineType());
//
//            if(gamepad1.back){
//                drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
//                drive.updatePoseEstimate();
//                angle = 0;
//                prevAngle = 0;
//                totalAngle = 0;
//            }
//
//            LLResult result = limelight.getLatestResult();
//
//
//            if (result.isValid()) {
//                // Access general information
//                targetFound = true;
//                botpose = result.getBotpose();
//
//
//                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
//
//
//                for (LLResultTypes.FiducialResult fr : fiducialResults) {
////                    yaw = fr.getTargetPoseRobotSpace().getOrientation().getYaw();
//                    yaw = fr.getRobotPoseFieldSpace().getOrientation().getYaw();
//                    pitch = fr.getRobotPoseFieldSpace().getOrientation().getPitch();
//                    roll = fr.getRobotPoseFieldSpace().getOrientation().getRoll();
//                    x_pose = fr.getRobotPoseFieldSpace().getPosition().x;
//                    y_pose = fr.getRobotPoseFieldSpace().getPosition().y;
//                    z_pose = fr.getTargetPoseRobotSpace().getPosition().z;
//                    a2 = fr.getTargetYDegrees();
//                    angle_radian = (LimeGlobals.a1 + LimeGlobals.a2) * (3.14159 / 180.0);
//                    dist = (LimeGlobals.h2 - LimeGlobals.h1)/ Math.tan(LimeGlobals.angle_radian);
//                    tx = fr.getTargetXDegrees();
//                    ty = fr.getTargetYDegrees();
//                    tYaw = fr.getTargetPoseRobotSpace().getOrientation().getYaw();
//
//                    DESIRED_ANGLE = yaw;
//                    telemetry.addData("Fiducial", " Yaw: %.2f", yaw);
//                    telemetry.addData("Fiducial", " Pitch: %.2f", pitch);
//                    telemetry.addData("Fiducial", " Roll: %.2f", roll);
//                }
//            }else {
//                yaw = 0;
//                corr = 0;
//                telemetry.addData("Limelight", "No data available");
//            }
//
//            if (gamepad1.left_bumper && !targetFound){
//                rangeError = 0;
//
//                Find_AprilTag(telemetry);
//
//            }
//            else if (gamepad1.left_bumper && targetFound) {
//
//                Track_AprilTag(telemetry);
//
//            } else {
//                rangeError = 0;
//                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
//                driveS  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
//                strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
//                turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
//                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", driveS, strafe, turn);
//                drive.driveFieldCentric(strafe,  driveS, -turn, drive.localizer.getPose().heading.toDouble());
//
//            }
//
//
//            angle = getContinuousIMU(Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
//
//            run_turret(angle + DESIRED_ANGLE, 0, 8192, currentPose);
//
//
//
//            print_stuffs(telemetry);
//
//            telemetry.update();
//
//        }
//        limelight.stop();
//    }
//
//    public static void moveRobot(double x, double y, double yaw) {
//        // Calculate wheel powers.
//        double frontLeftPower    =  x - y - yaw;
//        double frontRightPower   =  x + y + yaw;
//        double backLeftPower     =  x + y - yaw;
//        double backRightPower    =  x - y + yaw;
//
//        // Normalize wheel powers to be less than 1.0
//        double max = Math.max(abs(frontLeftPower), abs(frontRightPower));
//        max = Math.max(max, abs(backLeftPower));
//        max = Math.max(max, abs(backRightPower));
//
//        if (max > 1.0) {
//            frontLeftPower /= max;
//            frontRightPower /= max;
//            backLeftPower /= max;
//            backRightPower /= max;
//        }
//
//        // Send powers to the wheels.
//        drive.leftFront.setPower(frontLeftPower);
//        drive.rightFront.setPower(frontRightPower);
//        drive.leftBack.setPower(backLeftPower);
//        drive.rightBack.setPower(backRightPower);
//    }
//
//
//    public double  getContinuousIMU(double currentAngle) {
//        double delta = currentAngle - prevAngle;
//
//        // Handle wrap-around
//        if (delta > 180) {
//            delta -= 360;
//        } else if (delta < -180) {
//            delta += 360;
//        }
//
//        totalAngle += delta;
//        prevAngle = currentAngle;
//
//        return totalAngle;
//    }
//
//    public static void run_turret(double imu, double min_in_pos, double max_in_pos, double pose ){
//        pid = 0;
//
//        c = map(pose, min_in_pos, max_in_pos, 360, 0);
//
//        if (imu < 0){
//            c = -c;
//        }
//
//
//        error = 0;
//        double target = imu;
//
//        double derivative = 0;
//        if(imu < 0){
//            error = c + target;
//            integral = integral + error;
//            derivative = error - previous_error;
//
//            previous_error = error;
//            pid = kp*error + kd*derivative + ki*integral;
//            cr.setPower(pid);
//
//        }
//        else {
//            error = target - c;
//            integral = integral + error;
//            derivative = error - previous_error;
//
//            previous_error = error;
//            pid = kp*error + kd*derivative + ki*integral;
//            cr.setPower(pid);
//
//        }
//    }
//
//    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
//        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
//    }
//
//    public static double convert_m_to_inches(double meter){
//        return meter * 39.370;
//    }
//
//    public static void Track_AprilTag(Telemetry telemetry){
//        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
//        rangeError      = convert_m_to_inches(z_pose) - DESIRED_DISTANCE - offset;
//        double  headingError    = tx;
//        double  yawError        = tYaw;
//
//        // Use the speed and turn "gains" to calculate how we want the robot to move.
//        driveS  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//        turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
//        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//        moveRobot(driveS, strafe, -turn);
//    }
//    public static void Find_AprilTag(Telemetry telemetry){
//        telemetry.addLine("No Target Found - Rotating to find target");
//        if(Math.toDegrees( drive.localizer.getPose().heading.toDouble()) > 0){
//            rangeError      = 0;
//            headingError    = Math.toDegrees( drive.localizer.getPose().heading.toDouble()) - DESIRED_ANGLE;
//            double  yawError        = 0;
//
//            // Use the speed and turn "gains" to calculate how we want the robot to move.
//            driveS  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//            turn   = Range.clip(headingError * TURN_GAIN, -1, 1) ;
//            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//        }
//        else if(Math.toDegrees( drive.localizer.getPose().heading.toDouble()) < 0){
//            rangeError      = 0;
//            if(DESIRED_ANGLE < 0){
//                headingError    = Math.toDegrees( drive.localizer.getPose().heading.toDouble()) - DESIRED_ANGLE;
//            }
//            else {
//                headingError = DESIRED_ANGLE - Math.toDegrees(drive.localizer.getPose().heading.toDouble());
//            }
//            double  yawError        = 0;
//
//            // Use the speed and turn "gains" to calculate how we want the robot to move.
//            driveS  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//            turn   = Range.clip(headingError * TURN_GAIN, -1, 1) ;
//            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//        }
//        moveRobot(driveS, strafe, -turn);
//        telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", driveS, strafe, turn);
//
//    }
//
//    public static void print_stuffs(Telemetry telemetry){
//
//        telemetry.addData("corr", corr);
//        telemetry.addData("DESIRED ANGLE " ,DESIRED_ANGLE);
//        telemetry.addData("angle", angle);
//        telemetry.addData("c ", c);
//        telemetry.addData("pid", pid);
//        telemetry.addData("error", error);
//        telemetry.addData("current Position", currentPose);
//        telemetry.addData("headingError", headingError);
//        telemetry.addData("Range error: ", rangeError);
//        telemetry.addData("corr", corr);
//        telemetry.addData("Tx", tx);
//        telemetry.addData("Ty", ty);
//
//        telemetry.addData("T Yaw", tYaw);
//        telemetry.addData("x ", convert_m_to_inches(x_pose));
//        telemetry.addData("y ", convert_m_to_inches(y_pose));
//        telemetry.addData("z ", convert_m_to_inches(z_pose));
//        telemetry.addData("dist", dist);
//        telemetry.addData("x robot", drive.localizer.getPose().position.x);
//        telemetry.addData("y robot", drive.localizer.getPose().position.y);
//        telemetry.addData("heading real", Math.toDegrees(drive.localizer.getPose().heading.real));
//        telemetry.addData("heading img", Math.toDegrees(drive.localizer.getPose().heading.imag));
//        telemetry.addData("heading RAD", drive.localizer.getPose().heading.real);
//        telemetry.addData("heading RAD to double",Math.toDegrees( drive.localizer.getPose().heading.toDouble()));
//        telemetry.addData("heading RAD to double Rad",drive.localizer.getPose().heading.toDouble());
//    }
//
//
//
//
//
//}
