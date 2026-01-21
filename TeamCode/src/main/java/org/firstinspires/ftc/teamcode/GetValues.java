package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="GetValues", group="TeleOp")
@Config
public class GetValues extends LinearOpMode {
    public static RobotHardware robot;
//    private IntakeSubsystem intake;
//    private Elbow elbow;
//    private Shoulder shoulder;
    private MecanumDrive drive;
    public double botHeading;

    public double multiplier=1;

    public double b;
    public static double c;
    public static double endPos = 0;

    public static double target = 0;
    public static double error ;
    public static double integral;
    public static double previous_error;
    public double derivative;
    public static double pid;
    public double a;

    public static double kp = 0.02, ki = 0, kd = 0.02;

    public static double corr = 0, yaw = 0;
    private double totalAngle;
    private double angle;
    private double prevAngle;
    IMU imu;

    public double strafe = 0.7, speed = 0.7, turn = 0.7;
    public static boolean upFlag = false;
    public static boolean initFlag = false;
    public static boolean pickingFlag = false;

    public static boolean pickedFlag = false;

    public static List<Action> runningActions = new ArrayList<>();

    public static int lifterPos=0;
    @Override
    public void runOpMode() {

        robot = new RobotHardware();
        robot.init(hardwareMap, telemetry);
//        elbow = new Elbow(robot);
//        shoulder= new Shoulder(robot);
//        intake = new IntakeSubsystem(robot);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        robot.resetEncoder();

        double xRotation = 0;
        double yRotation = 0;
        double zRotation = 0;

        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
//        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        integral = 0;
        error = 0;
        previous_error = 0;
//        imu.resetYaw();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeInInit()) {

            drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
        }

        waitForStart();

        while (opModeIsActive()) {
            runningActions = updateAction();
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            Pose2d pose = drive.localizer.getPose();
//            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            double actualPos = robot.turretEncoder.getCurrentPosition();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            // TODO =============================================== INIT===========================================================

            if(gamepad1.a){
                robot.endgame.setPosition(endPos);
            }
            if(gamepad1.b){

            }
            if(gamepad1.y){
                robot.intakeRoller.setPower(Globals.intakeRollerReverse);
                robot.lowerFeeder.setPower(Globals.intakeRollerReverse);
                robot.upperFeeder.setPower(Globals.upperfeederReverse);
            }
            if(gamepad1.dpad_up){
                robot.shooter.setVelocity(Globals.shooterVel);
            }
            if(gamepad1.dpad_down){
                robot.shooter.setVelocity(0);
            }
            if(gamepad1.dpad_left){
                robot.intake.setPosition(Globals.intakeServoPick);
            }
            if(gamepad1.dpad_right){
                robot.hood.setPosition(Globals.hoodInit);
            }

            if (gamepad1.x)
            {
                target = a;
            }

            if (gamepad2.a)
            {
                robot.intakeRoller.setPower(Globals.intakeRollerOn);
                robot.lowerFeeder.setPower(Globals.lowerfeederOn);
            }
            if (gamepad2.b)
            {
                robot.intakeRoller.setPower(Globals.intakeRollerOn);
                robot.lowerFeeder.setPower(-Globals.lowerfeederOn);
            }
            if (gamepad2.y)
            {
                robot.intakeRoller.setPower(-Globals.intakeRollerOn);
                robot.lowerFeeder.setPower(Globals.lowerfeederOn);
            }

            if (gamepad2.x)
            {
                robot.intakeRoller.setPower(Globals.intakeRollerOn);
                robot.lowerFeeder.setPower(Globals.lowerfeederOff);

            }
            angle = getContinuousIMU(target);
            run_turret(angle, 0, 27845, actualPos, telemetry);

            // Todo ==================================== Robot Oriented ======================================================================



            telemetry.addData("Left Front Current : ", drive.leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right Front Current : ", drive.rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right Back Current : ", drive.rightBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Left Back Current : ", drive.leftBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine("---------------------------");

            telemetry.addData("Intake Current : ", robot.intakeRoller.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Lower Feeder : ", robot.lowerFeeder.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Upper Feeder : ", robot.upperFeeder.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Shooter : ", robot.shooter.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("Shooter Velocity : ", robot.shooter.getVelocity());



            telemetry.addLine("---------------------------");

            telemetry.update();


        }

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

    public static void run_turret(double imu, double min_in_pos, double max_in_pos, double pose, Telemetry telemetry){
        pid = 0;

        c = map(pose, min_in_pos, max_in_pos, 0, 360);

        if (imu < 0){
            c = -c;
        }


        double target = imu;

        double derivative = 0;
        if(imu < 0){
            telemetry.addLine("neagtive heading");
            error = c + target;
//            error = -error;
            integral = integral + error;
            derivative = error - previous_error;

            previous_error = error;
            pid = kp*error + kd*derivative + ki*integral;
            robot.turret1.setPower(-pid);
            robot.turret2.setPower(-pid);

        }
        else {
            telemetry.addLine("positive heading");

            error = target - c;
//            error = -error;
            integral = integral + error;
            derivative = error - previous_error;

            previous_error = error;
            pid = kp*error + kd*derivative + ki*integral;
            robot.turret1.setPower(-pid);
            robot.turret2.setPower(-pid);
        }
    }

    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }


    //heading follow




}
