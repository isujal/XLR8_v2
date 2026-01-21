package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.teleop.SB5.run_turret;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class TurretSubsystem {
    public static double c;
    public static double actualPos;
//    private static RobotHardware robot=RobotHardware.getInstance();
//    RobotHardware robot = RobotHardware.getInstance();
//        robot.init(hardwareMap,telemetry);
    private final CRServo turretL, turretR;
    public DcMotorEx turretEncoder=null;

    private final IMU imu;

    // ===== PID Settings =====
    public static double kp = 0.01, ki = 0.0, kd = 0.0005;
    private double integral = 0, prevError = 0;

    // TARGET ANGLE the turret must track
    private double targetAngleDeg = 0;

    public enum TurretState {
        IDLE,
        TRACKING_IMU
    }

    private TurretState state = TurretState.IDLE;

    public TurretSubsystem(CRServo tL, CRServo tR, IMU imu) {
        this.turretL = tL;
        this.turretR = tR;
        this.imu = imu;
    }

    public void setState(TurretState newState) {
        this.state = newState;
    }

    public TurretState getState() { return state; }

    /** Set the target angle the turret should face */
    public void setTarget(double angleDeg) {
        targetAngleDeg = angleDeg;
    }

    /** Core update loop called every TeleOp loop */
    public void update() {

        switch (state) {

            case IDLE:
                turretL.setPower(0);
                turretR.setPower(0);
                break;

            case TRACKING_IMU:
                c = map(actualPos, 0, 27845, 0, 360);

                if (targetAngleDeg < 0){
                    c = -c;
                }
//                run_turret(angle, 0, 27845, actualPos, telemetry);
                double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                double error = targetAngleDeg - yaw;

                // Normalize error (-180 to 180)
                error = (error + 540) % 360 - 180;

                integral += error;
                double derivative = error - prevError;
                prevError = error;

                double pid = kp * error + ki * integral + kd * derivative;

                turretL.setPower(-pid);
                turretR.setPower(pid);
                break;
        }
    }

    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
}
