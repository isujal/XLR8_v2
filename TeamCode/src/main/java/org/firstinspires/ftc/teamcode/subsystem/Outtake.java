package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Config
public class Outtake {

    public static RobotHardware robot = RobotHardware.getInstance();
    public static TurretState turretState = TurretState.INIT;
    public static TurretPIDState turretPIDState = TurretPIDState.INIT;
    public static ShooterState shooterState = ShooterState.INIT;
    public static HoodState hoodState = HoodState.INIT;

    public enum TurretState {
        INIT, TRACK,TRACK_OFF, SHOOT, SHOOT_OFF,SHOOT_NEAR,SHOOT_NEAR_OFF,BUFF_RED,BUFF_RED_OFF,BUFF_BLUE, IMU_TRACK
    }public enum TurretPIDState {
        INIT, TRACK, SHOOT,BUFF_RED,BUFF_BLUE
    }public enum ShooterState {
        INIT, FAR_BLUE, TELE_FAR, FAR, NEAR, RELEASE, OFF, SLOW
    }    public enum HoodState {
        INIT, FAR, NEAR_END, NEAR_START, RELEASE,hoodPos,AUTO_NEAR, AUTO_FAR
    }

    // Constructor
    public Outtake(RobotHardware robot) {Outtake.robot = robot;}

    // Update Enums
    public static void updateTurretState(@NonNull TurretState state){
        turretState=state;
        Globals.currentTurretState= Globals.turretInit;
        switch (state){
            case INIT:
                Globals.currentTurretState= Globals.turretInit;
                break;
            case TRACK:
                Globals.currentTurretState= Globals.turretTrack;
                break;
            case TRACK_OFF:
                Globals.currentTurretState= Globals.turretTrackOff;
                break;
            case SHOOT:
                Globals.currentTurretState= Globals.turretShoot;
                break;
            case SHOOT_OFF:
                Globals.currentTurretState= Globals.turretShootOff;
                break;
            case SHOOT_NEAR:
                Globals.currentTurretState= Globals.turretShootNear;
                break;
            case SHOOT_NEAR_OFF:
                Globals.currentTurretState= Globals.turretShootNearOff;
                break;
            case BUFF_RED:
                Globals.currentTurretState=Globals.turretBuffRed;
                break;
            case BUFF_RED_OFF:
                Globals.currentTurretState=Globals.turretBuffRedOff;
                break;
            case BUFF_BLUE:
                Globals.currentTurretState=Globals.turretBuffBlue;
                break;
            case IMU_TRACK:
                Globals.currentTurretState=Globals.turret_imu_track;
                break;

        }
    }
    public void updateTurretPIDState(@NonNull TurretPIDState state){
        turretPIDState=state;
        double currentTurretPIDState= Globals.turretInit;
        switch (state){
            case INIT:
                currentTurretPIDState= Globals.turretPIDInit;
                break;
            case TRACK:
                currentTurretPIDState= Globals.turretTrack;
                break;
            case SHOOT:
                currentTurretPIDState= Globals.turretShoot;
                break;
            case BUFF_RED:
                currentTurretPIDState=Globals.turretBuffRed;
                break;
            case BUFF_BLUE:
                currentTurretPIDState=Globals.turretBuffBlue;
                break;
        }
    }

    public static void updateShooterState(@NonNull ShooterState state){
        shooterState=state;
        double curretShooterStatePowerMode=Globals.shooterInit;
        Globals.curretShooterStateVelMode=Globals.shooterInitVel;
        switch (state){
            case INIT:
                curretShooterStatePowerMode=Globals.shooterInit;
                Globals.curretShooterStateVelMode=Globals.shooterInitVel;
                break;
            case FAR_BLUE:
                curretShooterStatePowerMode=Globals.shooterInit;
                Globals.curretShooterStateVelMode=Globals.shooterFarVelBlue;
                break;
            case FAR:
                curretShooterStatePowerMode=Globals.shooterFar;
                Globals.curretShooterStateVelMode=Globals.shooterFarVel;
                break;
            case TELE_FAR:
                curretShooterStatePowerMode=Globals.shooterFar;
                Globals.curretShooterStateVelMode=Globals.shooterFarVelTeleOP;
                break;
            case NEAR:
                curretShooterStatePowerMode=Globals.shooterNear;
                Globals.curretShooterStateVelMode=Globals.shooterNearVel;
                break;
            case RELEASE:
                curretShooterStatePowerMode=Globals.shooterRelease;
                Globals.curretShooterStateVelMode=Globals.shooterReleaseVel;
                break;
            case OFF:
                curretShooterStatePowerMode=Globals.shooterOff;
                Globals.curretShooterStateVelMode=Globals.shooterOffVel;
                break;
            case SLOW:
//                curretShooterStatePowerMode=Globals.shooterOff;
                Globals.curretShooterStateVelMode=Globals.shooterSlowVel;
                break;
        }

        if (!Globals.shooterMode) {
            extendShooter(curretShooterStatePowerMode);
        } else {
            extendShooterUsingVelocity(Globals.curretShooterStateVelMode);
        }
    }    public static void updateHoodState(@NonNull HoodState state){
        hoodState=state;
        double currentHoodState=Globals.hoodInit;
        switch (state){
            case INIT:
                currentHoodState=Globals.hoodInit;
                break;
            case FAR:
                currentHoodState=Globals.hoodFar;
                break;
            case AUTO_FAR:
                currentHoodState=Globals.hoodFarAuto;
                break;
            case NEAR_END:
                currentHoodState=Globals.hoodNearEnd;
                break;
            case AUTO_NEAR:
                currentHoodState=Globals.hoodNearAuto;
                break;
            case NEAR_START:
                currentHoodState=Globals.hoodNearStart;
                break;
            case RELEASE:
                currentHoodState=Globals.hoodRelease;
                break;
            case hoodPos:
                currentHoodState=Globals.hoodPos;
                break;
        }

        if (!Globals.interpolate){
            setHoodState(currentHoodState);
        }
        else {

        }
    }

    public static void extendShooter(double currentShooter){
        robot.shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooter.setPower(currentShooter);
    }
    public static void extendShooterUsingVelocity(double currentShooter){

        robot.shooter.setVelocity(currentShooter);
    }

    public static void setHoodState(double pos){
        robot.hood.setPosition(pos);
    }
    public static Action ShooterCommand(ShooterState state) {
        return new InstantAction(() -> updateShooterState(state));
    }
    public static Action TurretCommand(TurretState state) {
        return new InstantAction(() -> updateTurretState(state));
    }
    public static Action HoodCommand(HoodState state) {
        return new InstantAction(() -> updateHoodState(state));
    }

    public void setHood(HoodState state) {
        updateHoodState(state);
    }
    public void setShooter(ShooterState state) {
        updateShooterState(state);
    }


}