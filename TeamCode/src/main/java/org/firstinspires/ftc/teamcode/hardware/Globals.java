package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Globals {

    //TODO======================================================== INTAKE =================================================

    public static double intakeServoInit = 0.8;
    public static double intakeServoPick = 0.3;
    public static double turretCurrentValue, currentAngleT;
    public static double intakeServoAutoPick = 0.3;
    public static double intakeServoRelease=0.18

            ;  //0.512 //R 0.488
    public static double intakeRollerOn = -1;//L-0.813 R-0.187
    public static double intakeRollerOff = 0;
    public static double intakeRollerReverse = 1;
    public static double clrGain = 50.0;


    //TODO======================================================== FEEDER ====================================================

    public static double upperfeederOn = -1;
    public static double upperfeederOff = 0;
    public static double upperfeederReverse = 1;
    public static double upperfeederPow = -0.5;

    public static double lowerfeederOn = 1;
    public static double lowerfeederOff = 0;
    public static double lowerfeederReverse = -1;
    public static double lowerfeederPow = 0.5;

    //TODO======================================================== OUTTAKE =====  ===============================================

    public static double turret_imu_track = 0,turretInit = 0, turretInitNear = -50, turretInitBlueFar = 60, turretInitBlueNear = 45, turretTrack = -65,turretTrackOff = -62, turretShoot = 68, turretShootNear = 45,turretShootNearOff = 42, turretBuffRed = -45,turretBuffRedOff = -42, turretBuffBlue = 130;
    public static double currentTurretState;
    public static double curretShooterStateVelMode;
    public static int turretTarget = 0;
    public static double turretPIDInit = 0, turretPIDTrack = 0, turretPIDShoot = 0, turretPIDBuffRed = 0, turretPIDBuffBlue = 0;
    public static double shooterInit = 0, shooterFar = 0, shooterNear = 0, shooterRelease = 0, shooterOff = 0;
    public static int shooterInitVel = 0,shooterFarVelTeleOP = 1800,shooterFarVelBlue = 1620, shooterFarVel = 1800, shooterNearVel = 1400; // 1550
    public static int shooterReleaseVel = 1350, shooterOffVel = 0, shooterSlowVel = 1300;
    public static double kp_turret = 0.022; //0.015
    public static double ki_turret = 0; // 0
    public static double kd_turret = 0.15;  // 0.03
    public static double feed = 0.05;  // 0.03
//    ki_turret = 0, kd_turret = 0.03;
    public static double hoodInit = 0.91;  // previous value = 0.88
    public static double eg_init = 0.1;  // previous value = 0.93
    public static double eg_release = 1;  // previous vaalue = 0.93
    public static double eg_pos = 0;  // previous value = 0.93
    public static double hoodFar = 0.82, hoodFarAuto = 0.89, hoodNearAuto = 0.94, hoodNearEnd = 0.88, hoodNearStart = 1, hoodRelease = 0, hoodPos;
    public static boolean interpolate  = false, shooterMode = false;
    public static int shooterVel = 900;



    //TODO======================================================== SORTING ====================================================

    public static int counterFeed_PPGtoGPP =0, counterFeed_PGPtoPPG = 0, counterFeed_GPPtoPPG = 0, counterFeed_GPPtoPGP = 0, counterFeed = 0;
    public static boolean intakeFlag_PPGtoGPP = false, intakeFlag_PGPtoPPG = false, intakeFlag_GPPtoPPG = false, intakeFlag_GPPtoPGP = false, intakeFlag = false;

    public static ElapsedTime motionTimer = new ElapsedTime();
    public static int motionTime = 280;
    public static boolean motionFlag = false;
    public static boolean thirdBallFlag = false;


}