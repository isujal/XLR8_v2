package org.firstinspires.ftc.teamcode.subsystem;

//import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.hardware.Globals.motionTimer;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.sequences.IntakeSeq;

import java.util.ArrayList;
import java.util.List;

public class MOTIF {
    private static RobotHardware robot = RobotHardware.getInstance();
    public static List<Action> runningActions = new ArrayList<>();


    public enum MOTIF_STATE {
        GPP,
        PGP,
        PPG,
    }

    public static MOTIF_STATE motifState = MOTIF_STATE.GPP;
    public static MOTIF_STATE currentMotifState = MOTIF_STATE.GPP;





    public static void SORT_STATE(MOTIF_STATE current, MOTIF_STATE motif ) {
        switch (current) {
            case PPG:
                switch(motif){
                    case PPG:
                        Globals.intakeFlag = true;
                        break;
                    case PGP:
                        Globals.intakeFlag_PGPtoPPG = true;
                        break;
                    case GPP:
                        Globals.intakeFlag_PPGtoGPP = true;
                        break;
                }
                break;
            case PGP:
                switch(motif){
                    case PPG:
                        Globals.intakeFlag_PGPtoPPG = true;
                        break;
                    case PGP:
                        Globals.intakeFlag = true;
                        break;
                    case GPP:
                        Globals.intakeFlag_GPPtoPPG = true;
                        break;
                }
                break;
            case GPP:
                switch(motif){
                    case PPG:
                        Globals.intakeFlag_GPPtoPPG = true;
                        break;
                    case PGP:
                        Globals.intakeFlag_GPPtoPGP = true;
                        break;
                    case GPP:
                        Globals.intakeFlag = true;
                        break;
                }
                break;
        }


    }


    public static void Sequence_2_4(){




        if (Globals.intakeFlag_PGPtoPPG  && Globals.counterFeed_PGPtoPPG == 0)
        {
//                thirdBallFlag = false;
//            runningActions.add(IntakeSeq.IntakeStoreAction2(intake,outtake,feeder));

            Actions.runBlocking(
                    new ParallelAction(
                            Intake.IntakeCommand(Intake.IntakeServoState.IN),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON),
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                            Outtake.TurretCommand(Outtake.TurretState.INIT)
                    )
            );
        }
//            if (!Globals.intakeFlag_PGPtoPPG )
//            {
//                runningActions.add(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
//                        )
//                );
//            }

        if (!robot.feederBeam.getState() && Globals.counterFeed_PGPtoPPG==0 && Globals.intakeFlag_PGPtoPPG ){
            Globals.counterFeed_PGPtoPPG += 1;
            Actions.runBlocking(
                    new ParallelAction(
                            Intake.RollerCommand(Intake.IntakeRollerState.ON)
                    )
            );

        }
        if (robot.feederBeam.getState() && Globals.counterFeed_PGPtoPPG ==1 && Globals.intakeFlag_PGPtoPPG ){
            Globals.counterFeed_PGPtoPPG += 1;
            Actions.runBlocking(
                    new ParallelAction(
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON)
                    )
            );
        }
        if (Globals.counterFeed_PGPtoPPG==2 && !robot.intakeBeam.getState()&& Globals.intakeFlag_PGPtoPPG ){
            Globals.counterFeed_PGPtoPPG += 1;
//                zeroFlag = true;
        }
        if (Globals.counterFeed_PGPtoPPG==3 && robot.intakeBeam.getState()&& Globals.intakeFlag_PGPtoPPG ){
            Globals.counterFeed_PGPtoPPG += 1;
            Actions.runBlocking(
                    new ParallelAction(
                            Feeder.LFCommand(Feeder.LowerFeederState.ON)
                    )
            );
        }

        if (Globals.counterFeed_PGPtoPPG==4 && !robot.intakeBeam.getState()&& Globals.intakeFlag_PGPtoPPG ){
            Globals.counterFeed_PGPtoPPG += 1;
//                zeroFlag = false;

        }
        if (Globals.counterFeed_PGPtoPPG==5 && robot.intakeBeam.getState()&& Globals.intakeFlag_PGPtoPPG ){
//                motionFlag = true;
//                motionTimer.reset();
            Globals.counterFeed_PGPtoPPG = 0;
            gamepad1.rumble(1,1,400);
            Actions.runBlocking(
                    new ParallelAction(
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                            Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                    )
            );
            Globals.intakeFlag_PGPtoPPG  = false;
        }

    }




    public static void Sequence_3_8(){


        if (Globals.intakeFlag_GPPtoPPG && Globals.counterFeed_GPPtoPPG ==0)
        {
//                thirdBallFlag = false;
//                Actions.runBlocking(IntakeSeq.IntakeStoreAction(intake,outtake,feeder));

//                Globals.counterFeed_GPPtoPGP =;
            Actions.runBlocking(
                    new ParallelAction(
                            Intake.IntakeCommand(Intake.IntakeServoState.IN),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON),
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                            Outtake.TurretCommand(Outtake.TurretState.INIT)
                    )
            );
        }
//            if ()
//            {
//                Actions.runBlocking(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
//                        )
//                );
//            }

        if (!robot.intakeBeam.getState() && Globals.counterFeed_GPPtoPPG==0 && Globals.intakeFlag_GPPtoPPG){
            Globals.counterFeed_GPPtoPPG += 1;
            Actions.runBlocking(
                    new ParallelAction(
                            Intake.IntakeCommand(Intake.IntakeServoState.IN),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON),
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                            Outtake.TurretCommand(Outtake.TurretState.INIT)
                    )
            );

        }
        if (robot.intakeBeam.getState() && Globals.counterFeed_GPPtoPPG ==1 && Globals.intakeFlag_GPPtoPPG){
            Globals.counterFeed_GPPtoPPG += 1;
            Actions.runBlocking(
                    new ParallelAction(
                            Intake.IntakeCommand(Intake.IntakeServoState.IN),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON),
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.ON),
                            Outtake.TurretCommand(Outtake.TurretState.INIT)
                    )
            );
        }
        if (Globals.counterFeed_GPPtoPPG==2 && !robot.feederBeam.getState()&& Globals.intakeFlag_GPPtoPPG){
            Globals.counterFeed_GPPtoPPG += 1;
//                zeroFlag = true;
        }
        if (Globals.counterFeed_GPPtoPPG==3 && robot.feederBeam.getState()&& Globals.intakeFlag_GPPtoPPG){
            Globals.counterFeed_GPPtoPPG += 1;

        }

        if (Globals.counterFeed_GPPtoPPG==4 && !robot.intakeBeam.getState()&& Globals.intakeFlag_GPPtoPPG){
            Globals.counterFeed_GPPtoPPG += 1;
//                zeroFlag = false;

        }
        if (Globals.counterFeed_GPPtoPPG==5 && robot.intakeBeam.getState()&& Globals.intakeFlag_GPPtoPPG){
//                motionFlag = true;
//                motionTimer.reset();
            Globals.counterFeed_GPPtoPPG = 0;
            gamepad1.rumble(1,1,400);
            Actions.runBlocking(
                    new ParallelAction(
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                            Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                    )
            );
            Globals.intakeFlag_GPPtoPPG = false;
        }



    }

    public static void zeroFlag()
    {
        Globals.intakeFlag_PGPtoPPG = false;
        Globals.intakeFlag_GPPtoPPG = false;
        Globals.intakeFlag_GPPtoPGP = false;
        Globals.intakeFlag_PPGtoGPP = false;
        Globals.intakeFlag = false;
        Globals.counterFeed_PGPtoPPG = 0;
        Globals.counterFeed_GPPtoPPG = 0;
        Globals.counterFeed_GPPtoPGP = 0;
        Globals.counterFeed_PPGtoGPP = 0;
        Globals.counterFeed = 0;
    }




    public static void Sequence_6(){
        if (Globals.intakeFlag_GPPtoPGP && Globals.counterFeed_GPPtoPGP ==0)
        {
//                thirdBallFlag = false;
//                Actions.runBlocking(IntakeSeq.IntakeStoreAction(intake,outtake,feeder));

//                Globals.counterFeed_GPPtoPGP =;
            Actions.runBlocking(
                    new ParallelAction(
                            Intake.IntakeCommand(Intake.IntakeServoState.IN),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON),
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                            Outtake.TurretCommand(Outtake.TurretState.INIT)
                    )
            );
        }
//            if (!Globals.intakeFlag_GPPtoPGP)
//            {
//                Actions.runBlocking(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
//                        )
//                );
//            }

        if (!robot.intakeBeam.getState() && Globals.counterFeed_GPPtoPGP==0 && Globals.intakeFlag_GPPtoPGP){
            Globals.counterFeed_GPPtoPGP += 1;
            Actions.runBlocking(
                    new ParallelAction(
                            Intake.IntakeCommand(Intake.IntakeServoState.IN),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON),
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                            Outtake.TurretCommand(Outtake.TurretState.INIT)
                    )
            );

        }
        if (robot.intakeBeam.getState() && Globals.counterFeed_GPPtoPGP ==1 && Globals.intakeFlag_GPPtoPGP){
            Globals.counterFeed_GPPtoPGP += 1;
            Actions.runBlocking(
                    new ParallelAction(
                            Intake.IntakeCommand(Intake.IntakeServoState.IN),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON),
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.ON),
                            Outtake.TurretCommand(Outtake.TurretState.INIT)
                    )
            );
        }
        if (Globals.counterFeed_GPPtoPGP==2 && !robot.outtakeBeam.getState()&& Globals.intakeFlag_GPPtoPGP){
            Globals.counterFeed_GPPtoPGP += 1;
            Globals.thirdBallFlag = true;
            motionTimer.reset();
            Globals.motionFlag = false;
//                zeroFlag = true;
        }

        if (Globals.counterFeed_GPPtoPGP==3 && !robot.intakeBeam.getState()&& Globals.intakeFlag_GPPtoPGP){

            Actions.runBlocking(
                    new ParallelAction(
                            Intake.IntakeCommand(Intake.IntakeServoState.IN),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON),
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.ON),
                            Outtake.TurretCommand(Outtake.TurretState.INIT)
                    )
            );
            Globals.counterFeed_GPPtoPGP += 1;

        }

        if (Globals.counterFeed_GPPtoPGP==4 && robot.intakeBeam.getState()&& Globals.intakeFlag_GPPtoPGP){
            Globals.counterFeed_GPPtoPGP += 1;

            Actions.runBlocking(
                    new ParallelAction(
                            Intake.IntakeCommand(Intake.IntakeServoState.IN),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON),
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.ON),
                            Outtake.TurretCommand(Outtake.TurretState.INIT)
                    )
            );
//                zeroFlag = false;

        }

        if (Globals.counterFeed_GPPtoPGP==5 && !robot.feederBeam.getState()&& Globals.intakeFlag_GPPtoPGP){
            Globals.counterFeed_GPPtoPGP += 1;

            Actions.runBlocking(
                    new ParallelAction(
                            Intake.IntakeCommand(Intake.IntakeServoState.IN),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON),
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                            Outtake.TurretCommand(Outtake.TurretState.INIT)
                    )
            );
//                zeroFlag = false;

        }



        if (Globals.counterFeed_GPPtoPGP==6 && !robot.intakeBeam.getState()&& Globals.intakeFlag_GPPtoPGP){
            Globals.counterFeed_GPPtoPGP += 1;

            Actions.runBlocking(
                    new ParallelAction(
                            Intake.IntakeCommand(Intake.IntakeServoState.IN),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON),
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                            Outtake.TurretCommand(Outtake.TurretState.INIT)
                    )
            );
//                zeroFlag = false;

        }
        if (Globals.counterFeed_GPPtoPGP==7 && robot.intakeBeam.getState()&& Globals.intakeFlag_GPPtoPGP){
            Globals.counterFeed_GPPtoPGP = 0;
            gamepad1.rumble(1,1,400);
            Actions.runBlocking(
                    new ParallelAction(
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                            Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                    )
            );
            Globals.intakeFlag_GPPtoPGP = false;
//                zeroFlag = false;

        }
    }

    public static void sortArtifacts(boolean statusin, boolean statusout, boolean statusfeed, Intake intake, Outtake outtake, Feeder feeder){
        if (!statusin && Globals.counterFeed_GPPtoPGP == 0 && Globals.intakeFlag_GPPtoPGP){
            Globals.counterFeed_GPPtoPGP += 1;
            Actions.runBlocking(
                    new ParallelAction(
                            Intake.IntakeCommand(Intake.IntakeServoState.IN),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON),
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                            Outtake.TurretCommand(Outtake.TurretState.INIT)
                    )
            );

        }
        if (statusin && Globals.counterFeed_GPPtoPGP ==1 && Globals.intakeFlag_GPPtoPGP){
            Globals.counterFeed_GPPtoPGP += 1;
            Actions.runBlocking(
                    new ParallelAction(
                            Intake.IntakeCommand(Intake.IntakeServoState.IN),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON),
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.ON),
                            Outtake.TurretCommand(Outtake.TurretState.INIT)
                    )
            );
        }
        if (Globals.counterFeed_GPPtoPGP==2 && !statusout && Globals.intakeFlag_GPPtoPGP){
            Globals.counterFeed_GPPtoPGP += 1;
            Globals.thirdBallFlag = true;
            motionTimer.reset();
            Globals.motionFlag = false;
//                zeroFlag = true;
        }

        if (Globals.counterFeed_GPPtoPGP==3 && !statusin && Globals.intakeFlag_GPPtoPGP){

            Actions.runBlocking(
                    new ParallelAction(
                            Intake.IntakeCommand(Intake.IntakeServoState.IN),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON),
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.ON),
                            Outtake.TurretCommand(Outtake.TurretState.INIT)
                    )
            );
            Globals.counterFeed_GPPtoPGP += 1;

        }

        if (Globals.counterFeed_GPPtoPGP==4 && statusin && Globals.intakeFlag_GPPtoPGP){
            Globals.counterFeed_GPPtoPGP += 1;

            Actions.runBlocking(
                    new ParallelAction(
                            Intake.IntakeCommand(Intake.IntakeServoState.IN),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON),
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.ON),
                            Outtake.TurretCommand(Outtake.TurretState.INIT)
                    )
            );
//                zeroFlag = false;

        }

        if (Globals.counterFeed_GPPtoPGP==5 && !statusfeed && Globals.intakeFlag_GPPtoPGP){
            Globals.counterFeed_GPPtoPGP += 1;

            Actions.runBlocking(
                    new ParallelAction(
                            Intake.IntakeCommand(Intake.IntakeServoState.IN),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON),
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                            Outtake.TurretCommand(Outtake.TurretState.INIT)
                    )
            );
//                zeroFlag = false;

        }



        if (Globals.counterFeed_GPPtoPGP==6 && !statusin && Globals.intakeFlag_GPPtoPGP){
            Globals.counterFeed_GPPtoPGP += 1;

            Actions.runBlocking(
                    new ParallelAction(
                            Intake.IntakeCommand(Intake.IntakeServoState.IN),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON),
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                            Outtake.TurretCommand(Outtake.TurretState.INIT)
                    )
            );
//                zeroFlag = false;

        }
        if (Globals.counterFeed_GPPtoPGP==7 && statusin && Globals.intakeFlag_GPPtoPGP){
            Globals.counterFeed_GPPtoPGP = 0;
            gamepad1.rumble(1,1,400);
            Actions.runBlocking(
                    new ParallelAction(
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                            Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                    )
            );
            Globals.intakeFlag_GPPtoPGP = false;
//                zeroFlag = false;

        }





        if (Globals.intakeFlag_PGPtoPPG  && Globals.counterFeed_PGPtoPPG == 0)
        {
//                thirdBallFlag = false;
            Actions.runBlocking(IntakeSeq.IntakeStoreAction2(intake,outtake,feeder));
        }
//            if (!Globals.intakeFlag_PGPtoPPG )
//            {
//                Actions.runBlocking(
//                        new ParallelAction(
//                                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
//                                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
//                                Intake.RollerCommand(Intake.IntakeRollerState.OFF)
//                        )
//                );
//            }

        if (!statusfeed && Globals.counterFeed_PGPtoPPG==0 && Globals.intakeFlag_PGPtoPPG ){
            Globals.counterFeed_PGPtoPPG += 1;
            Actions.runBlocking(
                    new ParallelAction(
                            Intake.RollerCommand(Intake.IntakeRollerState.ON)
                    )
            );

        }
        if (statusfeed && Globals.counterFeed_PGPtoPPG ==1 && Globals.intakeFlag_PGPtoPPG ){
            Globals.counterFeed_PGPtoPPG += 1;
            Actions.runBlocking(
                    new ParallelAction(
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                            Intake.RollerCommand(Intake.IntakeRollerState.ON)
                    )
            );
        }
        if (Globals.counterFeed_PGPtoPPG==2 && !statusin && Globals.intakeFlag_PGPtoPPG ){
            Globals.counterFeed_PGPtoPPG += 1;
//                zeroFlag = true;
        }
        if (Globals.counterFeed_PGPtoPPG==3 && statusin && Globals.intakeFlag_PGPtoPPG ){
            Globals.counterFeed_PGPtoPPG += 1;
            Actions.runBlocking(
                    new ParallelAction(
                            Feeder.LFCommand(Feeder.LowerFeederState.ON)
                    )
            );
        }

        if (Globals.counterFeed_PGPtoPPG==4 && !statusin && Globals.intakeFlag_PGPtoPPG ){
            Globals.counterFeed_PGPtoPPG += 1;
//                zeroFlag = false;

        }
        if (Globals.counterFeed_PGPtoPPG==5 && statusin && Globals.intakeFlag_PGPtoPPG ){
//                motionFlag = true;
//                motionTimer.reset();
            Globals.counterFeed_PGPtoPPG = 0;
            gamepad1.rumble(1,1,400);
            Actions.runBlocking(
                    new ParallelAction(
                            Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                            Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                            Intake.RollerCommand(Intake.IntakeRollerState.OFF)
                    )
            );
            Globals.intakeFlag_PGPtoPPG  = false;
        }


    }
}
