package org.firstinspires.ftc.teamcode.sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;

import org.firstinspires.ftc.teamcode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class AutoInitSeq {
    public static Action InitAction(Intake intake, Outtake outtake, Feeder feeder) {
        return new ParallelAction(
                Intake.IntakeCommand(Intake.IntakeServoState.INIT),
                Intake.RollerCommand(Intake.IntakeRollerState.OFF),
                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                Outtake.TurretCommand(Outtake.TurretState.SHOOT_NEAR),
                Outtake.ShooterCommand(Outtake.ShooterState.OFF),
                Outtake.HoodCommand(Outtake.HoodState.NEAR_END)

        );
    }

    public static Action InitActionFar(Intake intake, Outtake outtake, Feeder feeder) {
        return new ParallelAction(
                Intake.IntakeCommand(Intake.IntakeServoState.AUTO_IN),
                Intake.RollerCommand(Intake.IntakeRollerState.OFF),
                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                Outtake.TurretCommand(Outtake.TurretState.SHOOT),
                Outtake.ShooterCommand(Outtake.ShooterState.OFF),
                Outtake.HoodCommand(Outtake.HoodState.FAR)

        );
    }

    public static Action InitActionFarBlue(Intake intake, Outtake outtake, Feeder feeder) {
        return new ParallelAction(
                Intake.IntakeCommand(Intake.IntakeServoState.INIT),
                Intake.RollerCommand(Intake.IntakeRollerState.OFF),
                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                Outtake.TurretCommand(Outtake.TurretState.TRACK_OFF),
                Outtake.ShooterCommand(Outtake.ShooterState.OFF),
                Outtake.HoodCommand(Outtake.HoodState.FAR)

        );
    }

    public static Action InitActionNearBlue(Intake intake, Outtake outtake, Feeder feeder) {
        return new ParallelAction(
                Intake.IntakeCommand(Intake.IntakeServoState.INIT),
                Intake.RollerCommand(Intake.IntakeRollerState.OFF),
                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                Feeder.LFCommand(Feeder.LowerFeederState.OFF),
                Outtake.TurretCommand(Outtake.TurretState.BUFF_RED),
                Outtake.ShooterCommand(Outtake.ShooterState.OFF),
                Outtake.HoodCommand(Outtake.HoodState.NEAR_END)

        );
    }
}