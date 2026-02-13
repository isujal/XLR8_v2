package org.firstinspires.ftc.teamcode.sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class IntakeSeq {
    public static RobotHardware robot = RobotHardware.getInstance();

    public static Action IntakeStoreAction(Intake intake, Outtake outtake, Feeder feeder) {
        return new ParallelAction(
                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                Intake.RollerCommand(Intake.IntakeRollerState.ON),
                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                Feeder.LFCommand(Feeder.LowerFeederState.ON)
//                Outtake.TurretCommand(Outtake.TurretState.INIT)
//                Outtake.HoodCommand(Outtake.HoodState.INIT)

        );
    }
    public static Action IntakeStoreAction2(Intake intake, Outtake outtake, Feeder feeder) {
        return new ParallelAction(
                Intake.IntakeCommand(Intake.IntakeServoState.IN),
                Intake.RollerCommand(Intake.IntakeRollerState.ON),
                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                Feeder.LFCommand(Feeder.LowerFeederState.ON)
//                Outtake.TurretCommand(Outtake.TurretState.SHOOT)
//                Outtake.HoodCommand(Outtake.HoodState.INIT)

        );
    }
    public static Action IntakeStoreLockAction(Intake intake, Outtake outtake, Feeder feeder) {
        return new ParallelAction(
                Feeder.UFCommand(Feeder.UpperFeederState.OFF),
                Feeder.LFCommand(Feeder.LowerFeederState.OFF)

        );
    }
}