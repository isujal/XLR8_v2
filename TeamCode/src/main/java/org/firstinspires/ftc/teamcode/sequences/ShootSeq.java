package org.firstinspires.ftc.teamcode.sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class ShootSeq {
    public static RobotHardware robot = RobotHardware.getInstance();

    public static Action ShootAction(Intake intake, Outtake outtake, Feeder feeder) {
        return new ParallelAction(
                Outtake.TurretCommand(Outtake.TurretState.TRACK),
                Outtake.ShooterCommand(Outtake.ShooterState.FAR)
        );
    }


}