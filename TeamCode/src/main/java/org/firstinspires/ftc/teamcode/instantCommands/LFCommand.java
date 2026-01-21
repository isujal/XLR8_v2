package org.firstinspires.ftc.teamcode.instantCommands;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystem.Feeder;

public class LFCommand {
    public LFCommand(Feeder feeder, Feeder.LowerFeederState state) {
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction( () -> feeder.updateLFState(state))
                )
        );
    }
}
