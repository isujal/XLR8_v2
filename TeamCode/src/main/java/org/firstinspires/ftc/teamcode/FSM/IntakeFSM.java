package org.firstinspires.ftc.teamcode.FSM;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class IntakeFSM {
    private final ElapsedTime intakeTimeout = new ElapsedTime();
    private static final double INTAKE_CONFIRM_TIMEOUT_MS = 350; // tune this
    private boolean latchedOn = false;

    private IntakeState state = IntakeState.OFF;
    private int storedCount = 0;

    private final BeamEdge feederEdge = new BeamEdge();
    private final BeamEdge intakeEdge = new BeamEdge();

    public void update(
            boolean togglePressed,
            RobotHardware robot,
            Intake intake,
            Feeder feeder
    )


    {
// 🔴 GLOBAL TOGGLE OFF KILL


        boolean feederBeam = robot.feederBeam.getState();
        boolean intakeBeam = robot.intakeBeam.getState();

        switch (state) {

            // ---------------- OFF ----------------
            case OFF:
                intake.setRoller(Intake.IntakeRollerState.OFF);
                feeder.setLF(Feeder.LowerFeederState.OFF);

                if (togglePressed) {
                    storedCount = 0;

                    feederEdge.sync(feederBeam);
                    intakeEdge.sync(intakeBeam);

                    state = IntakeState.RUNNING;
                }
                break;


            // ---------------- RUNNING ----------------
            case RUNNING:

                intake.setIntake(Intake.IntakeServoState.IN);

                if (storedCount < 2) {
                    intake.setRoller(Intake.IntakeRollerState.ON);
                } else {
                    intake.setRoller(Intake.IntakeRollerState.OFF);
                }

                if (storedCount == 0) {
                    feeder.setLF(Feeder.LowerFeederState.ON);
                } else {
                    feeder.setLF(Feeder.LowerFeederState.OFF);
                }

                if (storedCount == 0) {
                    feeder.setLF(Feeder.LowerFeederState.ON);
                } else {
                    feeder.setLF(Feeder.LowerFeederState.OFF);
                }


                // First artifact → feeder beam
                if (storedCount == 0 && feederEdge.rising(feederBeam)) {
                    state = IntakeState.WAIT_FEEDER_EXIT;
                }

                // Second / Third artifact → intake beam
                if (storedCount > 0 && intakeEdge.rising(intakeBeam)) {
                    intakeTimeout.reset();
                    state = IntakeState.WAIT_INTAKE_EXIT;
                }

                break;

            // --------- FEEDER EXIT CONFIRM ----------
            case WAIT_FEEDER_EXIT:
                if (!feederBeam) {
                    storedCount++; // = 1
                    state = IntakeState.RUNNING;

                }
                break;

            // --------- INTAKE EXIT CONFIRM ----------
            case WAIT_INTAKE_EXIT:


                // Normal exit detection
                if (!intakeBeam) {
                    storedCount++;
                    advanceAfterIntake();
                    break;
                }

                // 🔥 FORCE ADVANCE AFTER TIMEOUT (YOUR OLD counterFeed = 4)
                if (storedCount == 1 && intakeTimeout.milliseconds() > INTAKE_CONFIRM_TIMEOUT_MS) {
                    storedCount++; // assume second ball stored
                    advanceAfterIntake();
                }

                break;


            // ---------------- COMPLETE ----------------
            case COMPLETE:
                intake.setRoller(Intake.IntakeRollerState.OFF);
                feeder.setLF(Feeder.LowerFeederState.OFF);

                storedCount = 0;
                state = IntakeState.OFF;
                togglePressed = false;
                break;

        }
    }

    private void advanceAfterIntake() {
        if (storedCount >= 2) {   // 3rd ball just completed
            state = IntakeState.COMPLETE;
        } else {
            state = IntakeState.RUNNING;
        }
    }


    public IntakeState getState() { return state; }
    public int getStoredCount() { return storedCount; }


}
