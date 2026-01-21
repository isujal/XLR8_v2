package org.firstinspires.ftc.teamcode.prototyping;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

public class WaitForSensorAction implements Action {

    private final BooleanSupplier condition;
    private final long timeoutMs;
    private final long debounceMs;
    private final boolean waitForFalse;
    private final boolean triggerOnChange;

    private boolean lastState;
    private long startTime = 0;
    private final ElapsedTime debounceTimer = new ElapsedTime();

    public WaitForSensorAction(BooleanSupplier condition,
                               long timeoutMs,
                               long debounceMs,
                               boolean waitForFalse,
                               boolean triggerOnChange) {
        this.condition = condition;
        this.timeoutMs = timeoutMs;
        this.debounceMs = debounceMs;
        this.waitForFalse = waitForFalse;
        this.triggerOnChange = triggerOnChange;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        // -------- initialize() equivalent --------
        if (startTime == 0) {
            startTime = System.currentTimeMillis();
            debounceTimer.reset();
            try {
                lastState = condition.getAsBoolean();
            } catch (Exception e) {
                lastState = false;
            }
        }

        // -------- Timeout FIRST --------
        if (timeoutMs > 0 && (System.currentTimeMillis() - startTime) >= timeoutMs) {
            return false;
        }

        boolean sensorValue;
        try {
            sensorValue = condition.getAsBoolean();
        } catch (Exception e) {
            sensorValue = false;
        }

        // -------- triggerOnChange --------
        if (triggerOnChange) {
            if (sensorValue != lastState) {
                lastState = sensorValue;   // maintain identical state progression
                return false;
            }
        }

        // -------- Desired state --------
        boolean desiredStateMet = waitForFalse ? !sensorValue : sensorValue;

        // -------- Debounce --------
        if (desiredStateMet) {
            if (debounceTimer.milliseconds() >= debounceMs) {
                return false;
            }
        } else {
            debounceTimer.reset();
        }

        lastState = sensorValue;   // EXACT placement from FTCLib logic

        return true; // keep running
    }
}
