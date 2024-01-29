package org.team1540.robot2024.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.team1540.robot2024.Constants;

import java.util.Arrays;

import static org.team1540.robot2024.Constants.SwerveConfig.*;

/**
 * Helper class for refreshing CANivore TimeSynced status signals.
 */
public class PhoenixTimeSyncSignalRefresher {
    private BaseStatusSignal[] signals = new BaseStatusSignal[0];
    private double refreshFrequencyHz = -1;
    private boolean useTimeSync;
    private boolean isRefreshed;

    public PhoenixTimeSyncSignalRefresher(String canbus) {
        useTimeSync = CANBus.isNetworkFD(canbus);
    }

    /**
     * Registers status signals to be refreshed when {@link PhoenixTimeSyncSignalRefresher#refreshSignals()} is called
     */
    public void registerSignals(BaseStatusSignal... newSignals) {
        if (newSignals.length < 1) return;
        if (refreshFrequencyHz < 0) refreshFrequencyHz = newSignals[0].getAppliedUpdateFrequency();

        signals = Arrays.copyOf(signals, signals.length + newSignals.length);
        for (int i = 0; i < newSignals.length; i++) {
            // Check if we have mismatched update frequencies
            if (newSignals[i].getAppliedUpdateFrequency() != refreshFrequencyHz && useTimeSync) {
                useTimeSync = false;
                DriverStation.reportWarning(
                        "Status signals received have varying update rates, TimeSynced updates are now unavailable",
                        false);
            }
            signals[signals.length - 1 - i] = newSignals[i];
        }
    }

    /**
     * Refreshes all registered signals if not done so since {@link PhoenixTimeSyncSignalRefresher#periodic()} call
     */
    public void refreshSignals() {
        if (isRefreshed) return;

        // Refresh signals
        StatusCode status;
        if (useTimeSync) status = BaseStatusSignal.waitForAll(0.01, signals);
        else status = BaseStatusSignal.refreshAll(signals);

        // Check errors
        if (status == StatusCode.InvalidNetwork)
            DriverStation.reportError("One or more status signals passed to signal refresher has a different CAN bus, unable to refresh", false);
        if (status == StatusCode.RxTimeout)
            DriverStation.reportWarning("Refreshing signals timed out, check CAN bus wiring", false);

        isRefreshed = true;
    }

    public void periodic() {
        isRefreshed = false;
    }
}
