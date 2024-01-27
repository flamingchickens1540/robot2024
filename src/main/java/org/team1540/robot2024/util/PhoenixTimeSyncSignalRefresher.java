package org.team1540.robot2024.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;

import java.util.Arrays;

import static org.team1540.robot2024.Constants.SwerveConfig.*;

/**
 * Helper class for refreshing CANivore TimeSynced status signals.
 */
public class PhoenixTimeSyncSignalRefresher {
    private static BaseStatusSignal[] signals = new BaseStatusSignal[0];
    private static final boolean isCANFD = CANBus.isNetworkFD(CAN_BUS);

    public static void registerSignals(BaseStatusSignal... newSignals) {
        signals = Arrays.copyOf(signals, signals.length + newSignals.length);
        for (int i = 0; i < newSignals.length; i++) signals[signals.length - 1 - i] = newSignals[i];
    }

    public static void refreshSignals() {
        if (isCANFD) BaseStatusSignal.waitForAll(0.01, signals);
        else BaseStatusSignal.refreshAll(signals);
    }
}
