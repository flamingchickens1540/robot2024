package org.team1540.robot2024.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import static org.team1540.robot2024.Constants.SwerveConfig.CAN_BUS;

public class LoggedCanivoreStats {
    private static final CanivoreStatsAutoLogged stats = new CanivoreStatsAutoLogged();
    @AutoLog
    public static class CanivoreStats {
        public int OffCount;
        public float Utilization;
        public int ReceiveErrorCount;
        public int TransmitErrorCount;
        public int TxFullCount;
        public StatusCode Status;
    }

    public static void periodic() {
        CANBus.CANBusStatus status = CANBus.getStatus(CAN_BUS);
        stats.OffCount = status.BusOffCount;
        stats.Utilization =  status.BusUtilization;
        stats.ReceiveErrorCount = status.REC;
        stats.TransmitErrorCount = status.TEC;
        stats.TxFullCount = status.TxFullCount;
        stats.Status = status.Status;
        Logger.processInputs("SystemStats/CANBus_"+CAN_BUS, stats);
    }
}
