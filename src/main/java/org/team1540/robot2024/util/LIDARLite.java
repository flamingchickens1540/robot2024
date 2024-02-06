package org.team1540.robot2024.util;

import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import java.nio.ByteBuffer;

public class LIDARLite {
    private static final byte k_deviceAddress = 0x62; // Default device id

    private final byte m_port; // Default port is I2C.Port.kOnboard

    private final ByteBuffer m_buffer = ByteBuffer.allocateDirect(2);
    private MedianFilter filter = new MedianFilter(5);

    public LIDARLite(Port port) {
        m_port = (byte) port.value;
        I2CJNI.i2CInitialize(m_port);
    }

    public synchronized void startMeasuring() {
        writeRegister(0x04, 0x08 | 32); // Enable reference filter, averages 8 reference measurements for increased consistency
        writeRegister(0x11, 0xff); // Indefinite repetitions after initial distance measurement command
        writeRegister(0x00, 0x04); // Start measuring
    }

    public synchronized void stopMeasuring() {
        writeRegister(0x11, 0x00); // One measurement per distance measurement command. (Terminate the previous 0xff to register 0x11)
    }

    // TODO document that this returns inches or whatever?
    public synchronized double getDistance() {
        return filter.calculate(Units.inchesToMeters(getRawDistance()));
    }

    private double getRawDistance() {
        m_buffer.put(0, (byte) 0x8f); // Special combination for readShort which combines high and low byte registers
        I2CJNI.i2CWrite(m_port, k_deviceAddress, m_buffer, (byte) 1);
        I2CJNI.i2CRead(m_port, k_deviceAddress, m_buffer, (byte) 2);
        return m_buffer.getShort(0) / 2.54 + 6; // 2.54 to convert to inches and +6 tuning offset
    }

    private int writeRegister(int address, int value) {
        m_buffer.put(0, (byte) address);
        m_buffer.put(1, (byte) value);

        return I2CJNI.i2CWrite(m_port, k_deviceAddress, m_buffer, (byte) 2);
    }
}