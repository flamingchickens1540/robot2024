package org.team1540.robot2024.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ZonedAddressableLEDBuffer {
    private final boolean isInverted;
    private final int start;
    private final int length;
    private final AddressableLEDBuffer buffer;

    public ZonedAddressableLEDBuffer(AddressableLEDBuffer buffer, int start, int end, boolean isInverted) {
        if (start > end) {
            throw new IllegalArgumentException("start must be less than end");
        }
        this.buffer = buffer;
        this.isInverted = isInverted;
        this.start = start;
        this.length = end - start;
    }

    public void setRGB(int index, int r, int g, int b) {
        buffer.setRGB(this.getAbsoluteIndex(index), r, g, b);
    }

    public void setHSV(int index, int h, int s, int v) {
        buffer.setHSV(this.getAbsoluteIndex(index), h, s, v);
    }

    public void setLED(int index, Color color) {
        buffer.setLED(this.getAbsoluteIndex(index), color);
    }

    public void setLED(int index, Color8Bit color) {
        buffer.setLED(this.getAbsoluteIndex(index), color);
    }

    public Color getLED(int index) {
        return buffer.getLED(this.getAbsoluteIndex(index));
    }

    public Color8Bit getLED8Bit(int index) {
        return buffer.getLED8Bit(this.getAbsoluteIndex(index));
    }

    public int getLength() {
        return this.length;
    }

    private int getAbsoluteIndex(int index) {
        if (index >= length) {
            DriverStation.reportWarning("led index out of bounds", false);
            return 0;
        }
        if (this.isInverted) {
            return this.start + length - 1 - index;
        } else {
            return this.start + index;
        }
    }
}
