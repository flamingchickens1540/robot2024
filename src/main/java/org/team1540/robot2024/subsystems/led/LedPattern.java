package org.team1540.robot2024.subsystems.led;

public abstract class LedPattern {
    private final boolean isDynamic;

    protected LedPattern(boolean isDynamic) {
        this.isDynamic = isDynamic;
    }

    boolean isDynamic() {
        return isDynamic;
    }

    abstract void apply(ZonedAddressableLEDBuffer buffer);

}
