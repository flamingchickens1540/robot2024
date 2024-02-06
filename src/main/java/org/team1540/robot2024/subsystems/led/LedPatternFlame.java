package org.team1540.robot2024.subsystems.led;


import java.awt.*;
import java.util.Random;

public class LedPatternFlame extends LedPattern{
    private final int cooling;
    private final int sparking = 123;
    private boolean reverseDirection = true;
    private int[] temperatures = new int[40];

    private static Random generator = new Random();


    public LedPatternFlame(int cooling) {
        super(true);
        this.cooling = cooling;
    }

    public LedPatternFlame() {
        super(true);
        cooling = 62;
    }


    @Override
    void apply(ZonedAddressableLEDBuffer buffer) {

        for (int i = 0; i < buffer.getLength(); i++) {
            temperatures[i] = bit8Subtraction(temperatures[i], getRandomInt(0, ((cooling * 10) / buffer.getLength()) + 2));
        }

        for (int k = buffer.getLength() - 1; k >= 2; k--) {
            temperatures[k] = (temperatures[k - 1] + temperatures[k - 2] + temperatures[k - 2]) / 3;
        }

        if (getRandomInt(0, 255) < sparking) {
            int y = getRandomInt(0, 6);
            temperatures[y] = bit8Addition(temperatures[y], getRandomInt(160, 255));
        }

        for (int j = 0; j < buffer.getLength(); j++) {
            int temperature = temperatures[j];
            Color color = getHeatColor(temperature);
            int pixelnumber = reverseDirection ? (buffer.getLength() - 1) - j : j;
            buffer.setRGB(pixelnumber, color.getRed(), color.getGreen(), color.getBlue());
        }
    }
    private Color getHeatColor(int temperature) {
        Color finalColor;
        int scaledTemperature = (temperature * 191) / 255;
        int theHeat = scaledTemperature & 0x3F;
        theHeat <<= 2;
        if ((scaledTemperature & 0x80) != 0) {
            finalColor = new Color(255, 255, theHeat);
        } else if (((scaledTemperature & 0x40) != 0)) {
            finalColor = new Color(255, theHeat, 0);
        } else {
            finalColor = new Color(theHeat, 0, 0);
        }
        return finalColor;
    }

    private int getRandomInt(int min, int max) {
        return generator.nextInt(max - min + 1) + min;
    }

    private int bit8Subtraction(int a, int b) {
        return (a < b) ? 0 : (a - b);
    }

    private int bit8Addition(int a, int b) {
        return Math.min((a + b), 255);
    }
}
