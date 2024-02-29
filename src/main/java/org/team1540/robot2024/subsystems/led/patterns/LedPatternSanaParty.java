package org.team1540.robot2024.subsystems.led.patterns;

import edu.wpi.first.wpilibj.util.Color;
import org.team1540.robot2024.subsystems.led.ZonedAddressableLEDBuffer;

public class LedPatternSanaParty extends LedPattern{
    private final int speed;
    private final Color color1;
    private final Color color2;
    private final Color color3;
    private final int color1Hue;
    private final int color2Hue;
    private final int color3Hue;

    public LedPatternSanaParty(int speed){
        super(true);
        this.speed = speed;
        color1 = new Color(89, 201, 165);
        color2 = new Color(216, 30, 91);
        color3 = new Color(255, 253, 152);
        color1Hue = getHue(color1);
        color2Hue = getHue(color2);
        color3Hue = getHue(color3);
    }

    @Override
    public void apply(ZonedAddressableLEDBuffer buffer) {
        for (int index = 0; index < buffer.getLength(); index++){
            if(index%3 == 0){
                buffer.setHSV(index, color1Hue, 255, 255);
            } else if (index%3 == 1){
                buffer.setHSV(index, color2Hue, 255, 255);
            } else{
                buffer.setHSV(index, color3Hue, 255, 255);
            }
        }
    }
}
