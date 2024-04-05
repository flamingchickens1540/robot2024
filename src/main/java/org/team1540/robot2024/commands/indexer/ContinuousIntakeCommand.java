package org.team1540.robot2024.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.led.Leds;
import org.team1540.robot2024.subsystems.led.patterns.LedPattern;
import org.team1540.robot2024.subsystems.led.patterns.SimpleLedPattern;

public class ContinuousIntakeCommand extends Command {
    private final Indexer indexer;
    private final Leds leds;
    private final double percent;
    private final LedPattern detected = SimpleLedPattern.solid("#ffff00");
    private final Timer timer = new Timer();


    public ContinuousIntakeCommand(Indexer indexer, Leds leds, double percent) {
        this.indexer = indexer;
        this.leds = leds;
        this.percent = percent;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        if (indexer.isNoteStaged()) {
            indexer.stopIntake();
            leds.clearPatternAll(Leds.PatternLevel.INTAKE_PREREADY);
        } else {
            if (indexer.getIntakeVoltage() == 0) {
                timer.restart();
            }
            if (timer.hasElapsed(0.3) && indexer.getIntakeCurrent() > 30) {
                leds.setPatternAll(detected, Leds.PatternLevel.INTAKE_PREREADY);
            }
            indexer.setIntakePercent(percent);
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopIntake();
        leds.clearPatternAll(Leds.PatternLevel.INTAKE_PREREADY);
    }
}
