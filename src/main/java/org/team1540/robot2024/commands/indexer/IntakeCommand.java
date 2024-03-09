package org.team1540.robot2024.commands.indexer;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2024.subsystems.indexer.Indexer;

import java.util.function.BooleanSupplier;

public class IntakeCommand extends Command {
    private final Indexer indexer;
    private final double percent;
    private final boolean shouldUseBeambreak;
    private final BooleanSupplier noteInTramp;
    private final XboxController driver;
    private final XboxController copilot;

    public IntakeCommand(Indexer indexer, BooleanSupplier noteInTramp, double percent, XboxController driver, XboxController copilot) {
        this.indexer = indexer;
        this.noteInTramp = noteInTramp;
        this.percent = percent;
        this.shouldUseBeambreak = true;
        this.driver = driver;
        this.copilot = copilot;
        addRequirements(indexer);
    }

    public IntakeCommand(Indexer indexer, BooleanSupplier noteInTramp, double percent) {
        this(indexer, noteInTramp, percent, null, null);
    }

    public IntakeCommand(Indexer indexer, BooleanSupplier noteInTramp, double percent, boolean shouldUseBeambreak, XboxController driver, XboxController copilot) {
        this.indexer = indexer;
        this.noteInTramp = noteInTramp;
        this.percent = percent;
        this.shouldUseBeambreak = shouldUseBeambreak;
        this.driver = driver;
        this.copilot = copilot;
        addRequirements(indexer);
    }

    public IntakeCommand(Indexer indexer, BooleanSupplier noteInTramp, double percent, boolean shouldUseBeambreak) {
        this(indexer, noteInTramp, percent, shouldUseBeambreak, null, null);
    }

    @Override
    public void initialize() {
        indexer.setIntakePercent(percent);
        if (copilot != null) copilot.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
        if (driver != null) driver.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
    }

    @Override
    public boolean isFinished() {
        return shouldUseBeambreak && (noteInTramp.getAsBoolean() || indexer.isNoteStaged());
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopIntake();
        if (copilot != null) copilot.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        if (driver != null) driver.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
    }
}
