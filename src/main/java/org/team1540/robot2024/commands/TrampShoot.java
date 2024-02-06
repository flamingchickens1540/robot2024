package main.java.org.team1540.robot2024.commands;

public class TrampShoot extends Command {
    public TrampShoot(Tramp tramp) {
        this.tramp = tramp;
        addRequirements(tramp);
    }
    @Override
    public void initialize() {
        tramp.setPercent(0.5); //TODO: Tune this
    }

    @Override
    public void execute() {}

    @Override
    public void isFinished() {
        return !(tramp.isNoteStaged());
    }

    @Override
    public void end() {
        tramp.stop();
    }
}
