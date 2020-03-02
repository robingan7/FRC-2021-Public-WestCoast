package frc.robot.actions;
/**
 * Template action for something that only needs to be done once and has no need for updates.
 *
 * @see Action
 */
public abstract class RunOnceTimeAction implements Action {
    @Override
    public void start() {
        runOnce();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {}

    public abstract void runOnce();
}