package frc.robot.actions;

import edu.wpi.first.wpilibj.Timer;

/**
 * Action to wait for a given amount of time To use this Action, call runAction(new WaitAction(your_time))
 */
public class TimeOutAction implements Action {
    private final double timeToWait_;
    private double startTime_;

    public TimeOutAction(double timeToWait) {
        timeToWait_ = timeToWait;
    }

    @Override
    public void start() {
        startTime_ = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime_ >= timeToWait_;
    }

    @Override
    public void done() {}
}