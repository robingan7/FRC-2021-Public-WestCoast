package frc.robot.actions;

import edu.wpi.first.wpilibj.Timer;

public class DelayAction implements Action {

	private double seconds;
	private double start;

	public DelayAction(double seconds) {
		this.seconds = seconds;
	}

	@Override
	public void start() {
		Timer.delay(seconds);
		start = Timer.getFPGATimestamp();
	}

	@Override
	public boolean isFinished() {
		return Math.abs(Timer.getFPGATimestamp() - start) < seconds;
    }
    
    @Override
    public void update() {}

    @Override
    public void done() {}

}
