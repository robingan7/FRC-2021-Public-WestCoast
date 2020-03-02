package frc.robot.actions;

import frc.robot.subsystems.NicoDrivebase;
import frc.lib.control.Path;

public class DrivePathAction implements Action {

	private Path robotPath;
	private boolean isReversed;

	public DrivePathAction(Path robotPath, boolean isReversed) {
		this(robotPath, isReversed, true);
	}

	public DrivePathAction(Path robotPath, boolean isReversed, boolean isBlocking) {
		System.out.println("Set Drive Path from Constructor");
		this.robotPath = robotPath;
		this.isReversed = isReversed;
	}

	@Override
	public boolean isFinished() {
		if(NicoDrivebase.getInstance().isFinished()) {
			System.out.println("pathcomplete");
			return true;
		}
		return false;
	}

	@Override
	public void start() {
		System.out.println("Startttttt Drive Path");
		NicoDrivebase.getInstance().setAutoPath(robotPath, isReversed);
    }
    
    @Override
    public void update() {}

    @Override
    public void done() {
		System.out.println("Drive path is DONEEEEEE");
		NicoDrivebase.getInstance().stop();
	}

}
