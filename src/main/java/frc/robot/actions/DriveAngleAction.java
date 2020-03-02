package frc.robot.actions;

import frc.lib.math.Rotation;
import frc.lib.math.Translation2d;
import frc.robot.subsystems.NicoDrivebase;
import frc.robot.subsystems.RobotState;

public class DriveAngleAction implements Action {

	private Rotation angle;
	private Translation2d point;

	public DriveAngleAction(Rotation angle) {
		this.angle = angle;
	}

	public DriveAngleAction(Translation2d point) {
		this.point = point;
	}

	@Override
	public void start() {
		if (angle == null) {
			angle = RobotState.getInstance().getOdometry().translationMat.getAngle(point);
		}
		
		NicoDrivebase.getInstance().setRotation(angle);
		System.out.println("Angle: " + angle.getDegrees());
	}

	@Override
	public boolean isFinished() {
		return NicoDrivebase.getInstance().isFinished();
	}

    @Override
    public void update() {}

    @Override
    public void done() {}

}

