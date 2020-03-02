package frc.robot.actions;

import java.util.ArrayList;

import frc.lib.math.Translation2d;
import frc.robot.subsystems.NicoDrivebase;
import frc.lib.control.Path;
import frc.robot.subsystems.RobotState;

public class DriveToPointsAction implements Action {

	private ArrayList<Translation2d> points;
	private double speed;
	private boolean isReversed;

	public DriveToPointsAction(double speed, boolean isReversed, Translation2d... points) {
		this.points = new ArrayList<Translation2d>();
		this.speed = speed;
		this.isReversed = isReversed;
		for (Translation2d point : points) {
			this.points.add(point);
		}
	}

	@Override
	public void start() {
		System.out.println("Drive To Points");
		Path drivePath = new Path(RobotState.getInstance().getOdometry().translationMat);
		for (Translation2d point : points) {
			drivePath.addPoint(point.getX(), point.getY(), speed);
		}
		NicoDrivebase.getInstance().setAutoPath(drivePath, isReversed);
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