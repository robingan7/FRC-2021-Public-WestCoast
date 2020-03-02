package frc.robot.subsystems;

import frc.lib.other.CircularQueue;
import frc.lib.math.InterpolablePair;
import frc.lib.math.RigidTransform;
import frc.lib.math.Rotation;
import frc.lib.math.Translation2d;
import frc.robot.subsystems.NicoDrivebase;
import frc.robot.subsystems.AustinTurret.TurretState;
import frc.robot.cycles.Subsystem_Cycle;
import frc.robot.WayPoints;
import frc.robot.cycles.Cycle;
import frc.robot.cycles.ICycle_in;
import frc.robot.subsystems.AustinTurret;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState extends Subsystem_Cycle {

	private static RobotState instance = new RobotState();

	public static RobotState getInstance() {
		if(instance == null) {
			instance = new RobotState();
		}
		return instance;
	}

	private Limelight limelight;
	private NicoDrivebase driveBase;
	private RigidTransform currentOdometry;
	private CircularQueue<RigidTransform> vehicleHistory;
	private CircularQueue<Rotation> gyroHistory;

	private double currentDistance, oldDistance, deltaDistance;
	private Rotation rotationOffset;
	private Translation2d translationOffset, robotPositionByVision;

	//private AustinTurret turret;

	private final Cycle cycle = new Cycle() {
		
        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp) {
			synchronized (RobotState.this) {
				update();
			}
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
	};

	private RobotState() {
		vehicleHistory = new CircularQueue<>(100);
		gyroHistory = new CircularQueue<>(200);
		driveBase = NicoDrivebase.getInstance();
		//turret = AustinTurret.getInstance();
		limelight = Limelight.getInstance();
		currentOdometry = new RigidTransform(new Translation2d(), driveBase.getGyroAngle());
		rotationOffset = Rotation.fromDegrees(0);
		translationOffset = new Translation2d();
		robotPositionByVision = new Translation2d();
	}

	public synchronized Rotation getGyroAngle(long time) {
		return gyroHistory.getInterpolatedKey(time);
	}

	public synchronized RigidTransform getOdometry() {
		return currentOdometry;
	}

	public synchronized void resetOdometry() {
		driveBase.resetGyro();
		currentOdometry = new RigidTransform(new Translation2d().translateBy(translationOffset),
				Rotation.fromDegrees(0).rotateBy(rotationOffset));
		oldDistance = driveBase.getDistance();
	}

	/**
	 * Integrates the encoders and gyro to figure out robot position. A constant
	 * curvature is assumed
	 */
	public void update() {
		double leftDist = driveBase.getLeftDistance();
		double rightDist = driveBase.getRightDistance();

		//System.out.println("hhhh------: " + leftDist + " --- " + rightDist);

		/*
		 * Solve problem where Talon returns 0 for distance due to an error This
		 * causes an abnormal deltaPosition
		 */
		if (leftDist != 0 && rightDist != 0) {
			currentDistance = (leftDist + rightDist) / 2;
		} else {
			//System.out.println("IN DISTANCE ZEROOOOOOOO");
			return;
		}
		deltaDistance = currentDistance - oldDistance;
		Translation2d deltaPosition = new Translation2d(deltaDistance, 0);
		Rotation deltaRotation = driveBase.getGyroAngle().inverse().rotateBy(rotationOffset);
		synchronized (this) {
			deltaRotation = currentOdometry.rotationMat.inverse().rotateBy(deltaRotation);
			Rotation halfRotation = Rotation.fromRadians(deltaRotation.getRadians() / 2.0);
			currentOdometry = currentOdometry
					.transform(new RigidTransform(deltaPosition.rotateBy(halfRotation), deltaRotation));
			vehicleHistory.add(new InterpolablePair<>(System.nanoTime(), currentOdometry));
			gyroHistory.add(new InterpolablePair<>(System.nanoTime(), driveBase.getGyroAngle()));
		}
		oldDistance = currentDistance;

		// System.out.println("Position: " +
		// currentOdometry.translationMat.getX() + " " +
		// currentOdometry.translationMat.getY());
		// System.out.println("Gyro: " +
		// currentOdometry.rotationMat.getDegrees());
	}

	@Override
	public void registerEnabledCycles(ICycle_in enabledCycler) {
        enabledCycler.addSubsystem(cycle);
	}

	/**
	 *
	 * @param offset
	 */
	public synchronized void setInitialRotation(Rotation offset) {
		this.rotationOffset = offset;
	}

	public synchronized void setInitialTranslation(Translation2d offset) {
		this.translationOffset = offset;
		resetOdometry();
		//turret.resetControllers();
	}

	public synchronized void calculateRobotPositionByVision() {
		double distance = limelight.distance();
		double offset = 0;
		//double offset = turret.getAngleOffset();

		double x = distance * Math.cos(Math.toRadians(offset));
		double y = distance * Math.sin(Math.toRadians(offset));

		double xPos = WayPoints.kPowerPortPosition.getY() + y;
		double yPos = WayPoints.kPowerPortPosition.getX() + x;
		robotPositionByVision = new Translation2d(yPos, xPos);
	}

}