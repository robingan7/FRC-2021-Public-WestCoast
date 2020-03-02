package frc.robot.subsystems;

import frc.lib.utility.Utility;
import frc.robot.Constants;
import frc.robot.cycles.Cycle;
import frc.robot.cycles.ICycle_in;
import frc.robot.subsystems.Limelight.LedMode;
import frc.lib.control.SynchronousPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AustinTurret extends SingleMasterMotorSystem {
    private static AustinTurret instance = new AustinTurret();

	public static AustinTurret getInstance() {
		if(instance == null) {
			instance = new AustinTurret();
		}
		return instance;
	}

	private TurretState turretState;
	private TurretState prevState;
	private SetPointState setPointState;
	private SynchronousPIDController angleController, limelightController;
	private double pidGoal;
	private double angleOffset;
	private double prevRelativePos;

    private Limelight limelight = Limelight.getInstance();
	private NicoDrivebase drive = NicoDrivebase.getInstance();

	private final Cycle cycle = new Cycle() {
        @Override
        public void onStart(double timestamp) {
            synchronized (AustinTurret.this) {
            }
        }

        @Override
        public void onLoop(double timestamp) {
			synchronized (AustinTurret.this) {
				switch(turretState) {
					case SET_POINT:
						setAngle(false);
						break;
					case VISION:
						setLimelightAngle();
						break;
					case HOMING:
						setAngle(true);
						break;
					default:
						System.out.println("Invalid turret state: " + turretState);	
				}
			}
			
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
	};

	private AustinTurret() {
		super(Constants.kTurret);
		setState(TurretState.SET_POINT);
		setPointState = SetPointState.NORMAL;
		angleController = new SynchronousPIDController(constants_.kKp, constants_.kKi, constants_.kKd);//0.022, 0.000015, 0.002
		limelightController = new SynchronousPIDController(0.025, 0.00007, 0.0003);
		//angleController.enableContinuousInput(-180, 180);
		angleController.setTolerance(0.01);
		limelightController.enableContinuousInput(-29.8, 29.8);
		limelightController.setTolerance(0);
		pidGoal = 0;
		angleOffset = 0;
		prevRelativePos = 0;
	}

	public enum TurretState {
		SET_POINT, VISION, HOMING
	}

	private enum SetPointState {
		NORMAL, TO_POSITIVE_MAX, TO_NEGATIVE_MAX, LIMIT_TO_MAX, LIMIT_TO_MIN
	}
	
	/**
	 * @return
	 */ 
	private void setAngle(boolean isHoming) {
		double current = getAngle();
		double angle_drivebase = drive.getAngle();
		double nonNormalizeDesire = angle_drivebase + angleOffset;

		angle_drivebase = normalizeAngle(angle_drivebase);

		if(prevState == TurretState.VISION) {
			angleOffset = -(current - prevRelativePos + angle_drivebase);
			//System.out.println("Angle Offset: " + angleOffset + " --- " + current + " --- " + prevRelativePos);
		}
		prevState = turretState;

		double desire = normalizeAngle(angle_drivebase + angleOffset);
		
		if(isReachGoal(-current, desire, isHoming)) {
			prevRelativePos = current;
		}

		if(turretState == TurretState.HOMING) {
			desire = angle_drivebase - angleOffset;
			angleOffset = 0;
			prevRelativePos = 0;
			setPointState = SetPointState.NORMAL;
		}

		if(setPointState == SetPointState.LIMIT_TO_MAX 
			&& desire >= Constants.kTurretMaxAngle ) {
			desire = Constants.kTurretMaxAngle;
		} else if(setPointState == SetPointState.LIMIT_TO_MIN  
				&& desire <= Constants.kTurretMinAngle 
				) {
			desire = Constants.kTurretMinAngle;
		}

		double goal = 0;
		if(setPointState == SetPointState.NORMAL || 
			setPointState == SetPointState.LIMIT_TO_MAX ||
			setPointState == SetPointState.LIMIT_TO_MIN) {
			goal = deadZoneLimit(desire, nonNormalizeDesire);
		} else if(setPointState == SetPointState.TO_NEGATIVE_MAX) {
			//System.out.println(-current + " --- -175");
			goal = Constants.kTurretMinAngle;
		} else if(setPointState == SetPointState.TO_POSITIVE_MAX) {
			//System.out.println(-current + " --- 175");
			goal = Constants.kTurretMaxAngle;
		}

		setOpenLoop(angleController.calculate(-current, goal));
		isReachGoal(-current, goal, isHoming);
		//System.out.println("SetPoint State: " + setPointState);
	}

	private double deadZoneLimit(double desire, double actual_desire) {
		//System.out.println(desire + " --- " + actual_desire);
		if(desire >= Constants.kTurretMaxAngle) {
			if(actual_desire >= 179.5) {
				setPointState = SetPointState.TO_NEGATIVE_MAX;
				return Constants.kTurretMinAngle;
			}
			setPointState = SetPointState.LIMIT_TO_MAX;
			return Constants.kTurretMaxAngle;
		} else if(desire <= Constants.kTurretMinAngle) {
			if(actual_desire <= -179.5) {
				setPointState = SetPointState.TO_POSITIVE_MAX;
				return Constants.kTurretMaxAngle;
			}
			setPointState = SetPointState.LIMIT_TO_MIN;
			return Constants.kTurretMinAngle;
		}

		return desire;
	}

	private void setLimelightAngle() {
		prevState = TurretState.VISION;
		setOpenLoop(limelightController.calculate(limelight.xOffset(), 0));
	}

	public boolean isReachGoal(double current, double desire, boolean isHoming) {
		boolean isReach = isReachGoal(current, desire);
		if(isReach) {
			if(isHoming) {
				turretState = TurretState.SET_POINT;
			}
			//setOpenLoop(0.0);
			if(setPointState != SetPointState.NORMAL
			&& setPointState != SetPointState.LIMIT_TO_MAX
			 && setPointState != SetPointState.LIMIT_TO_MIN) {
				setPointState = SetPointState.NORMAL;
			}
		}

		return isReach;
	}

	public boolean isReachGoal(double current, double desire) {
		double diff = current - desire;
		double targetError = Constants.TurretTargetError;

		if(turretState == TurretState.VISION) {
			targetError = 0.01;
		}

		if(setPointState == SetPointState.TO_NEGATIVE_MAX || 
		setPointState == SetPointState.TO_POSITIVE_MAX) {
			targetError = 2;
		} 

		if(Math.abs(diff) < targetError && current * desire >= 0) {
			return true;
		}

		return false;
	}
	
	public double angleToFeedForward(double error) {
		return error * Constants.EncoderTicksPerDegree * (Constants.kTurrentTeeth / Constants.kTurretMotorTeeth);
	}

	public void setToAngle(double current, double angle) {
		System.out.println(current + " -- " +  angle);
		pidGoal = angleController.calculate(-current, angle);
		setOpenLoop(pidGoal);
	}
	
	/**
	 * turretDegree * DegreesPerEncoderTick = turretTeethRatio * currentPosition
	 * 
	 */
	public synchronized double getAngle() {
		double angle =  master_.getSelectedSensorPosition() * Constants.DegreesPerEncoderTick / (Constants.kTurrentTeeth / Constants.kTurretMotorTeeth);
		
		SmartDashboard.putNumber("debug angle", angle);//just for debugging
		return normalizeAngle(angle);
	}

	public double normalizeAngle(double angleInput) {
		if((int)angleInput == 0){
			return 0;
		}

		double absoulteValue = Math.abs(angleInput);

		int truncate = (int)absoulteValue / 360;

		if(truncate > 0) {
			absoulteValue = absoulteValue - truncate * 360;
		}

		if(angleInput < 0) {
			absoulteValue = 360 - absoulteValue;
		}

		return Utility.setAngleRange(absoulteValue);
	}
	
	@Override
	public synchronized void resetSensors() {
		turretState = TurretState.SET_POINT;
		master_.setSelectedSensorPosition(0, 0, 10);
		setOpenLoop(0.0);
		angleOffset = 0;
	}

	public synchronized void resetControllers() {
		angleController.reset();
		limelightController.reset();
		master_.setSelectedSensorPosition(0, 0, 10);
	}

	public synchronized void setState(TurretState state) {

		if(controlMode_ != ControlType.OPEN_LOOP) {
			controlMode_ = ControlType.OPEN_LOOP;
		}

		if(state == TurretState.VISION) {
			limelight.setLed(LedMode.ON);
		} else if(limelight.ledMode() != 1) {
			limelight.setLed(LedMode.OFF);
		}

		turretState = state;
	}

	public synchronized void addAngleOffset(double addVal) {
		angleOffset += addVal;
	}

	public synchronized double getAngleOffset() {
		return angleOffset;
	}

	@Override
	public void sendDataToSmartDashboard() {
		SmartDashboard.putNumber("Turret Tick", master_.getSelectedSensorPosition());
		SmartDashboard.putNumber("Turret Relative Angle", getAngle());
		SmartDashboard.putNumber("Turret Goal", pidGoal);
		SmartDashboard.putNumber("Turret Error", master_.getClosedLoopError());
	}

	@Override
	public void registerEnabledCycles(ICycle_in enabledCycler) {
		enabledCycler.addSubsystem(cycle);
		super.registerEnabledCycles(enabledCycler);
	}

}