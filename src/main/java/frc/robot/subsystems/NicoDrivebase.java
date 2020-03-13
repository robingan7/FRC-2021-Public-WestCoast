package frc.robot.subsystems;

import frc.robot.Constants;
import frc.lib.utility.Utility;
import frc.lib.control.RateLimiter;
import frc.lib.control.SynchronousPid;
import frc.lib.control.Path;
import frc.lib.control.PurePursuitController;
import frc.lib.math.Rotation;
import frc.robot.subsystems.RobotState;
import frc.robot.cycles.Subsystem_Cycle;
import frc.robot.cycles.Cycle;
import frc.robot.cycles.ICycle_in;
import frc.lib.utility.DriveSignal;
import frc.lib.motor.MotorUtility;
import frc.lib.math.Translation2d;
import frc.lib.control.SynchronousPIDController;
import frc.lib.utility.SM_HDrive;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import com.kauailabs.navx.frc.AHRS;

public class NicoDrivebase extends Subsystem_Cycle {

	public enum DriveState {
		TELEOP, PUREPURSUIT, TURN, DONE
	}

	public static class AutoDriveSignal {
		public DriveSignal command;
		public boolean isDone;

		public AutoDriveSignal(DriveSignal command, boolean isDone) {
			this.command = command;
			this.isDone = isDone;
		}
	}

	public static class FeedData {
        // Motion Profile Feedback
        public double timestamp;
        public int left_position_ticks;
        public int right_position_ticks;
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;
        public Rotation gyro_heading = Rotation.fromDegrees(0);
        public double left_voltage;
        public double right_voltage;

        //Feedforward 
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
		public double right_feedforward;
		public double middle_feedforward;
		public double middle_accel;

		public Translation2d robotInitialPosition;
		public double xDisplacment;
		public double yDisplacement;
    }

	private FeedData feedData_ = new FeedData(); 
	private static NicoDrivebase instance = new NicoDrivebase();

	public static NicoDrivebase getInstance() {
		if(instance == null) {
			instance = new NicoDrivebase();
		}
		return instance;
	}

	private AHRS gyroSensor = new AHRS(SPI.Port.kMXP);
	private TalonSRX leftMasterTalon, rightMasterTalon, middleWheel;
	private VictorSPX leftSlaveTalon, leftSlave2Talon, rightSlaveTalon, rightSlave2Talon;
	private PurePursuitController autonomousDriver;
	private SynchronousPid turnPID;
	private DriveState driveState;
	private RateLimiter moveProfiler, turnProfiler;
	private Solenoid shifter;
	private Rotation wantedHeading;
	private volatile double driveMultiplier;
	private Limelight limelight_ = Limelight.getInstance();
	private SynchronousPIDController middleWheelController;

	private final Cycle cycle = new Cycle() {
        @Override
        public void onStart(double timestamp) {
            synchronized (NicoDrivebase.this) {
                setWheelVelocity(new DriveSignal(0.05, 0.05));
            }
        }

        @Override
        public void onLoop(double timestamp) {
			synchronized (NicoDrivebase.this) {
				switch (driveState) {
					case TELEOP:
						break;
					case DONE:
						break;
					case PUREPURSUIT:
						updatePurePursuit();
						break;
					case TURN:
						updateTurn();
						break;
					default:
						System.out.println("Drive state is not valid" + driveState);
						break;		
				}
			}
			
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
	};
	
	private NicoDrivebase() {

		//shifter = new Solenoid(Constants.kGearShifter);
		leftMasterTalon = MotorUtility.createTalon(Constants.kLeftDriveMasterId);
		rightMasterTalon = MotorUtility.createTalon(Constants.kRightDriveMasterId);
		middleWheel = MotorUtility.createTalon(Constants.kMiddleWheelId);

		leftMasterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		rightMasterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

		leftSlaveTalon = MotorUtility.createVictor(Constants.kLeftDriveSlaveAId);
		leftSlave2Talon = MotorUtility.createVictor(Constants.kLeftDriveSlaveBId);
		rightSlaveTalon = MotorUtility.createVictor(Constants.kRightDriveSlaveAId);
		rightSlave2Talon = MotorUtility.createVictor(Constants.kRightDriveSlaveBId);
		
		leftMasterTalon.configFactoryDefault();
		rightMasterTalon.configFactoryDefault();
		leftSlaveTalon.configFactoryDefault();
		rightSlaveTalon.configFactoryDefault();
		leftSlave2Talon.configFactoryDefault();
		rightSlave2Talon.configFactoryDefault();
		configMotors();

		driveState = DriveState.DONE;

		turnPID = new SynchronousPid(1.0, 0, 1.2, 0); //P=1.0 OR 0.8
		turnPID.setOutputRange(Constants.HighDriveSpeed, -Constants.HighDriveSpeed);
		turnPID.setSetpoint(0);

		moveProfiler = new RateLimiter(Constants.TeleopAccLimit);
		turnProfiler = new RateLimiter(100);

		middleWheelController = new SynchronousPIDController(0.002, 0, 0);//0.002, 0.00003, 0.0001
		middleWheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		configHigh();
	}

	private void configAuto() {
		rightMasterTalon.config_kP(0, Constants.kRightAutoP, 10);
		rightMasterTalon.config_kD(0, Constants.kRightAutoD, 10);
		rightMasterTalon.config_kF(0, Constants.kRightAutoF, 10);
		leftMasterTalon.config_kP(0, Constants.kLeftAutoP, 10);
		leftMasterTalon.config_kD(0, Constants.kRightAutoD, 10);
		leftMasterTalon.config_kF(0, Constants.kLeftAutoF, 10);
		middleWheel.config_kP(0, Constants.kLeftAutoP, 10);
		middleWheel.config_kD(0, Constants.kRightAutoD, 10);
		middleWheel.config_kF(0, Constants.kLeftAutoF, 10);

		driveMultiplier = Constants.HighDriveSpeed;
		rightMasterTalon.configClosedloopRamp(12d / 200d, 10);
		middleWheel.configClosedloopRamp(12d / 200d, 10);
		leftMasterTalon.configClosedloopRamp(12d / 200d, 10);
	}

	private void configHigh() {
		rightMasterTalon.config_kP(0, Constants.kRightHighP, 10);
		rightMasterTalon.config_kD(0, Constants.kRightHighD, 10);
		rightMasterTalon.config_kF(0, Constants.kRightHighF, 10);
		rightMasterTalon.configClosedloopRamp(12d / 200d, 10);
		middleWheel.config_kP(0, Constants.kRightHighP, 10);
		middleWheel.config_kD(0, Constants.kRightHighD, 10);
		middleWheel.config_kI(0, 0.00001, 10);
		middleWheel.config_kF(0, Constants.kRightHighF, 10);
		middleWheel.configClosedloopRamp(12d / 200d, 10);
		leftMasterTalon.config_kP(0, Constants.kLeftHighP, 10);
		leftMasterTalon.config_kD(0, Constants.kRightHighD, 10);
		leftMasterTalon.config_kF(0, Constants.kLeftHighF, 10);
		leftMasterTalon.configClosedloopRamp(12d / 200d, 10);

		driveMultiplier = Constants.HighDriveSpeed;
	}

	private void configLow() {
		rightMasterTalon.config_kP(0, Constants.kRightLowP, 10);
		rightMasterTalon.config_kF(0, Constants.kRightLowF, 10);
		middleWheel.config_kP(0, Constants.kRightLowP, 10);
		middleWheel.config_kF(0, Constants.kRightLowF, 10);
		leftMasterTalon.config_kP(0, Constants.kLeftLowP, 10);
		leftMasterTalon.config_kF(0, Constants.kLeftLowF, 10);

		middleWheel.config_kI(0, 0.00001, 10);
		driveMultiplier = Constants.LowDriveSpeed;
	}

	public synchronized void setInitalRobotPosition(Translation2d point) {
		feedData_.robotInitialPosition = point;
	}

	public synchronized Translation2d getCurrentRobotPostion() {
		return new Translation2d(feedData_.robotInitialPosition.getX() +  feedData_.xDisplacment,
								feedData_.robotInitialPosition.getY() + feedData_.yDisplacement);
	}


	public void arcadeDrive(double moveValue, double rotateValue) {
		synchronized (this) {
			driveState = DriveState.TELEOP;
		}
		moveValue = scaleJoystickValues(moveValue);
		rotateValue = scaleJoystickValues(rotateValue);

		double leftMotorSpeed;
		double rightMotorSpeed;
		// Square values but keep sign
		moveValue = Math.copySign(Math.pow(moveValue, 2), moveValue);
		rotateValue = Math.copySign(Math.pow(rotateValue, 2), rotateValue);
		double maxValue = Math.abs(moveValue) + Math.abs(rotateValue);
		if (maxValue > 1) {
			moveValue -= Math.copySign(maxValue - 1, moveValue);
		}
		leftMotorSpeed = moveValue + rotateValue;
		rightMotorSpeed = moveValue - rotateValue;
		setOpenLoop(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
	}

	/**
	 * 
	 * @param forward
	 * @param turn
	 * 
	 * from 5805 original sandbox. shout to 2018 programming team & Joshua Ferrara 
	 */
	public void arcadeDrive5805(double forward, double turn) {
        double leftMotorOutput;
        double maxInputL = Math.copySign(Math.max(Math.abs(forward), Math.abs(turn)), forward);
        
        //find percent -1 to 1
        if (forward >= 0.0) {
            // First quadrant, else second quadrant
            if (turn >= 0.0) {
              leftMotorOutput = maxInputL;
            } else {
              leftMotorOutput = forward + turn;
            }
          } else {
            // Third quadrant, else fourth quadrant
            if (turn >= 0.0) {
              leftMotorOutput = forward + turn;
            } else {
              leftMotorOutput = maxInputL;
            }
          }
        
		  feedData_.left_feedforward = leftMotorOutput;
        
        double rightMotorOutput;
        double maxInputR = Math.copySign(Math.max(Math.abs(forward), Math.abs(turn)), forward);
        
        //find percent -1 to 1
        if (forward >= 0.0) {
            // First quadrant, else second quadrant
            if (turn >= 0.0) {
              rightMotorOutput = forward - turn;
            } else {
              rightMotorOutput = maxInputR;
            }
          } else {
            // Third quadrant, else fourth quadrant
            if (turn >= 0.0) {
              rightMotorOutput = maxInputR;
            } else {
              rightMotorOutput = forward - turn;
            }
          }
        feedData_.right_feedforward = rightMotorOutput;
	}

	public void arcadeDrive(double moveValue, double rotateValue, double middle) {
		double[] drive_values = SM_HDrive.getHDriveOutput(moveValue, middle);
		arcadeDrive(drive_values[0], rotateValue);

		feedData_.middle_feedforward = filterHDrive(drive_values[1]);
	}

	private void configMotors() {
		leftSlaveTalon.follow(leftMasterTalon);
		leftSlave2Talon.follow(leftMasterTalon);
		rightSlaveTalon.follow(rightMasterTalon);
		rightSlave2Talon.follow(rightMasterTalon);

		leftMasterTalon.setInverted(true);
		leftSlaveTalon.setInverted(true);
		leftSlave2Talon.setInverted(true);

		rightSlaveTalon.setInverted(false);
		rightSlave2Talon.setInverted(false);
		rightMasterTalon.setInverted(false);

		leftMasterTalon.setSensorPhase(false);
		rightMasterTalon.setSensorPhase(false);
		middleWheel.setSensorPhase(false);

		rightMasterTalon.setNeutralMode(NeutralMode.Brake);
		middleWheel.setNeutralMode(NeutralMode.Brake);
		leftMasterTalon.setNeutralMode(NeutralMode.Brake);
		rightSlaveTalon.setNeutralMode(NeutralMode.Brake);
		leftSlaveTalon.setNeutralMode(NeutralMode.Brake);
		rightSlave2Talon.setNeutralMode(NeutralMode.Brake);
		leftSlave2Talon.setNeutralMode(NeutralMode.Brake);
	}

	public synchronized void resetMotionProfile() {
		moveProfiler.reset();
	}

	public synchronized double getAngle() {
		return gyroSensor.getYaw();
	}

	public synchronized double getDistance() {
		return (getLeftDistance() + getRightDistance()) / 2;
	}

	public synchronized Rotation getGyroAngle() {
		// -180 through 180
		return Rotation.fromDegrees(gyroSensor.getAngle());
	}

	public synchronized double getLeftDistance() {
		return leftMasterTalon.getSelectedSensorPosition(0) / Constants.SensorTicksPerMotorRotation * Constants.kWheelDiameter
				* Math.PI * Constants.kLowGear / Constants.kHighGear / Constants.kGearNumberOfStages;
	}

	public synchronized double getRightDistance() {
		return rightMasterTalon.getSelectedSensorPosition(0) / Constants.SensorTicksPerMotorRotation * Constants.kWheelDiameter
				* Math.PI * Constants.kLowGear / Constants.kHighGear / Constants.kGearNumberOfStages;
	}

	public synchronized double getSpeed() {
		return ((leftMasterTalon.getSelectedSensorVelocity(0) + rightMasterTalon.getSelectedSensorVelocity(0))
				/ Constants.SensorTicksPerMotorRotation) / 10 / 2 * Constants.kWheelDiameter * Math.PI;
	}

	public synchronized double getLeftSpeed() {
		return leftMasterTalon.getSelectedSensorVelocity(0) / Constants.SensorTicksPerMotorRotation * 10
				* Constants.kWheelDiameter * Math.PI * Constants.kLowGear / Constants.kHighGear / Constants.kGearNumberOfStages;
	}

	public synchronized double getRightSpeed() {
		return rightMasterTalon.getSelectedSensorVelocity(0) / Constants.SensorTicksPerMotorRotation * 10
				* Constants.kWheelDiameter * Math.PI * Constants.kLowGear / Constants.kHighGear / Constants.kGearNumberOfStages;
	}

	public synchronized double scaleJoystickValues(double rawValue) {
		return Math.copySign(Utility.clampThenNormalize(Math.abs(rawValue), Constants.MinimumControllerInput,
				Constants.MaximumControllerInput, Constants.MinimumControllerOutput, Constants.MaximumControllerOutput),
				rawValue);
	}

	public synchronized void setAutoPath(Path autoPath, boolean isReversed) {
		driveState = DriveState.PUREPURSUIT;
		autonomousDriver = new PurePursuitController(autoPath, isReversed);
		autonomousDriver.resetTime();
		configAuto();
		updatePurePursuit();
	}

	public synchronized double getVoltage() {
		return (leftMasterTalon.getMotorOutputVoltage() + rightMasterTalon.getMotorOutputVoltage()
				+ leftSlaveTalon.getMotorOutputVoltage() + rightSlaveTalon.getMotorOutputVoltage()
				+ rightSlave2Talon.getMotorOutputVoltage() + leftSlave2Talon.getMotorOutputVoltage()) / 6;
	}

	private void setOpenLoop(DriveSignal setVelocity) {
		feedData_.left_feedforward = setVelocity.leftVelocity;
		feedData_.right_feedforward = setVelocity.rightVelocity;
	}

	private void setWheelVelocity(DriveSignal setVelocity) {
		if (Math.abs(setVelocity.rightVelocity) > Constants.HighDriveSpeed
				|| Math.abs(setVelocity.leftVelocity) > Constants.HighDriveSpeed) {
			DriverStation.getInstance();
			DriverStation.reportError("Velocity set over " + Constants.HighDriveSpeed + " !", false);
			return;
		}
		
		// inches per sec to rotations per min
		double leftSetpoint = (setVelocity.rightVelocity) * Constants.SensorTicksPerMotorRotation / (Constants.kWheelDiameter * Math.PI * 10)
				* (Constants.kHighGear / Constants.kLowGear) * Constants.kGearNumberOfStages;
		double rightSetpoint = (setVelocity.leftVelocity) * Constants.SensorTicksPerMotorRotation / (Constants.kWheelDiameter * Math.PI * 10) * (Constants.kHighGear / Constants.kLowGear)
				* Constants.kGearNumberOfStages;

		feedData_.left_feedforward = leftSetpoint;
		feedData_.right_feedforward = rightSetpoint;
	}


	public void setRotation(Rotation angle) {
		synchronized (this) {
			wantedHeading = angle;
			driveState = DriveState.TURN;
		}
		configHigh();
	}

	private void updateTurn() {
		double error = wantedHeading.rotateBy(RobotState.getInstance().getOdometry().rotationMat.inverse()).getDegrees();
		double deltaSpeed;
		//System.out.println(RobotState.getInstance().getOdometry().rotationMat.getDegrees());
		//System.out.println("error: " + error);
		deltaSpeed = turnPID.update(error);
		deltaSpeed = Math.copySign(
				Utility.clampThenNormalize(Math.abs(deltaSpeed), 0, 180, 0, Constants.HighDriveSpeed), deltaSpeed);

		if (Math.abs(error) < 3) { // was 2
			setWheelVelocity(new DriveSignal(0, 0));
			synchronized (this) {
				driveState = DriveState.DONE;
			} 
		} else {
			setWheelVelocity(new DriveSignal(-deltaSpeed, deltaSpeed));
		}
	}

	public synchronized void setShiftState(boolean state) {
		shifter.set(state);
		if (state) {
			configLow();
		} else {
			configHigh();
		}
	}

	private void updatePurePursuit() {
		AutoDriveSignal signal = autonomousDriver.calculate(RobotState.getInstance().getOdometry());
		if (signal.isDone) {
			synchronized (this) {
				driveState = DriveState.DONE;
			}
			configHigh();
		}

		setWheelVelocity(signal.command);
	}

	public void resetGyro() {
		gyroSensor.zeroYaw();
	}

	public void resetGyroDisplacement() {
		gyroSensor.resetDisplacement();
	}

	public double filterHDrive(double input) {
		if(Math.abs(input) > 0.15) {
			return input;
		}

		return 0;
	}

	@Override
	public synchronized void move_subsystem() {
		//System.out.println(feedData_.left_feedforward + " ----- " + feedData_.right_feedforward);
		if(driveState == DriveState.PUREPURSUIT || driveState == DriveState.TURN) {
			leftMasterTalon.set(ControlMode.Velocity, feedData_.left_feedforward);
			rightMasterTalon.set(ControlMode.Velocity, feedData_.right_feedforward);
		} else if(driveState == DriveState.TELEOP) {
			leftMasterTalon.set(ControlMode.PercentOutput, feedData_.left_feedforward);
			rightMasterTalon.set(ControlMode.PercentOutput, feedData_.right_feedforward);

			//feedData_.middle_feedforward = middleWheelController.calculate(middleWheel.getSelectedSensorPosition(), 10000);
			middleWheel.set(ControlMode.PercentOutput, -feedData_.middle_feedforward);
		} else {
			leftMasterTalon.set(ControlMode.PercentOutput, 0);
			rightMasterTalon.set(ControlMode.PercentOutput, 0);
			middleWheel.set(ControlMode.PercentOutput, 0);
		}
	}

	@Override
	public synchronized void update_subsystem() {
		double prevLeftTicks = feedData_.left_position_ticks;
		double prevRightTicks = feedData_.right_position_ticks;
		
        feedData_.left_position_ticks = leftMasterTalon.getSelectedSensorPosition(0);
        feedData_.right_position_ticks = rightMasterTalon.getSelectedSensorPosition(0);
        feedData_.left_velocity_ticks_per_100ms = leftMasterTalon.getSelectedSensorVelocity(0);
        feedData_.right_velocity_ticks_per_100ms = rightMasterTalon.getSelectedSensorVelocity(0);
        feedData_.gyro_heading = Rotation.fromDegrees(getAngle());

        double deltaLeftTicks = ((feedData_.left_position_ticks - prevLeftTicks) / Constants.SensorTicksPerMotorRotation) * Math.PI;
        if (deltaLeftTicks > 0.0) {
            feedData_.left_distance += deltaLeftTicks * Constants.kWheelDiameter;
        } else {
            feedData_.left_distance += deltaLeftTicks * Constants.kWheelDiameter;
        }

		double deltaRightTicks = ((feedData_.right_position_ticks - prevRightTicks) / Constants.SensorTicksPerMotorRotation) * Math.PI;
		
        if (deltaRightTicks > 0.0) {
            feedData_.right_distance += deltaRightTicks * Constants.kWheelDiameter;
        } else {
            feedData_.right_distance += deltaRightTicks * Constants.kWheelDiameter;
		}
		
		feedData_.xDisplacment = Utility.meterToInch(gyroSensor.getDisplacementY());
		feedData_.yDisplacement = Utility.meterToInch(gyroSensor.getDisplacementX());
	}

	@Override
	public boolean checkSubsystem() {
		boolean success = leftMasterTalon.getSensorCollection().getPulseWidthRiseToRiseUs() == 0;
		success = rightMasterTalon.getSensorCollection().getPulseWidthRiseToRiseUs() == 0 && success;
		configMotors();
		return success;
	}

	@Override
	public synchronized void stop() {
		setOpenLoop(new DriveSignal(0, 0));
		driveState = DriveState.DONE;
	}

	@Override
	public void registerEnabledCycles(ICycle_in enabledCycler) {
        enabledCycler.addSubsystem(cycle);
	}
	
	@Override 
    public void sendDataToSmartDashboard() {
        SmartDashboard.putNumber("right: ", rightMasterTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("left: ", leftMasterTalon.getSelectedSensorPosition());
		SmartDashboard.putNumber("midle: ", middleWheel.getSelectedSensorPosition());
		
		SmartDashboard.putNumber("Left Percent", leftMasterTalon.getMotorOutputPercent());
		SmartDashboard.putNumber("Right Percent", rightMasterTalon.getMotorOutputPercent());
		SmartDashboard.putNumber("Middle Percent", middleWheel.getMotorOutputPercent());

		SmartDashboard.putNumber("Left Velocity", leftMasterTalon.getSelectedSensorVelocity());
		SmartDashboard.putNumber("Right Velocity", rightMasterTalon.getSelectedSensorVelocity());
		SmartDashboard.putNumber("Middle Velocity", middleWheel.getSelectedSensorVelocity());
	}
	
	public synchronized boolean isFinished() {
		return driveState == DriveState.DONE;
	}

}
