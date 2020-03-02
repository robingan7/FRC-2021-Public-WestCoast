package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;

import frc.robot.Constants;

public class DevineShooter extends SingleMasterMotorSystem {
    private static DevineShooter instance = new DevineShooter();

	public static DevineShooter getInstance() {
        if(instance == null) {
            instance = new DevineShooter();
        }
		return instance;
    }

    public enum ShooterState {
		VELOCITY, PERCENT_OUTPUT, STOP
    }
    
    private ShooterState shooterState;
	private Servo angleChanger;
    
    private DevineShooter() {
        super(Constants.kShooter);
		angleChanger = new Servo(Constants.kShooterServoId);
        shooterState = ShooterState.STOP;
        setLowPosition();
    }

    public synchronized ShooterState getShooterState() {
        return shooterState;
    }

    public synchronized void setState(ShooterState state) {
        shooterState = state;
    }

    public synchronized void shootInAuto() {
        setOpenLoop(Constants.kAutoShootingPercentage);
    }

    public synchronized double getVal() {
        if(shooterState == ShooterState.PERCENT_OUTPUT) {
            master_.getStatorCurrent();
            return master_.getMotorOutputPercent();
        }

        return master_.getSelectedSensorVelocity();
    }

    public synchronized void setHighPosition() {
        angleChanger.set(Constants.kMaxServoAngle);
    }

    public synchronized void setLowPosition() {
        angleChanger.set(Constants.kMinServoAngle);
    }

    public synchronized boolean canShoot() {
        if(feedData_.feedforward == 0.0) {
            return false;
        }
        if(shooterState == ShooterState.PERCENT_OUTPUT) {
            return feedData_.feedforward == master_.getMotorOutputPercent();
        } else if(shooterState == ShooterState.VELOCITY) {
            return feedData_.feedforward == master_.getSelectedSensorVelocity();
        }

        return false;
    }

    @Override
    public void resetSensors() {
        setState(ShooterState.STOP);
        setLowPosition();
    }

    @Override
    public void stop() {
        shooterState = ShooterState.STOP;
        feedData_.feedforward = 0.0;
    }

    public synchronized void setVelocity(double velocity) {
        if(shooterState != ShooterState.VELOCITY) {
            shooterState = ShooterState.VELOCITY;
        }

        feedData_.feedforward = velocity;
    }

    @Override
    public synchronized void setOpenLoop(double percentage) {
        if(shooterState != ShooterState.PERCENT_OUTPUT) {
            shooterState = ShooterState.PERCENT_OUTPUT;
        }

        feedData_.feedforward = percentage;
    }

    @Override
    public synchronized void move_subsystem() {
        if(shooterState == ShooterState.PERCENT_OUTPUT) {
            master_.set(ControlMode.PercentOutput, feedData_.feedforward);
        } else if(shooterState == ShooterState.VELOCITY) {
            master_.set(ControlMode.Velocity, feedData_.feedforward);
        } else {
            master_.set(ControlMode.PercentOutput, 0.0);
        }
    }

    @Override
    public void sendDataToSmartDashboard() {
        SmartDashboard.putNumber("Shooter tick", master_.getSelectedSensorPosition());
        SmartDashboard.putNumber("Shooter output", master_.getMotorOutputPercent());
    }
}