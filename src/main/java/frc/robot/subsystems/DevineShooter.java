package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;

import frc.robot.Constants;
import frc.lib.utility.Utility;

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
        setHighPosition();
    }

    public synchronized void switchHood() {
        if(angleChanger.get() == Constants.kMaxServoAngle) {
            setLowPosition();
        } else if(angleChanger.get() == Constants.kMinServoAngle) {
            setHighPosition();
        }
    }

    public synchronized void reverse() {
        setOpenLoop(Constants.kShooterReverseSpeed);
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

    public synchronized void shootinA() {
        setOpenLoop(0.75);
    }
    public synchronized void shootInLowSpeed() {
        setOpenLoop(0.4);
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
            return Utility.isReachSpeedShooter(master_.getMotorOutputPercent(), feedData_.feedforward);
        } else if(shooterState == ShooterState.VELOCITY) {
            //return Utility.isReachSpeedShooter(master_.getSelectedSensorVelocity(), feedData_.feedforward);
            return true;
        }

        return false;
    }

    @Override
    public synchronized void resetSensors() {
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
        SmartDashboard.putNumber("Shooter output", master_.getMotorOutputPercent());
        SmartDashboard.putNumber("Shooter Servo", angleChanger.get());
    }
}