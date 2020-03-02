package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.cycles.*;

public class LukeIntake extends Subsystem_Cycle {
    private static LukeIntake instance = new LukeIntake();

	public static LukeIntake getInstance() {
        if(instance == null) {
            instance = new LukeIntake();
        }
		return instance;
    }

    public enum IntakeState {
        PERCENT_OUTPUT, STOP
    }
    
    private IntakeState intakeState;
    private double inverse;
    private boolean isInversePasser;
    private DevineShooter shooter;
    private Solenoid ballBlocker;
    private boolean forceClose;
    private double lastInverseTime;
    private VictorSPX frontRoller;
    private TalonSRX passer;

    public static class FeedData {
        // INPUTS
        public double timestamp;
        public int position_absolute;
        public double position_units;
        public double output_percent;
        public double output_voltage;
        public double master_current;
        public boolean wantReset;

        // OUTPUTS
        public double feedforwardFrontRoller;
        public double feedforwardPasser;
    }

    private FeedData feedData_ = new FeedData();

    private LukeIntake() {
        intakeState = IntakeState.STOP;
        inverse = 1;
        isInversePasser = false;
        lastInverseTime = 0;
        shooter = DevineShooter.getInstance();
        //ballBlocker = new Solenoid(Constants.kIntakeBallBlockerId);
        forceClose = true;
        frontRoller = new VictorSPX(Constants.kIntakeFrontRollerId);
        passer = new TalonSRX(Constants.kIntakePasserId);
    }

    public synchronized IntakeState getState() {
        return intakeState;
    }

    public synchronized void setState(IntakeState state) {
        intakeState = state;
    }

    public synchronized void setForceClose(boolean isForced) {
        forceClose = isForced;
    }

    public synchronized void setInverse(boolean isInverse) {
        if(isInverse) {
            inverse = -1;
        } else {
            inverse = 1;
        }
    }

    public synchronized void setHighSpeed() {
        setOpenLoop(Constants.kIntakeHighSpeedPercentageIntake, Constants.kIntakeHighSpeedPercentagePasser);
    }

    public synchronized void setLowSpeed() {
        setOpenLoop(Constants.kIntakeLowSpeedPercentageIntake, Constants.kIntakeLowSpeedPercentagePasser);
    }

    public synchronized void setOpenLoop(double percentageRoller, double percentagePasser) {
        if(intakeState != IntakeState.PERCENT_OUTPUT) {
            intakeState = IntakeState.PERCENT_OUTPUT;
        }

        feedData_.feedforwardFrontRoller = percentageRoller;
        feedData_.feedforwardPasser = percentagePasser;
    }

    @Override
    public synchronized void resetSensors() {
        setState(IntakeState.STOP);
    }
    
    @Override
    public synchronized void stop() {
        setState(IntakeState.STOP);
        feedData_.feedforwardFrontRoller = 0.0;
        feedData_.feedforwardPasser = 0.0;
    }

    @Override
    public synchronized void move_subsystem() {
        if(intakeState == IntakeState.PERCENT_OUTPUT) {
            frontRoller.set(ControlMode.PercentOutput, feedData_.feedforwardFrontRoller * inverse);
            if(isInversePasser) {
                if(Timer.getFPGATimestamp() - lastInverseTime > Constants.kTurnReverseDuration) {
                    isInversePasser = false;
                    passer.set(ControlMode.PercentOutput, feedData_.feedforwardPasser * inverse);
                } else {
                    passer.set(ControlMode.PercentOutput, Constants.kIntakeLowSpeedPercentagePasserInverse);
                }
            } else {
                if(passer.getSupplyCurrent() > Constants.kWantTurnReverseCurrent) {
                    isInversePasser = true;
                    passer.set(ControlMode.PercentOutput, Constants.kIntakeLowSpeedPercentagePasserInverse);
                    lastInverseTime = Timer.getFPGATimestamp();
                } else {
                    passer.set(ControlMode.PercentOutput, feedData_.feedforwardPasser * inverse);
                }
            }
        } else {
            passer.set(ControlMode.PercentOutput, 0.0);
            frontRoller.set(ControlMode.PercentOutput, 0.0);
        }
        //ballBlocker.set(!shooter.canShoot() && !forceClose);
    }

    @Override
    public void sendDataToSmartDashboard() {
        //SmartDashboard.putNumber("Intake percent output", master_.getMotorOutputPercent());
    }
}