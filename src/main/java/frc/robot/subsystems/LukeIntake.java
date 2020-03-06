package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.fasterxml.jackson.databind.deser.ContextualKeyDeserializer;
import com.fasterxml.jackson.databind.node.ContainerNode;
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
    private TalonSRX frontRoller;
    private TalonSRX passer;
    private VictorSPX upper;
    private VictorSPX bottom;

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
        public double feedforwardUpper;
        public double feedforwardBottom;
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
        frontRoller = new TalonSRX(Constants.kIntakeFrontRollerId);
        passer = new TalonSRX(Constants.kIntakePasserId);
        upper = new VictorSPX(Constants.kIntakeUpperId);
        bottom = new VictorSPX(Constants.kIntakeBottomId);
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

    public synchronized void setOpenLoop(double percentageRoller, double percentagePasser, double percentageUpper, double percentageBottom) {
        if(intakeState != IntakeState.PERCENT_OUTPUT) {
            intakeState = IntakeState.PERCENT_OUTPUT;
        }

        feedData_.feedforwardFrontRoller = percentageRoller;
        feedData_.feedforwardPasser = percentagePasser;
        feedData_.feedforwardUpper = percentageUpper;
        feedData_.feedforwardBottom = percentageBottom;
    }

    public synchronized void controlIntake(double joystick, boolean isRervsePasser) {
        double rollerInput = 0;
        double upperInput = 0;
        double passerInput = isRervsePasser ? Constants.kIntakePercentagePasserInverse : Constants.kIntakePercentagePasser;
        double bottomInput = 0;

        if(joystick > Constants.kIntakeJoystickDeadband) {
            if(joystick >= Constants.kIntakePercentageMaxUpper) {
                upperInput = Constants.kIntakePercentageMaxUpper;
            } else {
                upperInput = joystick;
            }
            
            bottomInput = Constants.kIntakePercentageMaxBottom;
            rollerInput = Constants.kIntakePercentageIntake;
            
        } else if(joystick < -Constants.kIntakeJoystickDeadband) {
            upperInput = Constants.kIntakePercentageMaxUpperInverse;
            bottomInput = Constants.kIntakePercentageMaxBottomInverse;
            passerInput = Constants.kIntakePercentagePasserInverse;
        }

        if(joystick <= Constants.kIntakeJoystickDeadband && joystick >= -Constants.kIntakeJoystickDeadband) {
            passerInput = 0;
        }

        setOpenLoop(rollerInput, passerInput, upperInput, bottomInput);
    }

    public synchronized void reverseIntakeInAuto() {
        controlIntake(-0.4, false);
    }

    public synchronized void intakeInAuto() {
        controlIntake(0.4, false);
    }

    private void passerCurrentLimit() {
        if(isInversePasser) {
            if((Timer.getFPGATimestamp() - lastInverseTime) > Constants.kTurnReverseDuration) {
                isInversePasser = false;
                passer.set(ControlMode.PercentOutput, -feedData_.feedforwardPasser * inverse);
            } else {
                passer.set(ControlMode.PercentOutput, -Constants.kIntakePercentagePasserInverse);
            }
        } else {
            if(-passer.getStatorCurrent() > Constants.kWantTurnReverseCurrent) {
                isInversePasser = true;
                passer.set(ControlMode.PercentOutput, -Constants.kIntakePercentagePasserInverse);
                lastInverseTime = Timer.getFPGATimestamp();
            } else {
                isInversePasser = false;
                passer.set(ControlMode.PercentOutput, -feedData_.feedforwardPasser * inverse);
            }
        }
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
        feedData_.feedforwardUpper = 0.0;
        feedData_.feedforwardBottom = 0.0;
    }

    @Override
    public synchronized void move_subsystem() {
        //System.out.println("in " + frontRoller.getMotorOutputPercent());
        if(intakeState == IntakeState.PERCENT_OUTPUT) {
            frontRoller.set(ControlMode.PercentOutput, -feedData_.feedforwardFrontRoller * inverse);
            upper.set(ControlMode.PercentOutput, feedData_.feedforwardUpper * inverse);
            bottom.set(ControlMode.PercentOutput, feedData_.feedforwardBottom * inverse);
            passerCurrentLimit();
        } else {
            passer.set(ControlMode.PercentOutput, 0.0);
            upper.set(ControlMode.PercentOutput, 0.0);
            frontRoller.set(ControlMode.PercentOutput, 0.0);
            bottom.set(ControlMode.PercentOutput, 0.0);
        }
        //ballBlocker.set(!shooter.canShoot() && !forceClose);
    }

    @Override
    public void sendDataToSmartDashboard() {
        SmartDashboard.putNumber("Passer current", passer.getStatorCurrent());
        SmartDashboard.putNumber("Roller current", frontRoller.getStatorCurrent());
    }

}