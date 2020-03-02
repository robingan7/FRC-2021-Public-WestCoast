package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.cycles.*;
import frc.lib.motor.MotorUtility;
import frc.lib.motor.LazyTalonSRX;

public abstract class SingleMasterMotorSystem extends Subsystem_Cycle {
    private static final int kPositionPIDSlot = 0;

    protected final Constants.SingleMasterMotorSystemConfig constants_;
    protected final TalonSRX master_;
    protected final VictorSPX[] slaves_;
   
    protected SingleMasterMotorSystem(final Constants.SingleMasterMotorSystemConfig constants){
        constants_ = constants;
        master_ = MotorUtility.createTalon(constants_.kMasterConstants.id);
        slaves_ = new VictorSPX[constants_.kSlaveConstants.length];

        MotorUtility.checkError(master_.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute),
                constants_.kName + ": Could not detect encoder: ");

        /*
        MotorUtility.checkError(master_.configForwardSoftLimitEnable(true, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not enable forward soft limit: ");

        MotorUtility.checkError(master_.configReverseSoftLimitEnable(true, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not enable reverse soft limit: ");*/

        MotorUtility.checkError(master_.config_kP(Constants.kSlotIdx, constants_.kKp, Constants.kLongCANTimeoutMs),
                constants_.kName + ": could not set kP: ");

        MotorUtility.checkError(master_.config_kI(Constants.kSlotIdx, constants_.kKi, Constants.kLongCANTimeoutMs),
                constants_.kName + ": could not set kI: ");

        MotorUtility.checkError(master_.config_kD(Constants.kSlotIdx, constants_.kKd, Constants.kLongCANTimeoutMs),
                constants_.kName + ": could not set kD: ");

        MotorUtility.checkError(master_.config_kF(Constants.kSlotIdx, constants_.kKf, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set kF: ");

        MotorUtility.checkError(master_.configMotionCruiseVelocity(constants_.kCruiseVelocity, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set cruise velocity: ");

        MotorUtility.checkError(master_.configMotionAcceleration(constants_.kAcceleration, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set acceleration: ");
    
        MotorUtility.checkError(master_.configContinuousCurrentLimit(constants_.kContinuousCurrentLimit),
                constants_.kName + ": Could not set continuous current limit.");

        MotorUtility.checkError(master_.configPeakCurrentLimit(constants_.kPeakCurrentLimit),
                constants_.kName + ": Could not set peak current limit.");

        MotorUtility.checkError(master_.configPeakCurrentDuration(constants_.kPeakCurrentDuration),
                constants_.kName + ": Could not set peak current duration.");
       
        master_.setInverted(constants_.kMasterConstants.invert_motor);

        master_.setNeutralMode(NeutralMode.Brake);
        
        master_.setSensorPhase(constants_.kMasterConstants.invert_sensor_phase);

        //-----------config didn't use in 2019 season ------------

        /*
        MotorUtility.checkError(master_.configNominalOutputForward(constants_.kNominalOutputForward, Constants.kLongCANTimeoutMs),
        constants_.kName + ": Could not set configNominalOutputForward()");

        MotorUtility.checkError(master_.configNominalOutputReverse(constants_.kNominalOutputReverse, Constants.kLongCANTimeoutMs),
            constants_.kName + ": Could not set configNominalOutputReverse()");
    
        MotorUtility.checkError(master_.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs),
            constants_.kName + ": Could not set voltage compensation saturation: ");
         MotorUtility.checkError(master_.configAllowableClosedloopError(Constants.kSlotIdx, constants_.kDeadband, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set deadband: ");

        MotorUtility.checkError(master_.configMaxIntegralAccumulator(kPositionPIDSlot, constants_.kPositionMaxIntegralAccumulator,
                Constants.kLongCANTimeoutMs), constants_.kName + ": Could not set max integral: ");

        MotorUtility.checkError(master_.config_IntegralZone(kPositionPIDSlot, constants_.kPositionIZone, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set i zone: ");

        MotorUtility.checkError(master_.configAllowableClosedloopError(kPositionPIDSlot, constants_.kPositionDeadband, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set deadband: ");

        MotorUtility.checkError(master_.configOpenloopRamp(constants_.kRampRate, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set voltage ramp rate: ");

        MotorUtility.checkError(master_.configClosedloopRamp(constants_.kRampRate, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set closed loop ramp rate: ");

         MotorUtility.checkError(master_.configVoltageCompSaturation(constants_.kMaxVoltage, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set voltage comp saturation.");        

        master_.enableVoltageCompensation(true);

        master_.enableCurrentLimit(true);

        master_.configVoltageMeasurementFilter(8);

        master_.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
        master_.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);
        master_.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, constants_.kStastusFrame8UpdateRate, 20);*/


        //--------------master and slave config-----------------
        master_.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);

        for (int i = 0; i < slaves_.length; ++i) {
            slaves_[i] = MotorUtility.createVictor(constants_.kSlaveConstants[i].id,
                    constants_.kMasterConstants.id);
            slaves_[i].setInverted(constants_.kSlaveConstants[i].invert_motor);
            slaves_[i].setNeutralMode(NeutralMode.Brake);
            slaves_[i].follow(master_);
        }

        stop();
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

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
        public double feedforward;
    }

    protected enum ControlType {
        OPEN_LOOP, MOTION_MAGIC, MOTION_PROFILE, STOP
    }

    protected FeedData feedData_ = new FeedData();
    protected ControlType controlMode_ = ControlType.OPEN_LOOP;
    protected final Cycle registerCycle_ = new Cycle() {
        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp) {
            if (feedData_.wantReset) {
                System.out.println(constants_.kName + ": Master Talon reset occurred; resetting frame rates.");
                master_.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
                master_.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);
                master_.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, constants_.kStastusFrame8UpdateRate, 20);

                // Reset encoder position to estimated position from absolute encoder
                if (constants_.kRecoverPositionOnReset) {
                    //master_.setSelectedSensorPosition(estimateSensorPositionFromAbsolute(), 0, Constants.kCANTimeoutMs);
                }
            }
            handleMasterReset(feedData_.wantReset);
            for (VictorSPX slave : slaves_) {
                if (slave.hasResetOccurred()) {
                    System.out.println(constants_.kName + ": Slave Talon reset occurred");
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    /**
     * call if the subsystem want to be reset, and should be override
     * @param reset the feeData.wantReset
     */
    public synchronized void handleMasterReset(boolean reset) {}

    @Override
    public synchronized void move_subsystem() {
        if (controlMode_ == ControlType.MOTION_MAGIC) {
            master_.set(ControlMode.Position, feedData_.feedforward);
        } else if (controlMode_ == ControlType.MOTION_PROFILE) {
            master_.set(ControlMode.Position, feedData_.feedforward);
        } else {
            master_.set(ControlMode.PercentOutput, feedData_.feedforward);
        } 
    }

    public synchronized void setSetpointMotionMagic(double units) {
        feedData_.feedforward = units;
        if (controlMode_ != ControlType.MOTION_MAGIC) {
            master_.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
            controlMode_ = ControlType.MOTION_MAGIC;
        }
    }

    /**
     * start percentage control
     */
    public synchronized void setOpenLoop(double percentage) {
        feedData_.feedforward = percentage;
        if (controlMode_ != ControlType.OPEN_LOOP) {
            controlMode_ = ControlType.OPEN_LOOP;
        }
    }

    @Override
	public void registerEnabledCycles(ICycle_in enabledCycler) {
        enabledCycler.addSubsystem(registerCycle_);
	}

    public synchronized double getPosition() {
        return master_.getSelectedSensorPosition();
    }

}