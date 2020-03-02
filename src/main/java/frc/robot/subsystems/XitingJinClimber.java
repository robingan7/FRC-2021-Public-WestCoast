package frc.robot.subsystems;

import frc.robot.Constants;
import frc.lib.control.SynchronousPIDController;
import frc.robot.cycles.Cycle;
import frc.robot.cycles.ICycle_in;
import frc.robot.cycles.Subsystem_Cycle;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class XitingJinClimber extends Subsystem_Cycle {
    private static XitingJinClimber instance = new XitingJinClimber();
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

    private VictorSPX master, slave;
    private FeedData feedData_ = new FeedData();

	public static XitingJinClimber getInstance() {
        if(instance == null) {
            instance = new XitingJinClimber();
        }
		return instance;
    }
    
    private XitingJinClimber() {
        master = new VictorSPX(Constants.kClimberMasterId);
        slave = new VictorSPX(Constants.kClimberSlaveId);
        slave.follow(master);
    }

    @Override
    public synchronized void resetSensors() {
        feedData_.feedforward = 0.0;
    }

    public synchronized void setOpenLoop(double percentage) {
        if(Math.abs(percentage) < 0.2) {
            feedData_.feedforward = 0.0;
            return;
        }
        feedData_.feedforward = percentage;
    }
    
    @Override
    public void move_subsystem() {
        master.set(ControlMode.PercentOutput, feedData_.feedforward);
    }
}