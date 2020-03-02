package frc.robot.subsystems;

import frc.robot.Constants;
import frc.lib.control.SynchronousPIDController;
import frc.robot.cycles.Cycle;
import frc.robot.cycles.ICycle_in;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class XitingJinClimberPID extends SingleMasterMotorSystem {
    private SynchronousPIDController heightController;
    private double goal;
    private static XitingJinClimberPID instance = new XitingJinClimberPID();

	public static XitingJinClimberPID getInstance() {
        if(instance == null) {
            instance = new XitingJinClimberPID();
        }
		return instance;
    }

    private final Cycle cycle = new Cycle() {
        @Override
        public void onStart(double timestamp) {
            synchronized (XitingJinClimberPID.this) {
            }
        }

        @Override
        public void onLoop(double timestamp) {
			synchronized (XitingJinClimberPID.this) {
                if(!isReachGoal()) {
                    setOpenLoop(heightController.calculate(-master_.getSelectedSensorPosition(), goal));
                } else {
                    setOpenLoop(0.0);
                }
			}
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };
    
    private XitingJinClimberPID() {
        super(Constants.kClimber);
        heightController = new SynchronousPIDController(constants_.kKp, constants_.kKi, constants_.kKd);
        goal = 0;
    }

    public synchronized void getHigh() {
        goal = Constants.kClimberMaxHeight;
    }

    public synchronized void getLow() {
        goal = 0;
    }

    @Override
    public synchronized void resetSensors() {
        master_.setSelectedSensorPosition(0, 0, 10);
		setOpenLoop(0.0);
    }

    @Override
	public void registerEnabledCycles(ICycle_in enabledCycler) {
		enabledCycler.addSubsystem(cycle);
		super.registerEnabledCycles(enabledCycler);
    }

    public synchronized boolean isReachGoal() {
        return Math.abs(goal -  master_.getSelectedSensorPosition()) < 2;
    }
    
    @Override
    public void sendDataToSmartDashboard() {
        SmartDashboard.putNumber("Climber tick", master_.getSelectedSensorPosition());
    }
}