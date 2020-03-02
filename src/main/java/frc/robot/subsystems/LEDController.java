package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.CANifier.PinValues;

import frc.robot.Constants;
import frc.robot.cycles.Subsystem_Cycle;
import frc.robot.cycles.Cycle;
import frc.robot.cycles.ICycle_in;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDController extends Subsystem_Cycle {
    private static LEDController instance;
    private CANifier canifier_;
    private LEDSignal currentSignal;
    private boolean signalChanged;

    private final Cycle cycle = new Cycle() {
        @Override
        public void onStart(double timestamp) {
            synchronized (LEDController.this) {
                //TODO: complete this on the actual robot
                //setLEDColor(newSignal);
            }
        }

        @Override
        public void onLoop(double timestamp) {
			synchronized (LEDController.this) {
			}
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };
    
    private LEDController() {
        canifier_ = new CANifier(Constants.kCanifierId);
        canifier_.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 2, Constants.kLongCANTimeoutMs);
        currentSignal = Constants.kOff;
        signalChanged = true;
    }

    public synchronized static LEDController getInstance() {
        if (instance == null) {
            instance = new LEDController();
        }
        return instance;
    }

    public synchronized void setLEDColor(double red, double green, double blue) {
        if (red != currentSignal.r_ ||
                green != currentSignal.g_ ||
                blue != currentSignal.b_) {
            currentSignal.r_ = red;
            currentSignal.g_ = green;
            currentSignal.b_ = blue;
            signalChanged = true;
        }
    }

    public synchronized void setLEDColor(LEDSignal newSignal) {
        setLEDColor(newSignal.r_, newSignal.g_, newSignal.b_);
    }

    @Override
    public synchronized void update_subsystem() {
        //mPeriodicInputs.has_ball_ = canifier_.getGeneralInput(CANifier.GeneralPin.LIMF);
        PinValues pinValues = new PinValues();
        canifier_.getGeneralInputs(pinValues);
    }

    @Override
    public synchronized void move_subsystem() {
        // A: Green
        // B: Red
        // C: Blue
        if (signalChanged) {
            canifier_.setLEDOutput(currentSignal.g_, CANifier.LEDChannel.LEDChannelA);
            canifier_.setLEDOutput(currentSignal.r_, CANifier.LEDChannel.LEDChannelB);
            canifier_.setLEDOutput(currentSignal.b_, CANifier.LEDChannel.LEDChannelC);
            signalChanged = false;
        }
    }

    @Override
    public synchronized void sendDataToSmartDashboard() {
       
    }

    @Override
    public void stop() {
        currentSignal = Constants.kOff;
        signalChanged = true;
        setLEDColor(Constants.kOff);
        update_subsystem();
    }

    @Override
    public synchronized void resetSensors() {
        canifier_.setQuadraturePosition(0, 0);
    }

    public static class LEDSignal {
        public double r_;
        public double g_;
        public double b_;

        public LEDSignal(double r, double g, double b) {
            r_ = r;
            g_ = g;
            b_ = b;
        }
    }

    @Override
	public void registerEnabledCycles(ICycle_in enabledCycler) {
        enabledCycler.addSubsystem(cycle);
	}
}