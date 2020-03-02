package frc.robot.cycles;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.lib.utility.CrashTrackerRunnable;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

public class Cycle_in implements ICycle_in{

    private boolean isRunning;//if the cycle_in has started

    public final double kPeriod = Constants.kLooperDt;
    private final Notifier notifier_;//cycles_ the runnable obj
    private final List<Cycle> cycles_;//subsystems that are in this cycle_in obj
    private final Object taskCurrentRun_ = new Object();//mock obj in order to run synchronized block 
    private double timestamp_ = 0;//old time stamp, it is always upadated
    private double dt_= 0;//difference between current time and old time stamp

    private final CrashTrackerRunnable runnable_= new CrashTrackerRunnable(){
    
        @Override
        public void runCrashTracker() {
            synchronized(runnable_){
                if(isRunning){
                    double current_time = Timer.getFPGATimestamp();
                    cycles_.forEach(c -> c.onLoop(current_time));
                    
                    dt_ = current_time - timestamp_;
                    timestamp_ = current_time;
                }
            }
        }
    };

    public Cycle_in(){
        notifier_ = new Notifier(runnable_);
        isRunning = false;
        cycles_ = new ArrayList<>();
    }

    @Override
    public synchronized void addSubsystem(Cycle cycle){
        synchronized(cycle){
            cycles_.add(cycle);
        }
    }

    public synchronized void start_all() {
        if (!isRunning) {
           //System.out.println("Starting cycles_");
            synchronized (taskCurrentRun_) {
                timestamp_ = Timer.getFPGATimestamp();
                cycles_.forEach(c -> c.onStart(timestamp_));
                isRunning = true;
            }
            notifier_.startPeriodic(kPeriod);
        }
    }

    public synchronized void stop_all() {
        if (isRunning) {
            //System.out.println("Stopping cycles_");
            notifier_.stop();
            synchronized (taskCurrentRun_) {
                isRunning = false;
                timestamp_ = Timer.getFPGATimestamp();
                for (Cycle cycle : cycles_) {
                    System.out.println("Stopping " + cycle);
                    cycle.onStop(timestamp_);
                }
            }
        }
    }

    @Override
    public void sendDataToSmartDashboard() {
        SmartDashboard.putNumber("cycle_in_dt", dt_);
    }
}