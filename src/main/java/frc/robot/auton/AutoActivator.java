package frc.robot.auton; 

import frc.robot.auton.AutoOptionBase;
import frc.lib.utility.CrashTrackerRunnable;

public class AutoActivator{
    private static AutoActivator instance_ = new AutoActivator();
    private Thread autoStarter_ = null;
    private AutoOptionBase currentOption_ = null;

    public AutoActivator(){}

    public static AutoActivator getInstance(){
        return instance_;
    }

    public void setAutoOption(AutoOptionBase newOption){
        currentOption_ = newOption;
        
        autoStarter_ = new Thread(new CrashTrackerRunnable(){
        
            @Override
            public void runCrashTracker() {
                if(currentOption_ != null){
                    currentOption_.run();
                } else {
                    System.err.println("The auto option is nullllllllll!!!!!!");
                }
            }
        });
    }

    public void start(){
        if(autoStarter_ != null){
            autoStarter_.start();
        }
    }

    public boolean isStarted() {
        return currentOption_ != null && 
        currentOption_.getIsActive() && 
        autoStarter_ != null && 
        autoStarter_.isAlive();
    }

    public void reset() {
        if (isStarted()) {
            stop();
        }

        currentOption_ = null;
    }

    public void stop() {
        if (currentOption_ != null) {
            currentOption_.stop();
        }

        autoStarter_ = null;
    }

    public AutoOptionBase getAutoOption() {
        return currentOption_;
    }

    public boolean isInterrupted() {
        if (currentOption_ == null) {
            return false;
        }
        return currentOption_.getIsInterrupted();
    }

    public void interrupt() {
        if (currentOption_ == null) {
            return;
        }
        currentOption_.interrupt();
    }

    public void resume() {
        if (currentOption_ == null) {
            return;
        }
        currentOption_.resume();
    }
}
