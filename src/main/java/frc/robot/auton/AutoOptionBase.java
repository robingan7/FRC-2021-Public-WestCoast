package frc.robot.auton;

import frc.robot.auton.AutoEndEarlyException;
import frc.robot.actions.Action;

import edu.wpi.first.wpilibj.DriverStation;
/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in auto modes (which are
 * routines that do actions).
 */

 public abstract class AutoOptionBase {
    protected final double updateRate_ = 1.0 / 50.0;
    protected boolean isActive_ = false;
    protected boolean isInterrupted_ = false;

    protected abstract void routine() throws AutoEndEarlyException;

    public void run(){
        isActive_ = true;

        try{
            routine();
        }catch(AutoEndEarlyException e){
            DriverStation.reportError("AUTO MODE DONE!!!! ENDED EARLY!!!!", false);
            return;
        }
        done();
    }

    public void runAction(Action action) throws AutoEndEarlyException {
        isActiveWithThrow();
        
        //wait for interrupt state to be cleared
        while(isActiveWithThrow() && isInterrupted_){
            sleepThread();
        }

        action.start();

        // Run action, stop action on interrupt, non active mode, or done
        while (isActiveWithThrow() && !action.isFinished() && !isInterrupted_) {
            action.update();

            sleepThread();
        }

        action.done();
    }

    public void sleepThread(){
        long waitTime = (long)(updateRate_ * 1000);
        try {
            Thread.sleep(waitTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void stop(){
        isActive_ = false;
    }

    public boolean getIsActive(){
        return isActive_;
    }

    public boolean isActiveWithThrow() throws AutoEndEarlyException {
        if (!getIsActive()) {
            throw new AutoEndEarlyException();
        }

        return getIsActive();
    }

    public void done() {
        System.out.println("Autonmous finish");
    }

    public void interrupt() {
        System.out.println("Auto mode interrrupted!");
        isInterrupted_ = true;
    }

    public void resume() {
        System.out.println("Auto mode resumed!");
        isInterrupted_ = false;
    }
    
    public boolean getIsInterrupted() {
        return isInterrupted_;
    }
 }