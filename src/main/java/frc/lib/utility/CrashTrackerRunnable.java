package frc.lib.utility;

public abstract class CrashTrackerRunnable implements Runnable {
    @Override
    public final void run(){
        try{
            runCrashTracker();
        }catch(Throwable t){
            System.err.println("Auto Crashhhhhhhhhhh!!!!!");
            throw t;
        }
    }

    public abstract void runCrashTracker();
}