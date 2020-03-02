package frc.robot.cycles;

import java.util.List;
import java.util.ArrayList;

/**
 * cycle_in class that contains multiple cycles
 * provides actual implementation for cycle
 */
public class Subsystem_Cycle_Manager implements ICycle_in{
   
    private final List<Subsystem_Cycle> allSubsystems_;//includes subsystems themselves
    private List<Cycle> cycles_ = new ArrayList<>();//includes cycles in subsystems

    public Subsystem_Cycle_Manager(List<Subsystem_Cycle> allSubsystems) {
        allSubsystems_ = allSubsystems;
    }

    //---------EnabledLoop-----------
    //provides actual implementation for cycle
    private class EnabledLoop implements Cycle {

        @Override
        public void onStart(double timestamp) {
            cycles_.forEach(c -> c.onStart(timestamp));
        }

        @Override
        public void onLoop(double timestamp) {
            allSubsystems_.forEach(Subsystem_Cycle::update_subsystem);
            cycles_.forEach(c -> c.onLoop(timestamp));
            allSubsystems_.forEach(Subsystem_Cycle::move_subsystem);
            sendDataToSmartDashboard();
        }
        @Override
        public void onStop(double timestamp) {
            cycles_.forEach(c -> c.onStop(timestamp));
        }
    }

     //---------DisabledLoop-----------
     //provides actual implementation for cycle
    private class DisabledLoop implements Cycle {

        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp) {
            allSubsystems_.forEach(Subsystem_Cycle::update_subsystem);
        }

        @Override
        public void onStop(double timestamp) {}
    }

    public void registerEnabledCycles(Cycle_in enabledCycler) {
        allSubsystems_.forEach((s) -> s.registerEnabledCycles(this));
        enabledCycler.addSubsystem(new EnabledLoop());
    }

    public void registerDisabledLoops(Cycle_in disabledCycler) {
        disabledCycler.addSubsystem(new DisabledLoop());
    }

    @Override
    public void addSubsystem(Cycle loop) {
        cycles_.add(loop);
    }

    @Override
    public void sendDataToSmartDashboard(){
        allSubsystems_.forEach(Subsystem_Cycle::sendDataToSmartDashboard);
    }

    public boolean checkSubsystems() {
        boolean ret_val = true;

        for (Subsystem_Cycle s : allSubsystems_) {
            ret_val &= s.checkSubsystem();
        }

        return ret_val;
    }


}