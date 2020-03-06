package frc.robot.actions;

import frc.robot.subsystems.LukeIntake;
import frc.robot.subsystems.DevineShooter;

public class GetReadyToShootAction extends RunOnceAction {

    @Override
    public void runOnce() {
        DevineShooter.getInstance().reverse();
        LukeIntake.getInstance().reverseIntakeInAuto();
    }

}