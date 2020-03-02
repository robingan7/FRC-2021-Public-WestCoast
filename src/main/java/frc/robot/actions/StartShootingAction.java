package frc.robot.actions;

import frc.robot.subsystems.DevineShooter;
import frc.robot.subsystems.DevineShooter.ShooterState;
import frc.robot.subsystems.LukeIntake;
import frc.robot.subsystems.LukeIntake.IntakeState;
import frc.robot.subsystems.AustinTurret;
import frc.robot.subsystems.AustinTurret.TurretState;

public class StartShootingAction extends DelayAction {
    public StartShootingAction(double duration) {
        super(duration);
    }

    @Override
    public void start() {
        //DevineShooter.getInstance().shootInAuto();
        //LukeIntake.getInstance().setHighSpeed();
        //AustinTurret.getInstance().setState(TurretState.VISION);
        super.start();
    }

    @Override
    public void update() {
        /*
        if(DevineShooter.getInstance().canShoot()) {
            LukeIntake.getInstance().setForceClose(false);
        }*/
    }

    @Override
    public void done() {
        //DevineShooter.getInstance().setState(ShooterState.STOP);
        //LukeIntake.getInstance().setForceClose(true);
        //AustinTurret.getInstance().setState(TurretState.SET_POINT);
    }

}