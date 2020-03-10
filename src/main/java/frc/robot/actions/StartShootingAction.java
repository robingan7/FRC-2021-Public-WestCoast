package frc.robot.actions;

import frc.robot.subsystems.DevineShooter;
import frc.robot.subsystems.DevineShooter.ShooterState;
import frc.robot.subsystems.LukeIntake;
import frc.robot.subsystems.LukeIntake.IntakeState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.AustinTurret;
import frc.robot.subsystems.AustinTurret.TurretState;

public class StartShootingAction extends DelayAction {
    private double delayTime;
    public StartShootingAction(double duration) {
        super(duration);
    }

    @Override
    public void start() {
        delayTime = Timer.getFPGATimestamp();
        DevineShooter.getInstance().shootInAuto();
        AustinTurret.getInstance().setState(TurretState.VISION);
        super.start();
    }

    @Override
    public void update() {
        if(DevineShooter.getInstance().canShoot()) {
            LukeIntake.getInstance().intakeInAuto();
        }
    }

    @Override
    public void done() {
        DevineShooter.getInstance().setState(ShooterState.STOP);
        AustinTurret.getInstance().setState(TurretState.SET_POINT);
    }

}