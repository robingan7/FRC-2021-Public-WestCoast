package frc.robot.joysticks;

public interface IOperator_Joystick{
    boolean isActivePanelControl();

    boolean isKillPanelControl();

    boolean isHomeTurret();

    boolean isAutoSteering();

    boolean isTurretMoveRight();

    boolean isTurretMoveLeft();

    double getElevator();

    boolean isIntake();

    boolean isReversePasser();

    boolean isShooting();

    boolean turretRight();

    boolean turretLeft();

    boolean isReverseIntake();

    boolean isTurretForceStop();

    boolean isHighSpeedShot();

    boolean isLowSpeedShot();

    boolean isAutoAimming();
}