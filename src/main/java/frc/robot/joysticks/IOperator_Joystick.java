package frc.robot.joysticks;

public interface IOperator_Joystick{
    boolean isActivePanelControl();

    boolean isKillPanelControl();

    boolean isHomeTurret();

    boolean isAutoSteering();

    boolean isTurretMoveRight();

    boolean isTurretMoveLeft();

    double getElevator();

    double getIntake();

    boolean isReversePasser();

    boolean isAutoAimming();
}