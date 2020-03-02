package frc.robot.joysticks;

/**
 * all the implementations of the driver joystick
 */
public interface IDrive_Joystick{
    double getSpeed();

    double getTurn();

    double getHDriveRight();

    double getHDriveLeft();

    boolean isActivePanelControl();

    boolean isKillPanelControl();

    boolean isAutoAimming();

    boolean isHomeTurret();

    boolean isAutoSteering();

    boolean isTurretMoveRight();

    boolean isTurretMoveLeft();
}