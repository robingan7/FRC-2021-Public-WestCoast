package frc.robot.joysticks;

/**
 * all the implementations of the driver joystick
 */
public interface IDrive_Joystick{
    double getSpeed();

    double getTurn();

    double getHDrive();

    double getHDriveRight();

    double getHDriveLeft();

    boolean isAutoAimming();

    boolean isSwitchHood();
}