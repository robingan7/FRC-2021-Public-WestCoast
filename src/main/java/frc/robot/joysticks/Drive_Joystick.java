package frc.robot.joysticks;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.Joystick;

public class Drive_Joystick implements IDrive_Joystick {
    private static Drive_Joystick mInstance = new Drive_Joystick();

    public static Drive_Joystick getInstance(){
        return mInstance;
    }

    private Joystick mDriverJoystick;

    private Drive_Joystick(){
        mDriverJoystick = new Joystick(Constants.kDriverJoystickId);
    }

    @Override
    public double getSpeed(){
        return mDriverJoystick.getRawAxis(1);
    }

    @Override
    public double getTurn(){
        return mDriverJoystick.getRawAxis(4);
    }

    public boolean isActivePanelControl() {
        return mDriverJoystick.getRawButton(1);
    }

    public boolean isKillPanelControl() {
        return mDriverJoystick.getRawButton(2);
    }

    @Override
    public double getHDriveRight() {
        return mDriverJoystick.getRawAxis(3);
    }

    @Override
    public double getHDriveLeft() {
        return -mDriverJoystick.getRawAxis(2);
    }

    @Override
    public boolean isAutoAimming() {
        return mDriverJoystick.getRawButton(3);
    }

    @Override
    public boolean isHomeTurret() {
        return mDriverJoystick.getRawButton(7);
    }

    @Override
    public boolean isAutoSteering() {
        return mDriverJoystick.getRawButton(6);
    }

    @Override
    public boolean isTurretMoveRight() {
        return mDriverJoystick.getPOV(0) == 90;
    }

    @Override
    public boolean isTurretMoveLeft() {
        return mDriverJoystick.getPOV(0) == 270;
    }

}