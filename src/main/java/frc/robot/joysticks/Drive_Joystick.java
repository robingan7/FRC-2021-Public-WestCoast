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
        return mDriverJoystick.getRawAxis(4);//4 for xbox 2 for ps4
    }

    @Override
    public double getHDriveRight() {
        return mDriverJoystick.getRawAxis(3);//3 for xbox 4 for ps4
    }

    @Override
    public double getHDriveLeft() {
        return -mDriverJoystick.getRawAxis(2);//2 for xbox 3 for ps4
    }

    @Override
    public boolean isAutoAimming() {
        return mDriverJoystick.getRawButton(6);//3 for xbox
    }

    @Override
    public boolean isSwitchHood() {
        return mDriverJoystick.getRawButton(5);
    }

    @Override
    public double getHDrive() {
        return mDriverJoystick.getRawAxis(0);
    }

}