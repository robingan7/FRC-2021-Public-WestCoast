package frc.robot.joysticks;

/**
 * the main control board contains the driver and operator joystick
 * and call their methods
 */
public class MainControlBoard implements IMainControlBoard{
    private static MainControlBoard mInstance = null;
    private IDrive_Joystick mDrive_Joystick;
    private IOperator_Joystick mIOperator_Joystick;

    public static MainControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new MainControlBoard();
        }
        return mInstance;
    }

    public MainControlBoard() {
        mDrive_Joystick  = Drive_Joystick.getInstance();
        mIOperator_Joystick = Operator_Joystick.getInstance();
    }

    @Override
    public double getSpeed() {
        return mDrive_Joystick.getSpeed();
    }

    @Override
    public double getTurn() {
        return mDrive_Joystick.getTurn();
    }

    @Override
    public boolean isActivePanelControl() {
        return mDrive_Joystick.isActivePanelControl();
    }

    @Override
    public boolean isKillPanelControl() {
        return mDrive_Joystick.isActivePanelControl();
    }

    @Override
    public boolean isAutoAimming() {
        return mDrive_Joystick.isAutoAimming();
    }

    @Override
    public boolean isHomeTurret() {
        return mDrive_Joystick.isHomeTurret();
    }

    @Override
    public boolean isAutoSteering() {
        return mDrive_Joystick.isAutoSteering();
    }

    @Override
    public boolean isTurretMoveRight() {
        return mDrive_Joystick.isTurretMoveRight();
    }

    @Override
    public boolean isTurretMoveLeft() {
        return mDrive_Joystick.isTurretMoveLeft();
    }

    @Override
    public double getHDriveRight() {
        return mDrive_Joystick.getHDriveRight();
    }

    @Override
    public double getHDriveLeft() {
        return mDrive_Joystick.getHDriveLeft();
    }

}