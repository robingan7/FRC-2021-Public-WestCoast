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
        return mIOperator_Joystick.isActivePanelControl();
    }

    @Override
    public boolean isKillPanelControl() {
        return mIOperator_Joystick.isActivePanelControl();
    }

    @Override
    public boolean isShooting() {
        return mIOperator_Joystick.isShooting();
    }

    @Override
    public boolean isHomeTurret() {
        return mIOperator_Joystick.isHomeTurret();
    }

    @Override
    public boolean isAutoSteering() {
        return mIOperator_Joystick.isAutoSteering();
    }

    @Override
    public boolean isTurretMoveRight() {
        return mIOperator_Joystick.isTurretMoveRight();
    }

    @Override
    public boolean isTurretMoveLeft() {
        return mIOperator_Joystick.isTurretMoveLeft();
    }

    @Override
    public double getHDrive() {
        return mDrive_Joystick.getHDrive();
    }

    @Override
    public double getHDriveRight() {
        return mDrive_Joystick.getHDriveRight();
    }

    @Override
    public double getHDriveLeft() {
        return mDrive_Joystick.getHDriveLeft();
    }

    @Override
    public double getElevator() {
        return mIOperator_Joystick.getElevator();
    }

    @Override
    public boolean isIntake() {
        return mIOperator_Joystick.isIntake();
    }

    @Override
    public boolean isReverseIntake() {
        return mIOperator_Joystick.isReverseIntake();
    }

    @Override
    public boolean isReversePasser() {
        return mIOperator_Joystick.isReversePasser();
    }

    @Override
    public boolean isSwitchHood() {
        return mDrive_Joystick.isSwitchHood();
    }

    @Override
    public boolean turretRight() {
        return mIOperator_Joystick.turretRight();
    }

    @Override
    public boolean turretLeft() {
        return mIOperator_Joystick.turretLeft();
    }

    @Override
    public boolean isTurretForceStop() {
        return mIOperator_Joystick.isTurretForceStop();
    }

    @Override
    public boolean isHighSpeedShot() {
        return mIOperator_Joystick.isHighSpeedShot();
    }

    @Override
    public boolean isLowSpeedShot() {
        return mIOperator_Joystick.isLowSpeedShot();
    }

    @Override
    public boolean isAutoAimming() {
        return mIOperator_Joystick.isAutoAimming();
    }

}
