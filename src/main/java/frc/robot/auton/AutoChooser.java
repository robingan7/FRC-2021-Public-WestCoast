package frc.robot.auton;

import frc.robot.auton.AutoOptionBase;
import frc.robot.subsystems.RobotState;
import frc.robot.WayPoints;
import frc.lib.math.Translation2d;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoChooser {
    public enum StartingPosition {
        FACE_OPPO_TRENCH, FACE_FRONT_BARRIER, FACE_POWER_PORT, PATHA, PATHB, 

        
    }

    public enum BeforeFirstShoot {
        TWO_ON_OPPO_TRENCH, 
        HALF_STEAL, 
        COMPLETE_STEAL, 
        FRONT_BARRIER,
        BACK_BARRIER,
        DO_NOTHING
    }

    public enum BeforeSecondShoot {
        FRONT_BARRIER_TO_BACK_BARRIER,
        FRONT_BARRIER_TO_HALF_TRENCH,
        BACK_BARRIER_TO_HALF_TRENCH,
        FULL_TRENCH,
        BACK_BARRIER_TO_FULL_TRENCH,//before two cells intake
        DO_NOTHING
    }

    public enum BeforeThirdShoot {
        FRONT_BARRIER,
        BACK_BARRIER,
        TRENCH,
        DO_NOTHING
    }

    public enum PanelControlMode {
        FMS, 
        ROTATE,
        GREEN,
        RED,
        BLUE,
        YELLOW
    }

    private BeforeFirstShoot beforeFirstShootSelected = null;
    private BeforeSecondShoot beforeSecondShootSelected = null;
    private BeforeThirdShoot beforeThirdShootSelected = null;
    private StartingPosition mCachedStartingPosition = null;
    private PanelControlMode mCachedPanelControl = null;

    private SendableChooser<BeforeFirstShoot> firstShooterChooser;
    private SendableChooser<BeforeSecondShoot> secondShooterChooser;
    private SendableChooser<BeforeThirdShoot> thridShooterChooser;
    private SendableChooser<StartingPosition> startPositionChooser_;
    private SendableChooser<PanelControlMode> panelControlMode_;

    private Optional<AutoOptionBase> autoOption_ = Optional.empty();
    private static Translation2d initialPoint;

    public AutoChooser() {
        initialPoint = WayPoints.kFacePowerPort;
        startPositionChooser_ = new SendableChooser<>();
        startPositionChooser_.setDefaultOption("FACE_OPPO_TRENCH", StartingPosition.FACE_OPPO_TRENCH);
        startPositionChooser_.addOption("FACE_FRONT_BARRIER", StartingPosition.FACE_FRONT_BARRIER);
        startPositionChooser_.addOption("FACE_POWER_PORT", StartingPosition.FACE_POWER_PORT);
        SmartDashboard.putData("Starting Position", startPositionChooser_);

        firstShooterChooser = new SendableChooser<>();
        firstShooterChooser.setDefaultOption("DO_NOTHING", BeforeFirstShoot.DO_NOTHING);
        firstShooterChooser.addOption("TWO_ON_OPPO_TRENCH", BeforeFirstShoot.TWO_ON_OPPO_TRENCH);
        firstShooterChooser.addOption("HALF_STEAL", BeforeFirstShoot.HALF_STEAL);
        firstShooterChooser.addOption("COMPLETE_STEAL", BeforeFirstShoot.COMPLETE_STEAL);
        firstShooterChooser.addOption("FRONT_BARRIER", BeforeFirstShoot.FRONT_BARRIER);
        firstShooterChooser.addOption("BACK_BARRIER", BeforeFirstShoot.BACK_BARRIER);
        SmartDashboard.putData("First Select", firstShooterChooser);

        secondShooterChooser = new SendableChooser<>();
        System.out.println(firstShooterChooser + " --- " + secondShooterChooser);
        secondShooterChooser.setDefaultOption("DO_NOTHING", BeforeSecondShoot.DO_NOTHING);
        secondShooterChooser.addOption("FRONT_BARRIER_TO_BACK_BARRIER", BeforeSecondShoot.FRONT_BARRIER_TO_BACK_BARRIER);
        secondShooterChooser.addOption("FRONT_BARRIER_TO_HALF_TRENCH", BeforeSecondShoot.FRONT_BARRIER_TO_HALF_TRENCH);
        secondShooterChooser.addOption("BACK_BARRIER_TO_HALF_TRENCH", BeforeSecondShoot.BACK_BARRIER_TO_HALF_TRENCH);
        secondShooterChooser.addOption("FULL_TRENCH", BeforeSecondShoot.FULL_TRENCH);
        secondShooterChooser.addOption("BACK_BARRIER_TO_FULL_TRENCH", BeforeSecondShoot.BACK_BARRIER_TO_FULL_TRENCH);
        SmartDashboard.putData("Second Select", secondShooterChooser);

        thridShooterChooser = new SendableChooser<>();
        thridShooterChooser.setDefaultOption("DO_NOTHING", BeforeThirdShoot.DO_NOTHING);
        thridShooterChooser.addOption("FRONT_BARRIER", BeforeThirdShoot.FRONT_BARRIER);
        thridShooterChooser.addOption("TRENCH", BeforeThirdShoot.TRENCH);
        thridShooterChooser.addOption("BACK_BARRIER", BeforeThirdShoot.BACK_BARRIER);
        SmartDashboard.putData("Third Select", thridShooterChooser);

        panelControlMode_ = new SendableChooser<>();
        panelControlMode_.setDefaultOption("FMS Auto", PanelControlMode.FMS);
        panelControlMode_.addOption("Rotate", PanelControlMode.ROTATE);
        panelControlMode_.addOption("Green", PanelControlMode.GREEN);
        panelControlMode_.addOption("Red", PanelControlMode.RED);
        panelControlMode_.addOption("Blue", PanelControlMode.BLUE);
        panelControlMode_.addOption("Yellow", PanelControlMode.YELLOW);
        SmartDashboard.putData("Panel Control", panelControlMode_);
    }

    public void updateModeCreator(boolean isOnlyControlPanel) {
        BeforeFirstShoot currentFirstShoot = firstShooterChooser.getSelected();
        BeforeSecondShoot currentSecondShoot = secondShooterChooser.getSelected();
        BeforeThirdShoot currentThirdShoot = thridShooterChooser.getSelected();
        StartingPosition startingPosition = startPositionChooser_.getSelected();
        PanelControlMode panelControl = panelControlMode_.getSelected();

        if ((beforeFirstShootSelected != currentFirstShoot || beforeSecondShootSelected != currentSecondShoot 
        || beforeThirdShootSelected != currentThirdShoot || 
        startingPosition != mCachedStartingPosition || mCachedPanelControl != panelControl) && !isOnlyControlPanel) {
            System.out.println("Auto selection changed, updating creator: First->" + currentFirstShoot.name()
                    + ", Second->" + currentSecondShoot.name() + ", Third->" + currentThirdShoot.name()
                    + ", starting position->" + startingPosition.name());
            beforeFirstShootSelected = currentFirstShoot;
            beforeSecondShootSelected = currentSecondShoot;
            beforeThirdShootSelected = currentThirdShoot;
            mCachedStartingPosition = startingPosition;
            autoOption_ = getAutoModeForParams(currentFirstShoot, currentSecondShoot, currentThirdShoot,  startingPosition);
        }

        mCachedPanelControl = panelControl;
    }

    private Optional<AutoOptionBase> getAutoModeForParams(BeforeFirstShoot firstShootSelected, 
            BeforeSecondShoot secondShootSelected, BeforeThirdShoot thirdShootSelected, StartingPosition position) {
        setInitialPoint(position);
        
        return Optional.of(new AutoCombiner(beforeFirstShootSelected, beforeSecondShootSelected, beforeThirdShootSelected));
    }

    public void reset() {
        autoOption_ = Optional.empty();
        beforeFirstShootSelected = null;
        beforeSecondShootSelected = null;
        beforeThirdShootSelected = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("Before First Shoot Selected", beforeFirstShootSelected.name());
        SmartDashboard.putString("Before Second Shoot Selected", beforeSecondShootSelected.name());
        SmartDashboard.putString("Before Third Shoot Selected", beforeThirdShootSelected.name());
        SmartDashboard.putString("Starting Position Selected", mCachedStartingPosition.name());
        SmartDashboard.putString("PanelControl Selected", mCachedPanelControl.name());
    }

    public Optional<AutoOptionBase> getAutoOption() {
        return autoOption_;
    }

    public Optional<PanelControlMode> getPanControl() {
        return Optional.of(mCachedPanelControl);
    }

    public void setInitialPoint(StartingPosition position) {
        switch(position) {
            case FACE_OPPO_TRENCH:
                setInitialPoint(WayPoints.kFaceOppoTrench);
                break;
            case FACE_POWER_PORT:
                setInitialPoint(WayPoints.kFacePowerPort);
                break;
            case FACE_FRONT_BARRIER:
                setInitialPoint(WayPoints.kFaceFrontBarrier);
                break;
            default:
                setInitialPoint(WayPoints.kFaceOppoTrench);
                break;
        }
    }

    public static Translation2d getInitialPoint() {
        return initialPoint;
    }

    public void setInitialPoint(Translation2d point) {
        RobotState.getInstance().setInitialTranslation(point);
        initialPoint = point;
    }
}