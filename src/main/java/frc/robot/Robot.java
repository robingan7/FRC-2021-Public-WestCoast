package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.cycles.Cycle_in;
import frc.robot.cycles.Subsystem_Cycle_Manager;
import frc.robot.joysticks.MainControlBoard;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DevineShooter.ShooterState;
import frc.robot.subsystems.LukeIntake.IntakeState;
import frc.robot.subsystems.Limelight.LedMode;
import frc.robot.subsystems.AustinTurret.TurretState;
import frc.robot.auton.AutoActivator;
import frc.robot.auton.AutoChooser;
import frc.robot.auton.AutoOptionBase;
import frc.lib.utility.DelayBoolean;
import frc.lib.utility.LatchDelayBoolean;

import com.ctre.phoenix.motorcontrol.*;

import java.util.Arrays;
import java.util.Optional;

public class Robot extends TimedRobot {
  private Cycle_in enabledCycler_ = new Cycle_in();
  private Cycle_in disabledCycler_ = new Cycle_in();

  private MainControlBoard controlBoard_ = MainControlBoard.getInstance();

  private final RobotState robotState_ = RobotState.getInstance();
  private final DevineShooter shooter_ = DevineShooter.getInstance();
  private final LukeIntake intake_ = LukeIntake.getInstance();
  private final NicoDrivebase drivebase_ = NicoDrivebase.getInstance();
  /*private final WheelOfFortune wheelOfFortune_ = WheelOfFortune.getInstance();*/
  private final XitingJinClimber climber_ = XitingJinClimber.getInstance();
  private final AustinTurret turret_ = AustinTurret.getInstance();
  private final Limelight limelight_ = Limelight.getInstance();
  private final LEDController ledController_ = LEDController.getInstance();
  private final Subsystem_Cycle_Manager subsystem_Cycle_Manager_ = new Subsystem_Cycle_Manager(
    Arrays.asList(
      drivebase_,
      robotState_,
      turret_,
      shooter_,
      intake_,
      climber_,
      /*wheelOfFortune_,
      */
      ledController_,
      limelight_
    )
  );

  private DelayBoolean startCPActivator = new DelayBoolean();
  private DelayBoolean killCPCPActivator = new DelayBoolean();
  private DelayBoolean resetTurretActivator = new DelayBoolean();
  private DelayBoolean turretMoveRightActivator = new DelayBoolean();
  private DelayBoolean turretMoveLeftActivator = new DelayBoolean();
  private LatchDelayBoolean autoAimming = new LatchDelayBoolean();
  private AutoChooser autoModeChooser_ = new AutoChooser();
  private AutoActivator autoModeActivator_;
  
  @Override
  public void robotInit() {
    try{
      /*wheelOfFortune_.resetSensors();*/
      intake_.resetSensors();
      climber_.resetSensors();
      shooter_.resetSensors(); 
      intake_.resetSensors();
      subsystem_Cycle_Manager_.registerEnabledCycles(enabledCycler_);
      subsystem_Cycle_Manager_.registerDisabledLoops(disabledCycler_);
      limelight_.setLed(LedMode.OFF);
    }catch(Throwable t){
      throw t;
    }
  }

  @Override
  public void robotPeriodic() {
    try {
      subsystem_Cycle_Manager_.sendDataToSmartDashboard();
      autoModeChooser_.outputToSmartDashboard();
    } catch (Throwable t) {
        throw t;
    }
  }

  @Override
  public void autonomousInit() {
    try{
      disabledCycler_.stop_all();
      turret_.resetSensors();

		  System.out.println("the autooooooo: " + autoModeActivator_.getAutoOption());
      if(autoModeChooser_.getAutoOption() != null) {
        autoModeActivator_.start();
      } else {
        System.out.println("Can't start with null autonomous mode");
      }

      enabledCycler_.start_all();
    }catch(Throwable t){
      throw t;
    }
  }

  @Override
  public void autonomousPeriodic() {
    try{

    }catch(Throwable t){
      throw t;
    }
  }

  @Override
  public void teleopInit() {
    SmartDashboard.putString("Robot State", "Start driving");
    try{
      disabledCycler_.stop_all();
      if (autoModeActivator_ != null) {
        autoModeActivator_.stop();
      }
      enabledCycler_.start_all();
      turret_.setState(TurretState.SET_POINT);
      turret_.resetControllers();
      limelight_.setLed(LedMode.OFF);
    }catch(Throwable t){
        throw t;
    }
  }

  @Override
  public void teleopPeriodic() {
    try{
      
      double timestamp = Timer.getFPGATimestamp();
      double speed = controlBoard_.getSpeed();
      double turn = controlBoard_.getTurn();
      double hDrive = controlBoard_.getHDriveLeft() + controlBoard_.getHDriveRight();
      boolean isActiveCP = controlBoard_.isActivePanelControl();
      boolean isKillCP = controlBoard_.isKillPanelControl();
      boolean isAutoAimming = controlBoard_.isAutoAimming();
      boolean isHomeTurret = controlBoard_.isHomeTurret();
      boolean isTurretMoveRight = controlBoard_.isTurretMoveRight();
      boolean isTurretMoveLeft = controlBoard_.isTurretMoveLeft();

      if(startCPActivator.canBeActived(isActiveCP, timestamp)) {
        autoModeChooser_.updateModeCreator(true);
        //wheelOfFortune_.activePanelControl(autoModeChooser_.getPanControl().get());
      }

      if(killCPCPActivator.canBeActived(isKillCP, timestamp)) {
        //wheelOfFortune_.resetSensors();
      }
      
      if(resetTurretActivator.canBeActived(isHomeTurret, timestamp)) {
        //turret_.setState(TurretState.HOMING);
      } else {
        if(autoAimming.canBeActived(isAutoAimming, timestamp)) {
          //turret_.setState(TurretState.SET_POINT);
          shooter_.setLowPosition();
          intake_.stop();
        } else {
          //turret_.setState(TurretState.VISION);
          intake_.setLowSpeed();
          shooter_.setHighPosition();
        }
      }

      if(turretMoveRightActivator.canBeActived(isTurretMoveRight, timestamp)) {
        //turret_.addAngleOffset(Constants.kTurretFineTuneAngle);
      } else if(turretMoveLeftActivator.canBeActived(isTurretMoveLeft, timestamp)) {
        //turret_.addAngleOffset(-Constants.kTurretFineTuneAngle);
      }
      
      
      shooter_.setOpenLoop(hDrive * 0.5);
      drivebase_.arcadeDrive(-speed, turn, hDrive);
    }catch(Throwable t){
      throw t;
    }
  }

  @Override
  public void testInit() {
    try {
      System.out.println("Starting check systems.");

      disabledCycler_.stop_all();
      enabledCycler_.stop_all();

      if (subsystem_Cycle_Manager_.checkSubsystems()) {
          System.out.println("ALL SYSTEMS PASSED");
      } else {
          System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
      }
    } catch (Throwable t) {
        throw t;
    }
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() { 
    try {
      enabledCycler_.stop_all();
      limelight_.setLed(LedMode.OFF);

      if (autoModeActivator_ != null) {
          autoModeActivator_.stop();
      }

      autoModeChooser_.reset();
      autoModeChooser_.updateModeCreator(false);
      autoModeActivator_ = new AutoActivator();

      //turret_.setState(TurretState.SET_POINT);
      //turret_.resetControllers();
      disabledCycler_.start_all();
    } catch (Throwable t) {
      throw t;
    }
  }

  @Override
  public void disabledPeriodic() {
    try{
      // Update auto modes
      autoModeChooser_.updateModeCreator(false);
      Optional<AutoOptionBase> autoMode = autoModeChooser_.getAutoOption();
      
      if (autoMode.isPresent() && autoMode.get() != autoModeActivator_.getAutoOption()) {
          System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
          autoModeActivator_.setAutoOption(autoMode.get());
      }
    }catch (Throwable t) {
        throw t;
    }
  }
}