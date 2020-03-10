package frc.robot;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import frc.lib.other.LEDSignal;

public class Constants {

    //use in pure pursuit controller
    public static final boolean LOGGING = true;

    //motion profile solot
    /**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
    public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;
    
    //--------cycle constant---------
    public static final double kLooperDt = 0.01;

	// Controller
	public static final double MinimumControllerInput = 0.15;
	public static final double MaximumControllerInput = 1;
	public static final double MinimumControllerOutput = 0;
	public static final double MaximumControllerOutput = 1;
    
    // Drivebase
    public static final int kLeftDriveMasterId = 12; //------change me!
    public static final int kLeftDriveSlaveAId = 14; //------change me!
    public static final int kLeftDriveSlaveBId = 16; //------change me!
    public static final int kRightDriveMasterId = 11; //------change me!
    public static final int kRightDriveSlaveAId = 13; //------change me!
    public static final int kRightDriveSlaveBId = 15; //------change me!
    public static final int kMiddleWheelId = 18; //------change me!

    public static final int kGearShifter = 4; //------change me!
    public static final int kFenseBlocker = 5; //------change me!

    public static final int kLeftDriveEncoderA = 0;
    public static final int kLeftDriveEncoderB = 1;
    public static final int kRightDriveEncoderA = 2;
    public static final int kRightDriveEncoderB = 3;

    public static final double kHighGear = 64d;//2019 robot: 60d
    public static final double kLowGear = 20d;//2019 robot: 24d
    public static final double kGearNumberOfStages = 3d;

    // Driving Motion Profile and PID
	public static final double HighDriveSpeed = 185;
	public static final double LowDriveSpeed = 95;
	public static final double kRightHighP = 0.02;
	public static final double kRightHighD = 0;
	public static final double kRightHighF = 0.035;
	public static final double kRightHighA = 0;
	public static final double kRightLowP = 0.1;
	public static final double kRightLowD = 0.1;
	public static final double kRightLowF = 0.05763730970902943999708309631717;

	public static final double kLeftHighP = 0.0;
	public static final double kLeftHighD = 0;
	public static final double kLeftHighF = 0.035;
	public static final double kLeftHighA = 0;
	public static final double kLeftLowP = 0.1;
	public static final double kLeftLowD = 0;
	public static final double kLeftLowF = 0.05763730970902943999708309631717;

	public static final double kRightAutoP = 0.12;
	public static final double kRightAutoD = 0.7;
	public static final double kRightAutoF = 0.035;
	public static final double kLeftAutoP = 0.12;
	public static final double kLeftAutoD = 0.7;
	public static final double kLeftAutoF = 0.035;
	public static final double TeleopAccLimit = 120;
    
    // Autonomous Driving
	public static final double TrackRadius = 12;
	public static final double kWheelDiameter = 6;
	public static final double MinimumTurningRadius = 40;
	public static final double MinPathSpeed = 20;
	public static final double MaxPathSpeed = 120;
	public static final double MinLookAheadDistance = 14;
    public static final double MaxLookAheadDistance = 30;

    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 10; //use for constructors

    //Joystick value
    public static final int kDriverJoystickId = 0;
    public static final int kOperatorJoystickId = 1;

    public static class VictorSRXConstants {
        public int id = -1;
        public boolean invert_motor = false;
        public boolean invert_sensor_phase = false;
    }

    public static class SingleMasterMotorSystemConfig {
        public String kName = "ERROR_ASSIGN_A_NAME";

        public VictorSRXConstants kMasterConstants = new VictorSRXConstants();
        public VictorSRXConstants[] kSlaveConstants = new VictorSRXConstants[0];

        public double kHomePosition = 0.0; // Units
        public double kTicksPerUnitDistance = 1.0;
        public double kKp = 0;  // Raw output / raw error
        public double kKi = 0;  // Raw output / sum of raw error
        public double kKd = 0;  // Raw output / (err - prevErr)
        public double kKf = 0;  // Raw output / velocity in ticks/100ms
        public double kKa = 0;  // Raw output / accel in (ticks/100ms) / s
        public double kMaxIntegralAccumulator = 0;
        public int kIZone = 0; // Ticks
        public int kDeadband = 0; // Ticks

        public double kPositionKp = 0;
        public double kPositionKi = 0;
        public double kPositionKd = 0;
        public double kPositionKf = 0;
        public double kPositionMaxIntegralAccumulator = 0;
        public int kPositionIZone = 0; // Ticks
        public int kPositionDeadband = 0; // Ticks

        public int kCruiseVelocity = 0; // Ticks / 100ms
        public int kAcceleration = 0; // Ticks / 100ms / s
        public double kRampRate = 0.0; // s
        public int kContinuousCurrentLimit = 20; // amps
        public int kPeakCurrentLimit = 60; // amps
        public int kPeakCurrentDuration = 200; // milliseconds
        public double kMaxVoltage = 12.0;

        public double kMaxUnitsLimit = Double.POSITIVE_INFINITY;
        public double kMinUnitsLimit = Double.NEGATIVE_INFINITY;

        public int kStastusFrame8UpdateRate = 1000;
        public boolean kRecoverPositionOnReset = false;

        public double kNominalOutputForward = 0;
        public double kNominalOutputReverse = 0;
    }

    // --------------Superstructure Constants--------------------
    // CHANGE ALL THE VALUES UNDER

    // Climber
    public static final int kClimberLimitSwitch = -1;
    public static final int kClimberMaxHeight = 1000;
    public static final SingleMasterMotorSystemConfig kClimber = new SingleMasterMotorSystemConfig();
    public static final int kClimberMasterId = 24;
    public static final int kClimberSlaveId = 25;
    /*
    static {
        kClimber.kName = "Climber";

        kClimber.kMasterConstants.id = 24;
        kClimber.kMasterConstants.invert_motor = false;
        kClimber.kMasterConstants.invert_sensor_phase = false;

        kClimber.kSlaveConstants = new VictorSRXConstants[1];

        kClimber.kSlaveConstants[0] = new VictorSRXConstants();
        kClimber.kSlaveConstants[0].id = 25;
        kClimber.kSlaveConstants[0].invert_motor = false;

        kClimber.kKp = 0.4;
        kClimber.kKi = 0.0001;
        kClimber.kKd = 0;
        kClimber.kKf = 0;
        kClimber.kKa = 0.0;
        kClimber.kMaxIntegralAccumulator = 0;
        kClimber.kIZone = 0; // Ticks
        kClimber.kDeadband = 0; // Ticks

        kClimber.kPositionMaxIntegralAccumulator = 0;
        kClimber.kPositionIZone = 400; // Ticks
        kClimber.kPositionDeadband = 0; // Ticks

        kClimber.kMaxUnitsLimit = 31.1; // inches
        kClimber.kMinUnitsLimit = 0.0; // inches

        kClimber.kCruiseVelocity = 300; //should be 3000
        kClimber.kAcceleration = 300; // should be 3000
        kClimber.kRampRate = 0; // s
        kClimber.kContinuousCurrentLimit = 14; // amps
        kClimber.kPeakCurrentLimit = 40; // amps
        kClimber.kPeakCurrentDuration = 14; // milliseconds

        kClimber.kNominalOutputForward = 0;
        kClimber.kNominalOutputReverse = 0;
    }*/

    // Intake
    public static final int kIntakeBallBlockerId = 7;

    public static final double kIntakePercentageIntake = 0.2;
    public static final double kIntakePercentagePasser = 0.35;
    public static final double kIntakePercentageMaxUpper = 0.25;
    public static final double kIntakePercentageMaxBottom = 0.25;

    public static final double kIntakePercentageMaxUpperInverse = -0.2;
    public static final double kIntakePercentageMaxBottomInverse = -0.2;
    public static final double kIntakePercentagePasserInverse = -0.1;
    public static final double kIntakePercentageRollerInverse = -0.2;

    public static final double kWantTurnReverseCurrent = 30;//in apms
    public static final double kTurnReverseDuration = 0.5;//seconds

    public static final double kIntakeJoystickDeadband = 0.15;

    public static final int kIntakePasserId = 27;
    public static final int kIntakeFrontRollerId = 29;
    public static final int kIntakeUpperId = 28;
    public static final int kIntakeBottomId = 30;

    /*
    public static final SingleMasterMotorSystemConfig kIntake = new SingleMasterMotorSystemConfig();
    static {
        kIntake.kName = "Intake";

        kIntake.kMasterConstants.id = 27;
        kIntake.kMasterConstants.invert_motor = false;
        kIntake.kMasterConstants.invert_sensor_phase = false;

        kIntake.kSlaveConstants = new VictorSRXConstants[0];

        kIntake.kKp = 0.6;
        kIntake.kKi = 0.0001;
        kIntake.kKd = 0;
        kIntake.kKf = 0;
        kIntake.kKa = 0.0;
        kIntake.kMaxIntegralAccumulator = 0;
        kIntake.kIZone = 0; // Ticks
        kIntake.kDeadband = 0; // Ticks

        kIntake.kPositionMaxIntegralAccumulator = 0;
        kIntake.kPositionIZone = 400; // Ticks
        kIntake.kPositionDeadband = 0; // Ticks

        kIntake.kMaxUnitsLimit = 31.1; // inches
        kIntake.kMinUnitsLimit = 0.0; // inches

        kIntake.kCruiseVelocity = 200; // should be 2000
        kIntake.kAcceleration = 300; // should be 3000
        kIntake.kRampRate = 0; // s
        kIntake.kContinuousCurrentLimit = 20; // amps
        kIntake.kPeakCurrentLimit = 40; // amps
        kIntake.kPeakCurrentDuration = 20; // milliseconds

        kIntake.kNominalOutputForward = 0.1;
        kIntake.kNominalOutputReverse = 0.1;
    }*/

    // Shooter
    public static final double kAutoShootingDuration = 1.7;
    public static final double kShooterReverseSpeed = -0.5;
    public static final double kAutoShootingPercentage = 0.5;//0.5 is good for trench
    public static final int kShooterServoId = 0;
    public static double kMaxServoAngle = 0.95;
    public static double kMinServoAngle = 0.5;
    public static final SingleMasterMotorSystemConfig kShooter = new SingleMasterMotorSystemConfig();
    static {
        kShooter.kName = "Shooter";

        kShooter.kMasterConstants.id = 22;
        kShooter.kMasterConstants.invert_motor = false;
        kShooter.kMasterConstants.invert_sensor_phase = true;

        kShooter.kSlaveConstants = new VictorSRXConstants[1];

        kShooter.kSlaveConstants[0] = new VictorSRXConstants();
        kShooter.kSlaveConstants[0].id = 23;
        kShooter.kSlaveConstants[0].invert_motor = true;

        kShooter.kKp = 0.6;
        kShooter.kKi = 0.0001;
        kShooter.kKd = 0;
        kShooter.kKf = 0;
        kShooter.kKa = 0.0;
        kShooter.kMaxIntegralAccumulator = 0;
        kShooter.kIZone = 0; // Ticks
        kShooter.kDeadband = 0; // Ticks

        kShooter.kPositionMaxIntegralAccumulator = 0;
        kShooter.kPositionIZone = 400; // Ticks
        kShooter.kPositionDeadband = 0; // Ticks

        kShooter.kMaxUnitsLimit = 31.1; // inches
        kShooter.kMinUnitsLimit = 0.0; // inches

        kShooter.kCruiseVelocity = 200; // should be 2000
        kShooter.kAcceleration = 300; // should be 3000
        kShooter.kRampRate = 0.005; // s
        kShooter.kContinuousCurrentLimit = 20; // amps
        kShooter.kPeakCurrentLimit = 40; // amps
        kShooter.kPeakCurrentDuration = 20; // milliseconds

        kShooter.kNominalOutputForward = 0.1;
        kShooter.kNominalOutputReverse = 0.1;
    }

    // Turret
    public static final double kTurretFineTuneAngle = 10;
    public static final int kTurretLimitSwitch1 = -1;
    public static final int kTurretLimitSwitch2 = -1;
	public static final double kTurretErrorToleration = 0.5;
	public static final double kTurretErrorTolerationVision = 0.01;
	public static final double kTurretErrorTolerationMaxMin = 4.5;
    public static final double kTurretMotorTeeth = 24;
    public static final double kTurrentTeeth = 165;
    public static final double kTurretMaxAngle = 225;
    public static final double kTurretMinAngle = -65;
    public static final double kTurretMaxMargin = 259.5;//259.5
    public static final double kTurretMinMargin = -99.5;//
    public static final double kTurretRadius = 6;

    public static final double EncoderTicksPerRotation = 4096;
    public static final double DegreesPerEncoderTick = 360 * (1d / EncoderTicksPerRotation);
    public static final double EncoderTicksPerDegree = (1d / 360) * EncoderTicksPerRotation;
    public static final double EncoderTicksPerRotationTurret = 4096 * kTurrentTeeth / kTurretMotorTeeth;
    
    public static final SingleMasterMotorSystemConfig kTurret = new SingleMasterMotorSystemConfig();
    static {
        kTurret.kName = "Turret";

        kTurret.kMasterConstants.id = 21;
        kTurret.kMasterConstants.invert_motor = false;
        kTurret.kMasterConstants.invert_sensor_phase = true;

        kTurret.kSlaveConstants = new VictorSRXConstants[0];

        kTurret.kKp = 0.006;
        kTurret.kKi = 0.00002;
        kTurret.kKd = 0.0015;
        kTurret.kKf = 0;
        kTurret.kKa = 0.0;
        kTurret.kMaxIntegralAccumulator = 0;
        kTurret.kIZone = 0; // Ticks
        kTurret.kDeadband = 0; // Ticks

        kTurret.kPositionMaxIntegralAccumulator = 0;
        kTurret.kPositionIZone = 400; // Ticks
        kTurret.kPositionDeadband = 0; // Ticks

        kTurret.kMaxUnitsLimit = 31.1; // inches
        kTurret.kMinUnitsLimit = 0.0; // inches

        kTurret.kCruiseVelocity = 20; // should be 2000
        kTurret.kAcceleration = 30; // should be 3000
        kTurret.kRampRate = 0; // s
        kTurret.kContinuousCurrentLimit = 20; // amps
        kTurret.kPeakCurrentLimit = 40; // amps
        kTurret.kPeakCurrentDuration = 20; // milliseconds

        kTurret.kNominalOutputForward = 0;
        kTurret.kNominalOutputReverse = 0;
    }

    // Wheel of Fortune
    public static final I2C.Port kColorSensorPort = I2C.Port.kOnboard;

     /**
     * TODO: colors need to be calibrated before the event
     */
    public static final Color kBlueTarget = ColorMatch.makeColor(0.136, 0.412, 0.450);
    public static final Color kGreenTarget = ColorMatch.makeColor(0.196, 0.557, 0.246);
    public static final Color kRedTarget = ColorMatch.makeColor(0.475, 0.371, 0.153);
    public static final Color kYellowTarget = ColorMatch.makeColor(0.293, 0.561, 0.144);

    // Rev Color threshold
    // blue 0.143, 0.427, 0.429
    // green 0.197, 0.561, 0.240
    // red 0.561, 0.232, 0.114
    // yellow 0.361, 0.524, 0.113

    public static final int kWheelOfFortActivator = 9;
    public static final SingleMasterMotorSystemConfig kWheelOfFortune = new SingleMasterMotorSystemConfig();
    static {
        kWheelOfFortune.kName = "Wheel of Fortune";

        kWheelOfFortune.kMasterConstants.id = 19;
        kWheelOfFortune.kMasterConstants.invert_motor = false;
        kWheelOfFortune.kMasterConstants.invert_sensor_phase = false;

        kWheelOfFortune.kSlaveConstants = new VictorSRXConstants[0];

        kWheelOfFortune.kKp = 0.6;
        kWheelOfFortune.kKi = 0.0001;
        kWheelOfFortune.kKd = 0;
        kWheelOfFortune.kKf = 0;
        kWheelOfFortune.kKa = 0.0;
        kWheelOfFortune.kMaxIntegralAccumulator = 0;
        kWheelOfFortune.kIZone = 0; // Ticks
        kWheelOfFortune.kDeadband = 0; // Ticks

        kWheelOfFortune.kPositionMaxIntegralAccumulator = 0;
        kWheelOfFortune.kPositionIZone = 400; // Ticks
        kWheelOfFortune.kPositionDeadband = 0; // Ticks

        kWheelOfFortune.kMaxUnitsLimit = 31.1; // inches
        kWheelOfFortune.kMinUnitsLimit = 0.0; // inches

        kWheelOfFortune.kCruiseVelocity = 200; // should be 2000
        kWheelOfFortune.kAcceleration = 300; // should be 3000
        kWheelOfFortune.kRampRate = 0; // s
        kWheelOfFortune.kContinuousCurrentLimit = 20; // amps
        kWheelOfFortune.kPeakCurrentLimit = 40; // amps
        kWheelOfFortune.kPeakCurrentDuration = 20; // milliseconds

        kWheelOfFortune.kNominalOutputForward = 0.1;
        kWheelOfFortune.kNominalOutputReverse = 0.1;
    }
    
	// Other(last three used in Utility.java)
	public static final double SensorTicksPerMotorRotation = 4096;
	public static final double ExpectedCurrentTolerance = 0;
	public static final double ExpectedRPMTolerance = 0;
    public static final double ExpectedPositionTolerance = 0;
    
    //limelight
    public static final String kLimelightName = "turrentLimelight";
    public static final String kLimelightTableName = "limelight";
    public static final double kLimelightHeight = 37.5;//change this
    public static final double kLimelightAngle = 25;//change this
    public static final double kPowerPortHieght = 54.5;//in inch change this 90.75, 75.5

    //canifier
    public static final int kCanifierId = 10;

    //LED Colors
    public static final LEDSignal kOff = new LEDSignal(0.0, 0.0, 0.0);

    public static final LEDSignal kIntakeIntakingDisk = new LEDSignal(0.0, 1.0, 0.0);
    public static final LEDSignal kIntakeIntakingCargo = new LEDSignal(1.0, 1.0, 1.0);

    public static final LEDSignal kIntakeExhuasting = new LEDSignal(1.0, 0.0, 0.0);

    public static final LEDSignal kRobotZeroed = new LEDSignal(0.0, 1.0, 0.0);
    public static final LEDSignal kFault = new LEDSignal(0.0, 0.0, 1.0);
    public static final LEDSignal kFaultElevator = new LEDSignal(1.0, 0.0, 1.0);

    public static final LEDSignal kHanging = new LEDSignal(0.0, 0.3, 1.0);
    public static final LEDSignal kMinimalPressure = new LEDSignal(0.0, 1.0, 1.0);
    public static final LEDSignal kOptimalPressure = new LEDSignal(1.0, 0.0, 0.0);
}