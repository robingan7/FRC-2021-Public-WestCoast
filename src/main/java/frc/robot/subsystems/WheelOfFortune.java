package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.auton.AutoChooser.PanelControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;

import com.revrobotics.ColorSensorV3;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;

public class WheelOfFortune extends SingleMasterMotorSystem {
  
  private ColorSensorV3 colorSensor_;
  private ColorMatch colorMatcher_;
  private MatcherState matcherState_ = MatcherState.NOTHING;
  private ColorOutput output_;
  private boolean isFinish;
  private ColorArray colorArray_;
  private Solenoid activator;

  private static WheelOfFortune instance = new WheelOfFortune();

	public static WheelOfFortune getInstance() {
    if(instance == null) {
			instance = new WheelOfFortune();
		}
		return instance;
	}

  private static class ColorArray {
    private String lastColor;
    private ArrayList<String> colors;
    private final int completeRotationCycles = 3 * 8;
    private Character targetColor;
    private final List<Character> colorTypes = Arrays.asList('R', 'G', 'B', 'Y');
    //TODO: every method has it should be modify according to the robot
    private int direction; 

    public ColorArray() {
      lastColor = "No";
      targetColor = 'N';
      colors = new ArrayList<String>();
      direction = 1;
    }

    public void addColor(String color, double confidence) {
      if(!color.equals(lastColor) && confidence > 0.9 && isNextColor(color)) {
        colors.add(color);
        lastColor = color;
        System.out.println("current color  " + color + " " + colors.size());

        if(targetColor != 'N') {

          int currentIndex = colorTypes.indexOf(color.charAt(0));
          int targetIndex = colorTypes.indexOf(targetColor);
          int difference = targetIndex - currentIndex;

          if(difference > 0) {
            if(Math.abs(difference) > Math.abs((currentIndex + colorTypes.size()) - targetIndex)) {
              direction = -1;
            } else {
              direction = 1;
            }
          } else if(difference < 0) {
            if(Math.abs(difference) < Math.abs(targetIndex - (currentIndex - colorTypes.size()))) {
              direction = -1;
            } else {
              direction = 1;
            }
          }
        }
      }
    }

    public boolean isCompleteRotation() {
      return colors.size() > completeRotationCycles;
    }

    public void reset() {
      lastColor = "No";
      targetColor = 'N';
      colors = new ArrayList<String>();
      direction = 1;
    }

    public void setTargetColor(Character color) {
      int index = colorTypes.indexOf(color) + 2;

      if(index >= colorTypes.size()) {
        index -= colorTypes.size();
      }

      targetColor = colorTypes.get(index);
    }

    public boolean isCompleteColorMatch() {
      return lastColor.charAt(0) == targetColor;
    }

    public int getDirection() {
      return direction;
    }

    private boolean isNextColor(String currentColor) {
      if(lastColor.equals("No")) {
        return true;
      }

      return currentColor.charAt(0) == colorTypes.get(getNextIndex(direction != 1));
    }

    private int getNextIndex(boolean reverse) {
      if(!reverse) {
        int index = colorTypes.indexOf(lastColor.charAt(0)) + 1;

        if(index >= colorTypes.size()) {
          index -= colorTypes.size();
        }

        return index;
      } else {
        int index = colorTypes.indexOf(lastColor.charAt(0)) - 1;

        if(index < 0) {
          index += colorTypes.size();
        }

        return index;
      }
    }
  }

  private static class ColorOutput {
		public double red;
		public double green;
		public double blue;
		public double confidence;
		public String detectedColor;

    public ColorOutput() {
      this.red = 0;
			this.green = 0;
			this.blue = 0;
			this.confidence = 0;
			this.detectedColor = "NO color";
    }

    public void update(double red, double green,
                          double blue, double confidence, 
                          String detectedColor) {
			this.red = red;
			this.green = green;
			this.blue = blue;
			this.confidence = confidence;
			this.detectedColor = detectedColor;
		}
  }
  
  public enum MatcherState {
		NOTHING, ROTATE_PANEL, COLOR_MATCH
  }

  public WheelOfFortune() {
    super(Constants.kWheelOfFortune);
    output_ = new ColorOutput();
    isFinish = true;
    colorArray_ = new ColorArray();
    colorSensor_ = new ColorSensorV3(Constants.kColorSensorPort);
    colorMatcher_ = new ColorMatch();
    activator = new Solenoid(Constants.kWheelOfFortActivator);

    colorMatcher_.addColorMatch(Constants.kBlueTarget);
    colorMatcher_.addColorMatch(Constants.kGreenTarget);
    colorMatcher_.addColorMatch(Constants.kRedTarget);
    colorMatcher_.addColorMatch(Constants.kYellowTarget);
    colorMatcher_.setConfidenceThreshold(0.80);
  }

  public synchronized Color getColor() {
    Color detectedColor = colorSensor_.getColor();
    String colorString;
    ColorMatchResult match = colorMatcher_.matchClosestColor(detectedColor);

    if (match.color == Constants.kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == Constants.kRedTarget) {
      colorString = "Red";
    } else if (match.color == Constants.kGreenTarget) {
      colorString = "Green";
    } else if (match.color == Constants.kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    output_.update(detectedColor.red, detectedColor.green,
                  detectedColor.blue, match.confidence, 
                  colorString);

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

    return match.color;
  }

  public synchronized boolean getIsFinish() {
    return isFinish;
  }

  public void updateRotation() {
    if(colorArray_.isCompleteRotation()) {
      isFinish = true;
      resetSensors();
      return;
    }

    colorArray_.addColor(output_.detectedColor, output_.confidence);
    setOpenLoop(0.5);
  }

  public void updateColorMatch() {
    if(colorArray_.isCompleteColorMatch()) {
      isFinish = true;
      resetSensors();
      return;
    }

    colorArray_.addColor(output_.detectedColor, output_.confidence);
    setOpenLoop(0.5 * colorArray_.getDirection());
  }

  public synchronized void setStartRotatePanel() {
    matcherState_ = MatcherState.ROTATE_PANEL;
    colorArray_.reset();
    isFinish = false;
  }

  public synchronized void setStartMatchColor(Character color) {
    matcherState_ = MatcherState.COLOR_MATCH;
    colorArray_.reset();
    colorArray_.setTargetColor(color);
    isFinish = false;
  }

  @Override
  public synchronized void resetSensors() {
    setOpenLoop(0.0);
    matcherState_ = MatcherState.NOTHING;
    colorArray_.reset();
  }

  @Override
  public synchronized void update_subsystem() {
    synchronized (WheelOfFortune.this) {
      switch (matcherState_) {
        case ROTATE_PANEL:
          updateRotation();
          break;
        case COLOR_MATCH:
          updateColorMatch();
          break;
        case NOTHING:
          break;
        default:
          System.out.println("Drive state is not valid" + matcherState_);
          break;	
      }
      getColor();
      activator.set(!isFinish);
    }
  }

  /**
   * this method is used in Robot.java
   */
  public void activePanelControl(PanelControlMode mode) {
    switch(mode) {
      case FMS:
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        
        if(gameData.length() > 0) {
          setStartMatchColor(gameData.charAt(0));
        } else {
          setStartRotatePanel();
        }
        break;
      case ROTATE:
        setStartRotatePanel();
        break;
      case GREEN:
        setStartMatchColor('G');  
        break;
      case RED:
        setStartMatchColor('R');
        break;
      case BLUE:
        setStartMatchColor('B');
        break;  
      case YELLOW:
        setStartMatchColor('Y');
        break;
      default:
        break;
    }
  }
}