package frc.robot.subsystems.leds;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.RainbowCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;

public class LEDs extends SubsystemBase {

    public enum BlinkState {
        SOLID,
        FAST,
        SlOW;
    }
    private final int LED_LENGTH = 59; //this number might be wrong
    private final int PORT = 0;

    private AddressableLED led;
    private AddressableLEDBuffer buffer;

    private int frame;

    private boolean teamColorsAnimation = false;

    
    private Color color1 = Color.kBlack;
    private Color color2 = Color.kBlack;
    private BlinkState blink = BlinkState.SOLID;

    private static LEDs instance;

    public static LEDs getInstance() {
        if (instance == null) {
            instance = new LEDs();
        }

        return instance;
    }

    private LEDs() {
        led = new AddressableLED(PORT);
        buffer = new AddressableLEDBuffer(LED_LENGTH);
        led.setLength(buffer.getLength());

        setDefaultCommand(Commands.either(
            Commands.runOnce(this::setTeamColors), 
            Commands.select(Map.ofEntries(
                Map.entry(BlinkState.SOLID, this.setSolidColor()),
                Map.entry(BlinkState.SlOW, this.blink(0.12)),
                Map.entry(BlinkState.FAST, this.blink(0.06))
            ), () -> blink), 
            DriverStation::isDisabled
        ).ignoringDisable(true));

        setTeamColors();
    };

    // Purple 143 139 189
    // Blue 1 195 203

    private void setTeamColors() {
        setColorNoOutput(174, 19, 186, false, 1, 12);
        setColorNoOutput(1, 195, 203, false, 12, 22);

        // Flip for other led strip
        setColorNoOutput(1, 195, 203, false, 22, 32);
        setColorNoOutput(174, 19, 186, false, 32, 42);

        setOutput();
    }

    public void setTopColor(Color color) {
        setColorNoOutput((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), false, 12, 22);
        setColorNoOutput((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), false, 22, 32);
        setOutput();
    }

    public void setBottomColor(Color color) {
        setColorNoOutput((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), false, 1, 12);
        setColorNoOutput((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), false, 32, 42);
        setOutput();
    }

    private void setColors(Color top, Color bottom) {
        setTopColor(top);
        setBottomColor(bottom);
    }

    public Command blink(double pause){
        return Commands.sequence(
            runOnce(() -> setColors(color1, color2)),
            Commands.waitSeconds(pause),
            runOnce(() -> setColors(color1, color2)),
            Commands.waitSeconds(pause)
        );
    }

    public Command setSolidColor(){
        return Commands.runOnce(() -> this.setColors(color1, color2));
    }

    public Command intakingAlgae() {
        return Commands.repeatingSequence(
                runOnce(() -> setColor(Color.kTurquoise)),
                Commands.waitSeconds(0.2),
                runOnce(() -> setColor(Color.kBlack)),
                Commands.waitSeconds(0.1));
    }

    public Command intakingCoral(){
        return Commands.repeatingSequence(
                runOnce(() -> setColor(Color.kMediumPurple)),
                Commands.waitSeconds(0.2),
                runOnce(() -> setColor(Color.kBlack)),
                Commands.waitSeconds(0.1));
    }

    public Command intookAlgae() {
        return Commands.repeatingSequence(
                runOnce(() -> setColor(Color.kGreen)),
                Commands.waitSeconds(0.2),
                runOnce(() -> setColor(Color.kBlack)),
                Commands.waitSeconds(0.1)).withTimeout(3);
    }

    public Command intookCoral() {
        return Commands.repeatingSequence(
                runOnce(() -> setColor(Color.kSteelBlue)),
                Commands.waitSeconds(0.2),
                runOnce(() -> setColor(Color.kBlack)),
                Commands.waitSeconds(0.1)).withTimeout(3);
    }

    public Command elevator(){
        return Commands.repeatingSequence(
            runOnce(() -> setColor(Color.kPowderBlue)),
            Commands.waitSeconds(0.2),
            runOnce(() -> setColor(Color.kBlack)),
            Commands.waitSeconds(0.1)).withTimeout(3);
    }

    public void setColor(Color color) {
        setStripRGB((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
    }

    private void setColorNoOutput(int a0, int a1, int a2, boolean colorModel, int start, int end) /*
                                                                                                   * false: RGB; true:
                                                                                                   * HSV
                                                                                                   */ {
        start = MathUtil.clamp(start, 0, LED_LENGTH);
        end = MathUtil.clamp(end, start, LED_LENGTH);

        for (int i = start; i < end; i++) {
            if (colorModel)
                buffer.setHSV(i, a0, a1, a2);
            else
                buffer.setRGB(i, a0, a1, a2);
        }
    }

    private void setColor(int a0, int a1, int a2, boolean colorModel, int start, int end) /* false: RGB; true: HSV */ {
        setColorNoOutput(a0, a1, a2, colorModel, start, end);
        setOutput();
    }

    public void setLEDRGB(int r, int g, int b, int led) {
        setColor(r, g, b, false, led, led);
    }

    public void setPartRGB(int r, int g, int b, int start, int end) {
        setColor(r, g, b, false, start, end);
    }

    public void setStripRGB(int r, int g, int b) {
        setColor(r, g, b, false, 0, LED_LENGTH);
    }

    public void setTopColorRGB(int r, int g, int b) {
        setColor(r, g, b, false, LED_LENGTH / 2, LED_LENGTH);
    }

    public void setBottomColorRGB(int r, int g, int b) {
        setColor(r, g, b, false, 0, LED_LENGTH / 2);
    }

    public void setPartHSV(int h, int s, int v, int start, int end) {
        setColor(h, s, v, true, start, end);
    }

    public void setStripHSV(int h, int s, int v) {
        setColor(h, s, v, true, 0, LED_LENGTH);
    }

    public void setTopColorHSV(int h, int s, int v) {
        setColor(h, s, v, true, LED_LENGTH / 2, LED_LENGTH);
    }

    public void setBottomColorHSV(int h, int s, int v) {
        setColor(h, s, v, true, 0, LED_LENGTH / 2);
    }

    // public void setStripAlliance() {
    // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
    // setStripRGB(255, 0, 0);
    // }
    // else {
    // setStripRGB(0, 0, 255);
    // }
    // }

    public void toggleTeamColorsAnimation() {
        if (teamColorsAnimation) {
            teamColorsAnimation = false;
        } else {
            teamColorsAnimation = true;
        }
    }

    private void updateTeamColor() {
        if ((frame / 10) % 2 == 0) {
            setStripRGB(143, 139, 189);
        } else {
            setStripRGB(1, 195, 203);
        }
    }

    public void setHSVIndex(int index, int h, int s, int v) {
        buffer.setHSV(index, h, s, v);
    }

    public int getLength() {
        return buffer.getLength();
    }

    public void setOutput() {
        led.setData(buffer);
        led.start();
    }
    
    @Override
    public void periodic() {
    // updates LEDs to show state of intake
    if(Swerve.getInstance().isSlowMode()) {
        color1 = Color.kBlack;
        color2 = Color.kBlack;
    } else if (OIConstants.aligned) {
        color1 = Color.kOrange;
        color2 = Color.kOrange;
    } else if(Constants.OIConstants.isScoringLeft && OIConstants.autoScoreMode == 4) {
        color1 = Color.kAqua;
        color2 = Color.kGreen;
    } else if(!Constants.OIConstants.isScoringLeft && OIConstants.autoScoreMode == 4) {
        color1 = Color.kPurple;
        color2 = Color.kGreen;
    } else if(Constants.OIConstants.isScoringLeft && OIConstants.autoScoreMode == 3) {
        color1 = Color.kAqua;
        color2 = Color.kYellow;
    } else if(!Constants.OIConstants.isScoringLeft && OIConstants.autoScoreMode == 3) {
        color1 = Color.kPurple;
        color2 = Color.kYellow;
    }  else if(Constants.OIConstants.isScoringLeft && OIConstants.autoScoreMode == 2) {
        color1 = Color.kAqua;
        color2 = Color.kRed;
    }  else if(!Constants.OIConstants.isScoringLeft && OIConstants.autoScoreMode == 2) {
        color1 = Color.kPurple;
        color2 = Color.kRed;
    } 


    if(Intake.getInstance().isStoring && !OIConstants.aligned) {
        blink = BlinkState.SlOW;
    } else {
        blink = BlinkState.SOLID;
    }


    }

}
