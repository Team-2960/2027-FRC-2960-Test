package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class LEDControl extends SubsystemBase {

    private static final int CANDLE_ID = 30;

    private static final int FIRST_LED = 0;// TODO rembmer there are 8 LEDS on the CANdle

    private static final int LAST_LED = 100;

    private static final int ANIMATION_SLOT = 0;

    private static final double BRIGHTNESS = 0.25;

    public static final RGBWColor OFF = new RGBWColor(0, 0, 0);
    public static final RGBWColor RED = new RGBWColor(255, 0, 0);
    public static final RGBWColor BLUE = new RGBWColor(0, 0, 255);
    public static final RGBWColor GREEN = new RGBWColor(0, 255, 0);
    public static final RGBWColor YELLOW = new RGBWColor(255, 180, 0);
    public static final RGBWColor ORANGE = new RGBWColor(255, 80, 0);
    public static final RGBWColor PURPLE = new RGBWColor(140, 0, 255);
    public static final RGBWColor WHITEBLUE = new RGBWColor(220, 235, 255);
    public static final RGBWColor WHITERED = new RGBWColor(255, 65, 65);

    private final CANdle candle = new CANdle(CANDLE_ID, Constants.rioBus);//TODO make sure its right bus

    private final SolidColor solidRequest = new SolidColor(FIRST_LED, LAST_LED);
    private final EmptyAnimation clearAnimation = new EmptyAnimation(ANIMATION_SLOT);

    private RGBWColor lastSolid = null;

    public LEDControl() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.GRB; // TODO make sure this matches LED strip
        config.LED.BrightnessScalar = BRIGHTNESS;
        candle.getConfigurator().apply(config);
        
        setDefaultCommand(this.getmodeDefaultCmd());
    }

    private void setSolid(RGBWColor color) {
        candle.setControl(clearAnimation);
        candle.setControl(solidRequest.withColor(color));
        lastSolid = color;
    }

    private Command animate(ControlRequest animation) {
        return startRun(
                () -> {
                    lastSolid = null;
                    candle.setControl(animation);
                },
                () -> {
                })
                .ignoringDisable(true);
    }

    public Command getSolidCmd(RGBWColor color) {
        return startRun(() -> setSolid(color), () -> {
        }).ignoringDisable(true);
    }

    public Command getOffCmd() {
        return getSolidCmd(OFF);
    }

    public Command getStrobeCmd(RGBWColor color, double frameRate) {
        return animate(
                new StrobeAnimation(FIRST_LED, LAST_LED)
                        .withSlot(ANIMATION_SLOT)
                        .withColor(color)
                        .withFrameRate(frameRate));
    }

    public Command getLarsonCmd(RGBWColor color) {
        return animate(
                new LarsonAnimation(FIRST_LED, LAST_LED)
                        .withSlot(ANIMATION_SLOT)
                        .withColor(color));
    }

    public Command getRainbowCmd() {
        return animate(
                new RainbowAnimation(FIRST_LED, LAST_LED).withSlot(ANIMATION_SLOT));
    }

    public Command getAllianceColorCmd() {
        return run(() -> {
            RGBWColor color = DriverStation.getAlliance()
                    .map(alliance -> alliance == Alliance.Red ? RED : BLUE)
                    .orElse(ORANGE);
            if (!color.equals(lastSolid)) {
                setSolid(color);
            }
        })
                .ignoringDisable(true);
    }

    public Command getmodeDefaultCmd() {
        return Commands.either(
                 getStrobeCmd(WHITEBLUE, 4).until(DriverStation::isEnabled),
                getSolidCmd(WHITEBLUE).until(DriverStation::isDisabled),
                DriverStation::isEnabled)
                .repeatedly()
                .ignoringDisable(true);
    }
}
