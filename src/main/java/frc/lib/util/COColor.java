package frc.lib.util;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.awt.Color;
import java.util.Objects;

public class COColor {
    public static final COColor kRed = new COColor(Color.red);
    public static final COColor kGreen = new COColor(Color.green);
    public static final COColor kBlue = new COColor(Color.blue);
    public static final COColor kCyan = new COColor(Color.cyan);
    public static final COColor kMagenta = new COColor(Color.magenta);
    public static final COColor kYellow = new COColor(Color.yellow);
    public static final COColor kWhite = new COColor(Color.white);
    public static final COColor kBlack = new COColor(Color.black);
    public static final COColor kOff = kBlack;
    public static final COColor kCOOrangePure = new COColor(255, 122, 28);
    public static final COColor kCOTealPure = new COColor(32, 146, 153);
    public static final COColor kCOOrangeLed = new COColor(255, 30, 0, kCOOrangePure);
    public static final COColor kCOTealLed = new COColor(0, 100, 35, kCOTealPure);

    public static final COColor kOrange = new COColor(255, 80, 0);
    public static final COColor kPurple = new COColor(255, 0, 255);
    public static final COColor kGray = new COColor(128, 128, 128);
    public static final COColor kPink = new COColor(255, 0, 100);
    public static final COColor kRust = new COColor(147, 58, 22);
    public static final COColor kLowBattery = kYellow;
    public static final COColor kGoodBattery = kGreen;

    public static final COColor[] kRainbow = {
        kWhite, kWhite, kWhite, kWhite, kWhite, kWhite, kWhite, kWhite, kOff, kRed, kOrange, kYellow, kGreen, kCyan,
        kBlue, kPurple, kOff, kOff
    };
    public static final COColor kRainbowRepresentation = new COColor(180, 255, 0);

    public static final COColor kCoralMode = kCOOrangeLed;
    public static final COColor kAlgaeMode = kCOTealLed;
    public static final COColor kCoralManual = kWhite;

    private final int red;
    private final int green;
    private final int blue;
    private final int alpha;
    private final int pureRed;
    private final int pureGreen;
    private final int pureBlue;

    public COColor(int r, int g, int b, int a, int pR, int pG, int pB) {
        red = r;
        green = g;
        blue = b;
        alpha = a;
        pureRed = pR;
        pureGreen = pG;
        pureBlue = pB;
    }

    public COColor(int r, int g, int b, int pR, int pG, int pB) {
        this(r, g, b, 255, pR, pG, pB);
    }

    public COColor(int r, int g, int b, int a) {
        this(r, g, b, a, r, g, b);
    }

    public COColor(int r, int g, int b) {
        this(r, g, b, 255);
    }

    public COColor(int r, int g, int b, int a, COColor pure) {
        this(r, g, b, a, pure.red, pure.green, pure.blue);
    }

    public COColor(int r, int g, int b, COColor pure) {
        this(r, g, b, 255, pure.red, pure.green, pure.blue);
    }

    public COColor() {
        this(0, 0, 0, 0);
    }

    public COColor(Color color) {
        this(color.getRed(), color.getGreen(), color.getBlue(), color.getAlpha());
    }

    public COColor(Color8Bit color) {
        this(color.red, color.green, color.blue);
    }

    public COColor(RGBWColor color) {
        this(color.Red, color.Green, color.Blue, color.White);
    }

    public Color getColor() {
        return new Color(red, green, blue);
    }

    public RGBWColor getRGBW() {
        return new RGBWColor(red, green, blue, alpha);
    }

    public Color8Bit getColor8Bit() {
        return new Color8Bit(red, green, blue);
    }

    public Color getPureColor() {
        return new Color(pureRed, pureGreen, pureBlue);
    }

    public RGBWColor getPureRGBW() {
        return new RGBWColor(pureRed, pureGreen, pureBlue, alpha);
    }

    public Color8Bit getPureColor8Bit() {
        return new Color8Bit(pureRed, pureGreen, pureBlue);
    }

    @Override
    public boolean equals(Object other) {
        if (other == null) {
            return false;
        }

        if (other.getClass() != this.getClass()) {
            return false;
        }

        COColor s = (COColor) other;
        return this.blue == s.blue && this.red == s.red && this.green == s.green && this.alpha == s.alpha;
    }

    @Override
    public String toString() {
        String redString = Integer.toHexString(red);
        if (redString.length() == 1) {
            redString = "0" + redString;
        }
        String greenString = Integer.toHexString(green);
        if (greenString.length() == 1) {
            greenString = "0" + greenString;
        }
        String blueString = Integer.toHexString(blue);
        if (blueString.length() == 1) {
            blueString = "0" + blueString;
        }
        return "#" + redString + greenString + blueString;
    }

    @Override
    public int hashCode() {
        return Objects.hash(red, green, blue, alpha);
    }
}
