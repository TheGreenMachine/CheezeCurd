package frc.team1816.states;

public class LEDState {
    public static final LEDState kOff = new LEDState(0.0, 0.0, 0.0);

    public static final LEDState kIntakeHasCube = new LEDState(.1, 0.0, 0.0);
    public static final LEDState kIntakeIntaking = new LEDState(0.0, .1, 0.0);

    public static final LEDState kFault = new LEDState(0.0, 0.0, .1);

    public static final LEDState kHanging = new LEDState(0.0, 0.03, .1);

    public static final LEDState kWantsCube = new LEDState(0.0, 0.025, .1);

    public LEDState(double b, double g, double r) {
        blue = b;
        green = g;
        red = r;
    }

    public void copyFrom(LEDState other) {
        this.blue = other.blue;
        this.green = other.green;
        this.red = other.red;
    }

    public double blue;
    public double green;
    public double red;
}

