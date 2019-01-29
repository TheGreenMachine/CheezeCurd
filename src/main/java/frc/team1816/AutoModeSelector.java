package frc.team1816;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1816.auto.AutoModeBase;
import frc.team1816.auto.creators.*;

import java.util.Optional;

public class AutoModeSelector {

    enum StartingPosition {
        LEFT,
        CENTER,
        RIGHT,
    }

    enum SwitchScalePosition {
        USE_FMS_DATA,
        LEFT_SWITCH_LEFT_SCALE,
        LEFT_SWITCH_RIGHT_SCALE,
        RIGHT_SWITCH_LEFT_SCALE,
        RIGHT_SWITCH_RIGHT_SCALE,
    }

    enum DesiredMode {
        DO_NOTHING,
        CROSS_AUTO_LINE,
        SIMPLE_SWITCH,
        SCALE_AND_SWITCH,
        ONLY_SCALE,
        LIVING_ROOM,
        SHOP,
    }

    private DesiredMode mCachedDesiredMode = null;
    private SwitchScalePosition mCachedSwitchScalePosition = null;
    private StartingPosition mCachedStartingPosition = null;

    private Optional<AutoModeCreator> mCreator = Optional.empty();

    private AutoFieldState mFieldState = AutoFieldState.getInstance();

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<SwitchScalePosition> mSwitchScalePositionChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Cross Auto Line", DesiredMode.CROSS_AUTO_LINE);
        mModeChooser.addOption("Living Room",DesiredMode.LIVING_ROOM);
        mModeChooser.addOption("Shop", DesiredMode.SHOP);
        SmartDashboard.putData("Auto mode", mModeChooser);

        mStartPositionChooser = new SendableChooser<>();
        mStartPositionChooser.setDefaultOption("Right", StartingPosition.RIGHT);
        mStartPositionChooser.addOption("Center", StartingPosition.CENTER);
        mStartPositionChooser.addOption("Left", StartingPosition.LEFT);
        SmartDashboard.putData("Starting Position", mStartPositionChooser);

        mSwitchScalePositionChooser = new SendableChooser<>();
        mSwitchScalePositionChooser.setDefaultOption("Use FMS Data", SwitchScalePosition.USE_FMS_DATA);
        mSwitchScalePositionChooser.addOption("Left Switch Left Scale", SwitchScalePosition.LEFT_SWITCH_LEFT_SCALE);
        mSwitchScalePositionChooser.addOption("Left Switch Right Scale", SwitchScalePosition.LEFT_SWITCH_RIGHT_SCALE);
        mSwitchScalePositionChooser.addOption("Right Switch Left Scale", SwitchScalePosition.RIGHT_SWITCH_LEFT_SCALE);
        mSwitchScalePositionChooser.addOption("Right Switch Right Scale", SwitchScalePosition.RIGHT_SWITCH_RIGHT_SCALE);
        SmartDashboard.putData("Switch and Scale Position", mSwitchScalePositionChooser);

    }

    void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition staringPosition = mStartPositionChooser.getSelected();
        SwitchScalePosition switchScalePosition = mSwitchScalePositionChooser.getSelected();
        if (mCachedDesiredMode != desiredMode || staringPosition != mCachedStartingPosition || switchScalePosition != mCachedSwitchScalePosition) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name() + ", starting position->" + staringPosition.name() + ", switch/scale position->" + switchScalePosition.name());
            mCreator = getCreatorForParams(desiredMode);
            if (switchScalePosition == SwitchScalePosition.USE_FMS_DATA) {
                mFieldState.disableOverride();
            } else {
                setFieldOverride(switchScalePosition);
            }
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = staringPosition;
        mCachedSwitchScalePosition = switchScalePosition;
    }

    private Optional<AutoModeCreator> getCreatorForParams(DesiredMode mode) {
        switch (mode) {
            case CROSS_AUTO_LINE:
                return Optional.of(new CrossAutoLineCreator());
            case LIVING_ROOM:
                return  Optional.of(new LivingRoomModeCreator());
            case SHOP:
                return Optional.of(new ShopModeCreator());
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    private void setFieldOverride(SwitchScalePosition switchScalePosition) {
        switch (switchScalePosition) {
            case LEFT_SWITCH_LEFT_SCALE:
                mFieldState.overrideSides("LLL");
                break;
            case LEFT_SWITCH_RIGHT_SCALE:
                mFieldState.overrideSides("LRL");
                break;
            case RIGHT_SWITCH_LEFT_SCALE:
                mFieldState.overrideSides("RLR");
                break;
            case RIGHT_SWITCH_RIGHT_SCALE:
                mFieldState.overrideSides("RRR");
                break;
            default:
                break;
        }
    }

    public void reset() {
        mCreator = Optional.empty();
        mFieldState.disableOverride();
        mCachedDesiredMode = null;
    }

    void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
        SmartDashboard.putString("SwitchScalePositionSelected", mCachedSwitchScalePosition.name());
    }

    Optional<AutoModeBase> getAutoMode(AutoFieldState fieldState) {
        if (mCreator.isEmpty()) {
            return Optional.empty();
        }
        if (fieldState.isOverridingGameData()) {
            System.out.println("Overriding FMS switch/scale positions!");
        }
        return Optional.of(mCreator.get().getStateDependentAutoMode(fieldState));
    }

}

