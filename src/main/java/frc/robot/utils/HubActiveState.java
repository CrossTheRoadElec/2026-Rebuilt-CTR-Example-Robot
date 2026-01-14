package frc.robot.utils;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HubActiveState {
    private HubActiveState(){}

    static HubActiveState _inst = new HubActiveState();
    public static HubActiveState getInstance(){ return _inst; }

    private class TimeSegment {
        public double start;
        public double end;
        public TimeSegment(double start, double end) {
            this.start = start;
            this.end = end;
        }
        public boolean isTimeWithin(double time) {
            return time >= start && time <= end;
        }
    }
    private final TimeSegment[] TeleopTimesForAutoWinner = new TimeSegment[] {
        new TimeSegment(0, 55), // Shift 4 combines with End game
        new TimeSegment((1 * 60) + 20, (1 * 60) + 45), // Shift 2
        new TimeSegment((2 * 60) + 10, (2 * 60) + 20) // Transition Shift
    };
    private final TimeSegment[] TeleopTimesForAutoLoser = new TimeSegment[] {
        new TimeSegment(0, 30), // End game
        new TimeSegment(55, (1 * 60) + 20), // Shift 3
        new TimeSegment((1 * 60) + 45, (2 * 60) + 20) // Shift 1 combines with transition shift
    };
    /* What to publish over networktables for telemetry */
    private final NetworkTable inst = NetworkTableInstance.getDefault().getTable("Hub");
    private final BooleanPublisher isHubActivePublisher = inst.getBooleanTopic("Active").publish();
    private final DoublePublisher timeUntilSwapPublisher = inst.getDoubleTopic("Time Until Swap").publish();

    /* True when our hub is active */
    private boolean isHubActive = false;
    private double timeUntilTransition = 0;

    private void updateStatesForTeleop() {
        if (!DriverStation.isTeleopEnabled()) return;

        String whoWonAuto = DriverStation.getGameSpecificMessage();
        boolean redIsWinner;
        switch (whoWonAuto) {
            case "R": redIsWinner = true; break;
            case "B": redIsWinner = false; break;
            default: return; // We don't have the message yet, we can't determine
        }

        /* Check to see if our current time is within a time that it doesn't matter what alliance we're on */
        double timeLeftInTeleop = DriverStation.getMatchTime();
        double timeUntilSwap = 150; // Start at 2:30 so it definitely gets cleared

        /* When we're in teleop we should definitely have the alliance color, so we can confidently use the DS alliance API */
        boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;

        /* This boils down to an XOR but we explicitly call out both cases so it's clear */
        boolean useWinnerTimes = (redIsWinner && isRedAlliance) || (!redIsWinner && !isRedAlliance);

        /* If we're the winner, use the winner times */
        if (useWinnerTimes) {
            for (TimeSegment seg : TeleopTimesForAutoWinner) {
                if (seg.isTimeWithin(timeLeftInTeleop)) {
                    timeUntilTransition = timeLeftInTeleop - seg.start;
                    isHubActive = true;
                    return;
                }
                double timeToStart = timeLeftInTeleop - seg.end;
                if (timeToStart > 0 && timeToStart < timeUntilSwap) {
                    /* Update our time until swap with this, since it's sooner */
                    timeUntilSwap = timeToStart;
                }
            }
        } else {
            for (TimeSegment seg : TeleopTimesForAutoLoser) {
                if (seg.isTimeWithin(timeLeftInTeleop)) {
                    timeUntilTransition = timeLeftInTeleop - seg.start;
                    isHubActive = true;
                    return;
                }
                double timeToStart = timeLeftInTeleop - seg.end;
                if (timeToStart > 0 && timeToStart < timeUntilSwap) {
                    /* Update our time until swap with this, since it's sooner */
                    timeUntilSwap = timeToStart;
                }
            }
        }
        timeUntilTransition = timeUntilSwap;
        isHubActive = false;
    }

    public void periodic() {
        /* Check what state we're in */
        if (DriverStation.isDisabled()) {
            /* If we're disabled, the hub is always inactive */
            isHubActive = false;
            timeUntilTransition = -1;
        } else if (DriverStation.isAutonomous()) {
            /* If we're autonomous, the hub is always active */
            isHubActive = true;
            timeUntilTransition = DriverStation.getMatchTime();
        } else if (DriverStation.isTeleop()) {
            /* If we're teleop, the hub will go through phases as outlined in the manual that we'll use to determine the active state */
            updateStatesForTeleop();
        } else {
            /* This is a state that we don't recognize, so make ti false */
            isHubActive = false;
        }

        isHubActivePublisher.accept(isHubActive);
        timeUntilSwapPublisher.accept(timeUntilTransition);
    }

    public boolean isOurHubActive() {
        return isHubActive;
    }

    public double timeUntilTransition() {
        return timeUntilTransition;
    }
}
