package ftc.evlib.vision.processors;

import com.google.common.collect.ImmutableList;

import java.util.List;

import ftc.electronvolts.util.TeamColor;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 11/15/16
 *
 * enum that represents the different types of beacon images
 */
public enum BeaconName {
    WHEELS,
    TOOLS,
    LEGOS,
    GEARS;

    private static final List<BeaconName> RED_BEACON_NAMES = ImmutableList.of(BeaconName.GEARS, BeaconName.TOOLS);
    private static final List<BeaconName> BLUE_BEACON_NAMES = ImmutableList.of(BeaconName.WHEELS, BeaconName.LEGOS);

    /**
     * Get the names of the beacons for a certain team color
     *
     * @param teamColor your team's color
     * @return the list of the beacon names. null if the given TeamColor was UNKNOWN
     */
    public static List<BeaconName> getNamesForTeamColor(TeamColor teamColor) {
        switch (teamColor) {
            case RED:
                return RED_BEACON_NAMES;
            case BLUE:
                return BLUE_BEACON_NAMES;
            default:
                throw new IllegalArgumentException("TeamColor given must be either red or blue.");
        }
    }
}
