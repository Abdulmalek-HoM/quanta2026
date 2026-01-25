package DecodeAuto;

public class TagConfiguration {
    /**
     * Randomization patterns as defined by the OBELISK on the DECODE field.
     * Each pattern determines the MOTIF colors for the 9 ramp indices.
     * 
     * GPP (ID 21): GATE positions get Green at indices 1,4,7 and Purple at others
     * PGP (ID 22): GATE positions get Green at indices 2,5,8 and Purple at others
     * PPG (ID 23): GATE positions get Green at indices 3,6,9 and Purple at others
     */
    public enum RandomizationPattern {
        GREEN_PURPLE_PURPLE, // GPP - Obelisk ID 21
        PURPLE_GREEN_PURPLE, // PGP - Obelisk ID 22
        PURPLE_PURPLE_GREEN, // PPG - Obelisk ID 23
        UNKNOWN
    }

    // Randomization Tags on the OBELISK (center of field)
    public static final int ID_PATTERN_GPP = 21; // Green-Purple-Purple
    public static final int ID_PATTERN_PGP = 22; // Purple-Green-Purple
    public static final int ID_PATTERN_PPG = 23; // Purple-Purple-Green

    // Shooting Goals
    public static final int ID_BLUE_SHOOTING_GOAL = 20; // Blue alliance shooting goal
    public static final int ID_RED_SHOOTING_GOAL = 24; // Red alliance shooting goal

    // All randomization tag IDs for detection
    public static final int[] RANDOMIZATION_TAGS = { 21, 22, 23 };

    // All shooting goal tag IDs
    public static final int[] SHOOTING_GOAL_TAGS = { 20, 24 };

    /**
     * Determine the randomization pattern from the detected AprilTag ID.
     * 
     * @param id The detected AprilTag ID from the obelisk
     * @return The corresponding RandomizationPattern, or UNKNOWN if not a valid
     *         pattern tag
     */
    public static RandomizationPattern getPatternFromId(int id) {
        if (id == ID_PATTERN_GPP)
            return RandomizationPattern.GREEN_PURPLE_PURPLE;
        if (id == ID_PATTERN_PGP)
            return RandomizationPattern.PURPLE_GREEN_PURPLE;
        if (id == ID_PATTERN_PPG)
            return RandomizationPattern.PURPLE_PURPLE_GREEN;
        return RandomizationPattern.UNKNOWN;
    }

    /**
     * Get the shooting goal tag ID for a given alliance.
     * 
     * @param isRedAlliance true if red alliance, false if blue alliance
     * @return The AprilTag ID for the alliance's shooting goal
     */
    public static int getShootingGoalId(boolean isRedAlliance) {
        return isRedAlliance ? ID_RED_SHOOTING_GOAL : ID_BLUE_SHOOTING_GOAL;
    }

    /**
     * Check if a tag ID corresponds to a randomization pattern tag.
     * 
     * @param id The AprilTag ID to check
     * @return true if the ID is a randomization tag (21, 22, or 23)
     */
    public static boolean isRandomizationTag(int id) {
        return id == ID_PATTERN_GPP || id == ID_PATTERN_PGP || id == ID_PATTERN_PPG;
    }

    /**
     * Check if a tag ID corresponds to a shooting goal tag.
     * 
     * @param id The AprilTag ID to check
     * @return true if the ID is a shooting goal tag (20 or 24)
     */
    public static boolean isShootingGoalTag(int id) {
        return id == ID_BLUE_SHOOTING_GOAL || id == ID_RED_SHOOTING_GOAL;
    }
}
