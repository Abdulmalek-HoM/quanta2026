package DecodeAuto;

public class TagConfiguration {
    public enum RandomizationPattern {
        GREEN_PURPLE_PURPLE, // Left / ID 1 or 4
        PURPLE_GREEN_PURPLE, // Center / ID 2 or 5
        PURPLE_PURPLE_GREEN, // Right / ID 3 or 6
        UNKNOWN
    }

    // Shooting Goals (Backdrop)
    public static final int ID_BLUE_SHOOTING_GOAL_CENTER = 2; // Example
    public static final int ID_RED_SHOOTING_GOAL_CENTER = 5;  // Example

    // Localization Aid Tags (e.g. Wall)
    public static final int[] LOCALIZATION_TAGS = {7, 8, 9, 10};

    // Helper to determine pattern from ID (assuming standard 1-3 Blue, 4-6 Red)
    public static RandomizationPattern getPatternFromId(int id) {
        if (id == 1 || id == 4) return RandomizationPattern.GREEN_PURPLE_PURPLE;
        if (id == 2 || id == 5) return RandomizationPattern.PURPLE_GREEN_PURPLE;
        if (id == 3 || id == 6) return RandomizationPattern.PURPLE_PURPLE_GREEN;
        return RandomizationPattern.UNKNOWN;
    }
}
