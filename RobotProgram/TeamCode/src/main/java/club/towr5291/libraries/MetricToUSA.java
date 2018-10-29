package club.towr5291.libraries;

public final class MetricToUSA {
    public static class Length {
        static final float mmPerin = 25.4f;
        static final float inPerm = 39.3701f;
        static final float cmPerin = 2.54f;
        static final float inPerkilo = 39370.08f;

        public static float getMmPerin() {
            return mmPerin;
        }

        public static float getInPerm() {
            return inPerm;
        }

        public static float getCmPerin() {
            return cmPerin;
        }

        public static float getInPerkilo() {
            return inPerkilo;
        }
    }
}
