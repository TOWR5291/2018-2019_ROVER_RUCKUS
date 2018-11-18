package club.towr5291.libraries;

public final class MetricToUSA {
    public static class Length {
        static final float mmPerin = 25.4f;
        static final float inPerm = 39.3701f;
        static final float cmPerin = 2.54f;
        static final float inPerkilo = 39370.08f;

        public static float MM_To_In(float input) {
            return input * mmPerin;
        }

        public static float In_To_M(float input) {
            return input / inPerm;
        }

        public static float Cm_To_In(float input) {
            return input / cmPerin;
        }
        public static float In_To_Cm(float input) {
            return input * cmPerin;
        }

        public static float getInPerkilo() {
            return inPerkilo;
        }
    }
}
