package club.towr5291.libraries;

public class LibraryMotorType {

    public double GEARRATIO;
    public double COUNTSPERROTATION;
    public double PULSEPERROTATION;

    private double dblENCODER_CPR_REV40SPUR = 1120;
    private double dblENCODER_CPR_REV20SPUR = 560;
    private double dblENCODER_CPR_REV20ORBITAL = 560;
    private double dblENCODER_CPR_ANDY20ORBITAL = 537.6;
    private double dblENCODER_CPR_ANDY20SPUR = 560;
    private double dblENCODER_CPR_ANDY40SPUR = 1120;
    private double dblENCODER_CPR_ANDY60SPUR = 1680;
    private double dblENCODER_CPR_ANDY3_7ORBITAL = 0;

    private double dblENCODER_PPR_REV40SPUR = 112;
    private double dblENCODER_PPR_REV20SPUR = 56;
    private double dblENCODER_PPR_REV20ORBITAL = 56;
    private double dblENCODER_PPR_ANDY20ORBITAL = 134.4;
    private double dblENCODER_PPR_ANDY20SPUR = 28;
    private double dblENCODER_PPR_ANDY40SPUR = 28;
    private double dblENCODER_PPR_ANDY60SPUR = 28;
    private double dblENCODER_PPR_ANDY3_7ORBITAL = 0;

    private double dblGearRatioREV40SPUR = 40;
    private double dblGearRatioREV20SPUR = 20;
    private double dblGearRatioREV20ORBITAL = 20;
    private double dblGearRatioANDY20ORBITAL = 19.2;
    private double dblGearRatioANDY20SPUR = 20;
    private double dblGearRatioANDY40SPUR = 40;
    private double dblGearRatioANDY60SPUR = 60;
    private double dblGearRatioANDY3_7ORBITAL = 3.7;

    public enum MotorTypes {
        REV40SPUR ("REV40SPUR"),
        REV20SPUR ("REV20SPUR"),
        REV20ORBIT ("REV20ORBIT"),
        ANDY20SPUR ("ANDY20SPUR"),
        ANDY40SPUR ("ANDY40SPUR"),
        ANDY60SPUR ("ANDY60SPUR"),
        ANDY20ORBIT ("ANDY20ORBIT"),
        ANDY3_7ORBIT ("ANDY3_7ORBIT");

        public String name;

        MotorTypes(String name){
            this.name = name;
        }

        public String toString() {
            return this.name;
        }

        public boolean isAndyMark(){
            if(this.name.equals("ANDY20SPUR") || this.name.equals("ANDY40SPUR") || this.name.equals("ANDY60SPUR") || this.name.equals("ANDY20ORBIT") || this.name.equals("ANDY3_7ORBIT")) {
                return true;
            } else {
                return false;
            }

        }
    }

    public LibraryMotorType(){
        //Nothing In here yet :)
    }

    public void loadData(MotorTypes motorTypes){
        switch(motorTypes){
            case REV40SPUR:
                this.COUNTSPERROTATION = this.dblENCODER_CPR_REV40SPUR;
                this.PULSEPERROTATION = this.dblENCODER_PPR_REV40SPUR;
                this.GEARRATIO = this.dblGearRatioREV40SPUR;
                break;

            case REV20SPUR:
                this.COUNTSPERROTATION = this.dblENCODER_CPR_REV20SPUR;
                this.PULSEPERROTATION = this.dblENCODER_PPR_REV20SPUR;
                this.GEARRATIO = this.dblGearRatioREV20SPUR;
                break;

            case REV20ORBIT:
                this.COUNTSPERROTATION = this.dblENCODER_CPR_REV20ORBITAL;
                this.PULSEPERROTATION = this.dblENCODER_PPR_REV20ORBITAL;
                this.GEARRATIO = this.dblGearRatioREV20ORBITAL;
                break;

            case ANDY20SPUR:
                this.COUNTSPERROTATION = this.dblENCODER_CPR_ANDY20SPUR;
                this.PULSEPERROTATION = this.dblENCODER_PPR_ANDY20SPUR;
                this.GEARRATIO = this.dblGearRatioANDY20SPUR;
                break;

            case ANDY40SPUR:
                this.COUNTSPERROTATION = this.dblENCODER_CPR_ANDY40SPUR;
                this.PULSEPERROTATION = this.dblENCODER_PPR_ANDY40SPUR;
                this.GEARRATIO = this.dblGearRatioANDY40SPUR;
                break;

            case ANDY60SPUR:
                this.COUNTSPERROTATION = this.dblENCODER_CPR_ANDY60SPUR;
                this.PULSEPERROTATION = this.dblENCODER_PPR_ANDY60SPUR;
                this.GEARRATIO = this.dblGearRatioANDY60SPUR;
                break;

            case ANDY20ORBIT:
                this.COUNTSPERROTATION = this.dblENCODER_CPR_ANDY20ORBITAL;
                this.PULSEPERROTATION = this.dblENCODER_PPR_ANDY20ORBITAL;
                this.GEARRATIO = this.dblGearRatioANDY20ORBITAL;
                break;

            case ANDY3_7ORBIT:
                this.COUNTSPERROTATION = this.dblENCODER_CPR_ANDY3_7ORBITAL;
                this.PULSEPERROTATION = this.dblENCODER_PPR_ANDY3_7ORBITAL;
                this.GEARRATIO = this.dblGearRatioANDY3_7ORBITAL;
                break;
        }
    }

    public double getGEARRATIO() {
        return GEARRATIO;
    }

    public double getCOUNTSPERROTATION() {
        return COUNTSPERROTATION;
    }

    public double getPULSEPERROTATION() {
        return PULSEPERROTATION;
    }
}