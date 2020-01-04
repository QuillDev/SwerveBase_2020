package SKCommon.Utils;

public final class qMath {
    private qMath() {
        throw new AssertionError("utility class");
    }

    //MathConstant
    public static final double twoPI = Math.PI * 2.0;
    /**
     * 
     * @param a a value to use in pythag 
     * @param b b value to use in pythag
     * @return return the c value of pythag theorum
     */
    public static double pythag(double a, double b) {
        return Math.sqrt( ( a * a) + (b * b) );
    }

    /**
     * @param metres
     * @return the amount of metres inputted in feet
     */
    public static double metresToFeet(double metres){
        return metres * 3.28084;
    }

    /**
     * @param metres
     * @return the amount of metres inputted in feet
     */
    public static double feetToMetres(double feet){
        return feet/3.28084;
    }

    /**
     * @param feet
     * @return the amount of inches inputted as feet
     */
    public static double inchesToFeet(double inches){
        return inches/12;
    }

    /**
     * @param inches
     * @return the amount of feet inputted to inces
     */
    public static double feetToInches(double feet){
        return feet*12;
    }
}