package frc.robot.Util;

public class MathUtil {
    public static int sign(double num){
        if(num > 0){
            return(1);
        }
        else if(num < 0){
            return(-1);
        }
        else{
            return(0);
        }
    }
}
