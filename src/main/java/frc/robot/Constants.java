package frc.robot;

public class Constants {
    public class DrivetrainConstants {
        public static final double kMaxSpeed = 5.41;
        public static final double kMaxAngularRate = kMaxSpeed * 39.37 / 20.75 * Math.PI;
    }
    public class KnuckleConstants {
        public static final int kMotorID = 0;
        public static final double kCurrentThreshold = 30;
        public static final double kHighSpeed = 1.0;
        public static final double kLowSpeed = 0.1;
    }
    public class ChuteConstants {
        public static final int kMotorID = 0;
        public static final double kCurrentThreshold = 30;
    }
    public class AlgaeScorerConstants{
        public static final int kMotorID = 0;
        public static final double kCurrentThreshold = 30;
        
    }
}
