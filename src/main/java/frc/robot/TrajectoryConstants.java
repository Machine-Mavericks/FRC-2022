package frc.robot;

/** Contains pre-defined paths for path following */
public class TrajectoryConstants {
    
    /**
     * Inner class containing TestPath points
     */
    public static class TestPath{
        public static final double[][] points = {{0.0,0.0},
                                                {2.0,0.0},
                                                {4.0,0.0}};
        public static final double startAngle = 0.0; 
        public static final double endAngle = 0.0;
        public static final double startVelocity = 0.0;
        public static final double endVelocity = 0.0;
        public static final double endRobotAngle = 0.0; 
        public static final boolean revserse = false;
        public static final boolean rotatePath = false;   // NOT tested. yet  Do not use for now Feb 19 2022 KN
        
    }
    
} // end class TrajectoryConstants