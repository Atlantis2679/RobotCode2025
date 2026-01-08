package frc.robot.allcommands;

public final class ObjectDetectionConstants {
    public final class ObjectHeights {
        private static double Algea_Height = 0.405;
        public static double getHeight(int objectClassID){
            switch (objectClassID) {
                case 1 -> {return Algea_Height;}
            }
            return -1;
        }
    }
}
