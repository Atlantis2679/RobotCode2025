package frc.robot.allcommands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

class ObjectDetectionTemp {
    public PhotonCamera camera;

    private static TargetCorner getObjectCenter(List<TargetCorner> corners){
        TargetCorner home = corners.get(0), opp=null;
        for (int i = 1; i<4; ++i) {
            if (corners.get(i).x!=home.x && corners.get(i).y!=home.y) {
                opp = corners.get(i);
                break;
            }
        }
        return new TargetCorner((home.x+opp.x)/2, (home.y+opp.y)/2);
    }
    public static Double getPixelHeight(PhotonTrackedTarget target){
        TargetCorner home = target.getDetectedCorners().get(0);
        for (int i = 1; i<4; ++i){
            if (target.getDetectedCorners().get(i).y!=home.y){
                return Math.abs(home.y-target.getDetectedCorners().get(i).y);
            }
        }
        return null;
    }

    public Vector<N3> getObjDistance(PhotonTrackedTarget target){
        Matrix<N3, N3> cameraMatrix = camera.getCameraMatrix().get();
        TargetCorner center = getObjectCenter(target.getDetectedCorners());

        double Z = cameraMatrix.get(1,1 ) //fy
                    *ObjectDetectionConstants.ObjectHeights.getHeight(target.getDetectedObjectClassID()) //The pixel height of the object
                    /getPixelHeight(target); //The pixel height of the object

        
    }
    
    
}