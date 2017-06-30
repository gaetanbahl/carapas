package maspack.geometry;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

/**
 * Object that can be used to query distance and direction from a point
 */
public interface DistanceQueryable {
   
   /**
    * Returns the distance of this object to a supplied point
    * @param pnt point from which to determine the distance
    * @return distance
    */
   public double getDistance(Point3d pnt);
   
   /**
    * Determines the distance of this object to a supplied point, and
    * the distance normal, which points in the direction of decreasing
    * distance
    * 
    * @param normal (output) computed distance normal, ignored if null
    * @param pnt point from which to determine the distance
    * @return distance
    */
   public double getDistanceAndNormal(Vector3d normal, Point3d pnt);

}
