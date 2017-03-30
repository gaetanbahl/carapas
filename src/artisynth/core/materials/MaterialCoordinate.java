package artisynth.core.materials;

import maspack.matrix.Point3d;

/**
 * 3D coordinate within an object, used for evaluating material-based
 * fields
 * @author Antonio
 *
 */
public interface MaterialCoordinate {

   /**
    * Initial position of the material point (i.e. the Lagrangian material coordinates)
    * @return 3D 'rest' position
    */
   public Point3d getRestPosition();
   
   /**
    * Initial position of the material point (i.e. the Lagrangian material coordinates)
    * @param rpos 3D 'rest' position
    */
   public void getRestPosition(Point3d rpos);
   
   /**
    * Absolute 3D position of the material point (i.e. the Eulerian coordinates)
    * @return pos 3D world position
    */
   public Point3d getWorldPosition();
   
   /**
    * Absolute 3D position of the material point (i.e. the Eulerian coordinates)
    * @param pos 3D world position
    */
   public void getWorldPosition(Point3d pos);
   
   /**
    * An index referring to a particular subvolume, such as a finite element
    * or integration region.  This is to aid in caching of information. 
    * @return subvolume index, or -1 if none exists
    */
   public int getSubvolumeIndex();
   
   /**
    * An index referring to a particular point within the subvolume ({@link #getSubvolumeIndex()}),
    * such as an integration point index.  This is to aid in caching of information.
    * @return subvolume index, or -1 if none exists
    */
   public int getCoordinateIndex();
   
}
