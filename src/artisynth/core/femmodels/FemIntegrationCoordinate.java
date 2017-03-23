package artisynth.core.femmodels;

import artisynth.core.materials.MaterialCoordinate;
import maspack.matrix.Point3d;

/**
 * Material coordinate that follows a FEM integration point 
 */
public class FemIntegrationCoordinate implements MaterialCoordinate {

   FemElement3d elem;
   IntegrationPoint3d ipnt;
   
   public FemIntegrationCoordinate() {
      elem = null;
      ipnt = null;
   }
   
   public FemIntegrationCoordinate(FemElement3d elem, IntegrationPoint3d ipnt) {
      set(elem, ipnt);
   }
   
   /**
    * Sets the parameters of the material coordinate
    * @param elem
    * @param ipnt
    */
   public void set(FemElement3d elem, IntegrationPoint3d ipnt) {
      this.elem = elem;
      this.ipnt = ipnt;
   }
   
   /**
    * @return the underlying element
    */
   public FemElement3d getElement() {
      return elem;
   }
   
   /**
    * @return the underlying integration point
    */
   public IntegrationPoint3d getIntegrationPoint() {
      return ipnt;
   }
   
   @Override
   public Point3d getRestPosition() {
      Point3d pos = new Point3d();
      ipnt.computeRestPosition(pos, elem);
      return pos;
   }
   
   @Override
   public void getRestPosition(Point3d pos) {
      ipnt.computeRestPosition(pos, elem);
   }
   
   @Override
   public Point3d getWorldPosition() {
      Point3d pos = new Point3d();
      ipnt.computePosition(pos, elem);
      return pos;
   }
   
   @Override 
   public void getWorldPosition(Point3d pos) {
      ipnt.computePosition(pos, elem);
   }
   
   /**
    * @return the finite element number
    */
   @Override
   public int getSubvolumeIndex() {
      return elem.getNumber();
   }
   
   /**
    * @return the integration point number within the finite element
    */
   @Override
   public int getCoordinateIndex() {
      return ipnt.getNumber();
   }

}
