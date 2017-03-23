package artisynth.core.mfreemodels;

import artisynth.core.materials.MaterialCoordinate;
import maspack.matrix.Point3d;

/**
 * Material coordinate that follows a FEM integration point 
 */
public class MFreeIntegrationCoordinate implements MaterialCoordinate {

   MFreeElement3d elem;
   MFreeIntegrationPoint3d ipnt;
   
   public MFreeIntegrationCoordinate() {
      elem = null;
      ipnt = null;
   }
   
   public MFreeIntegrationCoordinate(MFreeElement3d elem, MFreeIntegrationPoint3d ipnt) {
      set(elem, ipnt);
   }
   
   /**
    * Sets the parameters of the material coordinate
    * @param elem
    * @param ipnt
    */
   public void set(MFreeElement3d elem, MFreeIntegrationPoint3d ipnt) {
      this.elem = elem;
      this.ipnt = ipnt;
   }
   
   /**
    * @return the underlying element
    */
   public MFreeElement3d getElement() {
      return elem;
   }
   
   /**
    * @return the underlying integration point
    */
   public MFreeIntegrationPoint3d getIntegrationPoint() {
      return ipnt;
   }
   
   @Override
   public Point3d getRestPosition() {
      return ipnt.getRestPosition();
   }
   
   @Override
   public void getRestPosition(Point3d pos) {
      pos.set(ipnt.getRestPosition());
   }
   
   @Override
   public Point3d getWorldPosition() {
      Point3d pos = new Point3d();
      ipnt.getPosition();
      return pos;
   }
   
   @Override 
   public void getWorldPosition(Point3d pos) {
      pos.set(ipnt.getPosition());
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
