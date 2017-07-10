package artisynth.core.femmodels;

import artisynth.core.femmodels.ShellIntegrationPoint3d.NODE_POS;
import maspack.matrix.Matrix3d;
import maspack.matrix.Vector3d;

/**
 * Mainly used to copy integration point data into (e.g. computed jacobian)
 * and transfer to other methods to use like computeVolume() 
 * 
 * @author Danny Huang (dah208@mail.usask.ca). Feel free to contact me for help.
 */
public class ShellIntegrationData3d extends IntegrationData3d {

   public ShellIntegrationData3d() {
      super();
   }
   
   public static double computeRestJacobian (
      Matrix3d invJ0, ShellIntegrationPoint3d iPt) {

      Matrix3d J0 = new Matrix3d();
      
      Vector3d[] gco = iPt.getCoBaseVectors(NODE_POS.REST);
      
      J0.m00 = gco[0].x; J0.m01 = gco[1].x; J0.m02 = gco[2].x;
      J0.m10 = gco[0].y; J0.m11 = gco[1].y; J0.m12 = gco[2].y;
      J0.m20 = gco[0].z; J0.m21 = gco[1].z; J0.m22 = gco[2].z;
      
      return invJ0.fastInvert (J0);
   }
   
   public void computeRestJacobian (ShellIntegrationPoint3d iPt) {
      myDetJ0 = computeRestJacobian (myInvJ0, iPt);
      if (myDetJ0 <= 0) {
         System.out.println ("Warning: inverted rest element, det="+myDetJ0);
      }
   }
   
   @Deprecated
   public static double computeRestJacobian (
      Matrix3d invJ0, Vector3d[] GNs, FemNode3d[] nodes) { 
      throw new RuntimeException("Error :: Invoked obsolete and dead method.");
   }
   
   @Override
   @Deprecated
   public void computeRestJacobian (Vector3d[] GNs, FemNode3d[] nodes) {
      throw new RuntimeException("Error :: Invoked obsolete and dead method.");
   }
}
