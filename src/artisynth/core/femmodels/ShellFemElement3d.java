package artisynth.core.femmodels;

import artisynth.core.femmodels.ShellIntegrationPoint3d.NODE_POS;
import maspack.matrix.Matrix3d;
import maspack.util.InternalErrorException;

/**
 * Base class for a shell element. Compared to traditional elements, 
 * shell elements are thin elements that can better model surfaces.
 * Examples include water surfaces, clothing, and aluminium sheet.
 * 
 * The shell logic is mainly found in ShellIntegrationPoint3d,
 * ShellFemModel3d, ShellNodeNeighbor, and FemUtilties.
 */
public abstract class ShellFemElement3d extends FemElement3d {
   
   protected double myShellThickness = 0.01; 
   
   @Override
   public abstract ShellIntegrationPoint3d[] getIntegrationPoints();
   
   @Override
   public abstract ShellIntegrationPoint3d getWarpingPoint();
   
   @Override
   public abstract ShellIntegrationData3d[] getIntegrationData();
   
   @Override
   public abstract ShellIntegrationData3d getWarpingData();
   
   @Override
   public double computeVolumes () {
      return _computeVolume (/* isRest= */false);
   }

   @Override
   public double computeRestVolumes () {
      return _computeVolume (/* isRest= */true);
   }
   
   public double _computeVolume (boolean isRest) {
      double vol = 0;

      // For each integration point...
      ShellIntegrationPoint3d[] iPts = getIntegrationPoints ();
      for (int i = 0; i < iPts.length; i++) {
         ShellIntegrationPoint3d iPt = iPts[i];
         if (isRest) {
            iPt.computeJacobian(NODE_POS.REST);
         }
         else {
            iPt.computeJacobian(NODE_POS.CURRENT);
         }
         Matrix3d J = iPt.getJ();
         vol += J.determinant () * iPt.myWeight;
      }

      return vol;
   }
   
   public double getShellThickness() {
      return myShellThickness;
   }
   
   public void setShellThickness(double newThickness) {
      myShellThickness = newThickness;
      
      // Update static dependencies that depend on knowing the shell thickness.
      
      updateDirector0 ();
      updateCoContraVectors ();
      updateRestVolumeAndMass ();
   }
   
   /**
    * Update the rest director of each node.
    * 
    * This should be called whenever node.myAdjElements is updated or 
    * shell thickness is modified, both which the rest director depends on.
    */
   public void updateDirector0() {
      for (FemNode3d n : myNodes) {
         ((ShellFemNode3d) n).updateDirector0 ();
      }
   }
   
   /**
    * Update the covariant and contravariant base vectors of each 
    * integration point. 
    * 
    * This should be invoked at the beginning of each timestep and whenever
    * the node rest directors are updated.
    */
   public void updateCoContraVectors() {
      for (IntegrationPoint3d iPt : getIntegrationPoints()) {
         ((ShellIntegrationPoint3d)iPt).updateCoContraVectors();
      }
   }
   
   @Override
   protected ShellIntegrationPoint3d[] createIntegrationPoints (
      double[] integCoords) {
      int numi = integCoords.length/4;
      ShellIntegrationPoint3d[] pnts = new ShellIntegrationPoint3d[numi];
      if (integCoords.length != 4*numi) {
         throw new InternalErrorException (
            "Coordinate data length is "+integCoords.length+","
            + " expecting "+4*numi);
      }
      for (int k=0; k<numi; k++) {
         pnts[k] = ShellIntegrationPoint3d.create (
            this, integCoords[k*4], integCoords[k*4+1], integCoords[k*4+2],
            integCoords[k*4+3]);
         pnts[k].setNumber (k);
      }
      return pnts;
   }
}
