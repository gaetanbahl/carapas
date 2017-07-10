package artisynth.core.femmodels;

import artisynth.core.femmodels.ShellIntegrationPoint3d.NODE_POS;
import maspack.matrix.Matrix3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.util.InternalErrorException;

public abstract class ShellFemElement3d extends FemElement3d {
   
   /* Default thickness. Stability problems can occur if raised too high. */
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
      double vol = _computeVolume (/* isRest= */false);
      myVolumes[0] = vol;
      return vol;
   }

   @Override
   public double computeRestVolumes () {
      double vol = _computeVolume (/* isRest= */true);
      myRestVolumes[0] = vol;
      return vol;
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
   
   @Override
   protected ShellIntegrationPoint3d[] createIntegrationPoints (
      double[] integCoords) {
      return _createIntegrationPoints (this, integCoords);
   }

   protected ShellIntegrationPoint3d[] _createIntegrationPoints(
      ShellFemElement3d ele, double[] cdata) {
      int numi = cdata.length/4;
      ShellIntegrationPoint3d[] pnts = new ShellIntegrationPoint3d[numi];
      if (cdata.length != 4*numi) {
         throw new InternalErrorException (
            "Coordinate data length is "+cdata.length+", expecting "+4*numi);
      }
      for (int k=0; k<numi; k++) {
         pnts[k] = ShellIntegrationPoint3d.create (
            ele, cdata[k*4], cdata[k*4+1], cdata[k*4+2], cdata[k*4+3]);
         pnts[k].setNumber (k);
      }
      return pnts;
   }
}
