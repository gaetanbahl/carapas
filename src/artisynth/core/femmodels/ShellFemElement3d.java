package artisynth.core.femmodels;

import maspack.matrix.Matrix3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;

public abstract class ShellFemElement3d extends FemElement3d {
   
   protected double myShellThickness = 1; 
   
   //public abstract static IntegrationShellPoint3d[] createIntegrationPoints(
   //  FemShellElement3d ele, double[] cdata);
   
   @Override
   public abstract ShellIntegrationPoint3d[] getIntegrationPoints();
   
   @Override
   public abstract ShellIntegrationPoint3d getWarpingPoint();
   
   @Override
   public abstract ShellIntegrationData3d[] getIntegrationData();
   
   @Override
   public abstract ShellIntegrationData3d getWarpingData();
   
   public double computeVolume (boolean isRest) {
      Vector3d[] nodePos = new Vector3d[myNodes.length];
      for (int i = 0; i < myNodes.length; i++) {
         if (isRest) {
            nodePos[i] = myNodes[i].getRestPosition();
         }
         else {
            nodePos[i] = myNodes[i].getPosition();
         }
      }

      double vol = 0;

      // For each integration point...
      ShellIntegrationPoint3d[] iPts = getIntegrationPoints ();
      for (int i = 0; i < iPts.length; i++) {
         ShellIntegrationPoint3d iPt = iPts[i];
         iPt.computeJacobian(this);
         Matrix3d J = iPt.getJ();
         vol += J.determinant () * iPt.myWeight;
      }

      return vol;
   }
   
   @Override
   public double computeVolumes () {
      double vol = computeVolume (/* isRest= */false);
      myVolumes[0] = vol;
      return vol;
   }

   @Override
   public double computeRestVolumes () {
      double vol = computeVolume (/* isRest= */true);
      myRestVolumes[0] = vol;
      return vol;
   }
   
   public double getShellThickness() {
      return myShellThickness;
   }
}
