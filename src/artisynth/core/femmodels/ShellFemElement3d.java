package artisynth.core.femmodels;

public abstract class ShellFemElement3d extends FemElement3d {

   public double myShellThickness = 1;
   
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
}
