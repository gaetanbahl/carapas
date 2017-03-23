package artisynth.models.fem;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.materials.AnisotropicLinearMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.LinearishMaterial;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.workspace.RootModel;
import maspack.matrix.SparseBlockMatrix;
import maspack.render.RenderProps;

public class LinearTetTest extends RootModel {
   
   @Override
   public void build(String[] args) throws IOException {
      super.build(args);
      
      MechModel mech = new MechModel("mech");
      addModel(mech);
      
      double Youngs = 30000;
      double Poisson = 0.25;
      
      FemModel3d beam1 = createFEM();
      beam1.setName("Linear");
      beam1.setMaterial(new LinearMaterial(Youngs, Poisson, true));
      RenderProps.setFaceColor(beam1, Color.GREEN);
      RenderProps.setAlpha(beam1, 0.8);
      beam1.setSurfaceRendering(SurfaceRender.Shaded);
      mech.addModel(beam1);
      
      FemModel3d beam2 = createFEM();
      beam2.setName("Anisoropic");
      beam2.setMaterial(new AnisotropicLinearMaterial(Youngs, Poisson, true));
      RenderProps.setAlpha(beam2, 0.8);
      RenderProps.setFaceColor(beam2, Color.BLUE);
      beam2.setSurfaceRendering(SurfaceRender.Shaded);
      mech.addModel(beam2);
      
      FemModel3d beam3 = createFEM();
      beam3.setName("Linearish");
      beam3.setMaterial(new LinearishMaterial(Youngs, Poisson, true));
      RenderProps.setAlpha(beam3, 0.8);
      RenderProps.setFaceColor(beam3, Color.RED);
      beam3.setSurfaceRendering(SurfaceRender.Shaded);
      mech.addModel(beam3);
      
      // addMonitor(new StiffnessMonitor(beam1));
      addMonitor(new StiffnessMonitor(beam2));
      addMonitor(new StiffnessMonitor(beam3));
      
   }
   
   private static class StiffnessMonitor extends MonitorBase {
      FemModel3d fem;
      public StiffnessMonitor(FemModel3d fem) {
         this.fem = fem;
      }
      
      @Override
      public void initialize(double t) {
         super.initialize(t);
         apply(t, t);
      }
      
      @Override
      public void apply(double t0, double t1) {
         SparseBlockMatrix K = fem.getStiffnessMatrix();
         System.out.println(fem.getName() + " stiffness:");
         String str = K.toString("%.5f");
         str = str.replaceAll("-0.00000", "0.00000");
         System.out.println(str);
         
         System.out.println("Forces:");
         for (FemNode3d node : fem.getNodes()) {
            System.out.println(node.getInternalForce().toString("%5f"));
         }
      }
      
   }
   
   private static FemModel3d createFEM() {
      //FemModel3d beam = FemFactory.createHexGrid(null, 0.10, 0.02, 0.02, 10, 2, 2);
      FemModel3d beam = new FemModel3d();
      beam.addNode(new FemNode3d(0,0,0));
      beam.addNode(new FemNode3d(1,0,0));
      beam.addNode(new FemNode3d(0,1,0));
      beam.addNode(new FemNode3d(0,0,1));
      beam.addNode(new FemNode3d(1,1,1));
      FemNode3d[] nodes = beam.getNodes().toArray(new FemNode3d[5]);
      beam.addElement(FemElement3d.createElement(new FemNode3d[]{nodes[0], nodes[1], nodes[2], nodes[3]}));
      beam.addElement(FemElement3d.createElement(new FemNode3d[]{nodes[3], nodes[2], nodes[4], nodes[1]}));
      
      beam.getNode(0).setDynamic(false);
      beam.getNode(3).setDynamic(false);
      
      beam.invalidateStressAndStiffness();
      beam.updateStressAndStiffness();
      
      return beam;
   }
   

}
