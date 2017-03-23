package artisynth.models.fem;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.materials.AnisotropicLinearMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.ScaledFemMaterial;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.workspace.RootModel;
import maspack.matrix.Point3d;
import maspack.render.RenderProps;

public class ScaledMaterialTest extends RootModel {
   
   @Override
   public void build(String[] args) throws IOException {
      super.build(args);
      
      MechModel mech = new MechModel("mech");
      addModel(mech);
      
      double E = 100000;
      double nu = 0.3;
      
      FemModel3d femLinear = createFem("Linear");
      femLinear.setMaterial(new LinearMaterial(E, nu, true));
      RenderProps.setFaceColor(femLinear, Color.BLUE);
      RenderProps.setAlpha(femLinear, 0.8);
      mech.addModel(femLinear);
      femLinear.useNewStressAndStiffness = true;
      
      FemModel3d femScaled = createFem("Scaled");
      double scale = 10.0;
      femScaled.setMaterial(new ScaledFemMaterial(new LinearMaterial(E/scale, nu, true), scale));
      RenderProps.setFaceColor(femScaled, Color.RED);
      RenderProps.setAlpha(femScaled, 0.8);
      mech.addModel(femScaled);
      femScaled.useNewStressAndStiffness = true;
      
      FemModel3d femAnisotropic = createFem("Anisotropic");
      femAnisotropic.setMaterial(new AnisotropicLinearMaterial(E, nu, true));
      RenderProps.setFaceColor(femAnisotropic, Color.GREEN);
      RenderProps.setAlpha(femAnisotropic, 0.8);
      mech.addModel(femAnisotropic);
      femScaled.useNewStressAndStiffness = true;
      
   }
   
   private FemModel3d createFem(String name) {
      FemModel3d beam = FemFactory.createHexGrid(null, 0.1, 0.02, 0.02, 10, 2, 2);
      beam.setName(name);
      beam.setDensity(1000);
      
      //      double[][] init = {
      //                                {-0.05, -0.01, -0.01},
      //                                {-7.243071577045672E-4, -0.008977933025711345, -0.013331598913772696},
      //                                {0.04879670806752494, -0.008568800203973598, -0.01812361502845692},
      //                                {-0.05, 0.01, -0.01},
      //                                {-0.0010021801647414745, 0.011045342618434057, -0.01325623509526181},
      //                                {0.04864264121482746, 0.011435620439662729, -0.018152606970148397},
      //                                {-0.05, -0.01, 0.01},
      //                                {8.128446567077208E-4, -0.00892728976483528, 0.006683204518046685},
      //                                {0.05070882533076658, -0.008536943589232002, 0.0017650852684492648},
      //                                {-0.05, 0.01, 0.01},
      //                                {5.20787440040072E-4, 0.011041855450175981, 0.006605788036819612},
      //                                {0.050561854897302236, 0.011454830640207692, 0.001771859265543581},
      //                             };
      
      double eps = 1e-10;
      int idx = 0;
      for (FemNode3d node : beam.getNodes()) {
         Point3d pos = node.getPosition();
         if (pos.x < -0.05+eps) {
            node.setDynamic(false);
         }
         //node.setPosition(init[idx][0], init[idx][1], init[idx][2]);
         ++idx;
      }
      
      beam.setSurfaceRendering(SurfaceRender.Shaded);
      
      return beam;
   }
   
   @Override
   public StepAdjustment advance(double t0, double t1, int flags) {
            
      //      FemModel3d linear = (FemModel3d)(((MechModel)models().get("mech")).models().get("Linear"));
      //      System.out.println("Linear stiffness:");
      //      System.out.println( linear.getStiffnessMatrix().toString("%5f").replaceAll("-0.00000", "0.00000") );
      //      
      //      FemModel3d linearish = (FemModel3d)(((MechModel)models().get("mech")).models().get("Linearish"));
      //      System.out.println("Linearish stiffness:");
      //      System.out.println( linearish.getStiffnessMatrix().toString("%5f").replaceAll("-0.00000", "0.00000") );
      //      
      //      System.out.println("      double[][] init = {");
      //      for (FemNode3d node : linear.getNodes()) {
      //         Point3d pos = node.getPosition();
      //         System.out.println("         {" + pos.x + ", " + pos.y + ", " + pos.z + "},");
      //      }
      //      System.out.println("      };");
      
      return super.advance(t0, t1, flags);
   }
   
   
}
