package artisynth.models.fem;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.materials.FunctionMaterialField;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.SpatialScaledFemMaterial;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.workspace.RootModel;
import maspack.function.Function3x1Base;
import maspack.matrix.Point3d;
import maspack.render.RenderProps;

public class SpatiallyVaryingMaterial extends RootModel {
   
   @Override
   public void build(String[] args) throws IOException {
      super.build(args);
      
      MechModel mech = new MechModel("mech");
      addModel(mech);
      mech.setGravity(0, 0, 0);
      
      double E = 1000;
      double nu = 0.3;
      
      FemModel3d fem = createFem("Spatial");
      
      LinearMaterial lmat = new LinearMaterial(E, nu, true);
     
      class XFunc extends Function3x1Base {
         @Override
         public double eval(double x, double y, double z) {
            double w = (x+0.05)/0.1+1; // w : [1,2]
            double s = w*w*w*10;
            System.out.println("f(" + x +"," + y + "," + z + ") = " + s);
            return s;
         }         
      }
      FunctionMaterialField field = new FunctionMaterialField(new XFunc());
      SpatialScaledFemMaterial smat = new SpatialScaledFemMaterial(lmat, field);
      
      fem.setMaterial(smat);
      RenderProps.setFaceColor(fem, Color.BLUE);
      RenderProps.setAlpha(fem, 0.8);
      mech.addModel(fem);
      fem.useNewStressAndStiffness = true;
      
     
      
   }
   
   private FemModel3d createFem(String name) {
      FemModel3d beam = FemFactory.createHexGrid(null, 0.1, 0.02, 0.02, 20, 4, 4);
      beam.setName(name);
      beam.setDensity(1000);
     
      double eps = 1e-10;
      for (FemNode3d node : beam.getNodes()) {
         Point3d pos = node.getPosition();
         if (pos.x < -0.05+eps) {
            node.setDynamic(false);
         } else if (pos.x > 0.05-eps) {
            node.setDynamic(false);
            pos.x += 0.05;
         }
      }
      
      beam.setSurfaceRendering(SurfaceRender.Shaded);
      
      return beam;
   }
   
   
}
