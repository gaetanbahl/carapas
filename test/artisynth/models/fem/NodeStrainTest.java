package artisynth.models.fem;

import java.io.IOException;

import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.workspace.RootModel;
import maspack.matrix.AxisAngle;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;

public class NodeStrainTest extends RootModel {

   @Override
   public void build(String[] args) throws IOException {
      super.build(args);
      
      RigidTransform3d transform = new RigidTransform3d(Vector3d.ZERO, AxisAngle.ROT_X_90);
      
      FemModel3d fem1 = createFem();
      fem1.useNewStressAndStiffness = false;
      fem1.setComputeNodalStrain(true);
      fem1.setComputeNodalStress(true);
      fem1.setMaterial(new MooneyRivlinMaterial());
      
      FemModel3d fem2 = createFem();
      fem2.useNewStressAndStiffness = true;
      fem2.setComputeNodalStrain(true);
      fem2.setComputeNodalStress(true);
      
      // stress differently in x,y,z
      for (FemNode3d node : fem1.getNodes()) {
         node.getPosition().scale(1.1, 1.2, 1.3);
         Vector3d random = new Vector3d();
         random.setRandom(0.75, 1.25);
         node.getPosition().transform(transform);
         // node.getPosition().scale(random.x, random.y, random.z);
      }
      fem1.invalidateStressAndStiffness();
      fem1.updateStressAndStiffness();
      
      // stress differently in x,y,z
      for (int i=0; i<fem2.numNodes(); ++i) {
         fem2.getNode(i).setPosition(fem1.getNode(i).getPosition());
      }
      fem2.invalidateStressAndStiffness();
      fem2.updateStressAndStiffness();
      
      // print out stress and strains on nodes
      System.out.println("Fem 1:");
      for (FemNode3d node : fem1.getNodes()) {
         System.out.println("Node " + node.getNumber());
         System.out.println("Stress: ");
         System.out.println(node.getStress().toString("%.2f").replaceAll("-0.00", "0.00"));
         System.out.println("Strain: ");
         System.out.println(node.getStrain().toString("%.2f").replaceAll("-0.00", "0.00"));
      }
      
      // print out stress and strains on nodes
      System.out.println("Fem 2:");
      for (FemNode3d node : fem2.getNodes()) {
         System.out.println("Node " + node.getNumber());
         System.out.println("Stress: ");
         System.out.println(node.getStress().toString("%.2f").replaceAll("-0.00", "0.00"));
         System.out.println("Strain: ");
         System.out.println(node.getStrain().toString("%.2f").replaceAll("-0.00", "0.00"));
      }
   }
   
   private FemModel3d createFem() {
      FemModel3d fem = FemFactory.createHexGrid(null, 0.1, 0.1, 0.1, 1, 1, 1);
      fem.setMaterial(new LinearMaterial(30000, 0.25, true));
      fem.setDensity(1000);
      return fem;
   }
   
}
