package artisynth.demos.mech;

import java.io.IOException;

import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.workspace.RootModel;

public class SimpleRigidBody extends RootModel {
   
   @Override
   public void build(String[] args) throws IOException {
      super.build(args);
      
      MechModel mech = new MechModel("mech");
      addModel(mech);
      mech.setGravity(0,0,-9.8);

      RigidBody rb = RigidBody.createBox("box", 0.1, 0.1, 0.1, 1000);
      mech.addRigidBody(rb);
   }

}
