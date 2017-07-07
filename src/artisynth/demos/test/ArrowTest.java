package artisynth.demos.test;

import java.awt.Color;

import maspack.render.*;
import maspack.render.Renderer.LineStyle;
import artisynth.core.mechmodels.*;
import artisynth.core.workspace.RootModel;

public class ArrowTest extends RootModel {

   public void build (String[] args) {

      MechModel mech = new MechModel("mech");
      addModel (mech);

      Particle p0 = new Particle (0, 0, 0, 0);
      Particle p1 = new Particle (0, -0.1, 0, 0.05);
      mech.addParticle (p0);
      mech.addParticle (p1);

      RenderProps.setSphericalPoints (p0, 0.005, Color.WHITE);

      AxialSpring spr = new AxialSpring (1, 0, 0);

      RenderProps.setLineRadius (spr, 0.0025);
      RenderProps.setLineStyle (spr, LineStyle.SOLID_ARROW);
      RenderProps.setLineColor (spr, Color.CYAN);
      mech.attachAxialSpring (p0, p1, spr);
   }
}
