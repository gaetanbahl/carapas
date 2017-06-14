package artisynth.demos.mech;

import java.awt.Color;
import java.io.File;
import java.io.IOException;

import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.workspace.RootModel;
import maspack.fileutil.FileCacher;
import maspack.fileutil.uri.URIx;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.io.WavefrontReader;
import maspack.matrix.AxisAngle;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;

public class RigidBodySpin extends RootModel {
   
   private static PolygonalMesh loadStanfordBunny() {
      // read Standford bunny directly
      String bunnyURL = "http://graphics.stanford.edu/~mdfisher/Data/Meshes/bunny.obj";
      // bunny
      File bunnyFile = new File("tmp/data/stanford_bunny.obj");
      
      PolygonalMesh bunny = null;
      try {
         if (!bunnyFile.exists ()) {
            bunnyFile.getParentFile().mkdirs ();
            
            // read file directly from remote
            FileCacher cacher = new FileCacher();
            cacher.initialize();
            cacher.cache (new URIx(bunnyURL), bunnyFile);  
            cacher.release ();
         }

         WavefrontReader reader = new WavefrontReader(bunnyFile);

         bunny = new PolygonalMesh();
         reader.readMesh(bunny);
         //bunny.computeVertexNormals();
         // normalize bunny
         double r = bunny.computeRadius();
         Vector3d c = new Vector3d();
         bunny.computeCentroid(c);
         c.negate();
         bunny.scale(1.0 / r);
         c.z -= 0.5;
         bunny.transform(new RigidTransform3d(c, new AxisAngle(
            1, 0, 0, Math.PI / 2)));
         reader.close();
         
      } catch (IOException e1) {
         e1.printStackTrace();
         System.out.println("Unable to load stanford bunny... requires internet connection");
         bunny = null;
      }

      return bunny;
   }
   
   @Override
   public void build(String[] args) throws IOException {
      super.build(args);
      
      MechModel mech1 = new MechModel("mech1");
      mech1.setIntegrator(Integrator.Trapezoidal);
      addModel(mech1);
      RigidBody cube1 = new RigidBody("bunny1");
      cube1.setMesh(loadStanfordBunny());
      cube1.centerPoseOnCenterOfMass();
      RenderProps.setAlpha(cube1, 0.5);
      RenderProps.setFaceColor(cube1, Color.BLUE);
      cube1.setPose(0, 0.1, 0, 0, 0, 0);
      mech1.add(cube1);
      mech1.setGravity(0,0,0);
      cube1.setVelocity(0, 0, 0, 0.2, 0.2, 0);
      
      MechModel mech2 = new MechModel("mech2");
      mech2.setIntegrator(Integrator.Trapezoidal2);
      addModel(mech2);
      RigidBody cube2 = new RigidBody("bunny2");
      cube2.setMesh(loadStanfordBunny());
      cube2.centerPoseOnCenterOfMass();
      RenderProps.setAlpha(cube2, 0.5);
      RenderProps.setFaceColor(cube2, Color.RED);
      cube2.setPose(0, 0.1, 0, 0, 0, 0);
      mech2.add(cube2);
      mech2.setGravity(0,0,0);
      cube2.setVelocity(0, 0, 0, 0.2, 0.2, 0);
      
   }

}
