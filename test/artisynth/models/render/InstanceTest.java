package artisynth.models.render;

import java.io.File;
import java.io.IOException;

import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.RootModel;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderInstances;
import maspack.render.RenderObject;
import maspack.render.RenderObjectFactory;
import maspack.render.RenderProps;
import maspack.render.Renderable;
import maspack.render.RenderableBase;
import maspack.render.Renderer;
import maspack.render.Renderer.Shading;
import maspack.render.GL.GL3.GL3Object;
import maspack.render.GL.GL3.GL3Viewer;

public class InstanceTest extends RootModel {

   private static class InstanceRenderable extends RenderableBase {

      boolean init = false;
      RenderInstances rinst = null;
      RenderObject robj = null;

      
      File[] shaders = new File[]{ArtisynthPath.getSrcRelativeFile(GL3Object.class, "shaders/test_vertex.glsl"),
                                  ArtisynthPath.getSrcRelativeFile(GL3Object.class, "shaders/test_fragment.glsl")};
      
      private void initialize() {

         PolygonalMesh mesh = MeshFactory.createOctahedralSphere(0.5, 3);
         robj = RenderObjectFactory.createFromMesh(mesh, false, true);

         rinst = new RenderInstances();
         rinst.addScale(0.5);
         rinst.addInstance(new float[]{2,2,2});
         rinst.addInstance(new float[]{0,0,2});
         rinst.addInstance(new float[]{0,2,0});
         rinst.addInstance(new float[]{2,0,0});
         
         AffineTransform3d aff = new AffineTransform3d();
         aff.setRotation(new AxisAngle(1, 1, 1, Math.toRadians(30)));
         aff.applyScaling(2, 3, 1);
         aff.setTranslation(1, 1, 1);
         rinst.addInstance(aff);

         init = true;
      }

      @Override
      public RenderProps createRenderProps() {
         return new RenderProps();
      }

      @Override
      public void render(Renderer renderer, int flags) {
         if (!init) {
            initialize();
         }

         if (renderer instanceof GL3Viewer) {
            GL3Viewer viewer = (GL3Viewer)renderer;
            RenderProps rprops = getRenderProps();
            viewer.setShading(rprops.getShading());
            viewer.setFaceColoring(rprops, false);
            // viewer.drawTriangles(robj, 0);
            
            // viewer.setShaderOverride(shaders, shaders);
            viewer.drawTriangles(robj, 0, rinst);
            viewer.setShaderOverride(null, null);
         }

      }
      
      @Override
      public void updateBounds(Vector3d pmin, Vector3d pmax) {
         final Point3d p1 = new Point3d(-5,-5,-5);
         final Point3d p2 = new Point3d( 5, 5, 5);
         p1.updateBounds(pmin, pmax);
         p2.updateBounds(pmin, pmax);
      }
   }

   @Override
   public void build(String[] args) throws IOException {
      super.build(args);

      Renderable r = new InstanceRenderable();

      r.setRenderProps(r.createRenderProps());
      
      RenderProps.setVisible(r, true);
      RenderProps.setShading(r, Shading.FLAT);
      
      addRenderable(r);

   }

}
