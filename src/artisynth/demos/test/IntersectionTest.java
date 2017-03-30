package artisynth.demos.test;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.workspace.RootModel;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.RobustPreds;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;

public class IntersectionTest extends RootModel {
   
   @Override
   public void build(String[] args) throws IOException {
      super.build(args);
      
      PolygonalMesh mesh = new PolygonalMesh();
      Vertex3d[] v = new Vertex3d[9*2+8];
      Face[] f = new Face[8*6];
      int idx = 0;
      int fidx = 0;
      
      // plane with central vertex
      double c = 64;
      v[idx++] = mesh.addVertex(-c,-c,-c);
      v[idx++] = mesh.addVertex(-c,-c, 0);
      v[idx++] = mesh.addVertex(-c,-c, c);
      v[idx++] = mesh.addVertex(-c, 0,-c);
      v[idx++] = mesh.addVertex(-c, 0, 0);
      v[idx++] = mesh.addVertex(-c, 0, c);
      v[idx++] = mesh.addVertex(-c, c,-c);
      v[idx++] = mesh.addVertex(-c, c, 0);
      v[idx++] = mesh.addVertex(-c, c, c);
      f[fidx++] = mesh.addFace(v[0], v[1], v[4]);
      f[fidx++] = mesh.addFace(v[1], v[2], v[4]);
      f[fidx++] = mesh.addFace(v[2], v[5], v[4]);
      f[fidx++] = mesh.addFace(v[5], v[8], v[4]);
      f[fidx++] = mesh.addFace(v[8], v[7], v[4]);
      f[fidx++] = mesh.addFace(v[7], v[6], v[4]);
      f[fidx++] = mesh.addFace(v[6], v[3], v[4]);
      f[fidx++] = mesh.addFace(v[3], v[0], v[4]);
      
      // face for intersection
      PolygonalMesh edgeMesh = new PolygonalMesh();
      edgeMesh.addVertex(-2*c, 0, 0);
      edgeMesh.addVertex( 0, 0, 0);
      edgeMesh.addVertex( -c, 0, 2*c);
      edgeMesh.addFace(new int[]{1,2,0}); // first half-edge goes 0->1
      HalfEdge edge = edgeMesh.getFace(0).firstHalfEdge();
         
      Point3d p = new Point3d();
      int isects = 0;
      for (Face face : mesh.getFaces()) {
         if (RobustPreds.intersectEdgeFace(edge, face, p, false)) {
            ++isects;
            System.out.println("   intersects face " + face.getIndex());
         }
      }
      System.out.println("Testing edge-face intersection");
      System.out.println("    intersections: " + isects);
      if ((isects % 2) != 1) {
         // fail("vertex tangent test failed!");
         System.err.println("Intersection test failed! Should count as odd");
      }
      
      FixedMeshBody fm = new FixedMeshBody(mesh);
      addRenderable(fm);
      RenderProps.setFaceStyle(fm, FaceStyle.FRONT_AND_BACK);
      RenderProps.setFaceColor(fm, Color.ORANGE);
      RenderProps.setBackColor(fm, Color.BLUE);
      
      FixedMeshBody fm2 = new FixedMeshBody(edgeMesh);
      addRenderable(fm2);
      RenderProps.setFaceStyle(fm2, FaceStyle.FRONT_AND_BACK);
      RenderProps.setFaceColor(fm2, Color.ORANGE);
      RenderProps.setBackColor(fm2, Color.BLUE);
   }
   

}
