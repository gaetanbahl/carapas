package artisynth.core.femmodels;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;

public class TriMembraneElement extends FemElement3d {

   private static IntegrationPoint3d[] myDefaultIntegrationPoints;
   private static IntegrationPoint3d myWarpingPoint;
   private static FemElementRenderer myRenderer;
   
   private static double[] myNodeCoords = new double[] {
     0, 0, 0,
     1, 0, 0,
     0, 1, 0
   };
   
   static int[] myEdgeIdxs = new int[] 
      {
         2,   0, 1,
         2,   0, 2,
         2,   1, 2,
         2,   1, 0,
         2,   2, 0,
         2,   2, 1,
      };

   static int[] myFaceIdxs = new int[] 
      {
         3,   0, 2, 1,
         3,   0, 1, 2,
      };
   
   private static double[] myIntegrationCoords;
   static {
      double a = 1.0/6.0;
      double b = 2.0/3.0;
      double w = 1.0/3.0;
      myIntegrationCoords = new double[]
         {
          a, a, 0, w,
          b, a, 0, w,
          a, b, 0, w
         };
   }
   
   private static double[] myNodalExtrapolationMatrix = null;
   
   public TriMembraneElement() {
      myNodes = new FemNode3d[3];
   }
   
   public TriMembraneElement(FemNode3d n0, FemNode3d n1, FemNode3d n2) {
      myNodes = new FemNode3d[]{n0, n1, n2};
   }
   
   public TriMembraneElement (FemNode3d[] nodes) {
      if (nodes.length != 3) {
         throw new IllegalArgumentException ("nodes must have length 3");
      }
      myNodes = nodes.clone();
   }
   
   @Override
   public boolean integrationPointsMapToNodes() {
      return true;
   }
   
   @Override
   public int numIntegrationPoints() {
      return myIntegrationCoords.length/4;
   }

   @Override
   public double[] getIntegrationCoords() {
      return myIntegrationCoords;
   }

   @Override
   public double[] getNodalExtrapolationMatrix() {
      if (myNodalExtrapolationMatrix == null) {
         // map directly to node
         myNodalExtrapolationMatrix = new double[] {
           1,0,0,0,1,0,0,0,1
         };
      }
      return myNodalExtrapolationMatrix;
   }

   @Override
   public double getN(int i, Vector3d coords) {
      double s1 = coords.x;
      double s2 = coords.y;
      
      // on mid-wedge plane (r=0)
      switch(i) {
         case 0: return (1-s1-s2);
         case 1: return s1;
         case 2: return s2;
      }
      return 0;
   }

   @Override
   public void getdNds(Vector3d dNds, int i, Vector3d coords) {
      // sum of top and bottom of wedge (r=-1,1)
      double s1 = coords.x;
      double s2 = coords.y;
      switch(i) {
         case 0: dNds.set(-1, -1, (1-s1-s2));
         case 1: dNds.set(1, 0, s1);
         case 2: dNds.set(0, 1, s2);
      }
   }

   @Override
   public IntegrationPoint3d[] getIntegrationPoints() {
     if (myDefaultIntegrationPoints == null) {
        myDefaultIntegrationPoints = createIntegrationPoints();
     }
     return myDefaultIntegrationPoints;
   }

   @Override
   public IntegrationPoint3d getWarpingPoint() {
      if (myWarpingPoint == null) {
         myWarpingPoint = IntegrationPoint3d.create(this, 1.0/3, 1.0/3, 0, 1);
      }
      return myWarpingPoint;
   }

   @Override
   public int[] getEdgeIndices() {
      return myEdgeIdxs;
   }

   @Override
   public int[] getFaceIndices() {
      return myFaceIdxs;
   }

   @Override
   public boolean coordsAreInside(Vector3d coords) {
      double s1 = coords.x;
      double s2 = coords.y;
      double  r = coords.z;
      
      return (s1 >= 0 && s1 <= 1 && s2 >= 0 && s2 <= 1 && r >= -1e-16 && r <= 1e-16 &&
         (s1+s2) >= 0 && (s1+s2) <= 1 );
   }

   @Override
   public double[] getNodeCoords() {
      return myNodeCoords;
   }

   @Override
   public void renderWidget(Renderer renderer, double size, RenderProps props) {
      if (myRenderer == null) {
         myRenderer= new FemElementRenderer (this);
      }
      myRenderer.renderWidget (renderer, this, size, props);
   }

   @Override
   public void render(Renderer renderer, RenderProps rprops, int flags) {
      if (myRenderer == null) {
         myRenderer= new FemElementRenderer (this);
      }
      myRenderer.render (renderer, this, rprops);
   }
   
   @Override
   public boolean isInside(Point3d pnt) {
      
      Point3d p0 = myNodes[0].getPosition();
      Point3d p1 = myNodes[1].getPosition();
      Point3d p2 = myNodes[2].getPosition();
      
      Vector3d v1 = new Vector3d();
      Vector3d v2 = new Vector3d();
      Vector3d vs = new Vector3d();
      
      Vector3d n = new Vector3d();
      v1.sub(p1, p0);
      v2.sub(p2, p0);
      vs.sub(pnt, p0);
      n.cross(v1, v2);
      n.scale(1.0/Math.sqrt(n.norm()));  // rescale so ||n|| approx ||v1||||v2||
      
      double a = v1.dot(v1);
      double b = v1.dot(v2);
      double c = b;
      double d = v2.dot(v2);
      double r1 = v1.dot(vs);
      double r2 = v2.dot(vs);
      
      double det = a*d-b*c;
      
      Vector3d coords = new Vector3d();
      coords.x = (d*r1-b*r2)/det;
      coords.y = (-c*r1+a*r2)/det;
      
      // magnitude of projection as fraction of length of normal
      coords.z = vs.dot(n)/n.dot(n);
      
      // point in triangle
      return coordsAreInside(coords);
   }

}
