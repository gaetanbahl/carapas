package artisynth.core.femmodels;

import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.util.InternalErrorException;

/** 
 * Implementation of a triangle shell element with 3 shell nodes and 9 gauss 
 * points. Implementation is based on FEBio FEShellTri3G9::FEShellTri3G9()
 *  
 * @author Danny Huang (dah208@mail.usask.ca). Feel free to contact me for help.
 */
public class ShellTriElement extends ShellFemElement3d {

   /*** Variables and static blocks declarations ****/
   
   /* Expected arrangement of the initial node positions */
   protected static double[] myNodeCoords = new double[] {
      0, 0, 0,
      1, 0, 0,
      1, 1, 0,
   };

   protected static double[] myDefaultIntegrationCoords;
   public static final double[] INTEGRATION_COORDS_GAUSS_9;
   static {
      /* FEBio: FEShellTri3G9::FEShellTri3G9() */
      double a = 1/6.0;
      double b = 2/3.0;
      double w1 = 5/9.0;
      double w2 = 8/9.0;
      INTEGRATION_COORDS_GAUSS_9 = new double[] { 
         a, a, -b, a*w1,
         b, a, -b, a*w1,
         a, b, -b, a*w1, 
         
         a, a, 0, a*w2, 
         b, a, 0, a*w2, 
         a, b, 0, a*w2,
         
         a, a, b, a*w1, 
         b, a, b, a*w1, 
         a, b, b, a*w1
      };
      myDefaultIntegrationCoords = INTEGRATION_COORDS_GAUSS_9;
   }

   /*
    * Integration Points are basically the integration Coordinates as individual
    * objects: */

   protected ShellIntegrationPoint3d[] myIntegrationPoints = null;
   protected static ShellIntegrationPoint3d[] myDefaultIntegrationPoints;

   /* Mainly used to transfer integration point data (e.g. computed jacobian)
    * for other methods to use like computeVolume() */
   protected ShellIntegrationData3d[] myIntegrationData;
   
   /*
    * 3 edges in total. Each row is for a particular edge. Column #0 =
    * Number nodes comprising the edge. Column #1 = First node index of edge.
    * Column #2 = Second node index of edge.
    */
   static int[] myEdgeIdxs = new int[] { 
      2, 0, 1,
      2, 0, 2,
      2, 1, 2
   };

   /*
    * 1 face in total. Each row is for a particular face. Column #0 =
    * Number nodes comprising the face. Column #1 = First node index of face.
    * Column #2 = Second node index of face. Column #3 = Third node index of
    * face.
    */
   static int[] myFaceIdxs = new int[] { 
      3, 0, 1, 2
   };

   protected static FemElementRenderer myRenderer;

   
   
   /*** End of variables and static blocks declarations ****/

   public ShellTriElement () {
      myNodes = new ShellFemNode3d[myNodeCoords.length/3];
   }

   /**
    * Creates a new triangle element from three nodes. The node 
    * positions must abide to this.myNodeCoords, relatively speaking.
    */
   public ShellTriElement (ShellFemNode3d p0, ShellFemNode3d p1, ShellFemNode3d p2) {
      this ();
      setNodes (p0, p1, p2);
      
      p0.myAdjElements.add (this);
      p1.myAdjElements.add (this);
      p2.myAdjElements.add (this);
      
      myShellThickness = 0.01;
   }

   public void setNodes (ShellFemNode3d p0, ShellFemNode3d p1, ShellFemNode3d p2) {
      myNodes[0] = p0;
      myNodes[1] = p1;
      myNodes[2] = p2;
      invalidateRestData ();
   }

   /*** Methods pertaining to integration coordinates and points **/

   @Override
   public double[] getIntegrationCoords () {
      return myDefaultIntegrationCoords;
   }

   @Override
   public int numIntegrationPoints () {
      if (myIntegrationPoints != null) {
         return myIntegrationPoints.length;
      }
      // Divide by 4 because b/c each integration coordinate contains
      // (r, s, t, w) set.
      return myDefaultIntegrationCoords.length / 4;
   }

   @Override
   public ShellIntegrationPoint3d[] getIntegrationPoints () {
      if (myIntegrationPoints == null) {
         if (myDefaultIntegrationPoints == null) {
            // Wrap each Gauss coordinate (r, s, t, w) in an Integration
            // Point object.
            myDefaultIntegrationPoints =
               createIntegrationPoints (myDefaultIntegrationCoords);
         }
         myIntegrationPoints = myDefaultIntegrationPoints;
      }

      return myIntegrationPoints;
   }


   
   
   public void setIntegrationPoints (
      ShellIntegrationPoint3d[] ipnts, double[] nodalExtrapMat) {
      throw new RuntimeException("Unimplemented");
   }

   public static boolean mapIPointsToNodes (
      IntegrationPoint3d[] ipnts, double[] nodalExtrapMat, FemNode3d[] nodes) {
      throw new RuntimeException("Unimplemented");
   }

   public void setIntegrationPoints (
      ShellIntegrationPoint3d[] ipnts, double[] nodalExtrapMat, 
      boolean mapToNodes) {
      throw new RuntimeException("Unimplemented");
   }

   public void setNodalExtrapolationMatrix (double[] nem) {
      throw new RuntimeException("Unimplemented");
   }

   @Override
   public double[] getNodalExtrapolationMatrix () {
      throw new RuntimeException("Unimplemented");
   }

   @Override
   public ShellIntegrationPoint3d getWarpingPoint () {
      throw new RuntimeException("Unimplemented");
   }
   
   @Override
   public ShellIntegrationData3d getWarpingData() {
      throw new RuntimeException("Unimplemented");
   }

   @Override
   public boolean integrationPointsMapToNodes () {
      return false;
   }
   
   

   
   /*** End of Methods pertaining to integration coordinates and points **/

   /**
    * Compute shape function of particular node.
    * 
    * FEBio: FEShellTri3G9::FEShellTri3G9()
    * 
    * @param n
    * Node index
    * 
    * @param rst
    * Integration point (i.e. Gauss point)
    */
   @Override
   public double getN (int n, Vector3d rst) {
      double r = rst.x;
      double s = rst.y;

      switch (n) {
         case 0:
            return 1-r-s;
         case 1:
            return r;
         case 2:
            return s;
         default: {
            throw new IllegalArgumentException (
               "M function index must be in range [0," + (numNodes () - 1)
               + "]");
         }
      }
   }

   /**
    * Compute 1st derivative of shape function of particular node.
    * 
    * FEBio: FEShellTri3G9::FEShellTri3G9()
    */
   @Override
   public void getdNds (Vector3d dNds, int n, Vector3d rst) {
      switch (n) {
         case 0:
            dNds.x = -1;
            dNds.y = -1;
            break;
         case 1:
            dNds.x = 1;
            dNds.y = 0;
            break;
         case 2:
            dNds.x = 0;
            dNds.y = 1;
            break;
         default: {
            throw new IllegalArgumentException (
               "M function index must be in range [0," + (numNodes () - 1)
               + "]");
         }
      }
      dNds.z = 0;               // unused
   }


   @Override
   public int[] getEdgeIndices () {
      return myEdgeIdxs;
   }

   @Override
   public int[] getFaceIndices () {
      return myFaceIdxs;
   }

   @Override
   public boolean coordsAreInside (Vector3d coords) {
      throw new RuntimeException("Unimplemented");
   }

   @Override
   public double[] getNodeCoords () {
      return myNodeCoords;
   }

   @Override
   public void render (Renderer renderer, RenderProps props, int flags) {
      if (myRenderer == null) {
         myRenderer = new FemElementRenderer (this);
      }
      myRenderer.render (renderer, this, props);
   }

   @Override
   public void renderWidget (
      Renderer renderer, double size, RenderProps props) {
      if (myRenderer == null) {
         myRenderer = new FemElementRenderer (this);
      }
      myRenderer.renderWidget (renderer, this, size, props);
   }
   
   
   
   /*** Functions that ComputeNonLinearStressAndStiffness() depends on ***/
    
   @Override
   public ShellIntegrationData3d[] getIntegrationData() {
      ShellIntegrationData3d[] idata = doGetIntegrationData();
      if (!myIntegrationDataValid) {
         // compute rest Jacobians and such
         ShellIntegrationPoint3d[] ipnts = getIntegrationPoints();
         for (int i=0; i<idata.length; i++) {
            idata[i].computeRestJacobian (ipnts[i]);
         }
         myIntegrationDataValid = true;
      }
      return idata;
   }
   
   @Override
   protected ShellIntegrationData3d[] doGetIntegrationData() {
      ShellIntegrationData3d[] idata = myIntegrationData;
      if (idata == null) {
         int numPnts = getIntegrationPoints().length;
         idata = new ShellIntegrationData3d[numPnts];
         for (int i=0; i<numPnts; i++) {
            idata[i] = new ShellIntegrationData3d();
         }
         myIntegrationData = idata;
      }
      return idata;
   }
}
