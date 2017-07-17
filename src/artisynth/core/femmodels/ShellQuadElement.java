package artisynth.core.femmodels;

import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;

/** 
 * Implementation of a square shell element with 4 shell nodes and 8 gauss 
 * points. Implementation is based on FEBio FEShellQuad4G8::FEShellQuad4G8()
 */
public class ShellQuadElement extends ShellFemElement3d {

   /*** Variables and static blocks declarations ****/

   /* Expected arrangement of the initial node positions */
   protected static double[] myNodeCoords = new double[] {
      0, 0, 0,
      1, 0, 0,
      1, 1, 0,
      0, 1, 0 
   };

   protected static double[] myDefaultIntegrationCoords;
   public static final double[] INTEGRATION_COORDS_GAUSS_8;
   static {
      // FEBio: FEShellQuad4G8::FEShellQuad4G8()
      double a = 1 / Math.sqrt (3);
      double w = 1.0;
      INTEGRATION_COORDS_GAUSS_8 = new double[] { 
         -a, -a, -a, w,
         +a, -a, -a, w,
         
         +a, +a, -a, w,
         -a, +a, -a, w,
         
         -a, -a, +a, w,
         +a, -a, +a, w,
         
         +a, +a, +a, w,
         -a, +a, +a, w };
      myDefaultIntegrationCoords = INTEGRATION_COORDS_GAUSS_8;
   }

   /* Integration Points are basically the integration coordinates above
    * as individual objects: */

   protected ShellIntegrationPoint3d[] myIntegrationPoints = null;
   
   /* Mainly used to copy integration point data into (e.g. computed jacobian)
    * and transfer to other methods to use like computeVolume() */
   protected ShellIntegrationData3d[] myIntegrationData;

   /*
    * 4 edges in total. Each row is for a particular edge. Column #0 =
    * Number nodes comprising the edge. Column #1 = First node index of edge.
    * Column #2 = Second node index of edge.
    */
   static int[] myEdgeIdxs = new int[] { 
      2, 0, 1,
      2, 1, 2, 
      2, 2, 3, 
      2, 3, 0,
   };

   /*
    * 1 face in total. Each row is for a particular face. Column #0 =
    * Number nodes comprising the face. Columns #1-3: Node indices comprising 
    * the face.
    */
   static int[] myFaceIdxs = new int[] { 
      4, 0, 1, 2, 3
   };

   protected static FemElementRenderer myRenderer;


   
   /*** End of variables and static blocks declarations ****/

   public ShellQuadElement () {
      myNodes = new FemNode3d[myNodeCoords.length/3];
   }

   /**
    * Creates a new square shell element with four shell nodes. The node 
    * positions must abide to this.myNodeCoords, relatively speaking.
    */
   public ShellQuadElement(
      ShellFemNode3d p0, ShellFemNode3d p1, ShellFemNode3d p2, ShellFemNode3d p3, double thickness) {
      this ();
      setNodes (p0, p1, p2, p3);
      
      p0.myAdjElements.add (this);
      p1.myAdjElements.add (this);
      p2.myAdjElements.add (this);
      p3.myAdjElements.add (this);
      
      myShellThickness = thickness;
   }

   public void setNodes (
      ShellFemNode3d p0, ShellFemNode3d p1, ShellFemNode3d p2, ShellFemNode3d p3) {
      myNodes[0] = p0;
      myNodes[1] = p1;
      myNodes[2] = p2;
      myNodes[3] = p3;
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
         // Wrap each Gauss coordinate (r, s, t, w) in an Integration
         // Point object.
         myIntegrationPoints =
            createIntegrationPoints (myDefaultIntegrationCoords);
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

   
   

   /**
    * Compute shape function of particular node.
    * 
    * FEBio Porting Notes: Equivalent to computing H in
    * FEShellQuad4G8::FEShellQuad4G8()
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
      double rSign = 0;
      double sSign = 0;

      switch (n) {
         case 0:
            rSign = -1;
            sSign = -1;
            break;
         case 1:
            rSign = +1;
            sSign = -1;
            break;
         case 2:
            rSign = +1;
            sSign = +1;
            break;
         case 3:
            rSign = -1;
            sSign = +1;
            break;
         default: {
            throw new IllegalArgumentException (
               "M function index must be in range [0," + (numNodes () - 1)
               + "]");
         }
      }

      return 0.25 * (1 + rSign * r) * (1 + sSign * s);
   }

   /**
    * Compute 1st derivative of shape function of particular node.
    * 
    * FEBio Porting Notes: Equivalent to computing Hr and Hs in
    * FEShellQuad4G8::FEShellQuad4G8()
    */
   @Override
   public void getdNds (Vector3d dNds, int n, Vector3d rst) {
      switch (n) {
         case 0:
            dNds.x = -0.25 * (1 - rst.x);
            dNds.y = -0.25 * (1 - rst.y);
            break;
         case 1:
            dNds.x = +0.25 * (1 - rst.x);
            dNds.y = -0.25 * (1 + rst.y);
            break;
         case 2:
            dNds.x = +0.25 * (1 + rst.x);
            dNds.y = +0.25 * (1 + rst.y);
            break;
         case 3:
            dNds.x = -0.25 * (1 + rst.x);
            dNds.y = +0.25 * (1 - rst.y);
            break;
         default: {
            throw new IllegalArgumentException (
               "M function index must be in range [0," + (numNodes () - 1)
               + "]");
         }
      }
      dNds.z = 0;     // unused
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
