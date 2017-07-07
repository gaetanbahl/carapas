package artisynth.core.femmodels;

import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.util.InternalErrorException;

/* Implementation of FEShellQuad4G8::FEShellQuad4G8() */
public class ShellQuadElement extends ShellFemElement3d {

   /*** Variables and static blocks declarations ****/

   protected static double[] myNodeCoords = new double[] {
      0, 0, 0,
      1, 0, 0,
      1, 1, 0,
      0, 1, 0 
   };

   protected static double[] myDefaultIntegrationCoords;
   public static final double[] INTEGRATION_COORDS_GAUSS_8;
   static {
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

   /*
    * Integration Points are basically the integration Coordinates as individual
    * objects:
    */

   protected ShellIntegrationPoint3d[] myIntegrationPoints = null;
   protected static ShellIntegrationPoint3d[] myDefaultIntegrationPoints;

   private static ShellIntegrationPoint3d myWarpingPoint = null;
   protected ShellIntegrationData3d myWarpingData = null;

   /* Is there a 1-1 mapping between nodes and integration points? */
   private boolean myIPointsMapToNodes = true;

   /*
    * Matrix that assumes integration points of this element is 1-1 with the
    * local node coordinate positions. E.g. i-th integration point corresponds
    * to i-th node position. Matrix contains shape function values that is used
    * for computing things like node stress.
    * 
    * n x m matrix where: n = number of node coordinates (scaled) m = number of
    * integration points [n][m] = shapeFunc(m, n-th scaled node coordinate)
    */
   private double[] myNodalExtrapolationMatrix = null;

   /*
    * 6 edges in total (6 rows). Each row is for a particular edge. Column #0 =
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
    * 4 faces in total (4 rows). Each row is for a particular face. Column #0 =
    * Number nodes comprising the face. Column #1 = First node index of face.
    * Column #2 = Second node index of face. Column #3 = Third node index of
    * face.
    */
   static int[] myFaceIdxs = new int[] { 
      4, 0, 1, 2, 3
   };

   protected static FemElementRenderer myRenderer;

   protected ShellIntegrationData3d[] myIntegrationData;

   
   /*** End of variables and static blocks declarations ****/

   public ShellQuadElement () {
      myNodes = new FemNode3d[myNodeCoords.length/3];
   }

   /**
    * Creates a new TetraHedral element from four nodes. The first three nodes
    * should define a clockwise arrangement about a particular face.
    */
   public ShellQuadElement(
      ShellFemNode3d p0, ShellFemNode3d p1, ShellFemNode3d p2, ShellFemNode3d p3) {
      this ();
      setNodes (p0, p1, p2, p3);
      
      p0.myAdjElements.add (this);
      p1.myAdjElements.add (this);
      p2.myAdjElements.add (this);
      p3.myAdjElements.add (this);
   }

   /**
    * Sets the nodes of a TetraHedral element. The first three nodes should
    * define a clockwise arrangement about a particular face.
    */
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

   @Override
   protected ShellIntegrationPoint3d[] createIntegrationPoints (
      double[] integCoords) {

      return createIntegrationPoints (this, integCoords);
   }

   //@Override
   public static ShellIntegrationPoint3d[] createIntegrationPoints(ShellFemElement3d ele, double[] cdata) {
      int numi = cdata.length/4;
      ShellIntegrationPoint3d[] pnts = new ShellIntegrationPoint3d[numi];
      if (cdata.length != 4*numi) {
         throw new InternalErrorException (
            "Coordinate data length is "+cdata.length+", expecting "+4*numi);
      }
      for (int k=0; k<numi; k++) {
         pnts[k] = ShellIntegrationPoint3d.create (
            ele, cdata[k*4], cdata[k*4+1], cdata[k*4+2], cdata[k*4+3]);
         pnts[k].setNumber (k);
      }
      return pnts;
   }
   
   public void setIntegrationPoints (
      ShellIntegrationPoint3d[] ipnts, double[] nodalExtrapMat) {

      myIPointsMapToNodes = mapIPointsToNodes (ipnts, nodalExtrapMat, myNodes);
      setIntegrationPoints (ipnts, nodalExtrapMat, myIPointsMapToNodes);
   }

   /**
    * Enforce each i-th integration points to correspond to i-th node in terms
    * of the nodal extrapolation matrix.
    * 
    * @param ipnts
    * @param nodalExtrapMat
    * @param nodes
    * @return
    */
   public static boolean mapIPointsToNodes (
      IntegrationPoint3d[] ipnts, double[] nodalExtrapMat, FemNode3d[] nodes) {

      int nNodes = nodes.length;
      int nIPnts = ipnts.length;

      if (nIPnts < nNodes) {
         return false;
      }

      double dist, minDist;
      Point3d pos = new Point3d ();
      int closest;
      // For each node...
      for (int i = 0; i < nNodes; i++) {
         minDist = Double.MAX_VALUE;
         closest = i;
         // For each integPt ahead of it...
         for (int j = i; j < nIPnts; j++) {
            // pos = sigma( integPt.wt[n] * nodes[n].pos )
            ipnts[i].computePosition (pos, nodes);
            // Is this integPt closest to node?
            dist = pos.distance (nodes[i].getPosition ());
            if (dist < minDist) {
               closest = j;
               minDist = dist;
            }
         }

         // If node isn't closest to its respective integPt...
         if (closest != i) {
            // Swap closest and respective integPt shape func value in the
            // nodalExtraMat.
            IntegrationPoint3d tmp;
            double tmpd;
            tmp = ipnts[closest];
            ipnts[closest] = ipnts[i];
            ipnts[i] = tmp;
            for (int j = 0; j < nNodes; j++) {
               tmpd = nodalExtrapMat[j * ipnts.length + closest];
               nodalExtrapMat[j * ipnts.length + closest] =
                  nodalExtrapMat[j * ipnts.length + i];
               nodalExtrapMat[j * ipnts.length + i] = tmpd;
            }
         }
      }

      return true;
   }

   public void setIntegrationPoints (
      ShellIntegrationPoint3d[] ipnts, double[] nodalExtrapMat, boolean mapToNodes) {

      myIntegrationPoints = ipnts;
      myIPointsMapToNodes = mapToNodes;
      myNodalExtrapolationMatrix = nodalExtrapMat;
      myIntegrationData = null;
      clearState (); // trigger re-creating integration data
   }

   public void setNodalExtrapolationMatrix (double[] nem) {
      myNodalExtrapolationMatrix = nem;
   }

   @Override
   public double[] getNodalExtrapolationMatrix () {
      if (myNodalExtrapolationMatrix == null) {
         // Scale initial node coordinates by sqrt(3)
         Vector3d[] ncoords = getScaledNodeCoords (Math.sqrt (3), null);
         //
         myNodalExtrapolationMatrix =
            createNodalExtrapolationMatrix (
               ncoords, numIntegrationPoints (), new ShellQuadElement ());

         // For now, just use integration point values at corresponding nodes
         myNodalExtrapolationMatrix =
            new double[] { 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                           1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, };
      }
      return myNodalExtrapolationMatrix;
   }

   @Override
   public ShellIntegrationPoint3d getWarpingPoint () {
      if (myWarpingPoint == null) {
         myWarpingPoint =
            ShellIntegrationPoint3d.create (this, 1 / 3.0, 1 / 3.0, 0, 1);
      }
      return myWarpingPoint;
   }
   
   @Override
   public ShellIntegrationData3d getWarpingData() {
      ShellIntegrationData3d wdata = myWarpingData;
      if (wdata == null) {
         int numPnts = getIntegrationPoints().length;
         if (numPnts == 1) {
            // then integration and warping points/data are the same
            wdata = getIntegrationData()[0];
         }
         else {
            wdata = new ShellIntegrationData3d();
            wdata.computeRestJacobian (getWarpingPoint(), this);
         }
         myWarpingData = wdata;
      }
      return wdata;
   }

   @Override
   public boolean integrationPointsMapToNodes () {
      return myIPointsMapToNodes;
   }

   
   
   /*** End of Methods pertaining to integration coordinates and points **/

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
      dNds.z = 0;
   }

   /*** Methods pertaining to volume **/
   
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
      // TODO Auto-generated method stub
      return false;
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
            idata[i].computeRestJacobian (ipnts[i], this);
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
