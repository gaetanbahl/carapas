package artisynth.core.femmodels;

import artisynth.core.materials.SolidDeformation;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class ShellIntegrationPoint3d extends IntegrationPoint3d {

   protected enum NODE_POS { REST, CURRENT, RENDER };
   
   public ShellIntegrationPoint3d(
      int nnodes, int npvals, double s0, double s1, double s2, double w) {
      super(nnodes, npvals, s0, s1, s2, w);
   }
   
   public ShellIntegrationPoint3d(int nnodes) {
      super(nnodes);
   }
   
   /** 
    * Create an integration point for a given element at a specific set of
    * natural coordinates.
    *
    * @param elem element to create the integration point for
    * @param s0 first coordinate value
    * @param s1 second coordinate value
    * @param s2 third coordinate value
    * @param w weight 
    * @return new integration point
    */
   public static ShellIntegrationPoint3d create (ShellFemElement3d elem,
         double s0, double s1, double s2, double w) {

      int nnodes = elem.numNodes();
      int npvals = elem.numPressureVals();
      
      Vector3d coords = new Vector3d();
      Vector3d dNds = new Vector3d();
      VectorNd shapeWeights = new VectorNd(nnodes);
      VectorNd pressureWeights = new VectorNd(npvals);

      ShellIntegrationPoint3d pnt =
            new ShellIntegrationPoint3d (nnodes, npvals, s0, s1, s2, w);
      coords.set (s0, s1, s2);
      for (int i=0; i<nnodes; i++) {
         shapeWeights.set (i, elem.getN (i, coords));
         elem.getdNds (dNds, i, coords);
         pnt.setShapeGrad (i, dNds);
      }
      for (int i=0; i<npvals; i++) {
         pressureWeights.set (i, elem.getH (i, coords));
      }
      pnt.setShapeWeights (shapeWeights);
      pnt.setPressureWeights (pressureWeights);
      return pnt;
   }
   
   
   
   
   /*** Methods for computing covariant and contravariant bases vectors ***/
   
   public Vector3d[] getCoBaseVectors(ShellFemElement3d ele) {
      return _getCoBaseVectors(ele, NODE_POS.CURRENT);
   }
   
   public Vector3d[] getCoBaseVectors0(ShellFemElement3d ele) {
      return _getCoBaseVectors(ele, NODE_POS.REST);
   }
   
   /**
    * Compute the bases of this integration point in covector form.
    * 
    * FEBio: FESSIShellDomain::CoBaseVectors
    * 
    * @param ePosType
    * Type of node position to use.
    *   REST -> getLocalRestPosition()
    *   CURRENT -> getLocalPosition()
    *   RENDER -> myRenderCoords
    * 
    * @return
    * Three covector bases.
    */
   protected Vector3d[] _getCoBaseVectors(
      ShellFemElement3d ele, NODE_POS ePosType) {
      Vector3d[] g = new Vector3d[3];
      for (int i = 0; i < 3; i++) {
         g[i] = new Vector3d();
      }
      
      double t = getCoords().z;
      // DANNY TODO: account for m_dof
      
      //System.out.println ("----");
      
      // For each node...
      for (int n = 0; n < ele.getNodes().length; n++) {
         ShellFemNode3d node = (ShellFemNode3d) ele.getNodes()[n];
         Vector3d d = null;
         
         Point3d pos = null;
         if (ePosType == NODE_POS.REST) {
            pos = node.getLocalRestPosition();
            d = new Vector3d( node.myDirector0 );
         }
         else if (ePosType == NODE_POS.CURRENT) {
            pos = node.getLocalPosition();
            d = new Vector3d( node.myDirector0 );
            d.add( node.getDisplacement() );
            d.sub( node.getDir() );
         }
         else if (ePosType == NODE_POS.RENDER) {
            float[] rPos = node.myRenderCoords; 
            pos = new Point3d( rPos[0], rPos[1], rPos[2] );
            d = new Vector3d( node.myDirector0 );
         }
         
         if (n == 0) {
            System.out.println ("Node#0 direction: " + node.getDir());
            System.out.println ("Node#0 displacement: " + node.getDisplacement());
         }
            
         //d0.sub(ele.myNodes[n].myDofd);
         
         //Vector3d nodeNormal = getNodeNormal(node);
         //nodeNormal.normalize ();
         
         //nodeNormal.scale (d0.norm ());
         
         //d0.add(ele.myNodes[n].myDofu);
         //nodeNormal.sub (nodeNormal, d0);
         
         //d0.add(ele.myNodes[n].myDofu);
         //d0.sub (nodeNormal);
         //d0.absolute ();
         
         //d0.set (nodeNormal);
         
         //System.out.println ("Node #" + n + " normal: " + d0);
       
         //System.out.printf ("Node #%d director: %s\n", n, node.myDirector);
         //System.out.printf ("Surface normal: %s\n", nodeNormal);
 
         Vector3d g0Term = new Vector3d( d );
         g0Term.scale( -(1 - t)*0.5 );
         g0Term.add( pos );
         
         Vector3d g1Term = new Vector3d( g0Term );
         
         // dN
         g0Term.scale( getGNs()[n].x );
         g1Term.scale( getGNs()[n].y );
         
         Vector3d g2Term = new Vector3d( d );
         // N
         g2Term.scale( getShapeWeights().get(n) * 0.5 );
         
         g[0].add (g0Term);
         g[1].add (g1Term);
         g[2].add (g2Term);
      }
      
      return g;
   }

   public static Vector3d getNodeNormal(ShellFemNode3d node) {
      Vector3d normal = new Vector3d();
      for (ShellFemElement3d el : node.myAdjElements) {
         Vector3d elNormal = getElementNormal(el);
         normal.add (elNormal);
      }
      // Avg
      normal.scale (1.0/node.myAdjElements.size ());
      return normal;
   }
   
   public static Vector3d getElementNormal(FemElement3d el) {
      Point3d n0 = el.myNodes[0].getPosition ();
      Point3d n1 = el.myNodes[1].getPosition ();
      Point3d n2 = el.myNodes[2].getPosition ();
      Vector3d n1_0 = new Vector3d();
      n1_0.sub (n1, n0);
      Vector3d n2_0 = new Vector3d();
      n2_0.sub (n2, n0);
      Vector3d cross = new Vector3d();
      cross.cross (n1_0, n2_0);
      cross.normalize ();
      return cross;
   }
   
   public Vector3d[] getContraBaseVectors(ShellFemElement3d ele) {
      return _getContraBaseVectors(ele, NODE_POS.CURRENT);
   }
   
   public Vector3d[] getContraBaseVectors0(ShellFemElement3d ele) {
      return _getContraBaseVectors(ele, NODE_POS.REST);
   }
   
   /**
    * Compute the bases for this integration point in contravector
    * form.
    * 
    * FEBio: FESSIShellDomain::ContraBaseVectors
    * 
    * @param ePosType
    * Type of node position to use.
    *   REST -> getLocalRestPosition()
    *   CURRENT -> getLocalPosition()
    *   RENDER -> myRenderCoords
    * 
    * @return
    * Three contravector bases.
    */
   protected Vector3d[] _getContraBaseVectors(
      ShellFemElement3d ele, NODE_POS ePosType) {
      Vector3d[] g = _getCoBaseVectors(ele, ePosType);
      
      Matrix3d J = new Matrix3d (g[0].x, g[1].x, g[2].x, 
                                 g[0].y, g[1].y, g[2].y,
                                 g[0].z, g[1].z, g[2].z);
      double Jdet = J.fastInvert(J);
      if (Jdet <= 0) {
         throw new RuntimeException("Warning: getContraBaseVectors() detected "
                                    + "determinant <= 0: " + Jdet);
      }
      
      // Compute contravectors using inverted J.
      g[0].set( J.m00, J.m01, J.m02 );
      g[1].set( J.m10, J.m11, J.m12 );
      g[2].set( J.m20, J.m21, J.m22 );
      
      return g;
   }
   
   
   
   /*** Methods for computing jacobian and gradients ***/
   
   public void computeJacobian(ShellFemElement3d ele) { 
      _computeJacobian(ele, NODE_POS.CURRENT, null);
   }
   
   public void computeJacobian0(ShellFemElement3d ele) { 
      _computeJacobian(ele, NODE_POS.REST, null);
   }
   
   /**
    * Compute 3x3 jacobian matrix that represents dN/dx in matrix form. N is
    * the shape function and x is the node position.
    * 
    * The 3x3 jacobian matrix can be seen as a needed component for computing 
    * the 3x3 material stress matrix.
    * 
    * FEBio: Top portion of FESSIShellDomain::invjac0
    * 
    * @param ePosType
    * Type of node position to use.
    *   REST -> getLocalRestPosition()
    *   CURRENT -> getLocalPosition()
    *   RENDER -> myRenderCoords
    * 
    * @param gco 
    * Pre-computed covariant base vectors of this integration point. Otherwise,
    * pass null let function compute it from stratch.
    * 
    * @return
    * void. Call getJ() afterwards to retrieve result.
    */
   protected void _computeJacobian (
      ShellFemElement3d ele, NODE_POS ePosType, Vector3d[] gco) {
      myJ.setZero();
      
      if (gco == null) {
         gco = _getCoBaseVectors(ele, ePosType);
      }
      
      myJ.m00 = gco[0].x; myJ.m01 = gco[1].x; myJ.m02 = gco[2].x;
      myJ.m10 = gco[0].y; myJ.m11 = gco[1].y; myJ.m12 = gco[2].y;
      myJ.m20 = gco[0].z; myJ.m21 = gco[1].z; myJ.m22 = gco[2].z;
   }
   
   /**
    * Compute both the 3x3 Jacobian matrix and 3x3 Gradient form of dN/dx.
    * 
    * FEBio: FESSIShellDomain::defgrad.
    *        Called by FEElasticShellDomain::Update
    * 
    * @return 
    * void. Call getJ(), getF(), and getDetF() to retrieve results.
    */
   public void computeJacobianAndGradient (ShellFemElement3d ele) {
      Vector3d[] gco = getCoBaseVectors(ele);
      Vector3d[] gct = getContraBaseVectors0(ele);
      
      _computeJacobian(ele, NODE_POS.CURRENT, gco);
      
      F.setZero();
      for (int i = 0; i < 3; i++) {
         F.addOuterProduct (gco[i], gct[i]);
      }
      
      detF = F.determinant ();
   }

   /**
    * Compute both the 3x3 Jacobian matrix and 3x3 Gradient form of dN/dx.
    * 
    * FEBio: FESSIShellDomain::defgrad
    * 
    * @return
    * void. Call getJ() afterwards to retrieve Jacobian result. Gradient is 
    * stored in def parameter.
    */
   public void computeJacobianAndGradient (
      SolidDeformation def, ShellFemElement3d ele) {
      Vector3d[] gco = getCoBaseVectors(ele);
      Vector3d[] gct = getContraBaseVectors0(ele);
      
      _computeJacobian(ele, NODE_POS.CURRENT, gco);
      
      Matrix3d F = new Matrix3d();
      for (int i = 0; i < 3; i++) {
         F.addOuterProduct (gco[i], gct[i]);
      }
      
      def.setF (F);
   }

   /**
    * Compute 3x3 Gradient of dN/dx using render positions of nodes.
    * 
    * FEBio: FESSIShellDomain::defgrad
    * 
    * @return
    * void. Gradient is stored in Fmat parameter.
    */
   public void computeGradientForRender (Matrix3d Fmat, ShellFemElement3d ele) {
      Vector3d[] gco = _getCoBaseVectors(ele, NODE_POS.RENDER);
      Vector3d[] gct = getContraBaseVectors0(ele);
      
      _computeJacobian(ele, NODE_POS.RENDER, gco);
      
      Fmat.setZero();
      for (int i = 0; i < 3; i++) {
         Fmat.addOuterProduct (gco[i], gct[i]);
      }
   }      

   
   /**
    * Evaluate a vector function over the shell.
    * 
    * FEBio: FESSIShellDomain::evaluate
    * 
    * @param el
    * Shell element to evaluate the vector function over.
    * 
    * @param vn
    * Computed xyz-related vectors for each node. E.g. acceleration.
    * 
    * @param dvn
    * Computed uvw-related vectors for each node. E.g. rotational acceleration.
    * 
    * @return 
    * Scalar. Generalized vector for the entire shell based on vn and dvn.
    */
   public Vector3d evaluate(ShellFemElement3d el, Vector3d[] vn, Vector3d[] dvn) {
      Vector3d v = new Vector3d(); 
      double t = coords.z;
      VectorNd Ns = getShapeWeights();
      
      for (int n = 0; n < el.getNodes().length; n++) {
         ShellFemNode3d node = (ShellFemNode3d) el.getNodes()[n];
         
         double N = Ns.get(n);
         
         double mu = (1+t)/2.0*N;
         double md = (1-t)/2.0*N;
         
         v.scaledAdd(mu, vn[n]);
         v.scaledAdd(md, dvn[n]);
      }
      
      return v;
   }
   
   /*** FEBio: FEElasticShellDomain::Update relies on existing vec3d evaluate()
               function for its shell implementation. This means that 
               computePosition() here shouldn't need to be overridden. ***/
   
   // DANNY TODO: Need to override computeShapeGradient()?
   // In FEBio, shape_gradient is implemented in 
   // FEElasticSolidDomain2O::shape_gradient(). FEElasticSolidDomain2O is a 
   // subclass of FEElasticSolidDomain. The class implements 
   // discontinuous-Galerkin formulation for gradient elasticity.
   
   @Override
   @Deprecated
   public void computeJacobian (FemNode3d[] nodes) {
      throw new RuntimeException("Error :: Invoked obsolete and dead method.");
   }

   @Override
   @Deprecated
   public void computeJacobianAndGradient (FemNode3d[] nodes, Matrix3d invJ0) {
      throw new RuntimeException("Error :: Invoked obsolete and dead method.");
   }
   
   @Override
   @Deprecated
   public void computeJacobianAndGradient (Point3d[] nodePos, Matrix3d invJ0) {
      throw new RuntimeException("Error :: Invoked obsolete and dead method.");
   }
   
   @Override
   @Deprecated
   public void computeJacobianAndGradient (
      SolidDeformation def, FemNode3d[] nodes, Matrix3d invJ0) {
      throw new RuntimeException("Error :: Invoked obsolete and dead method.");
   }
   
   @Override
   @Deprecated
   public void computeGradientForRender (
      Matrix3d Fmat, FemNode3d[] nodes, Matrix3d invJ0) {
      throw new RuntimeException("Error :: Invoked obsolete and dead method.");
   }   
}
