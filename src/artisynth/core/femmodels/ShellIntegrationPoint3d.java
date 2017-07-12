package artisynth.core.femmodels;

import artisynth.core.materials.SolidDeformation;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

/**
 * Integration coordinate/point for a shell element.
 * 
 * Compared to the IntegrationPoint3d superclass, shell integration points
 * have methods to compute the co and contra bases vectors which the
 * shell integration points' jacobian and gradients rely on. In addition, 
 * the contra bases vectors are used directly when computing the
 * node stress force and stiffness.
 * 
 * The co and contra bases vectors are pre-emptively computed at the 
 * beginning of every timestep and then re-used throughout the timestep.
 * 
 * It's worth noting that this is done by calling updateCoContraVectors()
 * in ShellFemModel3d.updateStressAndStiffness() to pre-emptively compute the
 * co and contra base vectors at the beginning of every timestep.
 * 
 * Implementation is based on FEBio FESSIShellDomain::
 */
public class ShellIntegrationPoint3d extends IntegrationPoint3d {

   /* Node position type: Rest (i.e. Initial), Current, Render */
   protected enum NODE_POS { REST, CURRENT, RENDER };
   
   /* Element that this integration point belongs to */
   protected ShellFemElement3d myEle = null;
   
   /* Computed covariant and contravariant bases vectors are stored here.
    * These should be updated at every timestep via updateCoContrVectors().
    * 
    * gco = 3 covariant bases vectors.
    * gct = 3 convariant bases vectors.
    * 
    * 0 = rest node position
    * <no_suffix> = current node position
    * Rend = renderer node position
    */
   protected Vector3d[] myGco = null;
   protected Vector3d[] myGct = null;
   protected Vector3d[] myGco0 = null;
   protected Vector3d[] myGct0 = null;
   protected Vector3d[] myGcoRend = null;
   protected Vector3d[] myGctRend = null;

   
   
   public ShellIntegrationPoint3d(ShellFemElement3d ele,
      int nnodes, int npvals, double s0, double s1, double s2, double w) {
      super(nnodes, npvals, s0, s1, s2, w);
      initEleAndCoContraVectors(ele);
   }
   
   public ShellIntegrationPoint3d(ShellFemElement3d ele, int nnodes) {
      super(nnodes);
      initEleAndCoContraVectors(ele);
   }
   
   protected void initEleAndCoContraVectors(ShellFemElement3d ele) {
      myEle = ele;
      
      myGco = new Vector3d[3];
      myGct = new Vector3d[3];
      myGco0 = new Vector3d[3];
      myGct0 = new Vector3d[3];
      myGcoRend = new Vector3d[3];
      myGctRend = new Vector3d[3];
      for (int i = 0; i < 3; i++) {
         myGco[i] = new Vector3d();
         myGct[i] = new Vector3d();
         myGco0[i] = new Vector3d();
         myGct0[i] = new Vector3d();
         myGcoRend[i] = new Vector3d();
         myGctRend[i] = new Vector3d();
      }
   }
   
   
   /** 
    * Create an integration point for a given shell element at a specific set of
    * natural coordinates.
    *
    * Co and contra bases vectors are automatically updated for each 
    * integration point.
    *
    * @param elem 
    * Shell element to create the integration point for.
    * 
    * @param s0 
    * First coordinate value (known as r in FEBio, and x in Artisynth)
    * 
    * @param s1
    * Second coordinate value (known as s in FEBio, and y in Artisynth)
    * 
    * @param s2
    * Third coordinate value (known as t in FEBio, and z in Artisynth)
    * 
    * @param w 
    * Weighing of the integration point.
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
            new ShellIntegrationPoint3d (elem, nnodes, npvals, s0, s1, s2, w);
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
      
      pnt.updateCoContraVectors();
      
      return pnt;
   }
   
   
   
   
   /* --- Methods for computing covariant and contravariant bases vectors.
    * These vectors are used to compute the gradients and eventual
    * shell stress and stiffness. --- */
   
   
   /**
    * Update the covariant and contravariant bases vectors for all 3 node 
    * position types: (REST, CURRENT, RENDER).
    * 
    * Afterwards, these methods can now be called for the current timestep
    * in O(1) time.
    *   getCoBaseVectors()
    *   getContraBaseVectors()
    *   computeJacobian()
    *   computeJacobianAndGradient()
    *   
    * This method should be called at the beginning of every time step, such 
    * as in ShellFemModel3d.updateStressAndStiffness() 
    */
   public void updateCoContraVectors() {             
      computeCoBaseVectors(NODE_POS.REST);
      computeCoBaseVectors(NODE_POS.CURRENT);
      computeCoBaseVectors(NODE_POS.RENDER);
      
      computeContraBaseVectors(NODE_POS.REST);
      computeContraBaseVectors(NODE_POS.CURRENT);
      computeContraBaseVectors(NODE_POS.RENDER);
   }
   
   
   /**
    * Compute the covariant bases vectors of this integration point.
    * 
    * FEBio: FESSIShellDomain::CoBaseVectors
    * 
    * @param ePosType
    * Type of node position to use.
    *   REST -> getRestPosition()
    *   CURRENT -> getPosition()
    *   RENDER -> myRenderCoords
    * 
    * Postcond:
    *   Results are stored in this.myGco, myGco0, or myGcoRend, depending
    *   on ePosType.
    * 
    * @return 
    * void. Use getCoBaseVectors() to retrieve results.
    */
   protected void computeCoBaseVectors(NODE_POS ePosType) {
      Vector3d[] g = null;
      if (ePosType == NODE_POS.REST) {
         g = myGco0;
      }
      else if (ePosType == NODE_POS.CURRENT) {
         g = myGco;
      }
      else if (ePosType == NODE_POS.RENDER) {
         g = myGcoRend;
      }
      
      for (int i = 0; i < 3; i++) {
         g[i].setZero ();
      }
      
      double t = getCoords().z;

      // For each node...
      for (int n = 0; n < myEle.getNodes().length; n++) {
         ShellFemNode3d node = (ShellFemNode3d) myEle.getNodes()[n];
         Vector3d d = null;
         
         Point3d pos = null;
         if (ePosType == NODE_POS.REST) {
            pos = node.getRestPosition();
            d = new Vector3d( node.myDirector0 );
         }
         else if (ePosType == NODE_POS.CURRENT) {
            pos = node.getPosition();
            d = new Vector3d( node.myDirector0 );
            d.add( node.getDisplacement() );
            d.sub( node.getDir() );
         }
         else if (ePosType == NODE_POS.RENDER) {
            float[] rPos = node.myRenderCoords; 
            pos = new Point3d( rPos[0], rPos[1], rPos[2] );
            d = new Vector3d( node.myDirector0 );
            d.add( node.getDisplacement() );
            d.sub( node.getDir() );
         }
     
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
   }
   
   
   /**
    * Compute the contravariant bases vectors of this integration point.
    * 
    * FEBio: FESSIShellDomain::ContraBaseVectors
    * 
    * Precond:
    *   computeCoBaseVectors(ePosType) was called beforehand in the current
    *   timestep.
    *   
    * @param ePosType
    * Type of node position to use.
    *   REST -> getRestPosition()
    *   CURRENT -> getPosition()
    *   RENDER -> myRenderCoords
    * 
    * Postcond:
    *   Results are stored in this.myGct, myGct0, or myGctRend, depending
    *   on ePosType.
    * 
    * @return 
    * void. Use getContraBaseVectors() to retrieve results.
    */
   protected void computeContraBaseVectors(NODE_POS ePosType) {
      Vector3d[] g = null;
      if (ePosType == NODE_POS.REST) {
         g = myGct0;
      }
      else if (ePosType == NODE_POS.CURRENT) {
         g = myGct;
      }
      else if (ePosType == NODE_POS.RENDER) {
         g = myGctRend;
      }
      
      computeJacobian(ePosType);
      computeInverseJacobian();
      
      // Compute contravectors using inverted J.
      g[0].set( myInvJ.m00, myInvJ.m01, myInvJ.m02 );
      g[1].set( myInvJ.m10, myInvJ.m11, myInvJ.m12 );
      g[2].set( myInvJ.m20, myInvJ.m21, myInvJ.m22 );
   }
   
   
   /**
    * Retrieve the computed covariant bases vectors.
    * 
    * Precond:
    *   computeCoBaseVectors(eNodePos) or updateCoContrVectors() 
    *   was called beforehand in the current timestep.
    * 
    * @param ePosType
    * Type of node position to use.
    *   REST -> getRestPosition()
    *   CURRENT -> getPosition()
    *   RENDER -> myRenderCoords
    *   
    * @return
    * 3 covariant bases vectors.
    */
   public Vector3d[] getCoBaseVectors(NODE_POS eNodePos) {
      if (eNodePos == NODE_POS.REST) {
         return myGco0;
      }
      else if (eNodePos == NODE_POS.CURRENT) {
         return myGco;
      }
      else if (eNodePos == NODE_POS.RENDER) {
         return myGcoRend;
      }
      else {
         return null;
      }
   }

   
   /**
    * Retrieve the computed contravariant bases vectors.
    * 
    * Precond:
    *   computeContraBaseVectors(eNodePos) or updateCoContrVectors() was 
    *   called beforehand in the current timestep.
    * 
    * @param ePosType
    * Type of node position to use.
    *   REST -> getRestPosition()
    *   CURRENT -> getPosition()
    *   RENDER -> myRenderCoords
    *   
    * @return
    * 3 contravariant bases vectors.
    */
   public Vector3d[] getContraBaseVectors(NODE_POS eNodePos) {
      if (eNodePos == NODE_POS.REST) {
         return myGct0;
      }
      else if (eNodePos == NODE_POS.CURRENT) {
         return myGct;
      }
      else if (eNodePos == NODE_POS.RENDER) {
         return myGctRend;
      }
      else {
         return null;
      }
   }
   

   
   
   
   /*** Methods for computing jacobian and gradients ***/
   
   /**
    * Compute 3x3 jacobian matrix that represents dN/dx in matrix form. N is
    * the shape function and x is the node position.
    * 
    * The 3x3 jacobian matrix can be seen as a needed component for computing 
    * the 3x3 material stress matrix.
    * 
    * FEBio: Top portion of FESSIShellDomain::invjac0
    * 
    * Precond:
    *   computeCoBaseVectors(ePosType) or updateCoContraVectors()
    *     was called beforehand in the current timestep.
    * 
    * @param ePosType
    * Type of node position to use.
    *   REST -> getRestPosition()
    *   CURRENT -> getPosition()
    *   RENDER -> myRenderCoords
    * 
    * Postcond:
    *   Result stored in this.myJ
    * 
    * @return
    * void. Call getJ() afterwards to retrieve result.
    */
   public void computeJacobian (NODE_POS ePosType) {
      Vector3d[] g = null;
      if (ePosType == NODE_POS.REST) {
         g = myGco0;
      }
      else if (ePosType == NODE_POS.CURRENT) {
         g = myGco;
      }
      else if (ePosType == NODE_POS.RENDER) {
         g = myGcoRend;
      }
      
      myJ.m00 = g[0].x; myJ.m01 = g[1].x; myJ.m02 = g[2].x;
      myJ.m10 = g[0].y; myJ.m11 = g[1].y; myJ.m12 = g[2].y;
      myJ.m20 = g[0].z; myJ.m21 = g[1].z; myJ.m22 = g[2].z;
   }
   
   /**
    * Compute both the 3x3 Jacobian matrix and 3x3 Gradient form of dN/dx.
    * 
    * FEBio: FESSIShellDomain::defgrad.
    *        Called by FEElasticShellDomain::Update
    * 
    * Precond:
    *   computeCoBaseVectors(NODE_POS.CURRENT) and 
    *   computeContraBaseVectors(NODE_POS.REST)
    *   
    *   or
    *   
    *   updateCoContraVectors()
    *   
    *      were called beforehand for the current timestep.
    * 
    * @return 
    * void. Call getJ(), getF(), and getDetF() to retrieve results.
    */
   public void computeJacobianAndGradient () {
      computeJacobian(NODE_POS.CURRENT);
      
      F.setZero();
      for (int i = 0; i < 3; i++) {
         F.addOuterProduct (myGco[i], myGct0[i]);
      }
      
      detF = F.determinant ();
   }

   /**
    * Compute both the 3x3 Jacobian matrix and 3x3 Gradient form of dN/dx.
    * 
    * FEBio: FESSIShellDomain::defgrad
    * 
    * Precond:
    *   computeCoBaseVectors(NODE_POS.CURRENT) and 
    *   computeContraBaseVectors(NODE_POS.REST)
    *   
    *   or
    *   
    *   updateCoContraVectors()
    *   
    *      were called beforehand for the current timestep.
    * 
    * @return
    * void. Call getJ() afterwards to retrieve Jacobian result. Gradient is 
    * stored in def parameter.
    */
   public void computeJacobianAndGradient (SolidDeformation def) {
      computeJacobian(NODE_POS.CURRENT);
      
      Matrix3d F = new Matrix3d();
      for (int i = 0; i < 3; i++) {
         F.addOuterProduct (myGco[i], myGct0[i]);
      }
      
      def.setF (F);
   }

   /**
    * Compute 3x3 Gradient of dN/dx using render positions of nodes.
    * 
    * FEBio: FESSIShellDomain::defgrad
    * 
    * Precond:
    *   computeCoBaseVectors(NODE_POS.CURRENT) and 
    *   computeContraBaseVectors(NODE_POS.REST)
    *   
    *   or
    *   
    *   updateCoContraVectors()
    *   
    *      were called beforehand for the current timestep.
    * 
    * @return
    * void. Gradient is stored in Fmat argument.
    */
   public void computeGradientForRender (Matrix3d Fmat) {
      computeJacobian(NODE_POS.RENDER);
      
      Fmat.setZero();
      for (int i = 0; i < 3; i++) {
         Fmat.addOuterProduct (myGcoRend[i], myGct0[i]);
      }
   }

   
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
