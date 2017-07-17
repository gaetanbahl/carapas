package artisynth.core.femmodels;

import java.util.LinkedList;

import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.Point;
import artisynth.core.modelbase.TransformGeometryContext;
import artisynth.core.modelbase.TransformableGeometry;
import maspack.geometry.GeometryTransformer;
import maspack.matrix.Matrix;
import maspack.matrix.Matrix6d;
import maspack.matrix.Matrix6dBlock;
import maspack.matrix.Matrix6dDiagBlock;
import maspack.matrix.MatrixBlock;
import maspack.matrix.Point3d;
import maspack.matrix.SparseBlockMatrix;
import maspack.matrix.SparseNumberedBlockMatrix;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

/**
 * Shell node for a shell element. This extends FemNode3d to include 
 * 3 additional dof which is referred as the direction (u,w,v). The direction
 * of a shell node can be seen as the vector between the x,y,z rest position 
 * to the x,y,z current position when everything is calm.
 * 
 * The node's rest director, which is distinct from the direction, is a vector
 * that lies through the node to represent the element thickness at that
 * node point. The rest director vector is simply computed as the vertex 
 * normal of the node. Aside from representing thickness, the director is
 * used to compute the node stress and stiffness.
 * 
 * Currently, there are no implemented thickness nodes that sit above and below
 * the shell node.
 */
public class ShellFemNode3d extends FemNode3d {
   
   /* Director vector when node is at rest. This is ultimately used to compute
    * the current (i.e. non-rest) director vector:
    *   
    *   d = node.myDirector0 + node.position - node.myDir
    *   
    * This d vector is used to compute co and contra vectors to subsequently
    * compute the node stress, stiffness, and element volume.
    * 
    * node.myDir is the node's current direction, which is distinct from the
    * director and rest director.
    */
   protected Vector3d myDirector0 = new Vector3d();
   
   /* Adjacent elements of this node. This is updated in the constructor of an 
    * element. This is preferred over super.myElementDeps because myAdjElement 
    * will be updated earlier to compute the rest directors earlier in order 
    * for FemModel3d.addElement() to compute the volume during initialization */
   public LinkedList<ShellFemElement3d> myAdjElements = 
      new LinkedList<ShellFemElement3d>();
   
   /* Direction */
   public Vector3d myDir = new Vector3d();
   
   /* Direction velocity */
   public Vector3d myDirVel = new Vector3d();
   
   /* Direction force */
   public Vector3d myDirForce = new Vector3d();
   public Vector3d myInternalDirForce = new Vector3d();
   
   /* Target direction and direction velocity. Mainly used by setDynamic(false)
    * which holds the node at a fixed (i.e. target) position and direction. */
   protected Vector3d myTargetDir = new Vector3d();
   protected Vector3d myTargetDirVel = new Vector3d();
  
   
   
   public ShellFemNode3d() {
      super();
   }
   
   public ShellFemNode3d (Point3d p) {
      super(p);
   }

   public ShellFemNode3d (double x, double y, double z) {
      super(x,y,z);
   }
   
   
   
   public Vector3d getDirector0() {
      return myDirector0;
   }
   
   public void setDirector0(Vector3d d0) {
      myDirector0 = d0;
   }
   
   public Vector3d getDir() {
      return myDir;
   }
   
   @Override
   public int getPosState(double[] x, int idx) {
      idx = super.getPosState(x, idx);
      x[idx++] = myDir.x;
      x[idx++] = myDir.y;
      x[idx++] = myDir.z;
      return idx;
   }
   
   @Override 
   public int setPosState(double[] p, int idx) {
      idx = super.setPosState(p, idx);
      myDir.x = p[idx++];
      myDir.y = p[idx++];
      myDir.z = p[idx++];
      return idx;
   }
   
   @Override 
   public void addPosImpulse (
      double[] xbuf, int xidx, double h, double[] vbuf, int vidx) {

      xbuf[xidx  ] += h*vbuf[vidx  ];
      xbuf[xidx+1] += h*vbuf[vidx+1];
      xbuf[xidx+2] += h*vbuf[vidx+2];
      
      xbuf[xidx+3] += h*vbuf[vidx+3];
      xbuf[xidx+4] += h*vbuf[vidx+4];
      xbuf[xidx+5] += h*vbuf[vidx+5];
   }
   
   @Override
   public int getPosDerivative (double[] dxdt, int idx) {
      idx = super.getPosDerivative(dxdt, idx);
      dxdt[idx++] = myDirVel.x;
      dxdt[idx++] = myDirVel.y;
      dxdt[idx++] = myDirVel.z;
      return idx;
   }
   
   @Override
   public int getVelState (double[] v, int idx) {
      idx = super.getVelState(v, idx);
      v[idx++] = myDirVel.x;
      v[idx++] = myDirVel.y;
      v[idx++] = myDirVel.z;
      return idx;
   }
   
   @Override
   public int setVelState (double[] v, int idx) {
      idx = super.setVelState (v, idx);
      myDirVel.x = v[idx++];
      myDirVel.y = v[idx++];
      myDirVel.z = v[idx++];
      return idx;
   }
   
   public void getVelocity(VectorNd v6) {
      double[] v6buf = v6.getBuffer();
      getVelState(v6buf /*return value*/, 0);
   }
   
   @Override 
   public int getVelStateSize() {
      return 6;
   }
   
   @Override 
   public int getPosStateSize() {
      return 6;
   }
   
   @Override
   public void setState(Point pt) {
      super.setState(pt);
 
      ShellFemNode3d node = (ShellFemNode3d) pt;
      myDir.set( node.myDir );
      myDirVel.set( node.myDirVel );
   }
   
   @Override
   public int setState(VectorNd x, int idx) {
      idx = super.setState(x, idx);
      
      double[] xb = x.getBuffer();
      
      myDir.x = xb[idx++];
      myDir.y = xb[idx++];
      myDir.z = xb[idx++];
      
      idx = myState.getVel(xb, idx);
      myDirVel.x = xb[idx++];
      myDirVel.y = xb[idx++];
      myDirVel.z = xb[idx++];
      
      return idx;
   }
   
   @Override
   public boolean velocityLimitExceeded (double tlimit, double rlimit) {
      boolean isXyzVeloExceeded = super.velocityLimitExceeded (tlimit, rlimit);
      boolean isUvwVeloExceeded = 
         (myDirVel.containsNaN() || myDirVel.infinityNorm() > tlimit);
      return (isXyzVeloExceeded || isUvwVeloExceeded);
   }
   
   @Override
   public int getForce (double[] f, int idx) {
      idx = super.getForce(f, idx);
      f[idx++] = myDirForce.x;
      f[idx++] = myDirForce.y;
      f[idx++] = myDirForce.z;
      return idx;
   }
   
   @Override
   public int setForce (double[] f, int idx) {
      idx = super.setForce(f, idx);
      myDirForce.x = f[idx++];
      myDirForce.y = f[idx++];
      myDirForce.z = f[idx++];
      return idx;
   }
   
   @Override
   public void zeroForces() {
      super.zeroForces();
      myDirForce.setZero();
   }
   
   public void getInternalForce(VectorNd f6) {
      double[] f6b = f6.getBuffer();
      
      f6b[0] = this.myInternalForce.x;
      f6b[1] = this.myInternalForce.y;
      f6b[2] = this.myInternalForce.z;
      f6b[3] = this.myInternalDirForce.x;
      f6b[4] = this.myInternalDirForce.y;
      f6b[5] = this.myInternalDirForce.z;
   }
   
   public void subForce (VectorNd f6) {
      Vector3d fxyz = new Vector3d(f6.get(0), f6.get(1), f6.get(2));
      super.subForce(fxyz);
      this.myDirForce.x -= f6.get(3);
      this.myDirForce.y -= f6.get(4);
      this.myDirForce.z -= f6.get(5);
   }
   
   
   /* --- Methods pertaining to the mass and solve blocks --- */
   
   
   @Override
   public MatrixBlock createMassBlock() {
      return new Matrix6dDiagBlock();
   }
   
   @Override 
   protected void doGetMass(Matrix M, double m) {
      if (M instanceof Matrix6d) {
         Matrix6d M6 = (Matrix6d)M;
         M6.setDiagonal( new double[]{m,m,m,m,m,m} );
      }
      else {
         throw new IllegalArgumentException ("Matrix not instance of Matrix6d");
      }
   }
   
   @Override
   public void getInverseMass (Matrix Minv, Matrix M) {
      if (!(Minv instanceof Matrix6d)) {
         throw new IllegalArgumentException ("Minv not instance of Matrix6d");
      }
      if (!(M instanceof Matrix6d)) {
         throw new IllegalArgumentException ("M not instance of Matrix6d");
      }
      double inv = 1/((Matrix6d)M).m00;
      ((Matrix6d)Minv).setDiagonal(new double[]{inv, inv, inv, inv, inv, inv});
   }
   
   @Override 
   public void addSolveBlock(SparseNumberedBlockMatrix S) {
      int bi = getSolveIndex();
      Matrix6dBlock blk = new Matrix6dBlock();
      mySolveBlockNum = S.addBlock(bi, bi, blk);
   }

   @Override 
   public MatrixBlock createSolveBlock() {
      Matrix6dBlock blk = new Matrix6dBlock();
      return blk;
   }
   
   @Override 
   public void addToSolveBlockDiagonal(SparseNumberedBlockMatrix S, double d) {
      if (mySolveBlockNum != -1) {
         Matrix6dBlock blk = (Matrix6dBlock)S.getBlockByNumber(mySolveBlockNum);
         
         blk.m00 += d;
         blk.m11 += d;
         blk.m22 += d;
         blk.m33 += d;
         blk.m44 += d;
         blk.m55 += d;
      }
   }
   
   @Override
   public void addVelJacobian (SparseNumberedBlockMatrix S, double s) {
      if (myPointDamping != 0) {
         addToSolveBlockDiagonal (S, -s * myPointDamping);
      }
   }
   
   @Override
   public int mulInverseEffectiveMass (
      Matrix M, double[] a, double[] f, int idx) {
      a[idx++] = 0;
      a[idx++] = 0;
      a[idx++] = 0;
      a[idx++] = 0;
      a[idx++] = 0;
      a[idx++] = 0;
      return idx;
   }

   @Override
   public int getEffectiveMassForces (VectorNd f, double t, int idx) {
      double[] buf = f.getBuffer();
      // Note that if the point is attached to a moving frame, then that
      // will produce mass forces that are not computed here.
      buf[idx++] = 0;
      buf[idx++] = 0;
      buf[idx++] = 0;
      buf[idx++] = 0;
      buf[idx++] = 0;
      buf[idx++] = 0;
      return idx;
   }
   
   @Override
   public void setPointFrame(Frame frame) {
      throw new RuntimeException("Unimplemented.");
   }
   
   @Override
   public Frame getPointFrame() {
      return super.getPointFrame ();
   }
   
   @Override
   public void setVelocity (Vector3d vel) {
      throw new RuntimeException("Unimplemented.");
   }
   
   
   /* --- Methods pertaining to target states. Needed for 
    *     node.setDynamic(false) to work ***/
   
   
   @Override 
   public void resetTargets() {
      super.resetTargets ();
      myTargetDir.set (myDir);
      myTargetDirVel.set(myDirVel);
   }
   
   @Override
   public Point3d getTargetPosition() {
      return super.getTargetPosition();
   }
   
   @Override
   public void setTargetPosition (Point3d pos) {
      throw new RuntimeException("Unimplemented.");
   }
   
   @Override
   public int addTargetJacobian (SparseBlockMatrix J, int bi) {
      throw new RuntimeException("Unimplemented.");
   }
   
   @Override
   public Vector3d getTargetVelocity () {
      return super.getTargetVelocity ();
   }
   
   @Override
   public void setTargetVelocity (Vector3d vel) {
      throw new RuntimeException("Unimplemented.");
   }

   @Override
   public int getTargetVel (double[] velt, double s, double h, int idx) {
      idx = super.getTargetVel (velt, s, h, idx);
      velt[idx++] = myTargetDirVel.x;
      velt[idx++] = myTargetDirVel.y;
      velt[idx++] = myTargetDirVel.z;
      return idx;
   }

   @Override
   public int setTargetVel (double[] velt, int idx) {
      idx = super.setTargetVel(velt, idx);
      myTargetDirVel.x = velt[idx++];
      myTargetDirVel.y = velt[idx++];
      myTargetDirVel.z = velt[idx++];
      return idx;
   }

   @Override
   public int getTargetPos (double[] post, double s, double h, int idx) {
      idx = super.getTargetPos (post, s, h, idx);
      post[idx++] = myTargetDir.x;
      post[idx++] = myTargetDir.y;
      post[idx++] = myTargetDir.z;
      return idx;
   }

   @Override
   public int setTargetPos (double[] post, int idx) {
      idx = super.setTargetPos(post, idx);
      myTargetDir.x = post[idx++];
      myTargetDir.y = post[idx++];
      myTargetDir.z = post[idx++];
      return idx;
   }
   
   
   
   /* --- Methods pertaining to node neighbors. Override those methods that 
    * require instantiating a new fem node neighbor. In that case, we want to 
    * specifically instantiate shell-specific node neighbors. --- */
   
 
   @Override
   public NodeNeighbor addIndirectNeighbor (FemNode3d nbrNode) {
      ShellFemNodeNeighbor nbr = new ShellFemNodeNeighbor((ShellFemNode3d)nbrNode);
      if (myIndirectNeighbors == null) {
         myIndirectNeighbors = new LinkedList<NodeNeighbor>();
      }
      myIndirectNeighbors.add (nbr);
      return nbr;
   }

   @Override
   protected void registerNodeNeighbor (FemNode3d nbrNode) {
      NodeNeighbor nbr = getNodeNeighbor (nbrNode);
      if (nbr == null) {
         nbr = new ShellFemNodeNeighbor ((ShellFemNode3d)nbrNode);
         myNodeNeighbors.add (nbr);
      }
      else {
         nbr.myRefCnt++;
      }
   }
   
   
   
   
   
   /** 
    * This is invoked whenever a node is dragged via the x,y,z translation 
    * GUI tool. Specific for shell nodes, this overridden method will also 
    * translate the extra 3dof direction vector as well. */
   @Override
   public void transformGeometry (
      GeometryTransformer gt, TransformGeometryContext context, int flags) {
      super.transformGeometry (gt, context, flags);
      
      // If not simulating, set direction to zero.
      if ((flags & TransformableGeometry.TG_SIMULATING) == 0) {
         myDir.setZero ();
      }
      else {
         Point3d myDirPt = new Point3d(myDir);
         gt.transformPnt(myDirPt);
         myDir.set (myDirPt);
      }
   }
   
   
   /**
    * Update the rest director vector. 
    * 
    * Implementation is simply finding the vertex normal which is the 
    * average normal of the adjacent elements.
    * 
    * FEBio: FEMesh::InitShellsNew()
    * 
    * Precond:
    * Rest position (x,y,z) vectors are updated for each node of the adjacent
    * elements.
    * 
    * Postcond:
    * this.myDirector0 is set
    */
   public void updateDirector0() {
      myDirector0.setZero ();
      
      for (ShellFemElement3d e : myAdjElements) {
         int i = e.getNodeIndex (this);
         
         // Get next and prev nodes relative to i-th node.
         int n = (i+1) % e.numNodes();
         int p = (i==0) ? e.numNodes()-1 : i-1; 
         
         Vector3d iPos = e.myNodes[i].getRestPosition ();
         Vector3d nPos = e.myNodes[n].getRestPosition ();
         Vector3d pPos = e.myNodes[p].getRestPosition ();
        
         Vector3d n_i = new Vector3d();
         n_i.sub (nPos, iPos);
         
         Vector3d p_i = new Vector3d();
         p_i.sub (pPos, iPos);
         
         Vector3d dir = new Vector3d();
         dir.cross (n_i, p_i);
         dir.normalize ();
         dir.scale (e.getShellThickness());
         
         myDirector0.add(dir);
      }
      
      myDirector0.scale (1.0/myAdjElements.size ());
   }
}

