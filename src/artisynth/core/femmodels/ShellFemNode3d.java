package artisynth.core.femmodels;

import java.util.LinkedList;

import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.Point;
import maspack.matrix.Matrix;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3x3DiagBlock;
import maspack.matrix.Matrix6d;
import maspack.matrix.Matrix6dBlock;
import maspack.matrix.Matrix6dDiagBlock;
import maspack.matrix.MatrixBlock;
import maspack.matrix.Point3d;
import maspack.matrix.SparseBlockMatrix;
import maspack.matrix.SparseNumberedBlockMatrix;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.util.InternalErrorException;

public class ShellFemNode3d extends FemNode3d {
   
   public Vector3d myDirector0 = new Vector3d();
   
   /* Adjacent elements of this node */
   public LinkedList<ShellFemElement3d> myAdjElements = 
      new LinkedList<ShellFemElement3d>();
   
   /* Direction force */
   protected Vector3d myDirForce = new Vector3d();
   protected Vector3d myInternalDirForce = new Vector3d();
   
   /* Direction */
   protected Vector3d myDir = new Vector3d();
   
   /* Direction velocity */
   protected Vector3d myDirVel = new Vector3d();
   
   public Vector3d danF = new Vector3d();
   public Vector3d danDF = new Vector3d();
   
   
   public ShellFemNode3d() {
      super();
   }
   
   public ShellFemNode3d (Point3d p) {
      super(p);
   }

   public ShellFemNode3d (double x, double y, double z) {
      super(x,y,z);
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
      
      /* Pos and velo only. No director nor force */
      
      ShellFemNode3d node = (ShellFemNode3d) pt;
      myDir.set( node.myDir );
      myDirVel.set( node.myDirVel );
   }
   
   @Override
   public int setState(VectorNd x, int idx) {
      idx = super.setState(x, idx);
      
      double[] xb = x.getBuffer();
      
      /* Pos and velo only. No director nor force */
      
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
   
   /*** Methods pertaining to the mass and solve blocks ***/
   
   
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
      throw new RuntimeException("Unimplemented.");
   }
   
   @Override
   public void setVelocity (Vector3d vel) {
      throw new RuntimeException("Unimplemented.");
   }
   
   @Override
   public int addTargetJacobian (SparseBlockMatrix J, int bi) {
      throw new RuntimeException("Unimplemented.");
   }
   
   
   /*** Methods pertaining to node neighbors. Override those methods that 
    * require instantiating a new fem node neighbor. In that case, we want to 
    * specifically instantiate shell-specific node neighbors. ***/
   
 
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
}

