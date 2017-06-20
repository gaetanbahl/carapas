package artisynth.demos.mech;

import java.awt.Color;
import java.util.ArrayList;
import artisynth.core.mechmodels.ForceComponent;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.workspace.RootModel;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Matrix2x3;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3x2;
import maspack.matrix.Point3d;
import maspack.matrix.SparseNumberedBlockMatrix;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.FaceStyle;

/**
 * Implementation of the "Large Steps in Cloth Simulation" paper. Only
 * the forces were implemented (equation 7) for stretch, shear, and bend.
 * 
 * Follows the "Implementing Baraff & Witkin's Cloth Simulation" paper for 
 * the implementation. From here on, any equation (e.g. equation (4)) or 
 * symbol (e.g. wuHat) mentioned in this code refers to that paper unless
 * "original paper" (i.e. "Large Steps in Cloth Simulation") is mentioned.
 * 
 * @author Danny Huang <dah208@mail.usask.ca>
 * 
 * Feel free to contact me for help, especially if you plan on finish 
 * implementing this.
 */
public class Cloth extends RootModel implements ForceComponent {

   /* Length and width of cloth */
   public final int mMeshX = 500;
   public final int mMeshY = 250;
   
   /* Number of triangles in the cloth == mMeshXDiv * mMeshYDiv  */
   public final int mMeshXDiv = 30;
   public final int mMeshYDiv = 30;
  
   /* Mesh for the cloth */
   protected PolygonalMesh mMesh = null;
   
   /* Copy of initial cloth mesh. Must be flat such that z-position is 0 
    * across cloth vertices. Used to represent the cloth in UV coordinates */
   protected PolygonalMesh mMeshUV = null;
   
   public final Color mRearMeshColor = Color.RED;
   public final Color mMeshEdgeColor = Color.BLUE;
   
   protected MechModel mModel = null;
   protected RigidBody mClothRB = null;
   protected Point3d mClothRB_fixedPos = null;
   
   /* Particles that represent the cloth. At every frame, forces will be 
    * computed and applied to the particles, and the cloth mesh vertices will 
    * be updated to match with the particle locations 
    * 
    * Each particle has a corresponding cloth mesh vertex.
    * E.g. mParticles[ mMesh.getVertex(5) ] */
   
   protected Particle[] mParticles = null;
   public final double mParticleRadius = 1;
   public final Color mParticleColor = Color.CYAN;
   public final double mParticleMass = 1;
   public final double mParticleDamping = 0; 
  
   /* Triangle lengths and areas of the UV mesh.
    * 
    * Will use simply use x and y axes of the Artisynth world to represent
    * the u and v axes of the UV mesh (i.e. this.mMeshUV).
    * 
    * Example:
    * 
    * For triangle #4 of the UV mesh, its three vertices are x0, x1, x2.
    * 
    * mU1s[4] == (x1.x - x0.x) == Length between x1 and x0 on u axis
    * mV1s[4] == (x1.y - x0.y) == Length between x1 and x0 on v axis
    * mU2s[4] == (x2.x - x0.x) == Length between x2 and x0 on u axis
    * mV2s[4] == (x2.y - x0.y) == Length between x2 and x0 on v axis.
    * mUVAreas[4] == area of triangle on uv axes == equation (1) */
   
   protected double[] mU1s = null;
   protected double[] mV1s = null;
   protected double[] mU2s = null;
   protected double[] mV2s = null;
   protected double[] mUVAreas = null;
   
   /* Allows leeway for stretching cloth in a particular UV axis. 
    * For equation (13).*/
   
   protected double m_bu = 1;
   protected double m_bv = 1;
  
   /* u direction (wu) and v direction (wv) of triangles at current time step.
    * Equation (2). Indexed by triangle index. Equation (2) */
   
   protected Vector3d[] mWus = null;
   protected Vector3d[] mWvs = null;
   
   /* Derivatives of wu and wv. Relies on UV coordinates so constant throughout
    * simulation. For triangle #4:
    * 
    * mDWuxdxmx[4][0] == Derivative of wu with respect to vertex x0 of triangle.
    *                 == Equation (6).
    * mDWuxdxmx[4][1] == Derivative of wu with respect to vertex x1 of triangle.
    *                 == Equation (7).
    * mDWuxdxmx[4][2] == Derivative of wu with respect to vertex x2 of triangle.
    *                 == Equation (8).
    * 
    * mDWvxdxmx is for derivative of wu instead of wv. */
   
   protected double[][] mDWuxdxmx = null;
   protected double[][] mDWvxdxmx = null;
   
   /* List of common triangle edges.
    * mTriPairs.get(5).getFace() == 1st triangle of adjacent triangle pair #5.
    * mTriPairs.get(5).getOppositeFace() == 2nd triangle of adjacent triangle
    *                                       pair #5. */
   protected ArrayList<HalfEdge> mTriPairs = null;
   
   /* Four vertices of each adjacent triangle pair. 
    * mTriPairVtxs[5][2] == Vertex #2 of adjacent triangle pair #5.
    * 
    * Vertex #0 is the vertex belonging to only the 1st triangle of the triangle 
    * pair 
    * 
    * Vertex #1 and #2 are the vertices shared between the triangle pair.
    * 
    * Vertex #3 is the vertex belonging to only the 2nd triangle of the triangle
    * pair. */
   protected Vertex3d[][] mTriPairVtxs = null;
   
   /* Unit normals of triangles of current time step.
    * 
    * mTriPairUnitNormals[5][0] == Unit normal of 1st triangle of triangle 
    * pair #5.
    * 
    * mTriPairUnitNormals[5][1] == Unit normal of 2nd triangle of triangle 
    * pair #5. */
   protected Vector3d[][] mTriPairUnitNormals = null;
   
   /* Unit common edge between each triangle pair of current time step.
    * 
    * mTriPairUnitNormals[5] == Unit common edge for triangle pair #5. */
   protected Vector3d[] mTriPairCommonUnitEdges = null;
   
   /* Magnitudes of mTriPairUnitNormals before they were normalized into 
    * unit vectors. */
   protected double[][] mTriPairNormalMags = null;
   
   /* Magnitudes of mTriPairCommonUnitEdges before they were normalized into 
    * unit vectors. */
   protected double[] mTriPairCommonEdgeMags = null;
   
   /* Stretch force strength scaler */
   public final double mK_stretch = 0.1;
   
   /* Stretch force strength damping scaler */
   public final double mK_stretchDamp = 0.01;
   
   /* Shear force strength scaler */
   public final double mK_shear = 0.1;
   
   /* Shear force strength damping scaler */
   public final double mK_shearDamp = 0.01;
   
   /* Bend force strength scaler */
   public final double mK_bend = 100;
   
   /* Bend force strength damping scaler */
   public final double mK_bendDamp = 10;
   
   /* Maximum particle force allowed. Another form of damping to prevent
    * unstable oscillation. */
   public final double mMaxForceMag = 100;    

   
   /**
    *  Construct the cloth at the start 
    */
   public void build (String[] args) {
      mModel = new MechModel ("mech");
      mModel.setGravity(0, 0, 0);
      addModel(mModel);
      
      /* Create a flat plane for the cloth */
      mMesh = MeshFactory.createPlane(mMeshX, mMeshY, mMeshXDiv, mMeshYDiv);

      /* Attach rigid body to cloth mesh */
      mClothRB = new RigidBody();
      mClothRB.setMesh(mMesh);
      mClothRB_fixedPos = new Point3d( mClothRB.getPosition() );
      
      /* Copy the flat cloth to represent the cloth in UV coordinates */
      mMeshUV = mMesh.copy();
      
      /* Cloth will be govern by its particles instead */
      mClothRB.setDynamic(false);
      
      /* Make rear of mesh visible */
      mClothRB.getRenderProps().setFaceStyle(FaceStyle.FRONT_AND_BACK);
      mClothRB.getRenderProps().setBackColor(mRearMeshColor);
      
      /* Make mesh edges visible*/
      mClothRB.getRenderProps().setEdgeColor(mMeshEdgeColor);
      mClothRB.getRenderProps().setDrawEdges(true);
      
      /* Attach cloth to overall model */
      mModel.addRigidBody(mClothRB);
     
      /* Create a cloth particle for each mesh vertex */
      mParticles = new Particle[mMesh.numVertices()];
      for (int i = 0; i < mMesh.numVertices(); i++) {
         Vertex3d v = mMesh.getVertices().get(i);
         Particle p = new Particle(mParticleMass, v.getPosition());
         p.setPointDamping(mParticleDamping);
         /* Make particle visible using a sphere */
         RenderProps.setSphericalPoints(p, mParticleRadius, mParticleColor);
         mParticles[i] = p;
         mModel.addParticle(p);
      }
      
      /* Initiate any needed constants before computing cloth forces */
      initMeshUVTriangleLengths(mMeshUV);
      initMeshUVTriangleArea(mMeshUV);
      initWuWvDerivatives();
      initTriPairs(mMesh);
      initTriPairVtxs();
   }
   
   
   /**
    * Compute the triangle lengths of the cloth mesh in UV coordinates.
    * Relies on UV coordinates so constant throughout simulation.
    * 
    * @param flatMesh
    * Flatten mesh of cloth such that z-position == 0 across mesh.
    */
   protected void initMeshUVTriangleLengths(PolygonalMesh flatMesh) {
      mU1s = new double[flatMesh.numFaces()];
      mV1s = new double[flatMesh.numFaces()];
      mU2s = new double[flatMesh.numFaces()];
      mV2s = new double[flatMesh.numFaces()];
      for (Face tri : flatMesh.getFaces()) {
         Vertex3d[] triVtxs = tri.getTriVertices();
         Vector3d x0 = triVtxs[0].pnt;
         Vector3d x1 = triVtxs[1].pnt;
         Vector3d x2 = triVtxs[2].pnt;
         
         mU1s[tri.idx] = x1.x - x0.x;
         mV1s[tri.idx] = x1.y - x0.y;
         mU2s[tri.idx] = x2.x - x0.x;
         mV2s[tri.idx] = x2.y - x0.y;
      }
   }
   
   
   /**
    * Compute the area of every triangle of the cloth mesh in UV coordinates.
    * Relies on UV coordinates so constant throughout simulation.
    * 
    * Follows equation (1).
    * 
    * @param flatMesh
    * Flatten mesh of cloth such that z-position == 0 across mesh.
    */
   protected void initMeshUVTriangleArea(PolygonalMesh flatMesh) {
      mUVAreas = new double[ flatMesh.numFaces() ];
      for (Face tri : flatMesh.getFaces()) {
         Vector3d uv1 = new Vector3d( mU1s[tri.idx], mV1s[tri.idx], 0 );
         Vector3d uv2 = new Vector3d( mU2s[tri.idx], mV2s[tri.idx], 0 );
         mUVAreas[tri.idx] = 0.5 * uv1.cross(uv2).norm();
      }
   }
   
   
   /**
    * Compute the wu and wv derivatives for each triangle. Relies on UV
    * coordinates so constant throughout simulation.
    * 
    * See comment block above this.mDWuxdxmx and this.mDWvxdxmx details.
    * 
    * Follows equation (6) to (12).
    */
   protected void initWuWvDerivatives() {
      mDWuxdxmx = new double[mMesh.numFaces()][3];
      mDWvxdxmx = new double[mMesh.numFaces()][3];

      for (int t = 0; t < mMesh.numFaces(); t++) {
         double u1 = mU1s[t];
         double v1 = mV1s[t];
         double u2 = mU2s[t];
         double v2 = mV2s[t];
      
         mDWuxdxmx[t][0] = (v1-v2) / (u1*v2 - u2*v1);
         mDWuxdxmx[t][1] = (v2) / (u1*v2 - u2*v1);
         mDWuxdxmx[t][2] = (-v1) / (u1*v2 - u2*v1);
         
         mDWvxdxmx[t][0] = (u2-u1) / (u1*v2 - u2*v1);
         mDWvxdxmx[t][1] = (-u2) / (u1*v2 - u2*v1);
         mDWvxdxmx[t][2] = (u1) / (u1*v2 - u2*v1);
      }
   }
 
   
   /**
    * Initialize list of common triangle edges. A common triangle edge
    * is defined to be an edge such that two triangles share that same edge.
    * 
    * See comment block above this.mTriPairs for details.
    * 
    * @param mesh
    * Mesh of triangle faces.
    */
   protected void initTriPairs(PolygonalMesh mesh) {
      /* Unique list of common triangle edges */
      mTriPairs = new ArrayList<HalfEdge>();
      
      /* Example: triHalfEdges[3][1] means get triangle #3's edge #1 */
      HalfEdge[][] triHalfEdges = new HalfEdge[mesh.numFaces()][3];
      
      /* Initialize triHalfEdges[][] and clear visited flag for each edge */
      for (Face tri : mesh.getFaces()) {
         triHalfEdges[tri.idx][0] = tri.getEdge(0);
         triHalfEdges[tri.idx][1] = tri.getEdge(1);
         triHalfEdges[tri.idx][2] = tri.getEdge(2);
         
         triHalfEdges[tri.idx][0].clearVisited();
         triHalfEdges[tri.idx][1].clearVisited();
         triHalfEdges[tri.idx][2].clearVisited();
      }
         
      /* For each triangle edge */
      for (int t = 0; t < mesh.numFaces(); t++) {
         for (int e = 0; e < 3; e++) {
            HalfEdge edge = triHalfEdges[t][e];
            HalfEdge oppEdge = edge.opposite;
            
            /* If this triangle edge hasn't been visited before and is shared
             * between two triangles, add triangle edge to common triangle 
             * edge list. */
            if (!edge.isVisited() && oppEdge != null) {
               mTriPairs.add(edge);
               edge.setVisited();
               oppEdge.setVisited();
            }
         }
      }
   }
   
   
   /**
    * Determine the four vertices of each triangle pair.
    * 
    * See comment block above this.mTriPairVtxs for details.
    */
   protected void initTriPairVtxs() {
      mTriPairVtxs = new Vertex3d[mTriPairs.size()][4];
      
      for (int p = 0; p < mTriPairs.size(); p++) {
         HalfEdge triPair = mTriPairs.get(p);
         
         Face tri0 = triPair.getFace();
         Face tri1 = triPair.getOppositeFace();
         
         Vertex3d[] tri0Vtxs = tri0.getTriVertices();
         Vertex3d[] tri1Vtxs = tri1.getTriVertices();
         
         Vertex3d x0 = null;
         Vertex3d x1 = null;
         Vertex3d x2 = null;
         Vertex3d x3 = null;
         
         /* Let x0 of tri pair be uncommon vertex of tri0 */
         for (int v0 = 0; v0 < 3; v0++) {
            int numV0V1matches = 0;
            for (int v1 = 0; v1 < 3; v1++) {
               if (tri0Vtxs[v0] == tri1Vtxs[v1]) {
                  numV0V1matches++;
               }
            }
            /* If this vertex of tri0 didn't match with any vertex of tri1 */
            if (numV0V1matches == 0) {
               x0 = tri0Vtxs[v0];
               break;
            }
         }
         
         if (x0 == null) {
            throw new RuntimeException("Couldn't find first uncommon vertex of "
                                       + "triangle pair.");
         }
         
         /* Let x1 and x2 be common edge vertices */
         for (int v0 = 0; v0 < 3; v0++) {
            if (tri0Vtxs[v0] != x0) {
               if (x2 == null) {
                  x2 = tri0Vtxs[v0];
               }
               else if (x1 == null) {
                  x1 = tri0Vtxs[v0];
               }
            }
         }
         
         /* Let x3 of tri pair be uncommon vertex of tri1 */
         for (int v1 = 0; v1 < 3; v1++) {
            if (tri1Vtxs[v1] != x1 && tri1Vtxs[v1] != x2) {
               x3 = tri1Vtxs[v1];
               break;
            }
         }
         
         if (x3 == null) {
            throw new RuntimeException("Couldn't find second uncommon vertex of "
                                       + "triangle pair.");
         }
         
         mTriPairVtxs[p][0] = x0;
         mTriPairVtxs[p][1] = x1;
         mTriPairVtxs[p][2] = x2;
         mTriPairVtxs[p][3] = x3;
      }
   }
   
  
   
   
   
   
   
   /**
    * Align vertices of mesh with their corresponding particles .
    * 
    * Also enforce mMesh position to be fixed (but its particles and vertices
    * can still move around).
    */
   @Override
   public void render (Renderer renderer, int flags) {
      super.render(renderer, flags);
      for (int i = 0; i < mParticles.length; i++) {
         Particle p = mParticles[i];
         Vertex3d v = mMesh.getVertex(i);
         v.setPosition(p.getPosition());
      }
      mMesh.notifyVertexPositionsModified();
      mClothRB.setPosition( mClothRB_fixedPos );
   }
   
   /**
    * Call applyForces() at every time step.
    */
   @Override
   protected void doadvance(double t0, double t1, int flags) {
      super.doadvance(t0,t1,flags);
      applyForces(0);
   }

   /**
    * Zero both the internal and external forces for every particle
    */
   protected void resetForces() {
      for (Particle p : mParticles) {
         p.zeroExternalForces();
         p.zeroForces();
      }
   }
   
   /**
    * Enforce a maximum magnitude for the particle forces
    */
   protected void capForces() {
      for (Particle p : mParticles) {
         if (p.getExternalForce().norm() > mMaxForceMag) {
            p.getExternalForce().normalize();
            p.setScaledExternalForce(mMaxForceMag, p.getExternalForce());
         }
      }
   }
   
   
   

   /**
    * Called by doadvance() at every time step to compute and apply the 
    * cloth particle forces. 
    */
   @Override
   public void applyForces (double t) {
      computeWuWv();
      computeAdjTriPairNormalsAndCommonEdges();

      resetForces();
      
      applyStretchForces();
      applyShearForces();
      applyBendForces();
      
      capForces();
   }

   @Override
   public void addSolveBlocks (SparseNumberedBlockMatrix M) {
      // TODO Auto-generated method stub
   }

   @Override
   public void addPosJacobian (SparseNumberedBlockMatrix M, double s) {
      // TODO Auto-generated method stub
      // TODO This is where you compute equation (8) and (12) of the original
      //      paper.
   }

   @Override
   public void addVelJacobian (SparseNumberedBlockMatrix M, double s) {
      // TODO Auto-generated method stub
      // TODO This is where you compute the 2nd equation below equation (12) of 
      ///     of the original paper.
   }

   @Override
   public int getJacobianType () {
      // TODO Auto-generated method stub
      return 0;
   }
   
   
   
   
   /**
    * Compute wu and wv for each triangle.
    * 
    * Follows equation (3) and (4).
    */
   protected void computeWuWv() {
      /* Initialize memory for wu and wv if haven't done so */
      if (mWus == null || mWvs == null) {
         mWus = new Vector3d[ mMesh.numFaces() ];
         mWvs = new Vector3d[ mMesh.numFaces() ];
         for (int t = 0; t < mMesh.numFaces(); t++) {
            mWus[t] = new Vector3d();
            mWvs[t] = new Vector3d();
         }
      }
      
      for (Face tri : mMesh.getFaces()) {
         Vertex3d[] triVtxs = tri.getTriVertices();
         /* Note that ordering of x0, x1, x2 below doesn't matter as long as
          * you are consistent throughout the code. */
         Vector3d x0 = triVtxs[0].pnt;
         Vector3d x1 = triVtxs[1].pnt;
         Vector3d x2 = triVtxs[2].pnt;
         double u1 = mU1s[tri.idx];
         double v1 = mV1s[tri.idx];
         double u2 = mU2s[tri.idx];
         double v2 = mV2s[tri.idx];
         
         /* Compute wu */
         
         Vector3d wuTerm1 = new Vector3d();
         wuTerm1.sub(x1, x0);
         wuTerm1.scale(v2);
         
         Vector3d wuTerm2 = new Vector3d();
         wuTerm2.sub(x2, x0);
         wuTerm2.scale(v1);
         
         double wuDenominator = u1*v2 - u2*v1;
         
         Vector3d wu = mWus[tri.idx];
         wu.sub(wuTerm1, wuTerm2);
         wu.scale(1/wuDenominator);

         /* Compute wv */
         
         Vector3d wvTerm1 = new Vector3d();
         wvTerm1.sub(x1,x0);
         wvTerm1.scale(-u2);
         
         Vector3d wvTerm2 = new Vector3d();
         wvTerm2.sub(x2,x0);
         wvTerm2.scale(u1);
         
         double wvDenominator = u1*v2 - u2*v1;
         
         Vector3d wv = mWvs[tri.idx];
         wv.add(wvTerm1, wvTerm2);
         wv.scale(1/wvDenominator);
      }
   }
   
   
   /**
    * Compute the two unit normals and single common unit edge for each triangle 
    * pair. 
    * 
    * Magnitude are saved before the normal and edge vectors are normalized into 
    * unit vectors.
    * 
    * See comments above this.mTriPairUnitNormals, this.mTriPairCommonUnitEdges,
    * this.mTriPairNormalMags, and this.mTriPairCommonEdgeMags.
    * 
    * Follows equations in (24).
    */
   protected void computeAdjTriPairNormalsAndCommonEdges() {
      /* Initialize memory for normals and edges if haven't done so. */
      if (mTriPairUnitNormals == null || mTriPairCommonUnitEdges == null) {
         mTriPairUnitNormals = new Vector3d[mTriPairs.size()][2];
         mTriPairCommonUnitEdges = new Vector3d[mTriPairs.size()];
         
         for (int p = 0; p < mTriPairs.size(); p++) {
            mTriPairUnitNormals[p][0] = new Vector3d();
            mTriPairUnitNormals[p][1] = new Vector3d();
            mTriPairCommonUnitEdges[p] = new Vector3d();
         }
         
         mTriPairNormalMags = new double[mTriPairs.size()][2];
         mTriPairCommonEdgeMags = new double[mTriPairs.size()];
      }
      
      /* For each pair */
      for (int p = 0; p < mTriPairs.size(); p++) {
         /* Get four vertices of pair */
         Vector3d x0 = mTriPairVtxs[p][0].pnt;
         Vector3d x1 = mTriPairVtxs[p][1].pnt;
         Vector3d x2 = mTriPairVtxs[p][2].pnt;
         Vector3d x3 = mTriPairVtxs[p][3].pnt;
         
         /* Compute unit normal of 1st triangle of tri pair. Equation (25) */
         
         Vector3d unitNormalA_term1 = new Vector3d();
         unitNormalA_term1.sub(x2, x0);
         
         Vector3d unitNormalA_term2 = new Vector3d();
         unitNormalA_term2.sub(x1, x0);
         
         mTriPairUnitNormals[p][0].cross(unitNormalA_term1, unitNormalA_term2);
         
         /* Save magnitude */
         mTriPairNormalMags[p][0] = mTriPairUnitNormals[p][0].norm();
         
         /* Finally normalize into unit vector */
         mTriPairUnitNormals[p][0].scale( 1/mTriPairNormalMags[p][0] );
         
         /* Compute unit normal of 2nd triangle of tri pair. Equation (26) */
         
         Vector3d unitNormalB_term1 = new Vector3d();
         unitNormalB_term1.sub(x1, x3);
         
         Vector3d unitNormalB_term2 = new Vector3d();
         unitNormalB_term2.sub(x2, x3);
         
         mTriPairUnitNormals[p][1].cross(unitNormalB_term1, unitNormalB_term2);
         
         /* Save magnitude */
         mTriPairNormalMags[p][1] = mTriPairUnitNormals[p][1].norm();
         
         /* Finally normalize into unit vector */
         mTriPairUnitNormals[p][1].scale( 1/mTriPairNormalMags[p][1] );
         
         /* Compute unit common edge of tri pair */
         
         mTriPairCommonUnitEdges[p].sub(x1, x2);
         
         /* Save magnitude */
         mTriPairCommonEdgeMags[p] = mTriPairCommonUnitEdges[p].norm();
         
         /* Finally normalize into unit vector */
         mTriPairCommonUnitEdges[p].scale( 1/mTriPairCommonEdgeMags[p] );
      }
   }
   

   

   
   
   
   
   
   /**
    * Compute and apply the stretch force exerted on the particles. Damping
    * for the stretch forces will also be included. 
    * 
    * If a triangle is stretched or compressed, the particles of the triangle
    * will exert their stretch forces upon themselves to restore the 
    * triangle's rest state (i.e. no stretch or compression).
    * 
    * Follows equation (7) of original paper for stretch forces.
    */
   protected void applyStretchForces() {
      for (Face tri : mMesh.getFaces()) {
         Vector2d C = computeStretch_C(tri);
         
         Vertex3d[] triVtxs = tri.getTriVertices();
         for (int m = 0; m < 3; m++) {
            Matrix3x2 dC = computeStretch_dC(tri, m);
            
            /* Force of particle = -k * dCdp * C */
            Vector3d stretchForce = new Vector3d(
               -mK_stretch,-mK_stretch,-mK_stretch);
            stretchForce.x *= dC.m00*C.x + dC.m01*C.y;
            stretchForce.y *= dC.m10*C.x + dC.m11*C.y;
            stretchForce.z *= dC.m20*C.x + dC.m21*C.y;
            
            int v = triVtxs[m].getIndex();
            Particle p = mParticles[v];
            p.addExternalForce(stretchForce);
            
            /* Apply associated damping force.
             *  d = -k * dC * [ (dC)^T * dxdt ]
             * where d is the damping force of a particle.
             * where k is the stretch damping stiffness.
             * where T is the transpose symbol. */
            
            Vector3d stretchDampForce = new Vector3d(
               -mK_stretchDamp,-mK_stretchDamp,-mK_stretchDamp);
            
            // Compute (dC)^T
            Matrix2x3 dCT = new Matrix2x3();
            dCT.transpose(dC);
            
            // velo = dxdt = velocity
            Vector3d velo = p.getVelocity();
            
            // Cdot = (dC)^T * dxdt
            Vector2d Cdot = new Vector2d();
            Cdot.x = dCT.m00*velo.x + dCT.m01*velo.y + dCT.m02*velo.z;
            Cdot.y = dCT.m10*velo.x + dCT.m11*velo.y + dCT.m12*velo.z;
            
            // Compute -k * dC * [ (dC)^T * dxdt ]
            // i.e. -k * dCdp * Cdot
            stretchDampForce.x *= dC.m00*Cdot.x + dC.m01*Cdot.y;
            stretchDampForce.y *= dC.m10*Cdot.x + dC.m11*Cdot.y;
            stretchDampForce.z *= dC.m20*Cdot.x + dC.m21*Cdot.y;
            
            p.addExternalForce(stretchDampForce);
         }
      }
   }
   
   
   /**
    * Compute stretch condition C(x) where x is a triangle.
    * 
    * Follows equation (13).
    * 
    * @param tri
    * Triangle of mesh.
    * 
    * @return
    * 2x1 vector.
    */
   protected Vector2d computeStretch_C(Face tri) {
      double wuMag = mWus[tri.idx].norm();
      double wvMag = mWvs[tri.idx].norm();
      
      double alpha = mUVAreas[tri.idx] * 0.75;
      
      Vector2d C = new Vector2d();
      C.x = alpha * (wuMag - m_bu);
      C.y = alpha * (wvMag - m_bv);
              
      return C;
   }
   
   
   /**
    * Compute the 1st derivative of stretch condition C(x) where x is a 
    * triangle with respect to a particular vertex of the triangle.
    * 
    * Follows equation (14) and (16).
    * 
    * @param tri
    * Triangle of mesh.
    * 
    * @param m
    * Vertex index of triangle to respect by. [0,2].
    * 
    * @return
    * 3x2 matrix.
    */
   protected Matrix3x2 computeStretch_dC(Face tri, int m) {
      Matrix3x2 dC = new Matrix3x2();
      
      dC.setColumn(0, computeStretch_dCudxm(tri, m));
      dC.setColumn(1, computeStretch_dCvdxm(tri, m));
      
      return dC;
   }
   
   
   /**
    * Helper function for computeStretch_dC. Computes 1st column of the 3x2
    * dC matrix.
    * 
    * Follows equation (14).
    * 
    * @param tri
    * Triangle of mesh.
    * 
    * @param m
    * Vertex index of triangle to respect by. [0,2].
    * 
    * @return
    * 3x1 vector.
    */
   protected Vector3d computeStretch_dCudxm(Face tri, int m) {
      Vector3d dCudxm = new Vector3d();
      
      double alpha = mUVAreas[tri.idx] * 0.75;
      
      double dwuxdxmx = mDWuxdxmx[tri.idx][m];
      
      Vector3d wuHat = new Vector3d();
      wuHat.set( mWus[tri.idx] );
      wuHat.normalize();
      
      dCudxm.set( wuHat );
      dCudxm.scale( alpha );
      dCudxm.scale( dwuxdxmx );
      
      return dCudxm;
   }
   
   
   /**
    * Helper function for computeStretch_dC. Computes 2nd column of the 3x2
    * dC matrix.
    * 
    * Follows equation (16).
    * 
    * @param tri
    * Triangle of mesh.
    * 
    * @param m
    * Vertex index of triangle to respect by. [0,2].
    * 
    * @return
    * 3x1 vector.
    */
   protected Vector3d computeStretch_dCvdxm(Face tri, int m) {
      Vector3d dCvdxm = new Vector3d();
      
      double alpha = mUVAreas[tri.idx] * 0.75;
      
      double dwvxdxmx = mDWvxdxmx[tri.idx][m];
      
      Vector3d wvHat = new Vector3d();
      wvHat.set( mWvs[tri.idx] );
      wvHat.normalize();
      
      dCvdxm.set( wvHat );
      dCvdxm.scale( alpha );
      dCvdxm.scale( dwvxdxmx );
      
      return dCvdxm;  
   }
   
   
   /**
    * Compute and apply the shear forces exerted on the particles. Damping for 
    * the shear forces will also be included.
    * 
    * If a triangle is sheared (i.e. triangle angles not at their rest state),
    * the particles of the triangle will exert their shear forces upon
    * themselves to restore the triangle's rest angles.
    */
   protected void applyShearForces() {
      for (Face tri : mMesh.getFaces()) {
         double C = computeShear_C(tri);
      
         Vertex3d[] triVtxs = tri.getTriVertices();
         for (int m = 0; m < 3; m++) {
            Vector3d dC = computeShear_dC(tri, m);
            
            /* Force of particle = -k * dCdp * C */
            Vector3d shearForce = new Vector3d(
               -mK_shear, -mK_shear, -mK_shear);
            shearForce.x *= dC.x*C;
            shearForce.y *= dC.y*C;
            shearForce.z *= dC.z*C;
            
            Particle p = mParticles[triVtxs[m].getIndex()];
            p.addExternalForce(shearForce);
            
            /* Apply associated damping force.
             *  d = -k * dC * [ (dC)^T * dxdt ]
             * where d is the damping force of a particle.
             * where k is the stretch damping stiffness. 
             * where T is the transpose symbol. */
            
            Vector3d shearDampForce = new Vector3d(
               -mK_shearDamp, -mK_shearDamp, -mK_shearDamp);
            
            // Don't need to compute transpose of dC (i.e. (dC)^T) 
            // because dC is just a vector-3 and (dC)^T would just 
            // result in another vector-3.
            
            // v = dxdt = velocity
            Vector3d v = p.getVelocity();
            
            // Cdot = (dC)^T * dxdt
            double Cdot = dC.dot(v);
            
            // Compute -k * dC * [ (dC)^T * dxdt ]
            // i.e. -k * dCdp * Cdot
            shearDampForce.x *= dC.x*Cdot;
            shearDampForce.y *= dC.y*Cdot;
            shearDampForce.z *= dC.z*Cdot;
            
            p.addExternalForce(shearDampForce);
         }
      }
   }
   
   
   /**
    * Compute shear condition C(x) of mesh triangle x.
    * 
    * Follows equation (19).
    * 
    * @param tri
    * Triangle of mesh
    * 
    * @return
    * Scalar.
    */
   protected double computeShear_C(Face tri) {
      double alpha = mUVAreas[tri.idx] * 0.75;
      Vector3d wu = mWus[tri.idx];
      Vector3d wv = mWvs[tri.idx];
      
      return alpha * wu.dot(wv);
   }
   
   
   /**
    * Compute the 1st derivative of the shear condition C(x) of mesh triangle 
    * x with respect to a vertex of the triangle.
    * 
    * Follows equation (21).
    * 
    * @param tri
    * Triangle of mesh.
    * 
    * @param m
    * Vertex index of triangle to respect by. [0,2].
    * 
    * @return
    * 3x1 vector.
    */
   protected Vector3d computeShear_dC(Face tri, int m) {
      Vector3d dC = new Vector3d();
      dC.x = computeShear_dCdxms(tri, m, 0);
      dC.y = computeShear_dCdxms(tri, m, 1);
      dC.z = computeShear_dCdxms(tri, m, 2);

      return dC;
   }
   
   
   /**
    * Helper function for computeShear_dC(). Computes an element of the 
    * dC vector-3.
    * 
    * @param tri
    * Triangle of mesh.
    * 
    * @param m
    * Vertex index of triangle to respect by. [0,2].
    * 
    * @param s
    * Component index (e.g. x, y, or z) of vertex of triangle to respect 
    * by. [0,2].
    * 
    * @return
    * Scalar.
    */
   protected double computeShear_dCdxms(Face tri, int m, int s) {
      double alpha = mUVAreas[tri.idx] * 0.75;
      double dwudxmx = mDWuxdxmx[tri.idx][m];
      double wus = mWus[tri.idx].get(s);
      double wvs = mWvs[tri.idx].get(s);
      double dwvdxmx = mDWvxdxmx[tri.idx][m];
      
      return alpha * (dwudxmx*wvs + wus*dwvdxmx);
   }
   


   
   /**
    * Compute and apply the bend force exerted on the particles. Damping for
    * the bend forces will also be included.
    * 
    * For each adjacent pair of triangles, if the angle between the two
    * triangles isn't flat (e.g. 180 degrees), the particles of the triangle
    * will exert a bending force upon themselves to restore the angle back to 
    * 180 degrees. 
    * 
    * Fixed bug: Ensure mK_bend is large to overcome outside bending forces 
    * caused by shearing and stretching.
    */
   protected void applyBendForces() {
      for (int p = 0; p < mTriPairs.size(); p++) {
         double C = computeBend_C(p);
         for (int m = 0; m < 4; m++) {
            Vector3d dC = computeBend_dC(p, m);
            
            Vector3d bendForce = new Vector3d(-mK_bend, -mK_bend, -mK_bend);
            bendForce.x *= dC.x*C; 
            bendForce.y *= dC.y*C;
            bendForce.z *= dC.z*C;
             
            int particleIdx = mTriPairVtxs[p][m].getIndex();
            Particle particle = mParticles[particleIdx];
            particle.addExternalForce(bendForce);
            
            /* Apply associated damping force.
             *  d = -k * dC * [ (dC)^T * dxdt ]
             * where d is the damping force of a particle.
             * where k is the bend damping stiffness.
             * where T is the transpose symbol. */
             
            Vector3d bendDampForce = new Vector3d(
                    -mK_bendDamp, -mK_bendDamp, -mK_bendDamp);
             
            Vector3d Cdot = new Vector3d(particle.getVelocity());
            Cdot.x *= dC.x;
            Cdot.y *= dC.y;
            Cdot.z *= dC.z;
             
            bendDampForce.x *= dC.x*Cdot.x;
            bendDampForce.y *= dC.y*Cdot.y;
            bendDampForce.z *= dC.z*Cdot.z;
             
            particle.addExternalForce(bendDampForce);
         }
      }
   }
   
   
   /**
    * Compute bend condition C(x) where x is a triangle pair.
    * 
    * Follows equation (30).
    * 
    * @param p
    * Triangle pair index. this.TriPairs.get(p) should be valid.
    * 
    * @return
    * Scalar.
    */
   protected double computeBend_C(int p) {
      Vector3d unitNormalA = mTriPairUnitNormals[p][0];
      Vector3d unitNormalB = mTriPairUnitNormals[p][1];
      Vector3d unitCommonEdge = mTriPairCommonUnitEdges[p];
      
      double cosTheta = unitNormalA.dot(unitNormalB);
      
      Vector3d crossAB = new Vector3d();
      crossAB.cross(unitNormalA, unitNormalB);
      double sinTheta = crossAB.dot(unitCommonEdge);
      
      double C = Math.atan2(sinTheta, cosTheta);
      
      return C;
   }
   

   /**
    * Compute the 1st derivative of stretch condition C(x) where x is a
    * triangle pair with respect to a particular vertex of the triangle pair.
    * 
    * @param p
    * Triangle pair index. this.TriPairs.get(p) should be valid.
    * 
    * @param m
    * Vertex index of triangle pair to respect by. [0,3].
    * 
    * @return
    * 3x1 vector.
    */
   protected Vector3d computeBend_dC(int p, int m) {
      Vector3d dC = new Vector3d();
      dC.x = computeBend_dCdxms(p, m, 0);
      dC.y = computeBend_dCdxms(p, m, 1);
      dC.z = computeBend_dCdxms(p, m, 2);
      
      return dC;
   }
   
   
   /**
    * Helper function for computeBend_dC(). Computes an element of the dC
    * vector-3.
    * 
    * Follows equation (54).
    * 
    * @param p
    * Triangle pair index. this.TriPairs.get(p) should be valid.
    * 
    * @param m
    * Vertex index of triangle pair to respect by. [0,3].
    * 
    * @param s
    * Component index (e.g. x, y, or z) of vertex of triangle pair to respect 
    * by. [0,2].
    * 
    * @return
    * Scalar.
    */
   protected double computeBend_dCdxms(int p, int m, int s) {
      Vector3d unitNormalA = mTriPairUnitNormals[p][0];
      Vector3d unitNormalB = mTriPairUnitNormals[p][1];
      Vector3d unitCommonEdge = mTriPairCommonUnitEdges[p];
      
      double cosTheta = unitNormalA.dot(unitNormalB);
      
      Vector3d crossAB = new Vector3d();
      crossAB.cross(unitNormalA, unitNormalB);
      double sinTheta = crossAB.dot(unitCommonEdge);
      
      double dcosThetadxms = compute_dcosThetadxms(p, m, s);
      double dsinThetadxms = compute_dsinThetadxms(p, m, s);
      
      double dC = cosTheta*dsinThetadxms - sinTheta*dcosThetadxms;
      
      return dC;
   }
   
   
   /**
    * Helper function for computeBend_dCdxms(). Computes the 1st derivative of
    * cos theta with respect to a particular component of a vertex of a triangle
    * pair.
    * 
    * Follows equation (47).
    * 
    * @param p
    * Triangle pair index. this.TriPairs.get(p) should be valid.
    * 
    * @param m
    * Vertex index of triangle pair to respect by. [0,3].
    * 
    * @param s
    * Component index (e.g. x, y, or z) of vertex of triangle pair to respect 
    * by. [0,2].
    * 
    * @return
    * Scalar.
    */
   protected double compute_dcosThetadxms(int p, int m, int s) {
      Vector3d dnA = compute_dnAdxms(p, m, s);
      Vector3d nB = mTriPairUnitNormals[p][1];
      
      Vector3d nA = mTriPairUnitNormals[p][0];
      Vector3d dnB = compute_dnBdxms(p, m, s);
      
      double dcosTheta = dnA.dot(nB) * nA.dot(dnB);
      
      return dcosTheta;
   }
   
   
   /**
    * Helper function for computeBend_dCdxms(). Computes the 1st derivative of
    * sin theta with respect to a particular component of a vertex of a triangle
    * pair.
    * 
    * Follows equation (49).
    * 
    * @param p
    * Triangle pair index. this.TriPairs.get(p) should be valid.
    * 
    * @param m
    * Vertex index of triangle pair to respect by. [0,3].
    * 
    * @param s
    * Component index (e.g. x, y, or z) of vertex of triangle pair to respect 
    * by. [0,2].
    * 
    * @return
    * Scalar.
    */
   protected double compute_dsinThetadxms(int p, int m, int s) {
      Vector3d dnA = compute_dnAdxms(p, m, s);
      Vector3d nB = mTriPairUnitNormals[p][1];
      
      Vector3d nA = mTriPairUnitNormals[p][0];
      Vector3d dnB = compute_dnBdxms(p, m, s);
      
      Vector3d e = mTriPairCommonUnitEdges[p];
      Vector3d de = compute_dedxms(p, m, s);
      
      Vector3d leftCross = new Vector3d();
      leftCross.cross(dnA, nB);
      
      Vector3d rightCross = new Vector3d();
      rightCross.cross(nA, dnB);
      
      Vector3d addedCrosses = new Vector3d();
      addedCrosses.add(leftCross, rightCross);
      
      double leftTerm = addedCrosses.dot(e);
      
      Vector3d crossAB = new Vector3d();
      crossAB.cross(nA, nB);

      double rightTerm = crossAB.dot(de);
      
      double dsinTheta = leftTerm + rightTerm;
      
      return dsinTheta;
   }
   
   
   /**
    * Helper function for compute_dcosThetadxms() and compute_dsinThetadxms().
    * Computes the 1st order derivative of the unit normal of the 1st triangle
    * of a triangle pair with respect to a particular component of a vertex of
    * a triangle pair. 
    * 
    * Follows equation (40).
    * 
    * @param p
    * Triangle pair index. this.TriPairs.get(p) should be valid.
    * 
    * @param m
    * Vertex index of triangle pair to respect by. [0,3].
    * 
    * @param s
    * Component index (e.g. x, y, or z) of vertex of triangle pair to respect 
    * by. [0,2].
    * 
    * @return
    * Scalar.
    */
   protected Vector3d compute_dnAdxms(int p, int m, int s) {
      Vector3d x0 = mTriPairVtxs[p][0].pnt;
      Vector3d x1 = mTriPairVtxs[p][1].pnt;
      Vector3d x2 = mTriPairVtxs[p][2].pnt;
      
      Vector3d qAm = new Vector3d();
      if (m == 0) {
         qAm.sub(x2,x1);
      }
      else if (m == 1) {
         qAm.sub(x0,x2);
      }
      else if (m == 2) {
         qAm.sub(x1, x0);
      }
      else if (m == 3) {
         qAm.setZero();
      }
      
      Vector3d Ss = compute_Ss(qAm, s);
      Ss.scale( 1/mTriPairNormalMags[p][0] );
      
      return Ss;
   }
   
   
   /**
    * Helper function for compute_dcosThetadxms() and compute_dsinThetadxms().
    * Computes the 1st order derivative of the unit normal of the 2nd triangle
    * of a triangle pair with respect to a particular component of a vertex of
    * a triangle pair. 
    * 
    * Follows equation (41).
    * 
    * @param p
    * Triangle pair index. this.TriPairs.get(p) should be valid.
    * 
    * @param m
    * Vertex index of triangle pair to respect by. [0,3].
    * 
    * @param s
    * Component index (e.g. x, y, or z) of vertex of triangle pair to respect 
    * by. [0,2].
    * 
    * @return
    * Scalar.
    */
   protected Vector3d compute_dnBdxms(int p, int m, int s) {
      Vector3d x1 = mTriPairVtxs[p][1].pnt;
      Vector3d x2 = mTriPairVtxs[p][2].pnt;
      Vector3d x3 = mTriPairVtxs[p][3].pnt;
      
      Vector3d qBm = new Vector3d();
      if (m == 0) {
         qBm.setZero();
      }
      else if (m == 1) {
         qBm.sub(x2, x3);
      }
      else if (m == 2) {
         qBm.sub(x3, x1);
      }
      else if (m == 3) {
         qBm.sub(x1, x2);
      }
      
      Vector3d Ss = compute_Ss(qBm, s);
      Ss.scale( 1/mTriPairNormalMags[p][1] );
      
      return Ss;
   }
   

   /**
    * Get the s-th row of S(v) where v is a vector-3.
    * 
    * S(v) is defined in the 1st page of the 2nd paper.
    * 
    * @param v
    * Vector-3 to create the 3x3 skew-symmetric matrix of a vector cross
    * product.
    * 
    * @param s
    * Row index to extract from the 3x3 matrix.
    * 
    * @return
    * 3x1 vector. Transpose of extracted row.
    */
   protected Vector3d compute_Ss(Vector3d v, int s) {
      if (s == 0) {
         return new Vector3d(0, -v.z, v.y);
      }
      else if (s == 1) {
         return new Vector3d(v.z, 0, -v.x);
      }
      else if (s == 2) {
         return new Vector3d(-v.y, v.x, 0);
      }
      else {
         return null;
      }
   }
   
   
   /**
    * Helper function for compute_dsinThetadxms().
    * Computes the 1st order derivative of the unit common edge of the triangle 
    * pair with respect to a particular component of a vertex of a triangle
    * pair. 
    * 
    * Follows equation (42).
    * 
    * @param p
    * Triangle pair index. this.TriPairs.get(p) should be valid.
    * 
    * @param m
    * Vertex index of triangle pair to respect by. [0,3].
    * 
    * @param s
    * Component index (e.g. x, y, or z) of vertex of triangle pair to respect 
    * by. [0,2].
    * 
    * @return
    * Scalar.
    */
   protected Vector3d compute_dedxms(int p, int m, int s) {
      Matrix3d I = new Matrix3d();
      I.setIdentity();
      
      if (m == 0) {
         I.scale(0);
      }
      else if (m == 1) {
         I.scale(1);
      }
      else if (m == 2) {
         I.scale(-1);
      }
      else if (m == 3) {
         I.scale(0);
      }
      
      Vector3d de = new Vector3d();
      I.getColumn(s, de);
      
      de.scale( 1/mTriPairCommonEdgeMags[p] );
      
      return de;
   }
}
