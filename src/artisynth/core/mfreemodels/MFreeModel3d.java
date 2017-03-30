/**
 * Copyright (c) 2014, by the Authors: Antonio Sanchez (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.core.mfreemodels;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemNodeNeighbor;
import artisynth.core.femmodels.FemUtilities;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.materials.BulkIncompressibleBehavior;
import artisynth.core.materials.BulkIncompressibleBehavior.BulkPotential;
import artisynth.core.materials.ConstitutiveMaterial;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.IncompressibleBehavior;
import artisynth.core.materials.IncompressibleMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.SolidDeformation;
import artisynth.core.materials.ViscoelasticBehavior;
import artisynth.core.materials.ViscoelasticState;
import artisynth.core.mechmodels.Collidable;
import artisynth.core.mechmodels.CollidableDynamicComponent;
import artisynth.core.mechmodels.CollisionHandler;
import artisynth.core.mechmodels.ContactMaster;
import artisynth.core.mechmodels.ContactPoint;
import artisynth.core.mechmodels.DynamicComponent;
import artisynth.core.mechmodels.HasAuxState;
import artisynth.core.mechmodels.MechSystemModel;
import artisynth.core.mechmodels.MeshComponentList;
import artisynth.core.mechmodels.PointList;
import artisynth.core.modelbase.ComponentChangeEvent;
import artisynth.core.modelbase.ComponentChangeEvent.Code;
import artisynth.core.modelbase.CopyableComponent;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.ModelComponentBase;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.modelbase.TransformGeometryContext;
import artisynth.core.modelbase.TransformableGeometry;
import artisynth.core.util.ScalableUnits;
import maspack.function.ConstantFuntion3x1;
import maspack.function.Function3x1;
import maspack.geometry.AABBTree;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.BVFeatureQuery.ObjectDistanceCalculator;
import maspack.geometry.BVNode;
import maspack.geometry.BVTree;
import maspack.geometry.Boundable;
import maspack.geometry.GeometryTransformer;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.DenseMatrix;
import maspack.matrix.EigenDecomposition;
import maspack.matrix.Matrix;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3x1Block;
import maspack.matrix.Matrix3x3Block;
import maspack.matrix.Matrix6d;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixNd;
import maspack.matrix.NumericalException;
import maspack.matrix.Point3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SVDecomposition3d;
import maspack.matrix.SparseBlockMatrix;
import maspack.matrix.SparseMatrixNd;
import maspack.matrix.SparseNumberedBlockMatrix;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.matrix.VectorNi;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.properties.PropertyUtils;
import maspack.render.RenderList;
import maspack.render.Renderable;
import maspack.render.Renderer;
import maspack.render.color.ColorMapBase;
import maspack.render.color.HueColorMap;
import maspack.util.DoubleInterval;
import maspack.util.InternalErrorException;

public class MFreeModel3d extends FemModel implements TransformableGeometry,
ScalableUnits, MechSystemModel, Collidable, CopyableComponent {

   protected SparseBlockMatrix M = null;
   protected VectorNd b = null;

   protected PointList<MFreeNode3d> myNodes;
   // protected CountedList<MFreePoint3d> myEvaluationPoints;
   protected MFreeElement3dList myElements;
   protected MeshComponentList<MFreeMeshComp> myMeshList;

   protected AABBTree myElementTree;
   protected AABBTree myNodeTree;
   protected boolean myBVTreeValid;
   
   protected AABBTree myRestNodeTree; // rest nodes only
   
   protected MFreeMeshComp mySurfaceMesh;
   protected boolean mySurfaceMeshValid = false;
   protected int myCollidableIndex;
   
   // record inverted elements
   private double myMinDetJ; // used to record inverted elements
   private MFreeElement3d myMinDetJElement = null; // element with
   // "worst" DetJ

   public static boolean checkTangentStability = false;
   public static boolean abortOnInvertedElems = false;

   private int myNumInverted = 0; // counts number of inverted elements
   // static maspack.render.Material myInvertedMaterial =
   // maspack.render.Material.createDiffuse(1f, 0f, 0f, 0f, 32f);
   // private boolean myIncompressibleP = false;
   private double myIncompCompliance = 0;
   public static IncompMethod DEFAULT_HARD_INCOMP = IncompMethod.OFF;
   private IncompMethod myHardIncompMethod = DEFAULT_HARD_INCOMP;
   private boolean myHardIncompMethodValidP = false;
   public static IncompMethod DEFAULT_SOFT_INCOMP = IncompMethod.ELEMENT;
   private IncompMethod mySoftIncompMethod = DEFAULT_SOFT_INCOMP;
   private boolean mySoftIncompMethodValidP = false;

   // extra blocks in the solve matrix for soft nodel incomp stiffness;
   // needed for soft nodal incompressibility
   private boolean myNodalIncompBlocksAllocatedP = false;
   // incompressibility constraints attached to each FemNodeNeighbour;
   // needed for hard and soft nodal incompressibility
   private boolean myNodalIncompConstraintsAllocatedP = false;
   private boolean myHardIncompConfigValidP = false;
   private boolean myNodalRestVolumesValidP = false;
   //private boolean myHardIncompConstraintsChangedP = true;
   private double myHardIncompUpdateTime = -1;

   // total number of incompressibility constraints (GT.colSize(), not blocks)
   private int myNumIncompressConstraints = 0;

   private VectorNd myDg = null;
   private VectorNd myIncompressLambda = new VectorNd();
   
   // maximum number of pressure DOFs that can occur in an element
   private static int MAX_PRESSURE_VALS = 8;
   // maximum number of nodes for elements associated with nodal incompressibility
   private static int MAX_NODAL_INCOMP_NODES = 8;
   // temp space for computing pressures
   private VectorNd myPressures = new VectorNd(MAX_PRESSURE_VALS);
   private MatrixNd myRinv = new MatrixNd();
   // temp space for computing pressure stiffness
   private double[] myKp = new double[MAX_PRESSURE_VALS];
   // temp space for computing nodal incompressibility constraints
   private Vector3d[] myNodalConstraints = new Vector3d[MAX_NODAL_INCOMP_NODES];
   SVDecomposition3d SVD = new SVDecomposition3d();
   
   private static double DEFAULT_ELEMENT_WIDGET_SIZE = 0.0;
   private double myElementWidgetSize = DEFAULT_ELEMENT_WIDGET_SIZE;
   PropertyMode myElementWidgetSizeMode = PropertyMode.Inherited;

   protected MFreeAuxMaterialBundleList myAuxMaterialsList;

   protected boolean myClearMeshColoring = false;
   protected boolean myComputeNodalStress = false; /// XXX TODO: extrapmat in
   /// MFreeNode3d
   protected boolean myComputeNodalStrain = false;

   public static ColorMapBase defaultColorMap = createDefaultColorMap();
   protected ColorMapBase myColorMap;
   protected PropertyMode myColorMapMode = PropertyMode.Inherited;

   public static PropertyList myProps =
      new PropertyList(MFreeModel3d.class, FemModel.class);

   static {
      myProps.add(
         "softIncompMethod", "method of enforcing soft incompressibility",
         DEFAULT_SOFT_INCOMP);
      myProps.addInheritable(
         "elementWidgetSize:Inherited",
         "size of rendered widget in each element's center",
         DEFAULT_ELEMENT_WIDGET_SIZE, "[0,1]");
      myProps.addInheritable(
         "colorMap:Inherited", "color map for stress/strain", defaultColorMap,
         "CE");
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public void setElementWidgetSize(double size) {
      myElementWidgetSize = size;
      myElementWidgetSizeMode =
         PropertyUtils.propagateValue(
            this, "elementWidgetSize", myElementWidgetSize,
            myElementWidgetSizeMode);
   }

   public double getElementWidgetSize() {
      return myElementWidgetSize;
   }

   public void setElementWidgetSizeMode(PropertyMode mode) {
      myElementWidgetSizeMode =
         PropertyUtils.setModeAndUpdate(
            this, "elementWidgetSize", myElementWidgetSizeMode, mode);
   }

   public PropertyMode getElementWidgetSizeMode() {
      return myElementWidgetSizeMode;
   }

   @Override
   public void getCollidables(List<Collidable> list, int level) {
      list.add(this);
      // traverse forward for additional collidables (e.g. FemMeshComp)
      recursivelyGetLocalComponents(this, list, Collidable.class);
   }

   public void setComputeNodalStress(boolean enable) {
      if (enable != myComputeNodalStress) {
         myComputeNodalStress = enable;
         if (!enable) {
            // release memory used for computing stress
            for (MFreeNode3d n : myNodes) {
               n.setAvgStress(null);
            }
         }
      }
   }

   public void setComputeNodalStrain(boolean enable) {
      if (enable != myComputeNodalStrain) {
         myComputeNodalStrain = enable;
         if (!enable) {
            // release memory used for computing strain
            for (MFreeNode3d n : myNodes) {
               n.setAvgStrain(null);
            }
         }
      }
   }

   protected void setDefaultValues() {
      super.setDefaultValues();
      myDensity = DEFAULT_DENSITY;
      myStiffnessDamping = DEFAULT_STIFFNESS_DAMPING;
      myMassDamping = DEFAULT_MASS_DAMPING;
      myElementWidgetSize = DEFAULT_ELEMENT_WIDGET_SIZE;
      myElementWidgetSizeMode = PropertyMode.Inherited;
      myHardIncompMethod = DEFAULT_HARD_INCOMP;
      mySoftIncompMethod = DEFAULT_SOFT_INCOMP;
      setMaterial(new LinearMaterial());
      myColorMap = createDefaultColorMap();
   }

   public static ColorMapBase createDefaultColorMap() {
      return new HueColorMap(0.7, 0);
   }

   public MFreeModel3d() {
      this(null);
   }

   public MFreeModel3d(String name) {
      super(name);
      myNodalConstraints = new Vector3d[MAX_NODAL_INCOMP_NODES];
      for (int i = 0; i < MAX_NODAL_INCOMP_NODES; i++) {
         myNodalConstraints[i] = new Vector3d();
      }
      mySurfaceMesh = null;
      myRestNodeTree = null;      
   }

   protected void initializeChildComponents() {
      myNodes = new PointList<MFreeNode3d>(MFreeNode3d.class, "nodes", "n");
      myElements = new MFreeElement3dList("elements", "e");
      myAuxMaterialsList = new MFreeAuxMaterialBundleList("materials", "m");
      myMeshList =
         new MeshComponentList<MFreeMeshComp>(
            MFreeMeshComp.class, "geometry", "g");
      // myEvaluationPoints = new CountedList<MFreePoint3d>();

      addFixed(myNodes);
      addFixed(myElements);
      addFixed(myAuxMaterialsList);
      addFixed(myMeshList);

      super.initializeChildComponents();
   }

   public void addMaterialBundle(MFreeAuxMaterialBundle bundle) {
      if (!myAuxMaterialsList.contains(bundle)) {
         for (MFreeAuxMaterialElementDesc d : bundle.getElements()) {
            bundle.checkElementDesc(this, d);
         }
         myAuxMaterialsList.add(bundle);
      }
   }

   public boolean removeMaterialBundle(MFreeAuxMaterialBundle bundle) {
      return myAuxMaterialsList.remove(bundle);
   }

   public void clearMaterialBundles() {
      myAuxMaterialsList.removeAll();
   }

   public RenderableComponentList<MFreeAuxMaterialBundle> getMaterialBundles() {
      return myAuxMaterialsList;
   }

   protected void buildBVHierarchies() {
      myElementTree = new AABBTree();
      Boundable[] elements = new Boundable[numElements()];
      for (int i = 0; i < elements.length; i++) {
         elements[i] = myElements.get(i);
      }
      myElementTree.build(elements, numElements());

      myNodeTree = new AABBTree();
      Boundable[] nodes = myNodes.toArray(new Boundable[numNodes()]);
      myNodeTree.build(nodes, numNodes());

      myBVTreeValid = true;
   }
   
   private void updateBVHierarchies() {
      if (!myBVTreeValid) {
         myNodeTree.update();
         myElementTree.update();
         myBVTreeValid = true;
      }
   }

   public BVTree getNodeBVTree() {
      if (myNodeTree == null) {
         buildBVHierarchies();
      } else if (!myBVTreeValid) {
         updateBVHierarchies();
      }
      return myNodeTree;
   }

   public BVTree getElementBVTree() {
      if (myElementTree == null) {
         buildBVHierarchies();
      } else if (!myBVTreeValid) {
         updateBVHierarchies();
      }
      return myElementTree;
   }

   public PointList<MFreeNode3d> getNodes() {
      return myNodes;
   }

   public MFreeNode3d getNode(int idx) {
      return myNodes.get(idx);
   }

   @Override
   public MFreeNode3d getByNumber(int num) {
      return myNodes.getByNumber(num);
   }

   public MFreeElement3d getElementByNumber(int num) {
      return myElements.getByNumber(num);
   }

   @Override
   public RenderableComponentList<MFreeElement3d> getElements() {
      return myElements;
   }

   public void addNodes(List<MFreeNode3d> nodes) {
      myNodes.addAll(nodes);
   }

   public void addNode(MFreeNode3d p) {
      myNodes.add(p);
   }

   public void addNumberedNode(MFreeNode3d p, int number) {
      myNodes.addNumbered(p, number);
   }

   public boolean removeNode(MFreeNode3d p) {

      // check if any elements depend on this node
      LinkedList<MFreeElement3d> elems = p.getMFreeElementDependencies();
      for (MFreeElement3d elem : elems) {
         if (myElements.contains(elem)) {
            System.err.println("Error: unable to remove node because some elements still depend on it");
            return false;
         }
      }

      // make sure no surfaces depend on it
      for (MFreeMeshComp fm : myMeshList) {
         if (fm.hasNodeDependency(p)) {
            System.err.println("Error: unable to remove node because the mesh '" 
               + fm.getName() + "' still depends on it");
            return false;
         }
      }

      if (myNodes.remove(p)) {
         return true;
      }
      return false;
   }

   @Override
   public MFreeElement3d getElement(int idx) {
      return myElements.get(idx);
   }

   public void addElements(List<MFreeElement3d> regionList) {
      for (MFreeElement3d region : regionList) {
         addElement(region);
      }
   }

   public void addElement(MFreeElement3d e) {
      myElements.add(e);
   }

   public void addNumberedElement(MFreeElement3d e, int elemId) {
      myElements.addNumbered(e, elemId);
   }

   public boolean removeElement(MFreeElement3d e) {
      return myElements.remove(e);
   }

   public void clearElements() {
      myElements.removeAll();
      for (int i = 0; i < myNodes.size(); i++) {
         myNodes.get(i).setMass(0);
      }
   }

   public LinkedList<FemNodeNeighbor> getNodeNeighbors(FemNode3d node) {
      return node.getNodeNeighbors();
   }

   private LinkedList<FemNodeNeighbor> myEmptyNeighborList =
      new LinkedList<FemNodeNeighbor>();

   protected LinkedList<FemNodeNeighbor> getIndirectNeighbors(FemNode3d node) {
      LinkedList<FemNodeNeighbor> indirect;
      if ((indirect = node.getIndirectNeighbors()) != null) {
         return indirect;
      } else {
         // returning a default empty list if indirect == null
         return myEmptyNeighborList;
      }
   }

   public LinkedList<MFreeElement3d> getElementNeighbors(MFreeNode3d node) {
      return node.getMFreeElementDependencies();
   }

   @Override
   protected void updateNodeForces(double t) {
      if (!myStressesValidP) {
         updateStressAndStiffness();
      }
      boolean hasGravity = !myGravity.equals(Vector3d.ZERO);
      Vector3d fk = new Vector3d();
      Vector3d fd = new Vector3d();

      // gravity, internal and mass damping
      for (MFreeNode3d n : myNodes) {
         if (hasGravity) {
            n.addScaledForce(n.getMass(), myGravity);
         }
         fk.set(n.getInternalForce());
         fd.setZero();
         if (myStiffnessDamping != 0) {
            for (FemNodeNeighbor nbr : getNodeNeighbors(n)) {
               nbr.addDampingForce(fd);
            }
            // used for soft nodal-based incompressibility
            for (FemNodeNeighbor nbr : getIndirectNeighbors(n)) {
               nbr.addDampingForce(fd);
            }
            fd.scale(myStiffnessDamping);
         }
         fd.scaledAdd(myMassDamping * n.getMass(), n.getFalseVelocity(), fd);
         n.subForce(fk);
         n.subForce(fd);
      }

      // if (stepAdjust != null && myMinDetJ <= 0) {
      // stepAdjust.recommendAdjustment(0.5, "element inversion");
      // }
   }

   private void computePressuresAndRinv(
      MFreeElement3d e, FemMaterial mat, double scale) {

      BulkIncompressibleBehavior imat = mat.getIncompressibleBehavior();
      
      int npvals = e.numPressureVals();

      myRinv.setSize(npvals, npvals);
      myPressures.setSize(npvals);

      double[] pbuf = myPressures.getBuffer();
      double restVol = e.getRestVolume();

      if (npvals > 1) {
         myPressures.setZero();
         MFreeIntegrationPoint3d[] ipnts = e.getIntegrationPoints();
         IntegrationData3d[] idata = e.getIntegrationData();
         
         if (imat.getBulkPotential() != BulkPotential.QUADRATIC) {
            myRinv.setZero();
         }
         for (int k = 0; k < ipnts.length; k++) {
            MFreeIntegrationPoint3d pt = ipnts[k];
            pt.computeJacobian(e.getNodes());
            double detJ0 = idata[k].getDetJ0();
            double detJ = pt.getJ().determinant() / detJ0;
            double dV = detJ0 * pt.getWeight();
            double[] H = pt.getPressureWeights().getBuffer();
            for (int i = 0; i < npvals; i++) {
               pbuf[i] += H[i] * imat.getEffectivePressure(detJ) * dV;
            }
            if (imat.getBulkPotential() != BulkPotential.QUADRATIC) {
               double mod = imat.getEffectiveModulus(detJ);
               for (int i = 0; i < npvals; i++) {
                  for (int j = 0; j < npvals; j++) {
                     myRinv.add(i, j, H[i] * H[j] * mod * dV);
                  }
               }
            }
         }
         Matrix W = e.getPressureWeightMatrix();
         W.mul(myPressures, myPressures);
         myPressures.scale(1 / restVol);
         if (imat.getBulkPotential() == BulkPotential.QUADRATIC) {
            myRinv.set(W);
            myRinv.scale(scale*imat.getBulkModulus() / restVol);
         }
         else {
            // optimize later
            MatrixNd Wtmp = new MatrixNd(W);
            Wtmp.scale(scale / restVol);
            myRinv.mul(Wtmp);
            myRinv.mul(Wtmp, myRinv);
         }
      }
      else {
         double Jpartial = e.getVolume() / e.getRestVolume();
         pbuf[0] = (imat.getEffectivePressure(Jpartial) +
            0 * e.myLagrangePressures[0]);
         myRinv.set(0, 0, scale*imat.getEffectiveModulus(Jpartial) / restVol);
      }
   }
   
   private boolean hasActiveNodes() {
      for (int i = 0; i < myNodes.size(); i++) {
         if (myNodes.get(i).isActive()) {
            return true;
         }
      }
      return false;
   }
   
   private void configureHardIncomp() {
      if (!hasActiveNodes()) {
         return;
      }
      IncompMethod method = getHardIncompMethod();
      if (method == IncompMethod.NODAL) {
         configureHardNodalIncomp();
      }
      else if (method == IncompMethod.ELEMENT) {
         configureHardElementIncomp();
      }
      else {
         throw new IllegalArgumentException(
            "unsupported hard incompressibility method " + method);
      }
      myDg = new VectorNd(myNumIncompressConstraints);
      myHardIncompConfigValidP = true;
      //myHardIncompConstraintsChangedP = true;
   }

   private boolean setNodalIncompBlocksAllocated(boolean allocated) {
      if (myNodalIncompBlocksAllocatedP != allocated) {
         for (MFreeNode3d n : myNodes) {
            if (allocated) {
               for (FemNodeNeighbor nbr_i : getNodeNeighbors(n)) {
                  MFreeNode3d node_i = (MFreeNode3d)nbr_i.getNode();
                  for (FemNodeNeighbor nbr_j : getNodeNeighbors(n)) {
                     MFreeNode3d node_j = (MFreeNode3d)nbr_j.getNode();
                     if (node_i.getNodeNeighbor(node_j) == null &&
                     node_i.getIndirectNeighbor(node_j) == null) {
                        // System.out.println (
                        // "adding block at "+node_i.getSolveIndex()+" "+
                        // node_j.getSolveIndex());
                        node_i.addIndirectNeighbor(node_j);
                     }
                  }
               }
            }
            else {
               n.clearIndirectNeighbors();
            }
         }
         // XXX signal structure change for solve matrix
         myNodalIncompBlocksAllocatedP = allocated;
         return true;
      }
      else {
         return false;
      }
   }

   private void setNodalIncompConstraintsAllocated(boolean allocated) {
      if (myNodalIncompConstraintsAllocatedP != allocated) {
         for (FemNode3d n : myNodes) {
            if (allocated) {
               for (FemNodeNeighbor nbr : getNodeNeighbors(n)) {
                  nbr.setDivBlk(new Matrix3x1Block());
               }
            }
            else {
               for (FemNodeNeighbor nbr : getNodeNeighbors(n)) {
                  nbr.setDivBlk(null);
               }
            }
         }
         myNodalIncompConstraintsAllocatedP = allocated;
      }
   }

   private boolean volumeIsControllable(FemNode3d node) {
      return node.isActive();
   }
   
   private boolean hasControllableNodes(MFreeElement3d elem) {
      return elem.hasControllableNodes();
   }
   
   private void configureHardNodalIncomp() {

      if (!hasActiveNodes()) {
         return;
      }

      // determine which nodes have which controllablity
      int ci = 0; // constraint index
      for (FemNode3d n : myNodes) {
         int idx = -1;
         // XXX for now, only enforce incompressibility around nodes that are
         // free. That decreases the accuracy but increases the chance that the
         // resulting set of constraints will have full rank.
         // if (n.isActive()) {
         // idx = ci++;
         // }

         // This is what we should do, but it can lead to rank-deficient
         // constraint sets:
         if (volumeIsControllable(n)) {
            idx = ci++;
         }
         n.setIncompressIndex(idx);
      }
      myNumIncompressConstraints = ci;
      myIncompressLambda.setSize(ci);
      myIncompressLambda.setZero();

      setNodalIncompConstraintsAllocated(true);
   }

   private void configureHardElementIncomp() {

      int ci = 0; // constraint index
      for (MFreeElement3d e : myElements) {
         int npvals = e.numPressureVals();
         int cidx = -1;
         if (hasControllableNodes(e)) {
            cidx = ci;
            ci += npvals;
         }
         e.setIncompressIndex(cidx);
         for (int i = 0; i < npvals; i++) {
            e.myLagrangePressures[i] = 0;
         }
      }
      myNumIncompressConstraints = ci;
   }
   
   private void updateNodalRestVolumes() {

      for (MFreeNode3d n : myNodes) {
         n.myRestVolume = 0;
      }
      for (MFreeElement3d e : myElements) {
         MFreeNode3d[] nodes = e.getNodes();
         
         IntegrationData3d[] idata = e.getIntegrationData();
         MFreeIntegrationPoint3d[] ipnts = e.getIntegrationPoints();
         for (int k=0; k<ipnts.length; ++k) {
            VectorNd N = ipnts[k].getShapeWeights();
            for (int i = 0; i < nodes.length; i++) {
               nodes[i].myRestVolume += ipnts[k].getWeight()* idata[k].getDetJ0()*N.get(i);
            }
         }
      }
      myNodalRestVolumesValidP = true;
   }
   
   private void updateNodalPressures(IncompressibleMaterial imat) {
      for (MFreeNode3d n : myNodes) {
         n.myVolume = 0;
      }
      for (MFreeElement3d e : myElements) {
         MFreeNode3d[] nodes = e.getNodes();
         MFreeIntegrationPoint3d[] ipnts = e.getIntegrationPoints();
         IntegrationData3d[] idata = e.getIntegrationData();
         for (int k=0; k<ipnts.length; ++k) {
            VectorNd N = ipnts[k].getShapeWeights();
            for (int i = 0; i < nodes.length; i++) {
               nodes[i].myVolume += idata[k].getDv()*N.get(i);
            }
         }
      }
      for (MFreeNode3d n : myNodes) {
         if (volumeIsControllable(n)) {
            n.myPressure = imat.getEffectivePressure(n.myVolume / n.myRestVolume);
         }
         else {
            n.myPressure = 0;
         }
      }
   }

   private boolean requiresWarping(MFreeElement3d elem, FemMaterial mat) {
      if (mat.isCorotated()) {
         return true;
      }
      if (elem.numAuxiliaryMaterials() > 0) {
         for (ConstitutiveMaterial aux : elem.getAuxiliaryMaterials()) {
            if (aux.isCorotated()) {
               return true;
            }
         }
      }
      return false;
   }
   
   // DIVBLK
   public void updateStressAndStiffness() {

      // allocate or deallocate nodal incompressibility blocks
      setNodalIncompBlocksAllocated (getSoftIncompMethod()==IncompMethod.NODAL);
      
      for (MFreeNode3d n : myNodes) {
         n.getInternalForce().setZero();
         if (!myStiffnessesValidP) {
            for (FemNodeNeighbor nbr : getNodeNeighbors(n)) {
               nbr.zeroStiffness();
            }
            for (FemNodeNeighbor nbr : getIndirectNeighbors(n)) {
               nbr.zeroStiffness();
            }
         }
         if (myComputeNodalStress) {
            n.zeroStress();
         }
         if (myComputeNodalStrain) {
            n.zeroStrain();
         }
      }

      if (!myVolumeValid) {
         updateJacobians();
         updateVolume();
      }

      IncompMethod softIncomp = getSoftIncompMethod();

      if (softIncomp == IncompMethod.NODAL) {
         if (!myNodalRestVolumesValidP) {
            updateNodalRestVolumes();
         }
         setNodalIncompConstraintsAllocated(true);
         updateNodalPressures((IncompressibleMaterial)myMaterial);
         for (FemNode3d n : myNodes) {
            for (FemNodeNeighbor nbr : getNodeNeighbors(n)) {
               nbr.getDivBlk().setZero();
            }
         }
      }
      
      Matrix6d D = new Matrix6d();
      
      myMinDetJ = Double.MAX_VALUE;
      myMinDetJElement = null;
      myNumInverted = 0;

      double mins = Double.MAX_VALUE;
      MFreeElement3d minE = null;
      
      for (MFreeElement3d region : myElements) {
         FemMaterial mat = getElementMaterial(region);
         computeMaterialStressAndStiffness(region, mat, D, softIncomp);
         if (checkTangentStability) {
            double s = checkMatrixStability(D);
            if (s < mins) {
               mins = s;
               minE = region;
            }
         }
      }

      //      // incompressibility
      //      if (softIncomp == IncompMethod.ELEMENT) {
      //         // XXX currently done in MaterialStressAndStiffness
      //         // computeElementIncompressibility(D);
      //      } else 
      if (softIncomp == IncompMethod.NODAL && myMaterial != null && myMaterial.isIncompressible()) {
         computeNodalIncompressibility(myMaterial, D);
      }

      if (checkTangentStability && minE != null) {
         System.out.println("min s=" + mins + ", element " + minE.getNumber());
      }

      if (myNumInverted > 0) {
         System.out.println(
            "Warning: " + myNumInverted + " inverted elements, min detJ="
               + myMinDetJ + ", element " + myMinDetJElement.getNumber());
         if (abortOnInvertedElems) {
            throw new NumericalException("Inverted elements");
         }
      }

      if (!myStiffnessesValidP && mySolveMatrixSymmetricP) {
         for (FemNode3d n : myNodes) {
            int bi = n.getSolveIndex();
            if (bi != -1) {
               for (FemNodeNeighbor nbr : getNodeNeighbors(n)) {
                  int bj = nbr.getNode().getSolveIndex();
                  if (bj > bi) {
                     FemNodeNeighbor nbrT =
                        nbr.getNode().getNodeNeighborBySolveIndex(bi);
                     nbrT.setTransposedStiffness(nbr);
                  }
               }
               for (FemNodeNeighbor nbr : getIndirectNeighbors(n)) {
                  int bj = nbr.getNode().getSolveIndex();
                  if (bj > bi) {
                     FemNodeNeighbor nbrT =
                        nbr.getNode().getIndirectNeighborBySolveIndex(bi);
                     nbrT.setTransposedStiffness(nbr);
                  }
               }
            }
         }
      }
      myStiffnessesValidP = true;
      myStressesValidP = true;
   }
   
   public void updateStress() {
      // clear existing internal forces and maybe stiffnesses
      for (MFreeNode3d n : myNodes) {
         n.getInternalForce().setZero();
         for (FemNodeNeighbor nbr : getNodeNeighbors(n)) {
            nbr.zeroStiffness();
         }
         // used for soft nodal-based incompressibilty:
         for (FemNodeNeighbor nbr : getIndirectNeighbors(n)) {
            nbr.zeroStiffness();
         }
         if (myComputeNodalStress) {
            n.zeroStress();
         }
         if (myComputeNodalStrain) {
            n.zeroStrain();
         }
      }
      if (!myVolumeValid) {
         updateVolume();
      }
      IncompMethod softIncomp = getSoftIncompMethod();

      if (softIncomp == IncompMethod.NODAL) {
         updateNodalPressures((IncompressibleMaterial)myMaterial);
      }

      // compute new forces as well as stiffness matrix if warping is enabled
      // myMinDetJ = Double.MAX_VALUE;
      for (MFreeElement3d e : myElements) {
         FemMaterial mat = getElementMaterial(e);
         // computeNonlinearStressAndStiffness(
         //    e, mat, /* D= */null, softIncomp);
         computeMaterialStressAndStiffness(e, mat, /* D= */null, softIncomp);
         //         if (mat.isIncompressible() && softIncomp == IncompMethod.ELEMENT) {
         //            computeElementIncompressibility(e, mat, null);
         //         }
      }
      myStressesValidP = true;
   }

   private boolean softNodalIncompressAllowed() {
      return (myMaterial instanceof IncompressibleMaterial);
   }

   private boolean hardNodalIncompressAllowed() {
      return (myMaterial instanceof IncompressibleMaterial);
   }

   public IncompMethod getHardIncompMethod() {
      if (!myHardIncompMethodValidP) {
         if (!hardNodalIncompressAllowed()
            && (myHardIncompMethod == IncompMethod.NODAL
            || myHardIncompMethod == IncompMethod.AUTO
            || myHardIncompMethod == IncompMethod.ON)) {
            myHardIncompMethod = IncompMethod.ELEMENT;
         } else if (myHardIncompMethod == IncompMethod.AUTO
            || myHardIncompMethod == IncompMethod.ON) {
            if (myElements.size() > numActiveNodes()) {
               myHardIncompMethod = IncompMethod.NODAL;
            } else {
               myHardIncompMethod = IncompMethod.ELEMENT;
            }
         }
         myHardIncompMethodValidP = true;
      }
      return myHardIncompMethod;
   }

   public void setIncompressible(IncompMethod method) {
      myHardIncompMethod = method;
      myHardIncompMethodValidP = false;
   }

   private int numActiveNodes() {
      int num = 0;
      for (int i = 0; i < myNodes.size(); i++) {
         if (myNodes.get(i).isActive()) {
            num++;
         }
      }
      return num;
   }

   public IncompMethod getSoftIncompMethod() {
      if (!mySoftIncompMethodValidP) {
         if (!softNodalIncompressAllowed()
            && (mySoftIncompMethod == IncompMethod.NODAL
            || mySoftIncompMethod == IncompMethod.AUTO)) {
            mySoftIncompMethod = IncompMethod.ELEMENT;
         } else if (mySoftIncompMethod == IncompMethod.AUTO) {
            if (myElements.size() > numActiveNodes()) {
               mySoftIncompMethod = IncompMethod.NODAL;
            } else {
               mySoftIncompMethod = IncompMethod.ELEMENT;
            }
         }
         mySoftIncompMethodValidP = true;
      }
      return mySoftIncompMethod;
   }

   public void setSoftIncompMethod(IncompMethod method) {
      if (method == IncompMethod.ON || method == IncompMethod.OFF) {
         throw new IllegalArgumentException("Unsupported method: " + method);
      }
      mySoftIncompMethod = method;
      mySoftIncompMethodValidP = false;
   }

   public void setMaterial(FemMaterial mat) {
      super.setMaterial(mat);
   }

   private FemMaterial getElementMaterial(MFreeElement3d e) {
      FemMaterial mat = e.getMaterial();
      if (mat == null) {
         mat = myMaterial;
      }
      return mat;
   }

   public void updateJacobians() {
      for (MFreeElement3d region : myElements) {
         computeJacobianAndGradient(region);
      }
   }

   private void computeJacobianAndGradient(MFreeElement3d region) {

      MFreeIntegrationPoint3d[] ipnts = region.getIntegrationPoints();
      IntegrationData3d[] idata = region.getIntegrationData();
      region.setInverted(false);

      for (int i = 0; i < ipnts.length; i++) {
         MFreeIntegrationPoint3d ipnt = ipnts[i];
         IntegrationData3d idat = idata[i];
         ipnt.computeJacobianAndGradient(idat.getInvJ0());
         double detJ = ipnt.computeInverseJacobian();

         if (detJ < myMinDetJ) {
            myMinDetJ = detJ;
            myMinDetJElement = region;
         }
         if (detJ <= 0) {
            region.setInverted(true);
            myNumInverted++;
         }
      }
   }

   protected double checkMatrixStability(DenseMatrix D) {
      EigenDecomposition evd = new EigenDecomposition();
      evd.factorSymmetric(D, EigenDecomposition.OMIT_V);
      VectorNd eig = evd.getEigReal();
      double min = eig.get(0);
      double max = eig.get(eig.size() - 1);
      if (Math.abs(max) > Math.abs(min)) {
         return min / max;
      } else {
         return max / min;
      }
   }
   
   private void computeNodalIncompressibility(FemMaterial mat, Matrix6d D) {
      
      IncompressibleBehavior ib = mat.getIncompressibleBehavior();
      
      for (MFreeNode3d n : myNodes) {
         if (volumeIsControllable(n)) {
            double restVol = n.myRestVolume;
            myKp[0] =
            ib.getEffectiveModulus(n.myVolume / restVol) / restVol;
            // myKp[0] = 1;
            if (myKp[0] != 0) {
               for (FemNodeNeighbor nbr_i : getNodeNeighbors(n)) {
                  int bi = nbr_i.getNode().getSolveIndex();
                  for (FemNodeNeighbor nbr_j : getNodeNeighbors(n)) {
                     int bj = nbr_j.getNode().getSolveIndex();
                     if (!mySolveMatrixSymmetricP || bj >= bi) {
                        FemNodeNeighbor nbr =
                        nbr_i.getNode().getNodeNeighbor(nbr_j.getNode());
                        if (nbr == null) {
                           nbr = nbr_i.getNode().getIndirectNeighbor(nbr_j.getNode());
                        }
                        if (nbr == null) {
                           throw new InternalErrorException(
                              "No neighbor block at bi=" + bi + ", bj=" + bj);
                        }
                        else {
                           nbr.addDilationalStiffness(
                              myKp, nbr_i.getDivBlk(), nbr_j.getDivBlk());
                        }
                     }
                  }
               }
            }
         }
      }
   }

   private void computeElementIncompressibility(Matrix6d D) {
      for (MFreeElement3d e : myElements) {
         FemMaterial mat = getElementMaterial(e);
         if (mat.isIncompressible()) {
            computeElementIncompressibility(e, mat, D);
         }
      }
   }
   
   private void computeElementIncompressibility(
      MFreeElement3d e, FemMaterial imat, Matrix6d D) {

      IncompressibleBehavior ib = imat.getIncompressibleBehavior();
      
      MFreeIntegrationPoint3d[] ipnts = e.getIntegrationPoints();
      IntegrationData3d[] idata = e.getIntegrationData();

      double pressure = 0;
      MatrixNd Rinv = new MatrixNd(1, 1);
      Matrix6d Dinc = new Matrix6d();
      SymmetricMatrix3d Sinc = new SymmetricMatrix3d();

      double Jpartial = e.getVolume() / e.getRestVolume();
      pressure = ib.getEffectivePressure(Jpartial);
      Rinv.set(0, 0, ib.getEffectiveModulus(Jpartial) / e.getRestVolume());

      MatrixBlock[] constraints = e.getIncompressConstraints();
      for (int i = 0; i < e.myNodes.length; i++) {
         constraints[i].setZero();
      }

      ib.computePressureStress(Sinc, pressure);
      ib.computePressureTangent(Dinc, pressure);

      for (int k = 0; k < ipnts.length; k++) {
         MFreeIntegrationPoint3d pt = ipnts[k];
         IntegrationData3d dt = idata[k];
         pt.computeJacobianAndGradient(dt.getInvJ0());
         double detJ = pt.computeInverseJacobian();
         double dv = detJ * pt.getWeight();
         Vector3d[] GNx = pt.updateShapeGradient(pt.getInvJ());

         for (int i = 0; i < e.myNodes.length; i++) {
            FemNode3d nodei = e.myNodes[i];
            int bi = nodei.getSolveIndex();

            // if (e.isTermActive(i, i)) {
               FemUtilities.addStressForce(
                  nodei.getInternalForce(), GNx[i], Sinc, dv);
               FemUtilities.addToIncompressConstraints(
                  constraints[i], new double[] { 1 }, GNx[i], dv);
            // }
            if (bi != -1) {
               for (int j = 0; j < e.myNodes.length; j++) {
                  int bj = e.myNodes[j].getSolveIndex();
                  if (!mySolveMatrixSymmetricP || bj >= bi) {
                     e.addMaterialStiffness(
                        i, j, GNx[i], Dinc, GNx[j], dv);
                     e.addGeometricStiffness(
                        i, j, GNx[i], Sinc, GNx[j], dv);
                     e.addPressureStiffness(
                        i, j, GNx[i], pressure, GNx[j], dv);
                  }

               } // looping through nodes
            } // if bi != -1

         } // looping through nodes computing stress
      } // looping through ipnts

      for (int i = 0; i < e.myNodes.length; i++) {
         int bi = e.myNodes[i].getSolveIndex();
         if (bi != -1) {
            for (int j = 0; j < e.myNodes.length; j++) {
               int bj = e.myNodes[j].getSolveIndex();
               if (!mySolveMatrixSymmetricP || bj >= bi) {
                  e.addDilationalStiffness(
                     i, j, Rinv, constraints[i], constraints[j]);
               }
            }
         }
      }
   }

   // DIVBLK
   private void computeMaterialStressAndStiffness(
      MFreeElement3d e, FemMaterial mat, Matrix6d D,
      IncompMethod softIncomp) {

      MFreeIntegrationPoint3d[] ipnts = e.getIntegrationPoints();
      IntegrationData3d[] idata = e.getIntegrationData();
      MFreeNode3d[] nodes = e.getNodes();
      if (D != null) {
         D.setZero();
      }

      // compute corotation if required
      MFreeIntegrationPoint3d wpnt = null;
      RotationMatrix3d wR = null;  // warping rotation
      SymmetricMatrix3d wP = null; // warping strain
      if (requiresWarping(e, mat)) {
         wpnt = e.getWarpingPoint();
         IntegrationData3d data = e.getWarpingData();
         wpnt.computeJacobianAndGradient(data.getInvJ0());
         wR = new RotationMatrix3d();
         wP = new SymmetricMatrix3d();
         SVD.polarDecomposition(wR, wP, wpnt.getF());
      }
      
      // see if material is linear
      FemMaterial linMat = null;
      SymmetricMatrix3d linStrain = null;
      if (mat.isLinear()) {
         linMat = mat;
         linStrain = new SymmetricMatrix3d();
         if (linMat.isCorotated()) {
            linStrain.set(wP);
         }
         else {
            // get strain from warping point
            wpnt = e.getWarpingPoint();
            IntegrationData3d data = e.getWarpingData();
            wpnt.computeJacobianAndGradient(data.getInvJ0());
            linStrain.setSymmetric(wpnt.getF());
            wpnt = null;
         }
         // compute Cauchy strain
         linStrain.m00 -= 1;
         linStrain.m11 -= 1;
         linStrain.m22 -= 1;
      }

      // base linear material optimization
      if (linMat != null) {
         if (linMat.isCorotated()) {
            e.updateWarping(wR);
         }
         for (int i = 0; i < nodes.length; i++) {
            int bi = nodes[i].getSolveIndex();
            if (bi != -1) {
               FemNode3d n = nodes[i];
               if (!myStiffnessesValidP) {
                  for (int j = 0; j < nodes.length; j++) {
                     int bj = nodes[j].getSolveIndex();
                     if (!mySolveMatrixSymmetricP || bj >= bi) {
                        e.addNodeStiffness(i, j, linMat.isCorotated());
                     }
                  }
               }
               e.addNodeForce(n.getInternalForce(), i, linMat.isCorotated());
            }
         }

         // nodal stress and strain
         if (myComputeNodalStress || myComputeNodalStrain) {
            if (myComputeNodalStress) {
               wpnt.getStress().setZero();
               SolidDeformation def = new SolidDeformation();
               def.setF(wpnt.getF());
               def.setR(wR);
               def.setMaterialCoordinate(new MFreeIntegrationCoordinate(e, wpnt));
               linMat.computeStress(wpnt.getStress(), def, null, null);
               // linMat.addStress(wpnt.sigma, linStrain, linMat.isCorotated() ? wR : null);
               for (int i = 0; i < nodes.length; i++) {
                  nodes[i].addScaledStress(1.0 / nodes[i].numAdjacentElements(), wpnt.getStress());
               }
            }
            
            if (myComputeNodalStrain) {
               // XXX rotate strain? or is it w.r.t. rest coordinates?
               //               if (linMat.isCorotated()) {
               //                  linStrain.mulLeftAndTransposeRight(wR);
               //               }
               for (int i = 0; i < nodes.length; i++) {
                  nodes[i].addScaledStrain(1.0 / nodes[i].numAdjacentElements(), linStrain);
               }  
            }
         } // stress and strain
         
         // exit early if no other materials
         if (e.numAuxiliaryMaterials() == 0) {
            return;
         }
         
      } // end base linear material
      
      e.setInverted(false); // will check this below
      
      // separation of linear, corotated linear, and nonlinear
      // since they use different shape function gradients
      // due to small-strain assumption (J ~ I)
      // Linear: GNx0                (rest position)
      // Corotated linear: R*GNx0 
      // Nonlinear: GNx1             (deformed position)
      boolean hasLinear = false;
      boolean hasCorotatedLinear = false;
      boolean hasNonlinear = false;
      SymmetricMatrix3d linearStress = null;
      SymmetricMatrix3d corotatedLinearStress = null;
      SymmetricMatrix3d nonlinearStress = null;
      Matrix6d linearTangent = null;
      Matrix6d corotatedLinearTangent = null;
      Matrix6d nonlinearTangent = null;
      Vector3d[] GNx0 = null;
      Vector3d[] RGNx0 = null;
      Vector3d[] GNx1 = null;
      double dv0 = 1;  // rest volume fraction
      double dv1 = 1;   // deformed volume fraction
      
      // initialize stuffs
      if (mat.isLinear()) {
         if (mat.isCorotated()) {
            hasCorotatedLinear = true;
         } else {
            hasLinear = true;
         }
      } else {
         hasNonlinear = true;
      }
      if (e.numAuxiliaryMaterials() > 0) {
         for (ConstitutiveMaterial amat : e.getAuxiliaryMaterials()) {
            if (amat.isLinear()) {
               if (mat.isCorotated()) {
                  hasCorotatedLinear = true;
               } else {
                  hasLinear = true;
               }
            } else {
               hasNonlinear = true;
            }
         }
      }
      if (hasLinear) {
         linearStress = new SymmetricMatrix3d();
         linearTangent = new Matrix6d();
         GNx0 = new Vector3d[nodes.length];
         for (int i=0; i<nodes.length; ++i) {
            GNx0[i] = new Vector3d();
         }
      }
      if (hasCorotatedLinear) {
         corotatedLinearStress = new SymmetricMatrix3d();
         corotatedLinearTangent = new Matrix6d();
         RGNx0 = new Vector3d[nodes.length];
         for (int i=0; i<nodes.length; ++i) {
            RGNx0[i] = new Vector3d();
         }
      }
      if (hasNonlinear) {
         nonlinearStress = new SymmetricMatrix3d();
         nonlinearTangent = new Matrix6d();
         GNx1 = new Vector3d[nodes.length];
         for (int i=0; i<nodes.length; ++i) {
            GNx1[i] = new Vector3d();
         }
      }
      
      // temporaries for auxiliary computation
      SymmetricMatrix3d sigmaTmp = null;
      Matrix6d tangentTmp = null;
      if (e.numAuxiliaryMaterials() > 0) {
         sigmaTmp = new SymmetricMatrix3d();
         tangentTmp = new Matrix6d();
      }
      
      ViscoelasticBehavior veb = mat.getViscoBehavior();
      double vebTangentScale = 1;
      if (veb != null) {
         vebTangentScale = veb.getTangentScale();
      }
      
      // pressure and incompressibility
      int npvals = e.numPressureVals();
      double pressure = 0;
      BulkIncompressibleBehavior ib = null;
      if (mat.isIncompressible()) {
         ib = mat.getIncompressibleBehavior();
      }
      MatrixBlock[] constraints = null;
      SymmetricMatrix3d C = new SymmetricMatrix3d();

      double[] pbuf = myPressures.getBuffer();
      if (mat.isIncompressible() && softIncomp != IncompMethod.NODAL) {
         if (softIncomp == IncompMethod.ELEMENT) {
            computePressuresAndRinv (e, mat, vebTangentScale);
            if (D != null) {
               constraints = e.getIncompressConstraints();
               for (int i = 0; i < e.myNodes.length; i++) {
                  constraints[i].setZero();
               }
            }
         }
      }
      // XXX should we check if incompressible again?
      else if (softIncomp == IncompMethod.NODAL) {
         for (int i = 0; i < e.numNodes(); i++) {
            myNodalConstraints[i].setZero();
         }
      }
      
      // basic information regarding local deformation
      SolidDeformation def = new SolidDeformation();
      MFreeIntegrationCoordinate mcoord = new MFreeIntegrationCoordinate();
      def.setMaterialCoordinate(mcoord);
      
      for (int k = 0; k < ipnts.length; k++) {
         MFreeIntegrationPoint3d pt = ipnts[k];
         IntegrationData3d dt = idata[k];
         pt.computeJacobianAndGradient(dt.getInvJ0());
         

         // material rotation and deformation
         // we currently expect corotational materials to handle
         // their own transform of F, although we could
         // technically do it from here
         mcoord.set(e, pt);
         
         if (mat.isCorotated()) {
            def.setR(wR);
         } else {
            def.setR(RotationMatrix3d.IDENTITY);
         }
         def.setF(pt.getF());
         
         // anisotropy rotational frame
         Matrix3d Q = (dt.getFrame() != null ? dt.getFrame() : Matrix3d.IDENTITY);
         
         double detJ = pt.computeInverseJacobian();
         if (detJ < myMinDetJ) {
            myMinDetJ = detJ;
            myMinDetJElement = e;
         }
         if (detJ <= 0) {
            e.setInverted(true);
            myNumInverted++;
         }
            
         // compute shape function gradients and volume fractions
         if (hasLinear) {
            dv0 = idata[k].getDetJ0() * ipnts[k].getWeight();
            pt.computeShapeGradient(idata[k].getInvJ0(), GNx0);
            if (hasCorotatedLinear) {
               for (int i=0; i<nodes.length; ++i) {
                  wR.mul(RGNx0[i], GNx0[i]);
               }
            }
         } else if (hasCorotatedLinear) {
            dv0 = idata[k].getDetJ0() * ipnts[k].getWeight();
            pt.computeShapeGradient(idata[k].getInvJ0(), RGNx0);
            for (int i=0; i<nodes.length; ++i) {
               wR.mul(RGNx0[i]);
            }
         }
         if (hasNonlinear) {
            dv1 = detJ * pt.getWeight();
            pt.computeShapeGradient(pt.getInvJ(), GNx1);
         }

         // compute pressure
         pressure = 0;
         double[] H = null;
         if (softIncomp == IncompMethod.ELEMENT) {
            H = pt.getPressureWeights().getBuffer();
            for (int l = 0; l < npvals; l++) {
               pressure += H[l] * pbuf[l];
            }
         }
         else if (softIncomp == IncompMethod.NODAL) {
            pressure = nodes[k].myPressure;
         }
         else if (softIncomp == IncompMethod.FULL && mat.isIncompressible()) {
            pressure = ib.getEffectivePressure(detJ / dt.getDetJ0());
         }

         // System.out.println("MFree Pressure: " + pressure);
         pt.setAveragePressure(pressure);
         def.setAveragePressure(pressure);

         // clear stress/tangents
         if (hasLinear) {
            linearStress.setZero();
            if (linearTangent != null) {
               linearTangent.setZero();
            }
         }
         if (hasCorotatedLinear) {
            corotatedLinearStress.setZero();
            if (corotatedLinearTangent != null) {
               corotatedLinearTangent.setZero();
            }
         }
         if (hasNonlinear) {
            nonlinearStress.setZero();
            if (nonlinearTangent != null) {
               nonlinearTangent.setZero();
            }
         }

         // base material stress and tangent
         SymmetricMatrix3d stress;
         Matrix6d tangent;
         if (mat.isLinear()) {
            if (mat.isCorotated()) {
               stress = corotatedLinearStress;
               tangent = corotatedLinearTangent;
            } else {
               stress = linearStress;
               tangent = linearTangent;
            }
         } else {
            stress = nonlinearStress;
            tangent = nonlinearTangent;
         }
         
         mat.computeStress(stress, def, Q, null);
         if (tangent != null) {
            mat.computeTangent(tangent, stress, def, Q, null);
         }
         
         // reset pressure to zero for auxiliary materials
         pt.setAveragePressure(0);
         def.setAveragePressure(0);
         if (e.numAuxiliaryMaterials() > 0) {
            for (ConstitutiveMaterial aux : e.myAuxMaterials) {
               aux.computeStress(sigmaTmp, def, Q, mat);
               if (aux.isLinear()) {
                  if (aux.isCorotated()) {
                     corotatedLinearStress.add(sigmaTmp);
                  } else {
                     linearStress.add(sigmaTmp);
                  }
               } else {
                  nonlinearStress.add(sigmaTmp);
               }
               if (D != null) {
                  aux.computeTangent(tangentTmp, sigmaTmp, def, Q, mat);
                  if (aux.isLinear()) {
                     if (aux.isCorotated()) {
                        corotatedLinearTangent.add(tangentTmp);
                     } else {
                        linearTangent.add(tangentTmp);
                     }
                  } else {
                     nonlinearTangent.add(tangentTmp);
                  }
               }
            }
         }

         //XXX this seems to overwrite previous stress/stiffness
         // and does not include auxiliary materials
         pt.setAveragePressure(pressure); // bring back pressure term
         def.setAveragePressure(pressure);
         if (veb != null) {
            ViscoelasticState state = idata[k].getViscoState();
            if (state == null) {
               state = veb.createState();
               idata[k].setViscoState(state);
            }
            // veb.computeStress(pt.sigma, state);
            veb.computeStress(nonlinearStress, state);
            if (D != null) {
               veb.computeTangent(nonlinearTangent, state);
            }
         }
         else {
            dt.clearState();
         }

         for (int i = 0; i < e.myNodes.length; i++) {
            FemNode3d nodei = e.myNodes[i];
            int bi = nodei.getSolveIndex();

            // if (e.isTermActive(i, i)) {
               if (hasLinear) {
                  FemUtilities.addStressForce(
                     nodei.getInternalForce(), GNx0[i], linearStress, dv0);
               }
               if (hasCorotatedLinear) {
                  FemUtilities.addStressForce(
                     nodei.getInternalForce(), RGNx0[i], corotatedLinearStress, dv0);
               }
               if (hasNonlinear) {
                  FemUtilities.addStressForce(
                     nodei.getInternalForce(), GNx1[i], nonlinearStress, dv1);
               }
            // }

            if (D != null) {
               if (hasLinear) {
                  D.scaledAdd(dv0, linearTangent);
               }
               if (hasCorotatedLinear) {
                  D.scaledAdd(dv0, corotatedLinearTangent);
               }
               if (hasNonlinear) {
                  D.scaledAdd(dv1, nonlinearTangent);
               }

               double p = 0;
               double kp = 0;
               if (mat.isIncompressible() &&
                  softIncomp != IncompMethod.NODAL) {
                  if (softIncomp == IncompMethod.ELEMENT) {
                     FemUtilities.addToIncompressConstraints(constraints[i], 
                        H, GNx1[i], dv1);
                  }
                  else if (softIncomp == IncompMethod.FULL) {
                     double dV = dt.getDetJ0() * ipnts[k].getWeight();
                     kp = mat.getIncompressibleBehavior().getEffectiveModulus(detJ / dt.getDetJ0()) * dV;
                  }
                  p = pressure;
               }
               else if (softIncomp == IncompMethod.NODAL) {
                  myNodalConstraints[i].scaledAdd(dv1*ipnts[k].getShapeWeights().get(i), GNx1[i]);
               }

               // compute stiffness
               if (bi != -1) {
                  for (int j = 0; j < e.myNodes.length; j++) {
                     int bj = e.myNodes[j].getSolveIndex();
                     if (!mySolveMatrixSymmetricP || bj >= bi) {
                        if (hasLinear) {
                           e.addMaterialStiffness(i,j,GNx0[i], linearTangent, GNx0[j], dv0);
                        }
                        if (hasCorotatedLinear) {
                           e.addMaterialStiffness(i,j,RGNx0[i], corotatedLinearTangent, RGNx0[j], dv0);
                        }
                        if (hasNonlinear) {
                           e.addMaterialStiffness(i,j,GNx1[i], nonlinearTangent, GNx1[j], dv1);
                           e.addGeometricStiffness(i,j, GNx1[i], nonlinearStress, GNx1[j], dv1);
                           e.addPressureStiffness(i,j, GNx1[i], p, GNx1[j], dv1);   
                        }

                        if (kp != 0) {
                           e.addDilationalStiffness(i, j, vebTangentScale*kp, GNx1[i], GNx1[j]);
                        }
                     }
                  }
               }
            } // if D != null

            // nodal stress/strain
            double[] nodalExtrapMat = e.getNodalExtrapolationMatrix();
            if (nodalExtrapMat != null) {
               if (myComputeNodalStress) {
                  double a = nodalExtrapMat[i * ipnts.length + k];
                  if (a != 0) {
                     if (hasLinear) {
                        nodei.addScaledStress(
                           a / nodei.numAdjacentElements(), linearStress);
                     }
                     if (hasCorotatedLinear) {
                        nodei.addScaledStress(
                           a / nodei.numAdjacentElements(), corotatedLinearStress);
                     }
                     if (hasNonlinear) {
                        nodei.addScaledStress(
                           a / nodei.numAdjacentElements(), nonlinearStress);
                     }
                  }
               }

               if (myComputeNodalStrain) {
                  double a = nodalExtrapMat[i * ipnts.length + k];
                  if (a != 0) {
                     // pt.computeRightCauchyGreen(C);
                     def.computeRightCauchyGreen(C);
                     C.m00 -= 1;
                     C.m11 -= 1;
                     C.m22 -= 1;
                     C.scale(0.5);
                     nodei.addScaledStrain(
                        a / nodei.numAdjacentElements(), C);
                  }
               }

            }
         } // looping through nodes computing stress
         
         // nodal incompressibility constraints
         if (D != null && softIncomp == IncompMethod.NODAL) {
            for (FemNodeNeighbor nbr : getNodeNeighbors(e.myNodes[k])) {
               int j = e.getLocalNodeIndex(nbr.getNode());
               if (j != -1) {
                  nbr.getDivBlk().scaledAdd(1, myNodalConstraints[j]);
               }
            }
         }
         
      } // looping through ipnts


      // element-wise incompressibility constraints
      if (D != null) {
         if (mat.isIncompressible() && softIncomp == IncompMethod.ELEMENT) {
            boolean kpIsNonzero = false;
            for (int l = 0; l < npvals; l++) {
               double Jpartial = e.myVolumes[l] / e.myRestVolumes[l];
               myKp[l] =
                  ib.getEffectiveModulus(Jpartial) / e.myRestVolumes[l];
               if (myKp[l] != 0) {
                  kpIsNonzero = true;
               }
            }
            // double kp = imat.getEffectiveModulus(vol/restVol)/restVol;
            if (true) {
               for (int i = 0; i < e.myNodes.length; i++) {
                  int bi = e.myNodes[i].getSolveIndex();
                  if (bi != -1) {
                     for (int j = 0; j < e.myNodes.length; j++) {
                        int bj = e.myNodes[j].getSolveIndex();
                        if (!mySolveMatrixSymmetricP || bj >= bi) {
                           e.addDilationalStiffness(i,j,
                              myRinv, constraints[i], constraints[j]);
                        } // end filling in symmetric
                     } // end filling in dilatational stiffness
                  } // end checking if valid index
               } // end looping through nodes
            } // XXX ALWAYS??
         } // end soft elem incompress
      } // end checking if computing tangent
      
   }

   public void checkInversion() {
      myMinDetJ = Double.MAX_VALUE;
      myMinDetJElement = null;
      myNumInverted = 0;
      for (MFreeElement3d e : myElements) {
         // FemMaterial mat = getRegionMaterial(e);
         // if (!(mat instanceof LinearMaterial)) {
         computeJacobianAndGradient(e);
         // }
      }
   }

   public int numSurfaceMeshes() {
      return myMeshList.size();
   }

   public MeshBase getMesh(int idx) {
      return myMeshList.get(idx).getMesh();
   }

   @Override
   public Collidability getCollidable() {
      MFreeMeshComp mesh = mySurfaceMesh;
      if (mesh != null) {
         return Collidability.EXTERNAL;
      }
      return Collidability.OFF;
   }
   
   @Override
   public Collidable getCollidableAncestor() {
      return null;
   }

   @Override
   public boolean isCompound() {
      return false;
   }

   @Override
   public boolean isDeformable () {
      return true;
   }

   public Collection<MFreeMeshComp> getMeshes() {
      return myMeshList;
   }

   public MFreeMeshComp getMeshComponent(int idx) {
      return myMeshList.get(idx);
   }

   protected MFreeMeshComp findMesh(MeshBase mesh) {
      for (MFreeMeshComp mc : myMeshList) {
         if (mc.getMesh() == mesh) {
            return mc;
         }
      }
      return null;
   }

   public boolean removeMesh(MeshBase mesh) {

      // remove points
      MFreeMeshComp mc = findMesh(mesh);
      if (mc != null) {
         return myMeshList.remove(mc);
      }
      return false;
   }
   
   public MFreeMeshComp setSurfaceMesh(PolygonalMesh surface) {
      String meshName =
         ModelComponentBase.makeValidName(surface.getName(), null, myMeshList);
      if (meshName == null) {
         meshName = ModelComponentBase.makeValidName("surface", null, myMeshList);
      }
      return setSurfaceMesh(meshName, surface);
   }
   
   public MFreeMeshComp setSurfaceMesh(String name, PolygonalMesh surface) {
      surface.setFixed(false);
      surface.setColorsFixed(false);
      MFreeMeshComp surf = MFreeMeshComp.createEmbedded(this, surface);
      surf.setName(name);
      setSurfaceMesh(surf);
      return surf;
   }
   
   public void setSurfaceMesh(MFreeMeshComp mesh) {
      if (!(mesh.getMesh()instanceof PolygonalMesh)) {
         throw new IllegalArgumentException("Surface mesh must be of type PolygonalMesh");
      }
      if (mySurfaceMesh != null) {
         mySurfaceMesh.setCollidable(Collidability.INTERNAL);
         mySurfaceMesh.markSurfaceMesh(false);
      }
      mySurfaceMesh = mesh;
      if (mesh != null) {
         mySurfaceMesh.markSurfaceMesh(true);
         if (!myMeshList.contains(mesh)) {
            myMeshList.add(mySurfaceMesh);
         }
         mySurfaceMesh.setCollidable(Collidability.EXTERNAL);
      }
   }

   public MFreeMeshComp addMesh(MeshBase mesh) {
      String meshName =
         ModelComponentBase.makeValidName(mesh.getName(), null, myMeshList);
      return addMesh(meshName, mesh);
   }
   
   public MFreeMeshComp addMesh(String name, MeshBase mesh) {
      mesh.setFixed(false);
      mesh.setColorsFixed(false);
      MFreeMeshComp mmc = MFreeMeshComp.createEmbedded(this, mesh);
      mmc.setName(name);
      addMesh(mmc);
      return mmc;
   }
   
   public void addMesh(MFreeMeshComp mesh) {
      mesh.setCollidable (Collidability.INTERNAL);
      myMeshList.add(mesh);
   }

   public void clearMeshes() {
      myMeshList.clear();
      if (mySurfaceMesh != null) {
         mySurfaceMesh.markSurfaceMesh(false);
         mySurfaceMesh = null;
      }
      
   }

   protected boolean checkSolveMatrixIsSymmetric() {
      if (!myMaterial.hasSymmetricTangent()) {
         return false;
      }
      for (int i = 0; i < myElements.size(); i++) {
         MFreeElement3d e = myElements.get(i);
         FemMaterial m = e.getMaterial();
         if (m != null && !m.hasSymmetricTangent()) {
            return false;
         }
         if (e.numAuxiliaryMaterials() > 0) {
            for (ConstitutiveMaterial aux : e.myAuxMaterials) {
               if (!aux.hasSymmetricTangent()) {
                  return false;
               }
            }
         }
      }
      return true;
   }

   public int getJacobianType() {
      if (mySolveMatrixSymmetricP) {
         return Matrix.SYMMETRIC;
      } else {
         return Matrix.INDEFINITE;
      }
   }

   private void addBlockVelJacobian(
      SparseNumberedBlockMatrix M, MFreeNode3d node, FemNodeNeighbor nbr,
      double s) {

      if (nbr.getNode().getSolveIndex() != -1) {
         Matrix3x3Block blk =
            (Matrix3x3Block)M.getBlockByNumber(nbr.getBlockNumber());
         if (nbr.getNode() == node && node.isActive()) {
            nbr.addVelJacobian(blk, s, myStiffnessDamping, myMassDamping);
         } else {
            nbr.addVelJacobian(blk, s, myStiffnessDamping, 0);
         }
      }
   }

   public void addVelJacobian(SparseNumberedBlockMatrix M, double s) {

      if (!myStressesValidP || !myStiffnessesValidP) {
         updateStressAndStiffness();
      }

      for (int i = 0; i < myNodes.size(); i++) {
         MFreeNode3d node = myNodes.get(i);
         if (node.getSolveIndex() != -1) {
            for (FemNodeNeighbor nbr : getNodeNeighbors(node)) {
               addBlockVelJacobian(M, node, nbr, s);
            }
            for (FemNodeNeighbor nbr : getIndirectNeighbors(node)) {
               addBlockVelJacobian(M, node, nbr, s);
            }
         }
      }
   }

   private void addBlockPosJacobian(
      SparseNumberedBlockMatrix M, MFreeNode3d node, FemNodeNeighbor nbr,
      double s) {

      if (nbr.getNode().getSolveIndex() != -1) {
         Matrix3x3Block blk =
            (Matrix3x3Block)M.getBlockByNumber(nbr.getBlockNumber());
         nbr.addPosJacobian(blk, s);
      }

   }

   public void addPosJacobian(SparseNumberedBlockMatrix M, double s) {

      if (!myStressesValidP || !myStiffnessesValidP) {
         updateStressAndStiffness();
      }

      for (int i = 0; i < myNodes.size(); i++) {
         MFreeNode3d node = myNodes.get(i);
         if (node.getSolveIndex() != -1) {
            for (FemNodeNeighbor nbr : getNodeNeighbors(node)) {
               addBlockPosJacobian(M, node, nbr, s);
            }
            for (FemNodeNeighbor nbr : getIndirectNeighbors(node)) {
               addBlockPosJacobian(M, node, nbr, s);
            }
         }
      }
   }

   private void addStiffnessBlock(
      SparseNumberedBlockMatrix S, FemNodeNeighbor nbr, int bi) {

      int bj = nbr.getNode().getSolveIndex();
      Matrix3x3Block blk = null;
      int blkNum = -1;
      if (bj != -1) {
         blk = (Matrix3x3Block)S.getBlock(bi, bj);
         if (blk == null) {
            blk = new Matrix3x3Block();
            S.addBlock(bi, bj, blk);
         }
         blkNum = blk.getBlockNumber();
      }
      // nbr.setBlock(blk);
      nbr.setBlockNumber(blkNum);
   }

   public void addSolveBlocks(SparseNumberedBlockMatrix S) {

      for (int i = 0; i < myNodes.size(); i++) {
         FemNode3d node = myNodes.get(i);
         int bi = node.getSolveIndex();
         if (bi != -1) {
            for (FemNodeNeighbor nbr : getNodeNeighbors(node)) {
               addStiffnessBlock(S, nbr, bi);
            }
            for (FemNodeNeighbor nbr : getIndirectNeighbors(node)) {
               addStiffnessBlock(S, nbr, bi);
            }
         }
      }

   }

   public void recursivelyInitialize(double t, int level) {

      if (t == 0) {
         for (MFreeElement3d region : myElements) {
            region.invalidateRestData();
            region.setInverted(false);
         }
         for (MFreeNode3d n : myNodes) {
            n.zeroStress();
         }
         invalidateStressAndStiffness();
         updateAllRestVolumes();
      }

      super.recursivelyInitialize(t, level);
   }

   // public boolean recursivelyCheckStructureChanged() {
   // return false;
   // }

   public double integrate(Function3x1 fun) {
      double out = 0;
      if (!myStiffnessesValidP) {
         updateJacobians();
      }

      for (MFreeElement3d elem : myElements) {
         MFreeIntegrationPoint3d[] ipnts = elem.getIntegrationPoints();
         IntegrationData3d idata[] = elem.getIntegrationData();
         
         for (int i = 0; i < elem.numIntegrationPoints(); i++) {
            MFreeIntegrationPoint3d ipnt = ipnts[i];
            IntegrationData3d idat = idata[i];
            VectorNd shapeFunc = ipnt.getShapeWeights();

            for (int j = 0; j < elem.numNodes(); j++) {
               // if (elem.isTermActive(j, j)) {
                  double f =
                     fun.eval(ipnt.getPosition()) * ipnt.getWeight() * shapeFunc.get(j);
                  f = f * ipnt.getDetF() * idat.getDetJ0();
                  out += f;
               // }
            }
         }
      }

      return out;
   }

   public double integrateVolume() {
      return integrate(new ConstantFuntion3x1(1));
   }

   public void updateAllRestVolumes() {

      // clear nodal volumes
      for (MFreeNode3d node : myNodes) {
         node.setRestVolume(0);
         node.setPartitionRestVolume(0);
      }

      double totalVolume = 0;

      for (MFreeElement3d elem : myElements) {
         MFreeNode3d[] nodes = elem.getNodes();
         double elemVolume = 0;
         
         MFreeIntegrationPoint3d[] ipnts = elem.getIntegrationPoints();
         IntegrationData3d[] idata = elem.getIntegrationData();

         for (int i = 0; i < elem.numIntegrationPoints(); i++) {
            MFreeIntegrationPoint3d ipnt = ipnts[i];
            IntegrationData3d idat = idata[i];

            VectorNd shapeFunc = ipnt.getShapeWeights();

            double f = ipnt.getWeight() * idat.getDetJ0();

            // element
            elemVolume += f;

            for (int j = 0; j < elem.numNodes(); j++) {
               // if (elem.isTermActive(j, j)) {
                  double g = f * shapeFunc.get(j);
                  // total
                  totalVolume += g;
                  // nodal
                  nodes[j].addRestVolume(f);
                  nodes[j].addPartitionRestVolume(g);
               // }
            }
         }
         elem.setRestVolume(elemVolume);

      }

      myRestVolume = totalVolume;

   }

   public void updateAllVolumes() {

      double totalVolume = 0;
      if (!myStiffnessesValidP) {
         updateJacobians();
      }

      // clear nodal volumes
      for (MFreeNode3d node : myNodes) {
         node.setVolume(0);
         node.setPartitionVolume(0);
      }

      for (MFreeElement3d elem : myElements) {
         MFreeNode3d[] nodes = elem.getNodes();
         double elemVolume = 0;

         for (int i = 0; i < elem.numIntegrationPoints(); i++) {
            MFreeIntegrationPoint3d ipnt = elem.getIntegrationPoint(i);
            IntegrationData3d idat = elem.getIntegrationData(i);

            VectorNd shapeFunc = ipnt.getShapeWeights();

            double f = ipnt.getWeight() * ipnt.getDetF() * idat.getDetJ0();

            // element
            elemVolume += f;

            for (int j = 0; j < elem.numNodes(); j++) {
               // if (elem.isTermActive(j, j)) {
                  double g = f * shapeFunc.get(j);
                  // total
                  totalVolume += g;
                  // nodal
                  nodes[j].addVolume(f);
                  nodes[j].addPartitionVolume(g);
               // }
            }
         }
         elem.setVolume(elemVolume);

      }

      myVolume = totalVolume;
      myVolumeValid = true;
   }

   public double integrateMass() {
      return integrate(new ConstantFuntion3x1(myDensity));
   }

   public double computeConsistentMassMatrixEntry(
      MFreeNode3d node1, MFreeNode3d node2) {

      // collect elements
      ArrayList<MFreeElement3d> depElems = new ArrayList<MFreeElement3d>();
      for (MFreeElement3d elem : node1.getMFreeElementDependencies()) {
         if (!depElems.contains(elem)) {
            // if (elem.isTermActive(node1, node2)) {
               depElems.add(elem);
            // }
         }
      }

      if (!myStiffnessesValidP) {
         for (MFreeElement3d elem : depElems) {
            computeJacobianAndGradient(elem);
         }
      }

      double m = 0;
      for (MFreeElement3d elem : depElems) {

         //         int idx1 = elem.getNodeIdx(node1);
         //         int idx2 = elem.getNodeIdx(node2);
         //
         //         if (elem.isTermActive(idx1, idx2)) {
            for (int i = 0; i < elem.numIntegrationPoints(); i++) {
               MFreeIntegrationPoint3d ipnt = elem.getIntegrationPoint(i);
               IntegrationData3d idat = elem.getIntegrationData(i);

               m +=
                  myDensity * ipnt.getShapeCoordinate(node1)
                  * ipnt.getShapeCoordinate(node2) * ipnt.getWeight() * ipnt.getDetF()
                  * idat.getDetJ0();
            }
         // }

      }
      return m;

   }

   public SparseMatrixNd computeConsistentMassMatrix() {

      int nNodes = myNodes.size();
      SparseMatrixNd M = new SparseMatrixNd(nNodes, nNodes);

      updateJacobians();

      for (MFreeElement3d e : myElements) {
         for (int k = 0; k < e.numIntegrationPoints(); k++) {

            MFreeIntegrationPoint3d ipnt = e.getIntegrationPoint(k);
            IntegrationData3d idat = e.getIntegrationData(k);

            VectorNd shapeCoords = ipnt.getShapeWeights();

            for (int i = 0; i < e.numNodes(); i++) {
               for (int j = i; j < e.numNodes(); j++) {
                  // if (e.isTermActive(i, j)) {
                     int bi = e.getNode(i).getNumber();
                     int bj = e.getNode(j).getNumber();

                     double m =
                        myDensity * shapeCoords.get(i)
                        * shapeCoords.get(j) * ipnt.getWeight() * ipnt.getDetF()
                        * idat.getDetJ0();

                     M.set(bi, bj, M.get(bi, bj) + m);
                     if (i != j) {
                        M.set(bj, bi, M.get(bj, bi) + m);
                     }

                  // }
               }
            }
         }
      }

      return M;

   }

   public void updateNodeMasses(double totalMass, VectorNd massMatrixDiag) {

      if (totalMass <= 0) {
         totalMass = integrateMass();
      }

      if (massMatrixDiag == null) {
         SparseMatrixNd massMatrix = computeConsistentMassMatrix();
         massMatrixDiag = new VectorNd(massMatrix.rowSize());
         for (int i = 0; i < massMatrix.rowSize(); i++) {
            double rowSum = 0;
            for (int j = 0; j < massMatrix.colSize(); j++) {
               rowSum += massMatrix.get(i, j);
            }
            // rowSum += massMatrix.get(i, i);
            massMatrixDiag.set(i, rowSum);
         }
      }

      double mTrace = massMatrixDiag.sum();

      for (MFreeNode3d node : myNodes) {
         int i = node.getNumber();
         double m = totalMass / mTrace * massMatrixDiag.get(i);
         node.setMass(m);
      }

   }

   @Override
   public void initialize(double t0) {
      super.initialize(t0);
      updatePosState();
      updateVelState();
   }

   // @Override
   // public synchronized StepAdjustment advance(
   // double t0, double t1, int flags) {

   // initializeAdvance (t0, t1, flags);

   // if (t0 == 0) {
   // updateForces(t0);
   // }

   // if (!myDynamicsEnabled) {
   // mySolver.nonDynamicSolve(t0, t1, myStepAdjust);
   // recursivelyFinalizeAdvance(null, t0, t1, flags, 0);
   // }
   // else {
   // mySolver.solve(t0, t1, myStepAdjust);
   // DynamicComponent c = checkVelocityStability();
   // if (c != null) {
   // throw new NumericalException(
   // "Unstable velocity detected, component "
   // + ComponentUtils.getPathName(c));
   // }
   // recursivelyFinalizeAdvance(myStepAdjust, t0, t1, flags, 0);
   // // checkForInvertedElements();
   // }

   // finalizeAdvance (t0, t1, flags);
   // return myStepAdjust;
   // }

   /**
    * {@inheritDoc}
    */
   public DynamicComponent checkVelocityStability() {
      PointList<MFreeNode3d> nodes = getNodes();
      for (int i = 0; i < nodes.size(); i++) {
         MFreeNode3d node = nodes.get(i);
         Vector3d vel = node.getFalseVelocity();
         if (node.velocityLimitExceeded(myMaxTranslationalVel, 0)) {
            return node;
         }
      }
      return null;
   }

   public double getEnergy() {
      double e = 0;
      for (MFreeNode3d n : getNodes()) {
         e += n.getFalseVelocity().normSquared() / 2;
      }
      return e;
   }

   public void setSurfaceRendering(SurfaceRender mode) {
      super.setSurfaceRendering(mode);
      if (mode != SurfaceRender.Stress && mode != SurfaceRender.Strain) {
         myClearMeshColoring = true;
      }
   }
   
   public DoubleInterval getNodalPlotRange(SurfaceRender rendering) {

      if (!(rendering == SurfaceRender.Strain || 
      rendering == SurfaceRender.Stress)) {
         return null;
      }

      double min = Double.MAX_VALUE;
      double max = 0;
      for (int i = 0; i < myNodes.size(); i++) {
         FemNode3d node = myNodes.get(i);
         double s;
         if (rendering == SurfaceRender.Stress) {
            s = (float)node.getVonMisesStress();
         }
         else {
            s = (float)node.getVonMisesStrain();
         }
         if (s < min) {
            min = s;
         }
         if (s > max) {
            max = s;
         }
      }
      return new DoubleInterval(min, max);
   }
   
   private void updateStressPlotRange() {
      
      if (mySurfaceRendering != SurfaceRender.Stress &&
         mySurfaceRendering != SurfaceRender.Strain) {
         return;
      }

      if (myStressPlotRanging == Ranging.Auto) {
         myStressPlotRange.merge (getNodalPlotRange(mySurfaceRendering));
      } 

   }

   public void render(Renderer renderer, int flags) {
      super.render(renderer, flags);
   }

   public void prerender(RenderList list) {
      list.addIfVisible(myNodes);
      list.addIfVisible(myElements);
      list.addIfVisible(myMarkers);
      list.addIfVisible(myMeshList);
      
      updateStressPlotRange();
      
      list.addIfVisible(myMeshList);
      myAuxMaterialsList.prerender(list);
   }

   public void getSelection(LinkedList<Object> list, int qid) {}

   protected void clearCachedData(ComponentChangeEvent e) {
      super.clearCachedData(e);
      mySolveMatrix = null;
      myBVTreeValid = false;
      myRestNodeTree = null;
   }

   private void handleGeometryChange() {

      myBVTreeValid = false;
      myRestNodeTree = null;
      invalidateStressAndStiffness();
      updatePosState(); // should this be updateSlavePos()?
   }

   public void componentChanged(ComponentChangeEvent e) {
      if (e.getCode() == ComponentChangeEvent.Code.STRUCTURE_CHANGED) {
         clearCachedData(null);
         // should invalidate elasticity
      } else if (e.getCode() == ComponentChangeEvent.Code.GEOMETRY_CHANGED) {
         handleGeometryChange();
      }
      notifyParentOfChange(e);
   }

   @Override
   protected void notifyStructureChanged(Object comp) {
      clearCachedData(null);
      super.notifyStructureChanged(comp);
   }

   public void updateSlavePos() {
      super.updateSlavePos();

      // nodes
      for (MFreeNode3d node : myNodes) {
         node.updatePosAndVelState();
      }

      // integration points
      for (MFreeElement3d elem : myElements) {
         for (MFreeIntegrationPoint3d mfip : elem.getIntegrationPoints()) {
            mfip.updatePosAndVelState();
         }
         MFreePoint3d warp = elem.getWarpingPoint();
         if (warp != null) {
            warp.updatePosAndVelState();
         }
      }

      // meshes
      for (MFreeMeshComp mc : myMeshList) {
         mc.updateSlavePos();
      }

      myBVTreeValid = false;
   }

   public void recursivelyFinalizeAdvance(
      StepAdjustment stepAdjust, double t0, double t1, int flags, int level) {

      // special implementation of updateVolume that checks for inverted
      // Jacobians
      double minDetJ = Double.MAX_VALUE;
      myNumInverted = 0;

      for (MFreeElement3d e : myElements) {
         FemMaterial mat = getElementMaterial(e);
         double detJ = e.computeVolumes();
         e.setInverted(false);
         if (!(mat instanceof LinearMaterial)) {
            if (detJ < minDetJ) {
               minDetJ = detJ;
            }
            if (detJ <= 0) {
               e.setInverted(true);
               myNumInverted++;
            }
         }
      }
      if (stepAdjust != null && minDetJ <= 0) {
         stepAdjust.recommendAdjustment(0.5, "element inversion");
      }
   }

   public double updateVolume() {
      // myVolume = integrateVolume();
      // myVolumeValid = true;
      updateAllVolumes();
      return myVolume;
   }

   public void invalidateStressAndStiffness() {
      super.invalidateStressAndStiffness();
      // should invalidate matrices for incompressibility here. However, at the
      // moment these are being rebuilt for each calculation anyway
   }

   public void invalidateRestData() {
      super.invalidateRestData();
   }

   public void resetRestPosition() {
      for (FemNode3d n : myNodes) {
         n.resetRestPosition();
      }
      invalidateRestData();
      notifyParentOfChange(new ComponentChangeEvent(Code.STRUCTURE_CHANGED));
   }
   
   /**
    * Update the blocks uses in the incompressibility constraint matrices.
    * These are stored in the myDviBlk fields of each FemNodeNeighbor.
    * Derivative values for inactive nodes are stored in b.
    */
   // DIVBLK
   private void updateHardNodalIncompInfo(VectorNd b, double time) {

      b.setZero();
      for (MFreeNode3d n : myNodes) {
         if (n.getIncompressIndex() != -1) {
            for (FemNodeNeighbor nbr : getNodeNeighbors(n)) {
               // if (isControllable (nbr.myNode)) {
               nbr.getDivBlk().setZero();
               // }
            }
         }
      }
      int idx;
      for (MFreeElement3d e : myElements) {
         MFreeNode3d[] enodes = e.getNodes();
         double dg = 0;
         for (int i = 0; i < myNodalConstraints.length; i++) {
            myNodalConstraints[i].setZero();
         }
         
         IntegrationPoint3d[] pt = e.getIntegrationPoints();
         IntegrationData3d[] dt = e.getIntegrationData();
         for (int k=0; k<pt.length; ++k) {
            pt[k].computeJacobianAndGradient(e.myNodes, dt[k].getInvJ0());
            pt[k].computeInverseJacobian();
            pt[k].updateShapeGradient(pt[k].getInvJ());
         }
         
         for (int i = 0; i < enodes.length; i++) {
            MFreeNode3d n = enodes[i];
            
            for (int k=0; k<pt.length; ++k) {
               double detJ = pt[k].getInvJ().determinant();
               double dv = detJ * pt[k].getWeight()*pt[k].getShapeWeights().get(i);
               Vector3d[] GNx = pt[k].getShapeGradient();
               for (int l = 0; l < GNx.length; k++) {
                  myNodalConstraints[l].scaledAdd(dv, GNx[k]);
               }   
            }

            if ((idx = n.getIncompressIndex()) != -1) {
               for (FemNodeNeighbor nbr : getNodeNeighbors(n)) {
                  MFreeNode3d nnode = (MFreeNode3d)nbr.getNode();
                  int j = e.getLocalNodeIndex(nnode);
                  if (j != -1) {
                     // if (isControllable (nnode)) {
                     nbr.getDivBlk().scaledAdd(1, myNodalConstraints[j]);
                     // }
                  }
               }
               b.add(idx, dg);
            }
         }
      }
   }

   /**
    * Computes the average deformation gradient for an element.
    */
   protected void computeAvgGNx(MFreeElement3d e) {

      IntegrationPoint3d[] ipnts = e.getIntegrationPoints();
      IntegrationData3d[] idata = e.getIntegrationData();

      Vector3d[] avgGNx = null;
      MatrixBlock[] constraints = null;

      constraints = e.getIncompressConstraints();
      for (int i = 0; i < e.myNodes.length; i++) {
         constraints[i].setZero();
      }

      e.setInverted(false);
      for (int k = 0; k < ipnts.length; k++) {
         IntegrationPoint3d pt = ipnts[k];
         pt.computeJacobianAndGradient(e.myNodes, idata[k].getInvJ0());
         double detJ = pt.computeInverseJacobian();
         if (detJ <= 0) {
            e.setInverted(true);
            // if (abortOnInvertedElems) {
            // throw new NumericalException ("Inverted elements");
            // }
         }
         double dv = detJ * pt.getWeight();
         Vector3d[] GNx = pt.updateShapeGradient(pt.getInvJ());

         double[] H = pt.getPressureWeights().getBuffer();
         for (int i = 0; i < e.myNodes.length; i++) {
            FemUtilities.addToIncompressConstraints(
               constraints[i], H, GNx[i], dv);
         }
      }
   }

   
   private void updateHardElementIncompInfo(VectorNd b, double time) {
      int ci = 0;

      IncompMethod softIncomp = getSoftIncompMethod();
      b.setZero();

      for (MFreeElement3d e : myElements) {
         if (e.getIncompressIndex() != -1) {
            if (softIncomp != IncompMethod.ELEMENT ||
            !getElementMaterial(e).isIncompressible() ||
            time == 0) {
               // need to do this at time=0 since stresses may not have been
               // computed yet
               computeAvgGNx(e);
            }
            for (int k = 0; k < e.numPressureVals(); k++) {
               b.set(ci++, 0);
            }
         }
      }
   }

   /**
    * Updates the divergence matrix. Returns true if the matrix
    * was recreated.
    */
   private void updateHardIncompInfo(double time) {

      if (time != myHardIncompUpdateTime) {
         myHardIncompUpdateTime = time;

         if (!myHardIncompConfigValidP) {
            configureHardIncomp();
         }
         if (getHardIncompMethod() == IncompMethod.NODAL) {
            updateHardNodalIncompInfo(myDg, time);
         }
         else if (getHardIncompMethod() == IncompMethod.ELEMENT) {
            updateHardElementIncompInfo(myDg, time);
         }
         else {
            throw new IllegalArgumentException(
               "unsupported hard incompress method " + getHardIncompMethod());
         }
      }
   }
   
   public double updateConstraints(double t, int flags) {
      if (!myVolumeValid) {
         updateVolume();
      }
      if (getHardIncompMethod() != IncompMethod.OFF) {
         updateHardIncompInfo(t);
      }
      return 0;
   }

   public void getConstrainedComponents(List<DynamicComponent> list) {
      if (getHardIncompMethod() != IncompMethod.OFF) {
         list.addAll(myNodes);
      }
   }

   public int setBilateralImpulses(VectorNd lam, double h, int idx) {
      IncompMethod hardIncomp = getHardIncompMethod();
      if (hardIncomp == IncompMethod.NODAL) {
         lam.getSubVector(idx, myIncompressLambda);
         idx += myNumIncompressConstraints;
      }
      else if (hardIncomp == IncompMethod.ELEMENT) {
         double[] buf = lam.getBuffer();
         for (int i = 0; i < myElements.size(); i++) {
            MFreeElement3d e = myElements.get(i);
            if (e.getIncompressIndex() != -1) {
               for (int k = 0; k < e.numPressureVals(); k++) {
                  e.myLagrangePressures[k] = buf[idx++];
               }
            }
         }
      }
      return idx;
   }

   public void zeroImpulses() {
      IncompMethod hardIncomp = getHardIncompMethod();
      if (hardIncomp == IncompMethod.NODAL) {
         myIncompressLambda.setZero();
      }
      else if (hardIncomp == IncompMethod.ELEMENT) {
         for (int i = 0; i < myElements.size(); i++) {
            MFreeElement3d e = myElements.get(i);
            if (e.getIncompressIndex() != -1) {
               for (int k = 0; k < e.numPressureVals(); k++) {
                  e.myLagrangePressures[k] = 0;
               }
            }
         }
      }
   }

   public int getBilateralImpulses(VectorNd lam, int idx) {
      IncompMethod hardIncomp = getHardIncompMethod();
      if (hardIncomp == IncompMethod.NODAL) {
         lam.setSubVector(idx, myIncompressLambda);
         idx += myNumIncompressConstraints;
      }
      else if (hardIncomp == IncompMethod.ELEMENT) {
         double[] buf = lam.getBuffer();
         for (int i = 0; i < myElements.size(); i++) {
            MFreeElement3d e = myElements.get(i);
            if (e.getIncompressIndex() != -1) {
               for (int k = 0; k < e.numPressureVals(); k++) {
                  buf[idx++] = e.myLagrangePressures[k];
               }
            }
         }
      }
      return idx;
   }

   public void getBilateralSizes(VectorNi sizes) {
      IncompMethod hardIncomp = getHardIncompMethod();
      if (hardIncomp != IncompMethod.OFF) {
         if (!myHardIncompConfigValidP) {
            configureHardIncomp();
         }
         if (hardIncomp == IncompMethod.NODAL) {
            for (int i = 0; i < myNumIncompressConstraints; i++) {
               sizes.append(1);
            }
         }
         else if (hardIncomp == IncompMethod.ELEMENT) {
            for (int i = 0; i < myElements.size(); i++) {
               MFreeElement3d e = myElements.get(i);
               if (e.getIncompressIndex() != -1) {
                  sizes.append(e.numPressureVals());
               }
            }
         }
      }
   }

   // DIVBLK
   public int addBilateralConstraints(
      SparseBlockMatrix GT, VectorNd dg, int numb) {
      IncompMethod hardIncomp = getHardIncompMethod();
      if (hardIncomp != IncompMethod.OFF) {

         // add the necessary columns to the matrix
         int ncons = myNumIncompressConstraints;

         int bj = GT.numBlockCols();
         if (hardIncomp == IncompMethod.NODAL) {
            // for TET case, ncons equals number of constraint blocks
            for (int j = 0; j < ncons; j++) {
               GT.addCol(1);
            }
            // For controllable node, add the incompressibility constraint
            for (MFreeNode3d n : myNodes) {
               if (n.getIncompressIndex() != -1) {
                  for (FemNodeNeighbor nbr : getNodeNeighbors(n)) {
                     // if (isControllable (nbr.myNode)) {
                        GT.addBlock(
                           nbr.getNode().getSolveIndex(), bj, nbr.getDivBlk());
                        // }
                  }
                  bj++;
               }
            }
         }
         else if (hardIncomp == IncompMethod.ELEMENT) {
            for (MFreeElement3d e : myElements) {
               if (e.getIncompressIndex() != -1) {
                  MatrixBlock[] constraints;
                  constraints = e.getIncompressConstraints();
                  for (int i = 0; i < e.numNodes(); i++) {
                     MFreeNode3d n = e.myNodes[i];
                     // if (isControllable (n)) {
                     GT.addBlock(n.getSolveIndex(), bj, constraints[i]);
                        // }
                  }
                  bj++;
               }
            }
         }
         if (dg != null) {
            double[] dbuf = dg.getBuffer();
            for (int i = 0; i < ncons; i++) {
               dbuf[numb + i] = myDg.get(i);
            }
         }
         numb += ncons;
      }
      return numb;
   }
   
   private double getLocalVolumeError(
      int nidx, MFreeNode3d[] nodes, MFreeIntegrationPoint3d[] pt, IntegrationData3d[] dt) {

      double vol = 0;
      double volr = 0;
      for (int k=0; k<pt.length; ++k) {
         // pt.computeJacobian(nodes);
         double sw = pt[k].getShapeWeights().get(nidx);
         double detJ = pt[k].getJ().determinant();
         vol += detJ * pt[k].getWeight()*sw;
         volr += dt[k].getDetJ0()*pt[k].getWeight()*sw;
      }
      return (vol - volr);
   }

   public int getBilateralInfo(ConstraintInfo[] ginfo, int idx) {
      IncompMethod hardIncomp = getHardIncompMethod();
      if (hardIncomp != IncompMethod.OFF) {
         int ncols = myNumIncompressConstraints;
         int ci;

         double damping = 0;
         if (myIncompCompliance != 0) {
            // set critical damping
            double mass = getMass() / myNumIncompressConstraints;
            damping = 2 * Math.sqrt(mass / myIncompCompliance);
         }

         if (!myVolumeValid) {
            updateVolume();
         }
         if (hardIncomp == IncompMethod.NODAL) {
            for (ci = 0; ci < ncols; ci++) {
               ConstraintInfo gi = ginfo[idx + ci];
               gi.dist = 0; // values will be accumulated below
               gi.compliance = myIncompCompliance;
               gi.damping = damping;
               gi.force = 0;
            }

            for (MFreeElement3d elem : myElements) {

               MFreeNode3d[] nodes = elem.myNodes;
               MFreeIntegrationPoint3d[] ipnts = elem.getIntegrationPoints();
               IntegrationData3d[] idata = elem.getIntegrationData();
               double verr;
               for (int i = 0; i < nodes.length; i++) {
                  if ((ci = nodes[i].getIncompressIndex()) != -1) {
                     verr = getLocalVolumeError(i, nodes, ipnts, idata);
                     ginfo[idx + ci].dist += verr;
                  }
               }
            }
         }
         else if (hardIncomp == IncompMethod.ELEMENT) {
            ci = idx;
            for (MFreeElement3d e : myElements) {
               e.getRestVolume(); // makes sure rest volume is updated
               if (e.getIncompressIndex() != -1) {
                  for (int k = 0; k < e.numPressureVals(); k++) {
                     ConstraintInfo gi = ginfo[ci++];
                     gi.dist = e.myVolumes[k] - e.myRestVolumes[k];
                     gi.compliance = myIncompCompliance;
                     gi.damping = damping;
                     gi.force = 0;
                  }
               }
            }
         }
         idx += ncols;
      }
      return idx;
   }

   public static void addControls(
      ControlPanel controlPanel, FemModel femModel, ModelComponent topModel) {
      FemControlPanel.addFemControls(controlPanel, femModel, topModel);
      controlPanel.addWidget(femModel, "surfaceRendering");
      controlPanel.addWidget(femModel, "elementWidgetSize", 0, 1.0);
   }

   public void dispose() {}

   @Override
   public void scaleDistance(double s) {
      super.scaleDistance(s);
      myVolume *= (s * s * s);
   }

   /**
    * {@inheritDoc}
    */
   public boolean isDuplicatable() {
      return false;
   }

   public void transformGeometry(AffineTransform3dBase X) {
      TransformGeometryContext.transform(this, X, 0);
   }

   public void transformGeometry(
      GeometryTransformer gtr, TransformGeometryContext context, int flags) {

      // This is now handled in the node.transformGeometry():
      // for (MFreeNode3d n : myNodes) {
      // n.getRestPosition().transform(gtr);
      // }
      for (MFreeElement3d elem : myElements) {
         elem.invalidateRestData();
         for (MFreeIntegrationPoint3d ipnt : elem.getIntegrationPoints()) {
            ipnt.updateRestPosition();
            ipnt.updatePosAndVelState();
         }
      }
      updateLocalAttachmentPos();
      invalidateStressAndStiffness();
      updatePosState();

      if (myMinBound != null) {
         gtr.transformPnt(myMinBound);
      }
      if (myMaxBound != null) {
         gtr.transformPnt(myMaxBound);
      }
   }

   public void addTransformableDependencies(
      TransformGeometryContext context, int flags) {
      context.addAll(myNodes);
   }

   public boolean getCopyReferences(
      List<ModelComponent> refs, ModelComponent ancestor) {
      // TODO Auto-generated method stub
      return false;
   }

   public void getAuxStateComponents(List<HasAuxState> comps, int level) {
      // TODO Auto-generated method stub

   }

   // public CountedList<MFreePoint3d> getEvaluationPoints() {
   // return myEvaluationPoints;
   // }

   public void computeShapeMatrix() {

      M = new SparseBlockMatrix();
      b = new VectorNd();
      // for (int i = 0; i< numNodes(); i++) {
      // MFreeNode3d nodei = myNodes.get(i);
      // VectorNd coords = nodei.getNodeCoordinates();
      // ArrayList<MFreeNode3d> deps = nodei.getDependentNodes();
      // for (int j=0; j<deps.size(); j++) {
      // int col = deps.get(j).getNumber();
      // M.set(i,col,coords.get(j));
      // }
      // }

      // LUDecomposition lu = new LUDecomposition(M);
      // MatrixNd out = new MatrixNd();
      // lu.inverse(out);
      //
      // System.out.println("M = [");
      // for (int i=0; i<numNodes(); i++) {
      // for (int j=0; j<numNodes(); j++) {
      // System.out.print(" " + M.get(i, j));
      // }
      // System.out.println();
      // }
      // System.out.println("];");
      //
      // System.out.println("Mi = [");
      // for (int i=0; i<numNodes(); i++) {
      // for (int j=0; j<numNodes(); j++) {
      // System.out.print(" " + out.get(i, j));
      // }
      // System.out.println();
      // }
      // System.out.println("];");

   }

   @Override
   public void updateBounds(Vector3d pmin, Vector3d pmax) {
      // TODO Auto-generated method stub
      updatePosState();
      super.updateBounds(pmin, pmax);
      for (MFreeMeshComp mc : myMeshList) {
         mc.updateBounds(pmin, pmax);
      }
   }

   public ColorMapBase getColorMap() {
      return myColorMap;
   }

   public void setColorMap(ColorMapBase colorMap) {
      myColorMap = colorMap;
      myColorMapMode =
         PropertyUtils
         .propagateValue(this, "colorMap", colorMap, myColorMapMode);
   }

   public PropertyMode getColorMapMode() {
      return myColorMapMode;
   }

   public void setColorMapMode(PropertyMode mode) {
      if (mode != myColorMapMode) {
         myColorMapMode =
            PropertyUtils
            .setModeAndUpdate(this, "colorMap", myColorMapMode, mode);
      }
   }

   public void getVertexMasters(List<ContactMaster> mlist, Vertex3d vtx) {

      // XXX currently assumed vtx is instance of MFreeVertex3d
      // This will change once MFreeModel uses FemMeshComp equivalent
      if (vtx instanceof MFreeVertex3d) {
         MFreeVertex3d mvtx = (MFreeVertex3d)vtx;

         MFreeNode3d[] nodes = mvtx.getDependentNodes();
         VectorNd coords = mvtx.getNodeCoordinates();

         for (int j = 0; j < nodes.length; j++) {
            mlist.add(new ContactMaster(nodes[j], coords.get(j)));
         }
      } else {
         System.out.println("Unknown masters.");
      }
   }

   public boolean containsContactMaster(CollidableDynamicComponent comp) {
      return comp.getParent() == myNodes;
   }

   public boolean allowCollision(
      ContactPoint cpnt, Collidable other, Set<Vertex3d> attachedVertices) {
      if (CollisionHandler.attachedNearContact(cpnt, other, attachedVertices)) {
         return false;
      }
      return true;
   }

   public int getCollidableIndex() {
      return myCollidableIndex;
   }
   
   public void setCollidableIndex (int idx) {
      myCollidableIndex = idx;
   }
   
   @Override
   public void getMassMatrixValues(SparseBlockMatrix M, VectorNd f, double t) {
      int bi;

      for (int i = 0; i < myNodes.size(); i++) {
         FemNode3d n = myNodes.get(i);
         if ((bi = n.getSolveIndex()) != -1) {
            n.getEffectiveMass(M.getBlock(bi, bi), t);
            n.getEffectiveMassForces(f, t, M.getBlockRowOffset(bi));
         }
      }
   }

   @Override
   public void mulInverseMass(SparseBlockMatrix M, VectorNd a, VectorNd f) {

      double[] abuf = a.getBuffer();
      double[] fbuf = f.getBuffer();
      int asize = a.size();

      if (M.getAlignedBlockRow(asize) == -1) {
         throw new IllegalArgumentException(
            "size of 'a' not block aligned with 'M'");
      }
      if (f.size() < asize) {
         throw new IllegalArgumentException(
            "size of 'f' is less than the size of 'a'");
      }
      for (int i = 0; i < myNodes.size(); i++) {
         FemNode3d n = myNodes.get(i);
         int bk = n.getSolveIndex();
         if (bk != -1) {
            int idx = M.getBlockRowOffset(bk);
            if (idx < asize) {
               n.mulInverseEffectiveMass(M.getBlock(bk, bk), abuf, fbuf, idx);
            }
         }
      }
   }
   
   private static class NearestIPointCalculator implements ObjectDistanceCalculator {

      Point3d myPnt;
      MFreeIntegrationPoint3d myIpnt;
      MFreeElement3d myElem;
      double myDist;
      
      public NearestIPointCalculator(Point3d pnt) {
         myPnt = new Point3d(pnt);
         reset();
      }
      
      @Override
      public void reset() {
         myIpnt = null;
         myElem = null;
         myDist = Double.POSITIVE_INFINITY;
      }
      
      @Override
      public double nearestDistance(BVNode node) {
         return node.distanceToPoint(myPnt);
      }

      @Override
      public double nearestDistance(Boundable e) {
         MFreeElement3d elem = (MFreeElement3d)e;
         double dmin = Double.MAX_VALUE;
         for (MFreeIntegrationPoint3d cb : elem.getIntegrationPoints()) {
            double d = cb.getPosition().distance(myPnt);
            if (d < dmin) {
               dmin = d;
               myDist = dmin;
               myElem = elem;
               myIpnt = cb;
            }
         }
         return dmin;
      }

      @Override
      public MFreeElement3d nearestObject() {
         return myElem;
      }
      
      public MFreeIntegrationPoint3d nearestIPoint() {
         return myIpnt;
      }
      
      @Override
      public double nearestDistance() {
         return myDist;
      }
      
   }
   
   /**
    * Finds the containing element and node coordinates
    * @param pnt
    * @param coords natural coordinates
    * @param N shape function values
    * @return the containing element if it exists
    */
   public MFreeElement3d findContainingElement(Point3d pnt, Point3d coords, VectorNd N) {
      
      BVFeatureQuery query = new BVFeatureQuery();
      
      NearestIPointCalculator dcalc 
         = new NearestIPointCalculator(pnt);
      query.nearestObject(getElementBVTree(), dcalc);
      
      MFreeElement3d elem = dcalc.nearestObject();
      MFreeIntegrationPoint3d ipnt = dcalc.nearestIPoint();
      
      // try to compute coords
      coords.set(ipnt.getRestPosition());
      int n = elem.getNaturalCoordinates(coords, pnt, 1000, N);
      if (n >= 0 && elem.coordsAreInside(coords)) {
         return elem;
      }
      
      // otherwise we failed, but should be inside *SOME* element??
      
      return null;
      
   }
   
   /**
    * Finds the containing element and node coordinates
    * @param pnt
    * @param coords natural coordinates
    * @param N shape function values
    * @return the containing element if it exists
    */
   public MFreeNode3d[] findNaturalCoordinates(Point3d pnt, Point3d coords, VectorNd N) {
      
      BVFeatureQuery query = new BVFeatureQuery();
      
      NearestIPointCalculator dcalc 
         = new NearestIPointCalculator(pnt);
      query.nearestObject(getElementBVTree(), dcalc);
      
      MFreeElement3d elem = dcalc.nearestObject();
      MFreeIntegrationPoint3d ipnt = dcalc.nearestIPoint();
      
      // try to compute coords
      coords.set(ipnt.getRestPosition());
      int n = elem.getNaturalCoordinates(coords, pnt, 1000, N);
      return elem.getNodes();
   }
   
   /**
    * Finds the nearest element and node coordinates
    * @param nearest nearest point
    * @param N shape function evaluation at point
    * @return the nearest element
    */
   public MFreeElement3d findNearestElement(Point3d nearest, Point3d pnt, VectorNd N) {
      
      BVFeatureQuery query = new BVFeatureQuery();
      
      NearestIPointCalculator dcalc 
         = new NearestIPointCalculator(pnt);
      query.nearestObject(getElementBVTree(), dcalc);
      
      MFreeElement3d elem = dcalc.nearestObject();
      MFreeIntegrationPoint3d ipnt = dcalc.nearestIPoint();
      
      nearest.set(ipnt.getPosition());
      N.set(ipnt.getShapeWeights());
      
      return elem;
      
   }
   
   // builds a Stiffness matrix, where entries are ordered by node numbers
   public SparseBlockMatrix getStiffnessMatrix() {

      if (!myStressesValidP || !myStiffnessesValidP) {
         updateStressAndStiffness();
      }

      SparseBlockMatrix M = new SparseBlockMatrix();
      int nnodes = numNodes();
      int[] sizes = new int[nnodes];
      for (int i=0; i<nnodes; ++i) {
         sizes[i] = 3;
      }
      M.addRows (sizes, sizes.length);
      M.addCols (sizes, sizes.length);
      M.setVerticallyLinked (true);

      int idx = 0;
      for (MFreeNode3d node : getNodes()) {
         node.setIndex(idx++);
      }

      // create solve blocks
      for (MFreeNode3d node : getNodes()) {
         MatrixBlock blk = node.createSolveBlock();
         M.addBlock(node.getIndex(), node.getIndex(), blk);
      }
      for (int i = 0; i < myNodes.size(); i++) {
         MFreeNode3d node = myNodes.get(i);
         for (FemNodeNeighbor nbr : getNodeNeighbors(node)) {
            MFreeNode3d other = (MFreeNode3d)nbr.getNode();
            Matrix3x3Block blk = null;
            if (other != node) {
               blk = new Matrix3x3Block();
               M.addBlock(node.getIndex(), nbr.getNode().getIndex(), blk);
            } else {
               blk = (Matrix3x3Block)M.getBlock(node.getIndex(), node.getIndex());
            }
            nbr.addPosJacobian(blk, -1.0);
         }
      }

      // System.out.println ("symmetric=" + mySolveMatrix.isSymmetric(1e-6));

      return M;
   }
   
   private static class RestNode implements Boundable {

      MFreeNode3d node;
      Point3d[] pnts;
      
      public RestNode(MFreeNode3d node, double r) {
         this.node = node;
         pnts = new Point3d[2];
         Point3d pos = node.getRestPosition();
         pnts[0] = new Point3d(pos.x+r, pos.y+r, pos.z+r);
         pnts[1] = new Point3d(pos.x-r, pos.y-r, pos.z-r);
      }
      
      public MFreeNode3d getNode() {
         return node;
      }
      
      @Override
      public int numPoints() {
         return 2;
      }

      @Override
      public Point3d getPoint(int idx) {
         return pnts[idx];
      }

      @Override
      public void computeCentroid(Vector3d centroid) {
         centroid.set(node.getRestPosition());
      }

      @Override
      public void updateBounds(Vector3d min, Vector3d max) {
         pnts[0].updateBounds(min, max);
         pnts[1].updateBounds(min, max);
      }

      @Override
      public double computeCovariance(Matrix3d C) {
         return -1;
      }
   }
   
   private static AABBTree buildRestNodeTree(Collection<MFreeNode3d> nodes) {
      AABBTree nodeTree = new AABBTree();
      RestNode[] rnodes = new RestNode[nodes.size()];
      int idx = 0;
      for (MFreeNode3d node : nodes) {
         rnodes[idx] = new RestNode(node, node.getInfluenceRadius());
         ++idx;
      }
      nodeTree.build(rnodes, rnodes.length);
      return nodeTree;
   }
   
   /**
    * Finds nodes containing the given point at rest
    * @param pnt point to find nodes for
    * @param out output list of nodes influencing region
    * @return number of nodes found
    */
   public int findDependentNodesAtRest(Point3d pnt, List<MFreeNode3d> out) {
      
      AABBTree nodeTree = myRestNodeTree;
      if (nodeTree == null) {
         nodeTree = buildRestNodeTree(myNodes);
         myRestNodeTree = nodeTree;
      }

      ArrayList<BVNode> bvNodes = new ArrayList<BVNode>(16);
      nodeTree.intersectPoint(bvNodes, pnt);

      if (bvNodes.size() == 0) {
         return 0;
      }

      int count = 0;
      for (BVNode n : bvNodes) {
         Boundable[] elements = n.getElements();
         for (int i = 0; i < elements.length; i++) {
            RestNode rnode = (RestNode)elements[i];
            MFreeNode3d node = rnode.getNode();
            if (node.isInDomain(pnt, 0)) {
               out.add(node);
               ++count;
            }
         }
      }
      return count;
   }

   /**
    * Adds a marker to this FemModel. The element to which it belongs is
    * determined automatically. If the marker's current position does not lie
    * within the model and {@code project == true}, it will be projected onto 
    * the model's surface.
    * 
    * @param pos
    * position to place a marker in the model
    * @param project
    * if true and pnt is outside the model, projects to the nearest point
    * on the surface.  Otherwise, uses the original position.
    * 
    */
   public FemMarker addMarker(Point3d pos) {
      FemMarker mkr = new FemMarker();
      Point3d coord = new Point3d();
      VectorNd N = new VectorNd();
      
      MFreeNode3d[] nodes =  findNaturalCoordinates(pos, coord, N);
     
      mkr.setPosition(pos);
      double[] wgts = new double[N.size()];
      for (int i=0; i<N.size(); ++i) {
         wgts[i] = N.get(i);
      }
      mkr.setFromNodes(nodes, wgts);
      addMarker(mkr);
      return mkr;
   }
   
   public Renderable getSurfaceMeshComp() {
      return mySurfaceMesh;
   }
   
   public PolygonalMesh getSurfaceMesh() {
      if (mySurfaceMesh != null) {
         return (PolygonalMesh)mySurfaceMesh.getMesh();
      }
      return null;
   }

}
