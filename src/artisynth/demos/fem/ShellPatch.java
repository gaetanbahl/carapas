package artisynth.demos.fem;

import java.awt.Color;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.ShellFemElement3d;
import artisynth.core.femmodels.ShellFemModel3d;
import artisynth.core.femmodels.ShellFemNode3d;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.core.gui.ControlPanel;
import artisynth.core.materials.NeoHookeanMaterial;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.workspace.RootModel;
import maspack.geometry.Face;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer;

/**
 * Square patch of triangular shell elements, subjected to gravity. 
 * Some nodes will be held in-place to demonstrate shell bending forces.
 */
public class ShellPatch extends RootModel {
   protected ShellFemModel3d m_femShellModel = null;
   protected MechModel m_mechModel = null;

   // Width and length of square patch of shell elements
   public final int mMeshX = 13;                
   public final int mMeshY = 13;
   
   // Number of shell elements per row
   public final int mMeshXDiv = 13;
   
   // Number of shell elements per column
   public final int mMeshYDiv = 13;
   
   // Not part of shell elements but used to generate a square lattice of node
   // positions.
   protected PolygonalMesh mMesh = null;
   
   protected ShellFemNode3d[] m_nodes = null;
   
   // Overall density of shell patch
   protected final double m_density = 100;
   
   // Generic particle velocity damping
   protected final static double m_particleDamping = 0;
   
   // Element stiffness. 0 for water-like, 100 for aluminium-like
   protected final static double m_stiffnessDamping = 0.05;              
   
   // Affects bending strength
   protected final static double m_shellThickness = 0.1;
   
   // Affects bending and shear strength
   protected final double m_neoHookYoungsModulus = 100000000;
   protected final double m_neoHookPoissonsRatio = 0.33;
   
   // Rendering radius of nodes
   protected final double mNodeRadius = 0.1;
   
   // Dynamic nodes will be given this color.
   protected final Color mNodeDynamicColor = Color.CYAN;
   
   // Non-dynamic (i.e. frozen) nodes will be given this color.
   protected final Color mNodeNonDynamicColor = Color.RED;
   
   protected final Vector3d mGravity = new Vector3d(0, 0, -9.81);

   
   
   public void build (String[] args) {
      m_mechModel = new MechModel ("mech");
      m_femShellModel = new ShellFemModel3d();

      // Create a square lattice of node positions, represented as mesh 
      // vertices.
      mMesh = MeshFactory.createPlane(mMeshX, mMeshY, mMeshXDiv, mMeshYDiv);
      m_nodes = new ShellFemNode3d[mMesh.numVertices()];
      
      // For each 3-vertex mesh face...
      for (Face face : mMesh.getFaces()) {
         // Create a fem node for each corresponding mesh vertex of the mesh
         // face...
         Vertex3d[] triVtx = face.getTriVertices();
         for (Vertex3d vertex : triVtx) {
            int v = vertex.getIndex();
            if (m_nodes[v] == null) {
               m_nodes[v] = new ShellFemNode3d(vertex.getPosition());
               m_nodes[v].setIndex(v);
               m_femShellModel.addNumberedNode(m_nodes[v], v);
            }
         }
         
         ShellFemNode3d n0 = m_nodes[ triVtx[0].getIndex() ];
         ShellFemNode3d n1 = m_nodes[ triVtx[1].getIndex() ];
         ShellFemNode3d n2 = m_nodes[ triVtx[2].getIndex() ];

         // Create a shell fem element for these 3 fem nodes
         ShellTriElement ele = new ShellTriElement(n0, n1, n2, m_shellThickness);
         m_femShellModel.addElement(ele);
      }

      m_femShellModel.setMaterial (
         new NeoHookeanMaterial(m_neoHookYoungsModulus, m_neoHookPoissonsRatio));
      m_femShellModel.setStiffnessDamping (m_stiffnessDamping);
      m_femShellModel.setGravity (mGravity);
      m_femShellModel.setDensity (m_density);
      m_femShellModel.setParticleDamping (m_particleDamping);

      m_mechModel.addModel (m_femShellModel);
      addModel (m_mechModel);
      
      // Hold some nodes in-place
      for (ShellFemNode3d node : m_nodes) {
         node.setRenderProps( node.createRenderProps() );
         if (shouldBeFrozen(node.getIndex())) {
            node.getRenderProps ().setPointColor (mNodeNonDynamicColor);
            node.setDynamic (false);
         }
      }
      
      // Setup rendering options
      m_femShellModel.setSurfaceRendering (SurfaceRender.Shaded);
      RenderProps.setFaceColor (m_femShellModel, Color.PINK);
      RenderProps.setShininess (
         m_femShellModel, m_femShellModel.getRenderProps().getShininess() * 10);
      RenderProps.setVisible (m_femShellModel, true);
      RenderProps.setFaceStyle (m_femShellModel, Renderer.FaceStyle.FRONT);
      RenderProps.setPointStyle (m_femShellModel.getNodes(), 
                                 Renderer.PointStyle.SPHERE);
      RenderProps.setPointRadius (m_femShellModel.getNodes(), mNodeRadius);
      
      // Add control panel
      ControlPanel panel = new ControlPanel("shellPatchControlPanel");
      panel.addWidget (this, "shellThickness", m_shellThickness, 10);
      panel.addWidget (m_femShellModel, "particleDamping");
      panel.addWidget (m_femShellModel, "stiffnessDamping", 0, 2000);
      panel.addWidget (m_femShellModel, "density", 10, 1000);
      panel.addWidget (m_femShellModel, "gravity");
      panel.addWidget (m_femShellModel.getMaterial (), "YoungsModulus");
      panel.addWidget (m_femShellModel.getMaterial (), "PoissonsRatio");
      addControlPanel(panel);

      System.out.println ("Number of elements: " +
                              m_femShellModel.numElements());
   }    
   
   public static PropertyList myProps =
      new PropertyList (ShellPatch.class, RootModel.class);
   
   static {
      myProps.add("shellThickness", "Thickness of each shell element", 
                  m_shellThickness);
   }
   
   @Override
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public double getShellThickness() {
      return ((ShellFemElement3d)m_femShellModel.getElement(0)).getShellThickness();
   }
   
   public void setShellThickness(double newThickness) {
      for (FemElement3d e : m_femShellModel.getElements()) {
         ShellFemElement3d se = (ShellFemElement3d) e;
         se.setShellThickness (newThickness);
      }
      
      System.out.println ("Num control panels: " + this.getControlPanels ().size ());
   }
   
   /**
    * Should this node be non-dynamic?
    */
   public boolean shouldBeFrozen(int idx) {
      return (idx <= mMeshXDiv);        // bottom edge nodes
      
//      if (idx <= mMeshXDiv ||                   // bottom edge nodes
//          idx % (mMeshXDiv+1) == 0 ||           // left edge nodes
//          idx >= mMeshXDiv*(mMeshYDiv+1) &&     // top edge nodes
//              idx <= mMeshXDiv*(mMeshYDiv+2))
//         return true;
//      
//      // Right edge nodes
//      
//      int leftDigit = idx % mMeshXDiv;
//      int rightDigits = idx - leftDigit;
//      rightDigits /= mMeshXDiv;
//      
//      if (rightDigits == leftDigit+1)
//         return true;
//      
//      return false;
   }
}
