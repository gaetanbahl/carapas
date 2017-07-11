package artisynth.demos.fem;

import java.awt.Color;

import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.ShellFemModel3d;
import artisynth.core.femmodels.ShellFemNode3d;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.core.materials.NeoHookeanMaterial;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.workspace.RootModel;
import maspack.geometry.Face;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;

/**
 * Square patch of triangular shell elements, subjected to gravity. 
 * The boundary nodes will be held in-place.
 * 
 * @author Danny Huang (dah208@mail.usask.ca). Feel free to contact me for help.
 */
public class ShellPatch extends RootModel {
   protected ShellFemModel3d m_femShellModel = null;
   protected MechModel m_mechModel = null;

   // Width and length of square patch of shell elements
   public final int mMeshX = 15;                
   public final int mMeshY = 15;
   
   // Number of shell elements per row
   public final int mMeshXDiv = 15;
   
   // Number of shell elements per column
   public final int mMeshYDiv = 15;
   
   // Not part of shell elements but used to generate a square lattice of node
   // positions.
   protected PolygonalMesh mMesh = null;
   
   protected ShellFemNode3d[] m_nodes = null;
   
   // Overall density of shell patch
   protected final double m_density = 10000;
   
   // Generic particle velocity damping
   protected final double m_particleDamping = 2;
   
   // Element stiffness. 0 for water-like, 100 for aluminium-like
   protected final double m_stiffnessDamping = 0.5;                
   
   // Rendering radius of nodes
   protected final double mNodeRadius = 0.1;
   
   // Dynamic nodes will be given this color.
   protected final Color mNodeDynamicColor = Color.CYAN;
   
   // Non-dynamic (i.e. frozen) nodes will be given this color.
   protected final Color mNodeNonDynamicColor = Color.GRAY;
   
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
         ShellTriElement ele = new ShellTriElement(n0, n1, n2);
         m_femShellModel.addElement(ele);
      }

      m_femShellModel.setMaterial (new NeoHookeanMaterial());
      m_femShellModel.setStiffnessDamping (m_stiffnessDamping);
      m_femShellModel.setGravity (mGravity);
      m_femShellModel.setDensity (m_density);
      m_femShellModel.setParticleDamping (m_particleDamping);

      m_mechModel.addModel (m_femShellModel);
      addModel (m_mechModel);
      
      // For the nodes on the border of the shell patch, hold them in-place.
      for (ShellFemNode3d node : m_nodes) {
         node.setRenderProps( node.createRenderProps() );
         if (isBorderNode(node.getIndex())) {
            node.getRenderProps ().setPointColor (Color.GREEN);
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
      
      System.out.println ("Number of elements: " +
                              m_femShellModel.numElements());
   }   
   
   
   /**
    * Is the node on the border of the shell patch?
    */
   public boolean isBorderNode(int idx) {
//    return (idx == 0 || idx == 1);

      if (idx <= mMeshXDiv ||                   // bottom edge nodes
          idx % (mMeshXDiv+1) == 0 ||           // left edge nodes
          idx >= mMeshXDiv*(mMeshYDiv+1) &&     // top edge nodes
              idx <= mMeshXDiv*(mMeshYDiv+2))
         return true;
      
      // Right edge nodes
      
      int leftDigit = idx % mMeshXDiv;
      int rightDigits = idx - leftDigit;
      rightDigits /= mMeshXDiv;
      
      if (rightDigits == leftDigit+1)
         return true;
      
      return false;
   }
}
