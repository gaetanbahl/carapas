package artisynth.demos.fem;

import java.awt.Color;

import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.ShellFemModel3d;
import artisynth.core.femmodels.ShellFemNode3d;
import artisynth.core.femmodels.ShellQuadElement;
import artisynth.core.materials.NeoHookeanMaterial;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.workspace.RootModel;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;

/**
 * Interactive demo of a single square shell element of 4 shell nodes and 
 * 8 gauss points. Drag the nodes around.
 */
public class SingleShellQuad extends RootModel {
   protected ShellFemModel3d m_femShellModel;
   protected MechModel m_mechModel;

   protected ShellFemNode3d m_node0;
   protected ShellFemNode3d m_node1;
   protected ShellFemNode3d m_node2;
   protected ShellFemNode3d m_node3;
   
   protected final double m_density = 100;
   protected final double m_particleDamping = 10;              
   protected final double m_nodeRadius = 0.05;

   public void build (String[] args) {
      m_femShellModel = new ShellFemModel3d();
      
      m_node0 = new ShellFemNode3d (0, 0, 0);        
      m_node1 = new ShellFemNode3d (1, 0, 0);      
      m_node2 = new ShellFemNode3d (1, 1, 0);    
      m_node3 = new ShellFemNode3d (0, 1, 0);        
     
      ShellQuadElement el = new ShellQuadElement(m_node0, m_node1, m_node2, m_node3, 0.05);
      m_femShellModel.addNode (m_node0);
      m_femShellModel.addNode (m_node1);
      m_femShellModel.addNode (m_node2);
      m_femShellModel.addNode (m_node3);
      m_femShellModel.addElement (el);
      
      m_femShellModel.setMaterial (new NeoHookeanMaterial());
      m_femShellModel.setGravity (0, 0, 0);
      m_femShellModel.setDensity (m_density);
      m_femShellModel.setParticleDamping (m_particleDamping);

      m_femShellModel.setSurfaceRendering (SurfaceRender.Shaded);
      RenderProps.setFaceColor (m_femShellModel, Color.PINK);
      RenderProps.setShininess (m_femShellModel, m_femShellModel.getRenderProps().getShininess() * 10);
      RenderProps.setVisible (m_femShellModel, true);
      RenderProps.setFaceStyle (m_femShellModel, Renderer.FaceStyle.FRONT);
      RenderProps.setPointStyle (m_femShellModel.getNodes(), Renderer.PointStyle.SPHERE);
      RenderProps.setPointRadius (m_femShellModel.getNodes(), m_nodeRadius);

      m_mechModel = new MechModel ("mech");
      m_mechModel.addModel (m_femShellModel);
      addModel (m_mechModel);
   }   
   
   
   /**
    * For each node, draw a white arrow between the rest position and current
    * position (i.e. displacement).
    * 
    * Also, draw a red arrow of the 3dof direction vector, starting from the
    * node current position. You'll notice that this red arrow is identical
    * to the white arrow.
    */
   public void render (Renderer renderer, int flags) {
      super.render (renderer, flags);
      
      for (FemNode3d n : m_femShellModel.getNodes()) {
         ShellFemNode3d sn = (ShellFemNode3d) n;
         
         Vector3d restPos = sn.getRestPosition ();
         Vector3d curPos = sn.getPosition ();
         
         renderer.setColor (Color.WHITE);
         renderer.drawArrow (restPos, curPos, 0.01, true);
         
         Vector3d curDir = new Vector3d(curPos);
         curDir.add (sn.getDir ());
         
         renderer.setColor (Color.RED);
         renderer.drawArrow (curPos, curDir, 0.01, true);
        
      }
   }
}
