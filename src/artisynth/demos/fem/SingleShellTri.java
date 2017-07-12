package artisynth.demos.fem;

import java.awt.Color;

import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.ShellFemModel3d;
import artisynth.core.femmodels.ShellFemNode3d;
import artisynth.core.femmodels.ShellTriElement;
import artisynth.core.materials.NeoHookeanMaterial;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.workspace.RootModel;
import maspack.render.RenderProps;
import maspack.render.Renderer;

/**
 * Interactive demo of a single triangle shell element of 3 shell nodes and 
 * 9 gauss points. Drag the nodes around via force.
 * 
 * @author Danny Huang (dah208@mail.usask.ca). Feel free to contact me for help.
 */
public class SingleShellTri extends RootModel {
   protected ShellFemModel3d m_femShellModel;
   protected MechModel m_mechModel;

   protected ShellFemNode3d m_node0;
   protected ShellFemNode3d m_node1;
   protected ShellFemNode3d m_node2;
   protected ShellFemNode3d m_node3;
   
   protected final double m_density = 10000;
   protected final double m_particleDamping = 10;       
   protected final double m_nodeRadius = 0.05;

   public void build (String[] args) {
      m_femShellModel = new ShellFemModel3d();
      
      m_node0 = new ShellFemNode3d (0, 0, 0);      
      m_node1 = new ShellFemNode3d (1, 0, 0);       
      m_node2 = new ShellFemNode3d (1, 1, 0);        

      ShellTriElement triShell = new ShellTriElement(m_node0, m_node1, m_node2, 0.01);
      m_femShellModel.addNode (m_node0);
      m_femShellModel.addNode (m_node1);
      m_femShellModel.addNode (m_node2);
      m_femShellModel.addElement (triShell);

      m_femShellModel.setMaterial (new NeoHookeanMaterial());
      m_femShellModel.setDensity (m_density);
      m_femShellModel.setParticleDamping (m_particleDamping);
      m_femShellModel.setGravity (0,0,0);
      
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
}
