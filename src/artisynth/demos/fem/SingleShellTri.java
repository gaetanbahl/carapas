package artisynth.demos.fem;

import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.femmodels.*;
import artisynth.core.modelbase.*;
import artisynth.core.materials.*;
import artisynth.core.mechmodels.*;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.workspace.RootModel;
import artisynth.core.gui.*;
import artisynth.core.driver.*;

import java.awt.Color;
import java.awt.Point;
import java.util.ArrayList;

import javax.swing.JFrame;

import maspack.properties.PropertyList;
import maspack.render.*;
import maspack.geometry.Vertex3d;
import maspack.matrix.*;

/**
 * Interactive demo of a single triangle shell element of 3 shell nodes and 
 * 9 guass points. Drag the nodes around via force.
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
   
   protected final double m_density = 100;
   protected final double m_particleDamping = 100;            
   
   protected final double m_nodeRadius = 0.05;

   public void build (String[] args) {
      m_femShellModel = new ShellFemModel3d();
      
      m_node0 = new ShellFemNode3d (0, 0, 0);      
      m_node1 = new ShellFemNode3d (1, 0, 0);       
      m_node2 = new ShellFemNode3d (1, 1, 0);        

      ShellTriElement triShell = new ShellTriElement(m_node0, m_node1, m_node2);
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
