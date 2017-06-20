package artisynth.demos.fem;

import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.femmodels.*;
import artisynth.core.modelbase.*;
import artisynth.core.materials.*;
import artisynth.core.mechmodels.*;
import artisynth.core.workspace.RootModel;
import artisynth.core.gui.*;
import artisynth.core.driver.*;

import java.awt.Color;
import java.awt.Point;
import java.util.ArrayList;
import java.util.Iterator;

import javax.swing.JFrame;

import maspack.properties.PropertyList;
import maspack.render.*;
import maspack.matrix.*;

public class ShellPatch extends RootModel {
   protected ShellFemModel3d m_femShellModel = null;
   protected MechModel m_mechModel = null;

   protected ShellTriElement[] m_eles = null;
   protected FemNode3d[] m_nodes = null;

   final double m_density = 10000;
   final double m_damping = 100;
   
   public void build (String[] args) {
      m_femShellModel = new ShellFemModel3d();
      
      m_eles = new ShellTriElement[2];
      m_nodes = new FemNode3d[4];
      
      m_nodes[0] = new FemNode3d (0, 0, 0);
      m_nodes[1] = new FemNode3d (1, 0, 0);
      m_nodes[2] = new FemNode3d (1, 1, 0);
      m_nodes[3] = new FemNode3d (0, 1, 0);          // for 2nd ele.
      
      for (int i = 0; i < m_nodes.length; i++) {
         m_nodes[i].setIndex(i);
      }
      
      m_femShellModel.addNode(m_nodes[0]);
      m_femShellModel.addNode(m_nodes[1]);
      m_femShellModel.addNode(m_nodes[2]);
      m_femShellModel.addNode(m_nodes[3]);
      
      m_eles[0] = new ShellTriElement(m_nodes[0], m_nodes[1], m_nodes[2]);
      m_eles[1] = new ShellTriElement(m_nodes[3], m_nodes[0], m_nodes[2]);
      
      m_femShellModel.addElement(m_eles[0]);
      m_femShellModel.addElement(m_eles[1]);
      
      m_femShellModel.setSurfaceRendering (SurfaceRender.Shaded);

      RenderProps.setFaceColor (m_femShellModel, Color.PINK);
      RenderProps.setShininess (m_femShellModel, 
                          m_femShellModel.getRenderProps().getShininess() * 10);
      RenderProps.setVisible (m_femShellModel, true);
      RenderProps.setFaceStyle (m_femShellModel, Renderer.FaceStyle.FRONT);

      m_femShellModel.setMaterial (new NeoHookeanMaterial());

      m_mechModel = new MechModel ("mech");
      m_mechModel.addModel (m_femShellModel);
      addModel (m_mechModel);

      RenderProps.setPointStyle (m_femShellModel.getNodes(), Renderer.PointStyle.SPHERE);
      RenderProps.setPointRadius (m_femShellModel.getNodes(), 0.05);

      m_femShellModel.setGravity (0, 0, 0);
      m_femShellModel.setDensity (m_density);
      m_femShellModel.setParticleDamping (m_damping);

      m_mechModel.setProfiling (true);
      
   }   
}
