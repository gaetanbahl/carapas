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
import java.util.LinkedList;

import javax.swing.JFrame;

import maspack.properties.PropertyList;
import maspack.render.*;
import maspack.geometry.Face;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.*;

public class ShellPatch extends RootModel {
   protected ShellFemModel3d m_femShellModel = null;
   protected MechModel m_mechModel = null;

   // 10,10,10,10
   public final int mMeshX = 2;                
   public final int mMeshY = 1;
   public final int mMeshXDiv = 2;
   public final int mMeshYDiv = 1;
   
   protected PolygonalMesh mMesh = null;
   
   protected ShellFemNode3d[] m_nodes = null;

   final double m_density = 10000;
   final double m_damping = 100;
   
   public final double mNodeRadius = 0.1;
   
   public void build (String[] args) {
      m_mechModel = new MechModel ("mech");
      m_femShellModel = new ShellFemModel3d();

      mMesh = MeshFactory.createPlane(mMeshX, mMeshY, mMeshXDiv, mMeshYDiv);
      m_nodes = new ShellFemNode3d[mMesh.numVertices()];
      
      for (Face face : mMesh.getFaces()) {
         Vertex3d[] triVtx = face.getTriVertices();
         
//         Vector3d centroid = new Vector3d();
//         face.computeCentroid (centroid);
//         Particle fParticle = new Particle(1, new Point3d(centroid));
//         RenderProps.setSphericalPoints(fParticle, face.idx/100.0, Color.BLUE);
//         m_mechModel.addParticle (fParticle);
         
//         System.out.println ("My face: " + face.idx);
//         System.out.println ("  My nodes are: " + triVtx[0].getIndex () + triVtx[1].getIndex () + triVtx[2].getIndex ());
         
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
         //System.out.println ("Clockwise: " + isClockwise(triVtx));
         
         // Add femElement using 3 femNodes.
         ShellTriElement ele = new ShellTriElement(n0, n1, n2);
         m_femShellModel.addElement(ele);
      }
      
      m_femShellModel.setSurfaceRendering (SurfaceRender.Shaded);

      RenderProps.setFaceColor (m_femShellModel, Color.PINK);
      RenderProps.setShininess (m_femShellModel, 
                          m_femShellModel.getRenderProps().getShininess() * 10);
      RenderProps.setVisible (m_femShellModel, true);
      RenderProps.setFaceStyle (m_femShellModel, Renderer.FaceStyle.FRONT);

      m_femShellModel.setMaterial (new NeoHookeanMaterial());

      m_mechModel.addModel (m_femShellModel);
      addModel (m_mechModel);

      RenderProps.setPointStyle (m_femShellModel.getNodes(), Renderer.PointStyle.SPHERE);
      RenderProps.setPointRadius (m_femShellModel.getNodes(), mNodeRadius);

      m_femShellModel.setGravity (0, 0, 0);
      m_femShellModel.setDensity (m_density);
      m_femShellModel.setParticleDamping (m_damping);

      //m_mechModel.setProfiling (true);
   }   
   
   public boolean isClockwise(Vertex3d[] triVtx) {
      return isClockwise(triVtx[0].pnt, triVtx[1].pnt, triVtx[2].pnt);
   }
   
   public boolean isClockwise(Point3d v0, Point3d v1, Point3d v2) {
      double edge01 = (v1.x - v0.x)*(v1.y + v0.y);
      double edge12 = (v2.x - v1.x)*(v2.y + v1.y);
      double edge20 = (v0.x - v2.x)*(v0.y + v2.y);
      return (edge01 + edge12 + edge20 > 0);
   }
}
