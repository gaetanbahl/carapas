/**
 * Copyright (c) 2014, by the Authors: Bruce Haines (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package maspack.geometry;

import java.util.LinkedList;

import javax.media.opengl.GL2;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.render.GLRenderable;
import maspack.render.GLRenderer;
import maspack.render.GLSelectable;
import maspack.render.RenderList;


// This is a class to make grid points renderable and selectable.


public class SignedDistanceGridCell implements GLSelectable {

   private int vertex[] = new int[3];
   private double distance;
   private int myIndex;
   private SignedDistanceGrid myGrid;
   private boolean isSelected;
   protected float[] pointColour = new float[] { 0, 1, 0 };
   protected float[] selectedColour = new float[] { 0, 1, 0 };
   protected int myPointSize = 5;
   
   public SignedDistanceGridCell() {
   }
  
   public SignedDistanceGridCell (int idx, SignedDistanceGrid grid) {
      myGrid = grid;
      myIndex = idx;
      int[] myGridSize = myGrid.getGridSize();
      vertex[2] = 
         (myIndex / (myGridSize[0] * myGridSize[1]));
      vertex[1] = 
         (myIndex - vertex[2] * myGridSize[0] * myGridSize[1]) / myGridSize[0];
      vertex[0] = 
         (myIndex % (myGridSize[0]));
   }
   
   public void setVertex (int x, int y, int z) {
      int[] myGridSize = myGrid.getGridSize();
      vertex[0] = x;
      vertex[1] = y;
      vertex[2] = z;
      myIndex = x + y * myGridSize[0] + z * myGridSize[0] * myGridSize[1];
   }
   
   public void setDistance (double d) {
      distance = d;
   } 
   
   public double getDistance () {
      return distance;
   }
   
   public void setIndex (int index) {
      myIndex = index;
      int[] myGridSize = myGrid.getGridSize();
      vertex[2] = 
         (myIndex / (myGridSize[0] * myGridSize[1]));
      vertex[1] = 
         (myIndex - vertex[2] * myGridSize[0] * myGridSize[1]) / myGridSize[0];
      vertex[0] = 
         (myIndex % (myGridSize[0]));
   }
   
   public void prerender (RenderList list) {
   }
   
   public void render (GLRenderer renderer, int flags) {
      GL2 gl = renderer.getGL2().getGL2();
      double meshVertex[] = new double[3];
      meshVertex = myGrid.getMeshCoordinatesFromGrid (
         vertex[0], vertex[1], vertex[2]);
      
      gl.glEnable (GL2.GL_POINT_SMOOTH);   // Render the point.
      gl.glPointSize (3);
      renderer.setColor (pointColour);
      gl.glBegin (GL2.GL_POINTS);
      gl.glVertex3d (meshVertex[0], meshVertex[1], meshVertex[2]);
      gl.glEnd();

      Vector3d normal = new Vector3d();
      normal = myGrid.getNormal (vertex[0], vertex[1], vertex[2]);
      
      gl.glLineWidth (1.0f);  // Render the normal.
      gl.glBegin (GL2.GL_LINES);
      gl.glVertex3d (meshVertex[0], meshVertex[1], meshVertex[2]);
      gl.glVertex3d (meshVertex[0] + normal.x * 0.1,
                     meshVertex[1] + normal.y * 0.1,
                     meshVertex[2] + normal.z * 0.1);
      gl.glEnd ();
   }
   
//   public void handleSelection (LinkedList<GLRenderable> pathlist, int[] namestack, int idx) {
//	   pathlist.add (this);
//      System.out.println (
//         "Index: " + myIndex +
//         " | Distance: " + distance +
//         " | ClosestFace: " + myGrid.getClosestFace (myIndex));
//   }
//   
   public void getSelection (LinkedList<Object> list, int qid) {
   }
   
   public boolean isSelectable() {
      return true;
   }
   
   public int numSelectionQueriesNeeded() {
      return -1;
   }

   public boolean isSelected () {
      return isSelected;
   }
   
   public void selectPoint (boolean selected) {
      isSelected = selected;
   }
   
   public int[] getPoint() {
      return vertex;
   }
   
   public int getRenderHints() {
      return 0;
   }
   
   public void setColour (float r, float g, float b) {
      pointColour[0] = r;
      pointColour[1] = g;
      pointColour[2] = b;
   }
   
   public void updateBounds (Point3d pmin, Point3d pmax) {
   }
   
}