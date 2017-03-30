package maspack.geometry;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;

import artisynth.core.modelbase.RenderableComponentBase;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.PointLineRenderProps;
import maspack.render.RenderList;
import maspack.render.RenderObject;
import maspack.render.RenderProps;
import maspack.render.Renderer;

/**
 * Signed-distance grid with recursively splitting partitioning tree 
 * (i.e. oct-tree with variable partitions per dimension)
 */
public class AdaptiveSignedDistanceGrid extends RenderableComponentBase {
   
   static double OUTSIDE = Double.MAX_VALUE;
   static int DEFAULT_DIVISIONS = 2;
   static double DEFAULT_MAX_TOLERANCE = 1e-12; // times appropriate mesh radius
   static double DEFAULT_MARGIN = 1e-12;        // times approximate mesh radius
   static int DEFAULT_MAX_DEPTH = 8;            // maximum depth
   
   PolygonalMesh mesh;
   Vector3d gridWidths;
   int[] gridCells;
   RigidTransform3d trans;
   double tolerance;
   int maxDepth;
   
   GridCell root;
   
   private static class GridNode {
      double phi;
      Face closest;
      public GridNode(){
         phi = Double.POSITIVE_INFINITY;
         closest = null;
      }
   }
   
   /**
    * 3D integer grid coordinate
    */
   private class GridCoord3d {
      int[] x;
      int[] y;
      int[] z;
      
      private GridCoord3d() {
         x = null;
         y = null;
         z = null;
      }
      
      public int depth() {
         for (int i=x.length; i-->0; ) {
            if (x[i] > 0 || y[i] > 0 || z[i] > 0) {
               return i;
            }
         }
         return 0;
      }
      
      public GridCoord3d(int depth, int x, int y, int z) {
         this.x = new int[depth+1];
         this.y = new int[depth+1];
         this.z = new int[depth+1];
         if (x < (gridCells[0]+1) || depth == 0) {
            this.x[depth] = x;
         } else {
            this.x[depth-1] = 1;
         }
         if (y < (gridCells[1]+1) || depth == 0) {
            this.y[depth] = y;
         } else {
            this.y[depth-1] = 1;
         }
         if (z < (gridCells[2]+1) || depth == 0) {
            this.z[depth] = z;
         } else {
            this.z[depth-1] = 1;
         }
      }
      
      public void computePosition(Point3d pnt) {
         pnt.setZero();
         for (int i=0; i<x.length; ++i) {
            pnt.x += gridWidths.x*x[i]/Math.pow(gridCells[0], i+1);
            pnt.y += gridWidths.y*y[i]/Math.pow(gridCells[1], i+1);
            pnt.z += gridWidths.z*z[i]/Math.pow(gridCells[2], i+1);
         }
      }
      
      public GridCoord3d plus(int depth, int x, int y, int z) {
         GridCoord3d out = new GridCoord3d();
         int ndepth = Math.max(depth+1, this.x.length);
         out.x = new int[ndepth];
         out.y = new int[ndepth];
         out.z = new int[ndepth];
         
         for (int i=0; i<this.x.length; ++i) {
            out.x[i] = this.x[i];
            out.y[i] = this.y[i];
            out.z[i] = this.z[i];
         }
         
         // add with carry
         for (int i=depth+1; x > 0 && i-->0;) {
            out.x[i] += x;
            if (out.x[i] >= (gridCells[0]+1) && i > 0) {
               int v = out.x[i];
               out.x[i] = v % (gridCells[0]+1);
               x = v / (gridCells[0]+1);
            } else {
               x = 0;
            }
         }
         
         // add with carry
         for (int j=depth+1; y > 0 && j-->0;) {
            out.y[j] += y;
            if (out.y[j] >= (gridCells[1]+1) && j > 0) {
               int v = out.y[j];
               out.y[j] = v % (gridCells[1]+1);
               y = v / (gridCells[1]+1);
            } else {
               y = 0;
            }
         }
         
         // add with carry
         for (int k=depth+1; z > 0 && k-->0;) {
            out.z[k] += z;
            if (out.z[k] >= (gridCells[2]+1) && k > 0) {
               int v = out.z[k];
               out.z[k] = v % (gridCells[2]+1);
               z = v / (gridCells[2]+1);
            } else {
               z = 0;
            }
         }
         
         return out;
      }
      
      @Override
      public int hashCode() {
         final int prime = 31;
         int out = 0;
         int d = 1;
         for (int i=0; i<x.length; ++i) {
            out += x[i]*d;
            d *= (gridCells[0]+1);
         }
         out *= prime;
         
         d = 1;
         for (int i=0; i<y.length; ++i) {
            out += y[i]*d;
            d *= (gridCells[1]+1);
         }
         out *= prime;
         
         d = 1;
         for (int i=0; i<z.length; ++i) {
            out += z[i]*d;
            d *= (gridCells[2]+1);
         }
         return out;
      }
      
      private boolean equals(int[] x1, int[] x2) {
         
         int[] lx, hx;
         if (x1.length < x2.length) {
            lx = x1;
            hx = x2;
         } else {
            lx = x2;
            hx = x1;
         }
         
         for (int i=0; i<lx.length; ++i) {
            if (lx[i] != hx[i]) {
               return false;
            }
         }
         for (int i=lx.length; i<hx.length; ++i) {
            if (hx[i] != 0) {
               return false;
            }
         }
         
         return true;
      }
      
      @Override
      public boolean equals(Object obj) {
         if (this == obj) {
            return true;
         }
         if (obj == null || getClass() != obj.getClass()) {
            return false;
         }
         
         GridCoord3d other = (GridCoord3d)obj;
         if (!equals(x, other.x) || !equals(y, other.y) || !equals(z, other.z)) {
            return false;
         }
         return true;
      }
      
      public String toString() {
         StringBuilder sb = new StringBuilder();
         sb.append("{ [");
         for (int i=0; i<x.length; ++i) {
            if (i > 0) {
               sb.append(',');
            }
            sb.append(x[i]);
         }
         sb.append("], [");
         for (int i=0; i<y.length; ++i) {
            if (i > 0) {
               sb.append(',');
            }
            sb.append(y[i]);
         }
         sb.append("], [");
         for (int i=0; i<z.length; ++i) {
            if (i > 0) {
               sb.append(',');
            }
            sb.append(z[i]);
         }
         sb.append("] }");
         return sb.toString();
      }
   }
   
   private class GridCell {
      
      GridNode[] corners;
      GridCell[] children; // gridCells[0]*gridCells[1]*gridCells[2] number of children
      
      GridCell() {
         corners = new GridNode[8]; // 8 corners
         children = null;
      }
      
      public int getCellIndex(int x, int y, int z) {
         return x + y*gridCells[0]+z*gridCells[0]*gridCells[1];
      }
      
      public int getCornerIndex(int x, int y, int z) {
         return x + 2*y + 4*z;
      }

      /**
       * Finds the leaf grid cell containing the given point
       * @param coord coordinate within grid we are seeking
       * @return corresponding cell
       */
      private GridCell getCell(GridCoord3d coord) {
         int d = coord.depth();
         GridCell cell = root;
         for (int i=0; i<=d; ++i) {
            if (cell.children == null) {
               return cell;
            }
            cell = cell.children[cell.getCellIndex(coord.x[i], coord.y[i], coord.z[i])];
         }
         return cell;
      }
      
      public double getDistance(Point3d pnt, Vector3d widths) {
         
         // go down into child if exists
         if (children != null) {
            // go into child
            widths.scale(1.0/gridCells[0], 1.0/gridCells[1], 1.0/gridCells[2]);
            
            // compute which square it belongs to
            int x = (int)(pnt.x/widths.x);
            int y = (int)(pnt.y/widths.y);
            int z = (int)(pnt.z/widths.z);
            
            // correct for if exactly on boundary
            if (x >= gridCells[0]) {
               x = gridCells[0]-1;
            }
            if (y >= gridCells[1]) {
               y = gridCells[1]-1;
            }
            if (z >= gridCells[2]) {
               z = gridCells[2]-1;
            }
            
            int idx = getCellIndex(x, y, z);
            
            // go deeper
            pnt.add(-x*widths.x, -y*widths.y, -z*widths.z); // adjust origin
            return children[idx].getDistance(pnt, widths);
            
         }
         
         // trilinear interpolation
         double n1 = corners[getCornerIndex(0,0,0)].phi;
         double n2 = corners[getCornerIndex(0,0,1)].phi;
         double n3 = corners[getCornerIndex(0,1,0)].phi;
         double n4 = corners[getCornerIndex(0,1,1)].phi;
         double n5 = corners[getCornerIndex(1,0,0)].phi;
         double n6 = corners[getCornerIndex(1,0,1)].phi;
         double n7 = corners[getCornerIndex(1,1,0)].phi;
         double n8 = corners[getCornerIndex(1,1,1)].phi;
         
         double u = pnt.x/widths.x;
         double v = pnt.y/widths.y;
         double w = pnt.z/widths.z;
         
         double d = (1-u)*((1-v)*( (1-w)*n1 + w*n2) + v*( (1-w)*n3 + w*n4))
            + u*((1-v)*( (1-w)*n5 + w*n6) + v*( (1-w)*n7 + w*n8));
         
         return d;
      }
      
      public double getDistanceAndNormal(Point3d pnt, Vector3d widths, Vector3d nrm) {
         
         // go down into child if exists
         if (children != null) {
            // go into child
            widths.scale(1.0/gridCells[0], 1.0/gridCells[1], 1.0/gridCells[2]);
            
            // compute which square it belongs to
            int x = (int)(pnt.x/widths.x);
            int y = (int)(pnt.y/widths.y);
            int z = (int)(pnt.z/widths.z);
            
            // correct for if exactly on boundary
            if (x >= gridCells[0]) {
               x = gridCells[0]-1;
            }
            if (y >= gridCells[1]) {
               y = gridCells[1]-1;
            }
            if (z >= gridCells[2]) {
               z = gridCells[2]-1;
            }
            
            int idx = getCellIndex(x, y, z);
            
            // go deeper
            pnt.add(-x*widths.x, -y*widths.y, -z*widths.z); // adjust origin
            return children[idx].getDistanceAndNormal(pnt, widths, nrm);
            
         }
         
         // trilinear interpolation
         double n1 = corners[getCornerIndex(0,0,0)].phi;
         double n2 = corners[getCornerIndex(0,0,1)].phi;
         double n3 = corners[getCornerIndex(0,1,0)].phi;
         double n4 = corners[getCornerIndex(0,1,1)].phi;
         double n5 = corners[getCornerIndex(1,0,0)].phi;
         double n6 = corners[getCornerIndex(1,0,1)].phi;
         double n7 = corners[getCornerIndex(1,1,0)].phi;
         double n8 = corners[getCornerIndex(1,1,1)].phi;
         
         double u = pnt.x/widths.x;
         double v = pnt.y/widths.y;
         double w = pnt.z/widths.z;
         
         double d = (1-u)*((1-v)*( (1-w)*n1 + w*n2) + v*( (1-w)*n3 + w*n4))
            + u*((1-v)*( (1-w)*n5 + w*n6) + v*( (1-w)*n7 + w*n8));
         
         // normal
         nrm.x = ((1-v)*( (1-w)*n5 + w*n6) + v*( (1-w)*n7 + w*n8)
                  - ((1-v)*( (1-w)*n1 + w*n2) + v*( (1-w)*n3 + w*n4)))/widths.x;
         
         nrm.y = ((1-u)*((1-w)*n3 + w*n4) + u*((1-w)*n7 + w*n8) 
                  - ((1-u)*((1-w)*n1 + w*n2) + u*((1-w)*n5 + w*n6)))/widths.y;
         
         nrm.z = ((1-u)*((1-v)*n2 + v*n4) + u*((1-v)*n6 + v*n8) 
                  -((1-u)*((1-v)*n1+v*n3) + u*((1-v)*n5 + v*n7)))/widths.z;
         nrm.normalize();   
         
         return d;
      }
 
   }
  
   public double getDistance(Point3d pnt) {
      // transform
      Point3d p = new Point3d();
      
      // transform into grid-aligned
      p.inverseTransform(mesh.getMeshToWorld(), pnt);
      p.inverseTransform(trans);
      Vector3d w = new Vector3d(gridWidths); // width
      return root.getDistance(p, w);
   }
   
   public double getDistanceAndNormal(Point3d pnt, Vector3d nrm) {
      // transform
      Point3d p = new Point3d();
      
      // transform to mesh-space
      p.inverseTransform(mesh.getMeshToWorld(), pnt);
      // transform into grid-aligned
      p.inverseTransform(trans);
      Vector3d w = new Vector3d(gridWidths); // width
      double d= root.getDistanceAndNormal(p, w, nrm);
      
      // transform the normal back to world
      nrm.transform(trans);
      nrm.transform(mesh.getMeshToWorld());
      
      return d;
   }
   
   
   public AdaptiveSignedDistanceGrid(PolygonalMesh mesh, RigidTransform3d trans, int[] divisions, 
      double margin, double tol, int maxDepth) {
      
      this.mesh = mesh;
      if (!mesh.isTriangular()) {
         mesh.triangulate();
      }
      
      // temporarily set mesh transform to identity
      RigidTransform3d meshToWorld = new RigidTransform3d();
      mesh.getMeshToWorld(meshToWorld);
      mesh.setMeshToWorld(RigidTransform3d.IDENTITY);
   
      Vector3d widths;
      if (trans == null) {
         OBB obb = new OBB(mesh);
         trans = obb.getTransform();
         widths = obb.getHalfWidths();
         Vector3d t = new Vector3d();
         t.transform(trans, widths);
         trans.p.sub(t); // origin in bottom-left
         widths.scale(2.0);
      } else {
         trans = new RigidTransform3d(trans); // make protected copy
         Point3d pmin = new Point3d(Double.POSITIVE_INFINITY,
            Double.POSITIVE_INFINITY,
            Double.POSITIVE_INFINITY);
         Point3d pmax = new Point3d(Double.NEGATIVE_INFINITY,
            Double.NEGATIVE_INFINITY, 
            Double.NEGATIVE_INFINITY);
         
         Point3d p = new Point3d();
         for (Vertex3d vtx : mesh.getVertices()) {
            p.inverseTransform(trans, vtx.pnt);
            p.updateBounds(pmin, pmax);
         }
         widths = new Vector3d(pmax);
         widths.sub(pmin);
         trans.p.add(pmin);
      }
      this.trans = trans;
      
      if (divisions == null) {
         gridCells = new int[]{DEFAULT_DIVISIONS, DEFAULT_DIVISIONS, DEFAULT_DIVISIONS};
      } else {
         gridCells = Arrays.copyOf(divisions, 3);
      }
      
      if (margin < 0) {
         margin = DEFAULT_MARGIN*mesh.getRadius();
      }
      
      // move transform down a bit due to margin
      trans.p.add(-margin, -margin, -margin);
      widths.add(2*margin, 2*margin, 2*margin);
      
      // tolerance to stop splitting
      if (tol <= 0) {
         tol = DEFAULT_MAX_TOLERANCE*mesh.getRadius();
      }
      this.tolerance = tol;
      
      // widths of grid cells
      gridWidths = new Vector3d(widths.x, widths.y, widths.z);
      
      if (maxDepth < 0) {
         maxDepth = DEFAULT_MAX_DEPTH;
      }
      this.maxDepth = maxDepth;
      
      // empty grid-square for root
      root = null;
      
      // now we build
      build();
      
      // restore mesh transform
      mesh.setMeshToWorld(meshToWorld);
   }
   
   private void getMeshCoordinates(Point3d origin, Vector3d widths,
      int x, int y, int z, Point3d pnt) {
      pnt.x = x*widths.x+origin.x;
      pnt.y = y*widths.y+origin.y;
      pnt.z = z*widths.z+origin.z;
      pnt.transform(trans);
   }
   
   private GridCell createCell(GridCoord3d origin, int depth, HashMap<GridCoord3d,GridNode> nodeMap) {
      
      GridCell out = new GridCell();
      
      // populate eight corners
      for (int i=0; i<2; ++i) {
         for (int j=0; j<2; ++j) {
            for (int k=0; k<2; ++k) {
               GridCoord3d coord = origin.plus(depth, i*gridCells[0], j*gridCells[1], k*gridCells[2]);
               // get or create node at location
               GridNode node = nodeMap.get(coord);
               if (node == null) {
                  node = new GridNode();
                  nodeMap.put(coord, node);
               }
               out.corners[out.getCornerIndex(i, j, k)] = node;
            }
         }
      }
      
      return out;
   }
   
   private static boolean overlapTest(Face f, RigidTransform3d trans, Vector3d origin,
      Vector3d widths, double threshold, Vector3d dir) {
      
      final double[][] corners = {{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1},
                                  {1,1,0}, {1,0,1}, {0,1,1}, {1,1,1}};
      
      double faceMin = Double.POSITIVE_INFINITY;
      double faceMax = Double.NEGATIVE_INFINITY;   
      double boxMin = Double.POSITIVE_INFINITY;
      double boxMax = Double.NEGATIVE_INFINITY;
      Point3d pnt = new Point3d();
      
      // face range
      for (Iterator<Vertex3d> vit = f.vertexIterator(); vit.hasNext();) {
         Vertex3d vtx = vit.next();
         double d = dir.dot(vtx.pnt);
         if (d < faceMin) {
            faceMin = d;
         }
         if (d > faceMax) {
            faceMax = d;
         }
      }
      
      // box range, put into mesh coordinates
      for (int j=0; j<8; ++j) {
         pnt.set(
            origin.x+corners[j][0]*widths.x, 
            origin.y+corners[j][1]*widths.y, 
            origin.z+corners[j][2]*widths.z);
         pnt.transform(trans);
         double d = dir.dot(pnt);
         
         if (d < boxMin) {
            boxMin = d;
         }
         if (d > boxMax) {
            boxMax = d;
         }
      }
      
      // check for overlapping interval
      boolean overlap = ( (boxMin <= faceMax+threshold) && (faceMin <= boxMax+threshold));
      return overlap;
   }
   
   /**
    * Tests if a face intersects a box
    * @param f
    * @param trans
    * @param origin
    * @param widths
    * @param threshold
    * @return
    */
   private static boolean faceBoxIntersection(Face f, RigidTransform3d trans, 
      Point3d origin, Vector3d widths, double threshold) {
      
      //      System.out.println("FBI: ");
      //      System.out.println("   f={");
      //      for (Iterator<Vertex3d> vit=f.vertexIterator(); vit.hasNext();) {
      //         Vertex3d vtx = vit.next();
      //         System.out.println("      (" +vtx.getWorldPoint().toString("%.2g") + ")");
      //      }
      //      System.out.println("   }");
      //      System.out.println("   b={");
      //      final double[][] corners = {{0,0,0}, {1,1,1}};
      //      Point3d pnt = new Point3d();
      //      for (int i=0; i<corners.length; ++i) {
      //         pnt.set(origin);
      //         pnt.add(corners[i][0]*widths.x, corners[i][1]*widths.y, corners[i][2]*widths.z);
      //         pnt.transform(trans);
      //         System.out.println("      (" + pnt.toString("%.2g") + ")");
      //      }
      //      System.out.println("   }");
      
      // uses separating axis theorem
      Vector3d dir = new Vector3d();
      
      // three axes of box
      for (int i=0; i<3; ++i) {
         trans.R.getColumn(i, dir); // direction is in WORLD coordinates, origin/widths in box coordinates
         if (!overlapTest(f, trans, origin, widths, threshold, dir)) { 
            return false;
         }
      }
      
      // face normal
      f.computeNormal(dir);
      if (!overlapTest(f, trans, origin, widths, threshold, dir)) { 
         return false;
      }
      
      // cross product edges
      Vector3d edir = new Vector3d();
      for (Iterator<HalfEdge> hit = f.edgeIterator(); hit.hasNext(); ) {
         HalfEdge he = hit.next();
         edir.sub(he.head.pnt, he.tail.pnt);
         
         for (int j=0; j<3; ++j) {
            trans.R.getColumn(j, dir);
            dir.cross(edir);
            double n2 = dir.norm();
            if (n2 > 0) {
               dir.normalize();
               if (!overlapTest(f, trans, origin, widths, threshold, dir)) {
                  return false;
               }
            }
         }
      }
      
      return true;
   }
   
   private void computeDistances(GridCell cell, int depth,
      GridCoord3d gridCoord, HashMap<GridCoord3d, GridNode> nodeMap,
      Face face, Point3d faceMin, Point3d faceMax) {
      
      Point3d origin = new Point3d();
      gridCoord.computePosition(origin);
      
      // convert to cell coordinate
      Vector3d boxWidth = new Vector3d(
         gridWidths.x/Math.pow(gridCells[0],depth),
         gridWidths.y/Math.pow(gridCells[1],depth),
         gridWidths.z/Math.pow(gridCells[2],depth)
         );
      Vector3d widths = new Vector3d(boxWidth.x/gridCells[0], boxWidth.y/gridCells[1], boxWidth.z/gridCells[2]);
      
      // Convert to grid coordinates.      
      Point3d pnt = new Point3d();
      Point3d closest = new Point3d();

      // calculating distance and closestFace.
      pnt.set(origin);
      for (int z = 0; z < 2; ++z) {
         for (int y = 0; y < 2; ++y) {
            for (int x = 0; x < 2; ++x) {
               // Get mesh coordinates
               getMeshCoordinates (origin, widths, 
                  x*gridCells[0], y*gridCells[1], z*gridCells[2], pnt);
               
               // Get the distance from this point to the face.
               face.nearestPoint (closest, pnt);
               double distance = pnt.distance (closest);
               int index = cell.getCornerIndex(x, y, z);
               if (distance < cell.corners[index].phi) {
                  cell.corners[index].phi = distance;
                  cell.corners[index].closest = face;
               }
            }
         }
      }
      
      // If cell actually intersects face, recursively go down
      if (widths.norm() >= tolerance && depth < maxDepth) {
         if (faceBoxIntersection(face, trans, origin, boxWidth, tolerance)) {
            if (cell.children == null) {
               int numGridCells = gridCells[0]*gridCells[1]*gridCells[2];
               cell.children = new GridCell[numGridCells];
            }
            
            Point3d o = new Point3d(origin);   
            for (int z=0; z<gridCells[2]; ++z) {
               o.z = origin.z + z*widths.z;
               for (int y=0; y<gridCells[1]; ++y) {
                  o.y = origin.y + y*widths.y;
                  for (int x=0; x<gridCells[0]; ++x) {
                     o.x = origin.x + x*widths.x;
                     
                     int childIdx = cell.getCellIndex(x, y, z);
                     GridCell child = cell.children[childIdx];
                     GridCoord3d childCoord = gridCoord.plus(depth, x, y, z);
                     if (child == null) {
                        child = createCell(childCoord, depth+1, nodeMap);
                        cell.children[childIdx] = child;
                     }
                     
                     computeDistances(child, depth+1, childCoord, nodeMap, face, faceMin, faceMax);
                  }
               }
            } // done looping over children
         } // if box intersects face
      } // if not too deep
      
   }
   
   private void sweep(GridCell cell, int depth, GridCoord3d coord, int dx, int dy, int dz) {
      
//      // XXX sweep in the (dx, dy, dz) direction, check closest neighbour using maxdepth
//      if (cell.children != null) {
//         for (int z=0; z<gridCells[2]; ++z) {
//            for (int y=0; y<gridCells[1]; ++y) {
//               for (int x=0; x<gridCells[0]; ++x) {
//                  o.x = origin.x + x*widths.x;
//                  
//                  int childIdx = cell.getCellIndex(x, y, z);
//                  GridCell child = cell.children[childIdx];
//                  GridCoord3d childCoord = gridCoord.plus(depth, x, y, z);
//                  if (child == null) {
//                     child = createCell(childCoord, depth+1, nodeMap);
//                     cell.children[childIdx] = child;
//                  }
//                  
//                  sweep(child, depth+1, childCoord, dx, dy, dz);
//               }
//            }
//         } // done looping over children
//      }
      
   }
   
   private void build() {
      
      Point3d faceMin = new Point3d();
      Point3d faceMax = new Point3d();
      Point3d point = new Point3d();
      
      // create root
      HashMap<GridCoord3d, GridNode> nodeMap = new HashMap<>();
      GridCoord3d rootCoord = new GridCoord3d(0, 0,0,0);
      root = createCell(rootCoord, 0, nodeMap);
      
      // Loop over every triangle
      for (Face face : mesh.getFaces()) {
         faceMin.set (Double.POSITIVE_INFINITY,
            Double.POSITIVE_INFINITY,
            Double.POSITIVE_INFINITY);
         faceMax.set (Double.NEGATIVE_INFINITY,
            Double.NEGATIVE_INFINITY, 
            Double.NEGATIVE_INFINITY);
         
         // loop over all vertices, transform to distance grid coordinates
         for (Iterator<Vertex3d> vit = face.vertexIterator(); vit.hasNext();) {
            Vertex3d vtx = vit.next();
            point.inverseTransform(trans, vtx.pnt); // put it in box coordinates
            point.updateBounds(faceMin, faceMax);
         }
         
         computeDistances(root, 0, rootCoord, nodeMap, face, faceMin, faceMax);
      }
      
      // sweep direction
      // Done all triangles.
      // Sweep, propagating values throughout the grid volume.
      for (int pass = 0; pass < 2; pass++) {
         sweep(root, 0, rootCoord, +1, +1, +1);
         sweep(root, 0, rootCoord, -1, -1, -1);
         sweep(root, 0, rootCoord, +1, +1, -1);
         sweep(root, 0, rootCoord, -1, -1, +1);
         sweep(root, 0, rootCoord, +1, -1, +1);
         sweep(root, 0, rootCoord, -1, +1, -1);
         sweep(root, 0, rootCoord, +1, -1, -1);
         sweep(root, 0, rootCoord, -1, +1, +1);
      }
      
      nodeMap.clear();  // clear memory
      
   }
   
   private void buildCellPoints(RenderObject robj, HashMap<GridCoord3d,Integer> gridMap,
      GridCell root, int depth, GridCoord3d coord) {
      
      // corners
      Point3d pnt = new Point3d();
      for (int i=0; i<2; ++i) {
         for (int j=0; j<2; ++j) {
            for (int k=0; k<2; ++k) {
               GridCoord3d pcoord = coord.plus(depth, 
                  i*gridCells[0], j*gridCells[1], k*gridCells[2]);
               Integer idx = gridMap.get(pcoord);
               if (idx == null) {
                  pcoord.computePosition(pnt);
                  pnt.transform(trans);
                  idx = robj.vertex((float)(pnt.x), (float)(pnt.y), (float)(pnt.z));
                  gridMap.put(pcoord, idx);
                  robj.addPoint(idx);
               }
            }
         }
      }
      
      // children
      for (int i=0; i<gridCells[0]; ++i) {
         for (int j=0; j<gridCells[1]; ++j) {
            for (int k=0; k<gridCells[2]; ++k) {
               int idx = root.getCellIndex(i, j, k);
               if (root.children != null && root.children[idx] != null) {
                  GridCoord3d pcoord = coord.plus(depth, i, j, k);
                  buildCellPoints(robj, gridMap, root.children[idx], depth+1, pcoord);
               }
            }
         }
      }
   }
   
   private void buildCellEdges(RenderObject robj, HashMap<GridCoord3d,Integer> gridMap,
      GridCell root, int depth, GridCoord3d coord) {
      
      Point3d pnt = new Point3d();
      
      // 8 corners
      int[] corners = new int[8];
      int[][] c = {{0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}, 
                   {1,1,0}, {1,0,1}, {0,1,1}, {1,1,1}
      };
      int[][] edges = {{0,1},  {0,2}, {0,3}, {1,4}, {1, 5}, {2, 4}, 
                       {2, 6}, {3,5}, {3,6}, {4,7}, {5,7}, {6, 7}};
      
      for (int i=0; i<c.length; ++i) {
         GridCoord3d pcoord = coord.plus(depth, 
            c[i][0]*gridCells[0], c[i][1]*gridCells[1], c[i][2]*gridCells[2]);
         Integer idx = gridMap.get(pcoord);
         if (idx == null) {
            pcoord.computePosition(pnt);
            pnt.transform(trans);
            idx = robj.vertex((float)pnt.x,(float)pnt.y,(float)pnt.z);
         }
         corners[i] = idx;
      }
      
      // 12 edges
      for (int[] edge : edges) {
         robj.addLine(corners[edge[0]], corners[edge[1]]);
      }
      
      for (int i=0; i<gridCells[0]; ++i) {
         for (int j=0; j<gridCells[1]; ++j) {
            for (int k=0; k<gridCells[2]; ++k) {
               int idx = root.getCellIndex(i, j, k);
               if (root.children != null && root.children[idx] != null) {
                  GridCoord3d pcoord = coord.plus(depth, i, j, k);
                  buildCellEdges(robj, gridMap, root.children[idx], depth+1, pcoord);
               }
            }
         }
      }
   }
   
   static PropertyList myProps = new PropertyList(AdaptiveSignedDistanceGrid.class, RenderableComponentBase.class);
   static {
      myProps.add("renderProps", "render props", createDefaultRenderProps());
   }
   @Override
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   private static RenderProps createDefaultRenderProps() {
      return new PointLineRenderProps();
   }
   
   @Override
   public RenderProps createRenderProps() {
      return new PointLineRenderProps();
   }
   
   RenderObject robj = null;
   
   @Override
   public void prerender(RenderList list) {
      super.prerender(list);
      //      if (robj == null) {
      //         robj = new RenderObject();
      //         GridCoord3d rootCoord = new GridCoord3d(0,0,0,0);
      //         HashMap<GridCoord3d,Integer> gridMap = new HashMap<>();
      //         buildCellPoints(robj, gridMap, root, 0, rootCoord);
      //         buildCellEdges(robj, gridMap, root, 0, rootCoord);
      //         gridMap.clear();
      //      }
      
   }

   @Override
   public void render(Renderer renderer, int flags) {
      
      //      RigidTransform3d m2w = mesh.getMeshToWorld();
      //      if (m2w != null) {
      //         renderer.pushModelMatrix();
      //         renderer.mulModelMatrix(m2w);
      //      }
      //      
      //      RenderProps rprops = getRenderProps();
      //      renderer.setPointColoring(rprops, false);
      //      renderer.drawPoints(robj, 0, rprops.getPointStyle(), 
      //         rprops.getPointStyle() == PointStyle.POINT ? rprops.getPointSize() : rprops.getPointRadius());
      //      renderer.setLineColoring(rprops, false);
      //      renderer.drawLines(robj, 0, rprops.getLineStyle(), 
      //         rprops.getLineStyle() == LineStyle.LINE ? rprops.getLineWidth() : rprops.getLineRadius());
      //      
      //      
      //      if (m2w != null) {
      //         renderer.popModelMatrix();
      //      }
   }
   
}
