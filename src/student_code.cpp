#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
      // TODO Part 1.
      // must use evaluatedLevels / control points to track level for recursiveness
      std::vector<Vector2D> currLevel;

      for (int i = 0; i < points.size() - 1; ++i) {
          Vector2D pointNew = (1 - this->t)*points[i] + points[i+1] * this->t;
          currLevel.push_back(pointNew);
      }
      
      // return std::vector<Vector2D>();
      return currLevel;
      
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      std::vector<Vector3D> currLevel;

      for (int i = 0; i < points.size() - 1; ++i) {
          Vector3D pointNew = (1 - t)*points[i] + points[i+1] * t;
          currLevel.push_back(pointNew);
      }
      
      return currLevel;
    // return std::vector<Vector3D>();
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      // stop recursion when points is on final level (point.size() = 1)
      if (points.size() == 1) {
          return points[0];
      }
      
      std::vector<Vector3D> currLevel;

      for (int i = 0; i < points.size() - 1; ++i) {
          Vector3D curveNew = (1 - t)*points[i] + points[i+1] * t;
          currLevel.push_back(curveNew);
      }
      
      // return std::vector<Vector2D>();
      return this->evaluate1D(currLevel, t);
    // return Vector3D();
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
      // must utilize class variable control points
      std::vector<Vector3D> bezCurve;
      // vector<Vector3D> contPoints = this->controlPoints;

      for (int i = 0; i < this->controlPoints.size(); ++i) {
          Vector3D curveNew = this->evaluate1D(this->controlPoints[i], u);
          bezCurve.push_back(curveNew);
      }
      
      // return std::vector<Vector2D>();
      return this->evaluate1D(bezCurve, v);
    // return Vector3D();
  }

  Vector3D Vertex::normal( void ) const
  {
      // TODO Part 3.
      // Returns an approximate unit normal at this vertex, computed by
      // taking the area-weighted average of the normals of neighboring
      // triangles, then normalizing.
      // face in halfedgemesh.h has example, follow roughly
      // use HalfedgeCIter per hint
      
      Vector3D norm(0., 0., 0.);
      HalfedgeCIter h = halfedge();
      
      do {
          Vector3D iPos = h->vertex()->position;
          Vector3D jPos = h->next()->vertex()->position;
          
          norm += cross(iPos, jPos);
          h = h->next();
          
      } while (h != halfedge());
      
      // .unit() returns unit vector
      return norm.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
      // TODO Part 4.
      // This method should flip the given edge and return an iterator to the flipped edge.
      // 5 edges, 10 half edges, 4 vertices
      // method to follow from hints:
      // define all elemenets from e0, then make new mesh of flipped edge
      // half edge points to twin, next, vertex, edge, face
      // vertex points to point, halfedge
      // edge points to half edge
      // face points to half edge
      // Following provided template for edge flipping: 15462.courses.cs.cmu.edu/fall2015content/misc/HalfedgeEdgeOpImplementationGuide.pdf
      // guide was found from provided supplementary notes
      
      // set initial mesh
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();
      
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();
      
      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();
      
      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();
      
      // check for boundary edges, check inside half edges, can't flip boundary
      if (h0->isBoundary() || h1->isBoundary() || h2->isBoundary() || h3->isBoundary() || h4->isBoundary() || h5->isBoundary()) {
          return e0;
      }
      
      // reassign everything, use setNeighbors for halfedges: next, twin, vertex, edge, face
      // do not assign next or face for outside half edges
      
      h0->setNeighbors(h1, h3, v3, e0, f0);
      h1->setNeighbors(h2, h7, v2, e2, f0);
      h2->setNeighbors(h0, h8, v0, e3, f0);
      h3->setNeighbors(h4, h0, v2, e0, f1);
      h4->setNeighbors(h5, h9, v3, e4, f1);
      h5->setNeighbors(h3, h6, v1, e1, f1);
      h6->setNeighbors(h6->next(), h5, v2, e1, h6->face());
      h7->setNeighbors(h7->next(), h1, v0, e2, h7->face());
      h8->setNeighbors(h8->next(), h2, v3, e3, h8->face());
      h9->setNeighbors(h9->next(), h4, v1, e4, h9->face());
      
      v0->halfedge() = h2;
      v1->halfedge() = h5;
      v2->halfedge() = h3;
      v3->halfedge() = h0;
      
      e0->halfedge() = h0;
      e1->halfedge() = h5;
      e2->halfedge() = h1;
      e3->halfedge() = h2;
      e4->halfedge() = h4;
      
      f0->halfedge() = h0;
      f1->halfedge() = h3;
      
      return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
      // TODO Part 5.
      // This method should split the given edge and return an iterator to the newly inserted vertex.
      // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      // 1 new vertex, 2 new faces, 3 new edges, more pointers
      
      // reuse original assignment as from flipEdge()
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();
      
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();
      
      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();
      
      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();
      
      // check for boundary edges, check inside half edges, can't split boundary
      if (h0->isBoundary() || h1->isBoundary() || h2->isBoundary() || h3->isBoundary() || h4->isBoundary() || h5->isBoundary()) {
          return e0->halfedge()->vertex();
      }
      
      // create new elements
      HalfedgeIter h10, h11, h12, h13, h14, h15;
      VertexIter v4;
      EdgeIter e5, e6, e7;
      FaceIter f2, f3;
      
      return e0->halfedge()->vertex();
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.

  }
}
