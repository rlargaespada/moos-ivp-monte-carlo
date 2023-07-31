#ifndef GCS2D_GCSVERTEX_HEADER
#define GCS2D_GCSVERTEX_HEADER

#include <string>

#include "convex_sets.h"


class GCSVertex
{
 public:
  // todo: constructors

 public:
  const std::string& name() const { return (m_name); }

  // todo: additional costs and constraints
  // todo: get solution

  // todo: incoming and outgoing edges: getters, setters, removers
 private:
  std::string m_name;
  ConvexSet m_set;
  // x, ell, costs, constraints
  // incoming and outgoing edges
};

#endif  // GCS2D_GCSVERTEX_HEADER
