#pragma once
#include <algorithm>
#include <array>
#include <cassert>
#include <vector>

#include "AABB.h"


namespace b2dl {

// type aliases
typedef uint32_t bvh_index_t;
typedef std::vector<bvh_index_t> bvh_vector_t;

static const bvh_index_t bvh_invalid_index = -1;

struct node_t {

  // for a leaf this will be a fat aabb and non terminal nodes will be regular
  aabb_t aabb;

  // left and right children
  std::array<bvh_index_t, 2> child;

  // index of the parent node
  bvh_index_t parent;

  // user provided data
  void *user_data;

  // query if this node is a neaf
  bool is_leaf() const {
    return child[0] == bvh_invalid_index;
  }

  void replace_child(bvh_index_t prev, bvh_index_t with) {
    if (child[0] == prev) {
      child[0] = with;
    }
    else {
      assert(child[1] == prev);
      child[1] = with;
    }
  }
};

struct bvh_t {

  bvh_t();

  // remove all nodes from the tree
  void clear();

  // create a new node in the tree
  bvh_index_t insert(const aabb_t &aabb, void *user_data);

  // remove a node from the tree
  void remove(bvh_index_t index);

  // move an existing node in the tree
  void move(bvh_index_t index, const aabb_t &aabb);

  // return a nodes user data
  void *user_data(bvh_index_t index) const {
    assert(index >= 0 && index < bvh_index_t(_nodes.size()));
    return _nodes[index].user_data;
  }

  // get a node from the tree
  const node_t &get(bvh_index_t index) const {
    assert(index >= 0 && index < bvh_index_t(_nodes.size()));
    return _nodes[index];
  }

  // get the root node of the tree
  const node_t &root() const {
    assert(_root != bvh_invalid_index);
    return _nodes[_root];
  }

  // return true if there are no nodes
  bool empty() const {
    return _root == bvh_invalid_index;
  }

  // find all overlaps with a given bounding-box
  bool find_overlaps(const aabb_t &bb, bvh_vector_t &overlaps) const;

  // find all overlaps with a given node
  // note: overlaps will also contain 'node' itself
  bool find_overlaps(bvh_index_t node, bvh_vector_t &overlaps) const;
  
  // return a quality metric for this tree
  float quality() const {
    return _quality(_root);
  }

  const bvh_index_t root_index() const {
    return _root;
  }

  // this is the growth size for fat aabbs (they will be expanded by this)
  float growth;

protected:

  // bubble up tree recalculating aabbs
  void _recalc_aabbs(bvh_index_t);

  // return a quality metric for this subtree
  float _quality(bvh_index_t) const;

  // walk up the tree recalculating the aabb
  void _touched_aabb(bvh_index_t i);

  // optimize the children of this node
  void _optimize(node_t &node);

  // sanity checks for the tree
  void _validate(bvh_index_t index);

  // insert node into the tree
  void _insert(bvh_index_t node);

  // find the best sibling leaf node for a given aabb
  bvh_index_t _find_best_sibling(const aabb_t &aabb) const;

  // insert 'node' into 'leaf'
  bvh_index_t _insert_into_leaf(bvh_index_t leaf, bvh_index_t node);

  // unlink this node from the tree but dont add it to the free list
  void _unlink(bvh_index_t index);

  // return true if a node is a leaf
  bool _is_leaf(bvh_index_t index) const;

  // access a node by index
  node_t &_get(bvh_index_t index) {
    assert(index != bvh_invalid_index);
    return _nodes[index];
  }

  // access a node by index
  const node_t &_get(bvh_index_t index) const {
    assert(index != bvh_invalid_index);
    return _nodes[index];
  }

  // get a child
  node_t &_child(bvh_index_t index, int32_t child) {
    return _get(_get(index).child[child]);
  }

  // release all nodes
  void _free_all();

  // allocate a new node from the free list
  bvh_index_t _new_node();

  // add a node to the free list
  void _free_node(bvh_index_t index);

  static const uint32_t _max_nodes = 1024 * 32;

  // free and taken bvh nodes
  std::array<node_t, _max_nodes> _nodes;
  // start index of the free list
  bvh_index_t _free_list;
  // root node of the bvh
  bvh_index_t _root;
};

} // namespace bvh
