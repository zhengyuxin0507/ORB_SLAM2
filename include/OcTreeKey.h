#ifndef OCTOMAP_OCTREE_KEY_H
#define OCTOMAP_OCTREE_KEY_H

#include <ciso646>
#include <assert.h>
#include "stdint.h"
#include "stddef.h"

namespace ORB_SLAM2 {

  typedef uint16_t key_type;
  
  class OcTreeKey {
    
  public:  
    OcTreeKey () {}
    OcTreeKey (key_type a, key_type b, key_type c){ 
      k[0] = a; 
      k[1] = b; 
      k[2] = c;         
    }
    
    OcTreeKey(const OcTreeKey& other){
      k[0] = other.k[0]; 
      k[1] = other.k[1]; 
      k[2] = other.k[2];
    }
    
    bool operator== (const OcTreeKey &other) const { 
      return ((k[0] == other[0]) && (k[1] == other[1]) && (k[2] == other[2]));
    }
    
    bool operator!= (const OcTreeKey& other) const {
      return( (k[0] != other[0]) || (k[1] != other[1]) || (k[2] != other[2]) );
    }
    
    OcTreeKey& operator=(const OcTreeKey& other){
      k[0] = other.k[0]; k[1] = other.k[1]; k[2] = other.k[2];
      return *this;
    }
    
    const key_type& operator[] (unsigned int i) const { 
      return k[i];
    }
    
    key_type& operator[] (unsigned int i) { 
      return k[i];
    }

    key_type k[3];

    /// Provides a hash function on Keys
    struct KeyHash{
      size_t operator()(const OcTreeKey& key) const{
        // a simple hashing function 
	// explicit casts to size_t to operate on the complete range
	// constanst will be promoted according to C++ standard
        return static_cast<size_t>(key.k[0])
          + 1447*static_cast<size_t>(key.k[1])
          + 345637*static_cast<size_t>(key.k[2]);
      }
    };
    
  };

  /**
   * Computes the key of a child node while traversing the octree, given
   * child index and current key
   *
   * @param[in] pos index of child node (0..7)
   * @param[in] center_offset_key constant offset of octree keys
   * @param[in] parent_key current (parent) key
   * @param[out] child_key  computed child key
   */
  inline void computeChildKey (unsigned int pos, key_type center_offset_key,
                                          const OcTreeKey& parent_key, OcTreeKey& child_key) {
    // x-axis
    if (pos & 1) child_key[0] = parent_key[0] + center_offset_key;
    else         child_key[0] = parent_key[0] - center_offset_key - (center_offset_key ? 0 : 1);
    // y-axis
    if (pos & 2) child_key[1] = parent_key[1] + center_offset_key;
    else         child_key[1] = parent_key[1] - center_offset_key - (center_offset_key ? 0 : 1);
    // z-axis
    if (pos & 4) child_key[2] = parent_key[2] + center_offset_key;
    else         child_key[2] = parent_key[2] - center_offset_key - (center_offset_key ? 0 : 1);
  }
  
  /// generate child index (between 0 and 7) from key at given tree depth
  inline uint8_t computeChildIdx(const OcTreeKey& key, int depth){
    uint8_t pos = 0;
    if (key.k[0] & (1 << depth)) 
      pos += 1;
    
    if (key.k[1] & (1 << depth)) 
      pos += 2;
    
    if (key.k[2] & (1 << depth)) 
      pos += 4;
    
    return pos;
  }

  /**
   * Generates a unique key for all keys on a certain level of the tree
   *
   * @param level from the bottom (= tree_depth - depth of key)
   * @param key input indexing key (at lowest resolution / level)
   * @return key corresponding to the input key at the given level
   */
  inline OcTreeKey computeIndexKey(key_type level, const OcTreeKey& key) {
    if (level == 0)
      return key;
    else {
      key_type mask = 65535 << level;
      OcTreeKey result = key;
      result[0] &= mask;
      result[1] &= mask;
      result[2] &= mask;
      return result;
    }
  }

} // namespace

#endif
