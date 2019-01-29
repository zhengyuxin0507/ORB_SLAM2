#ifndef Octree_H
#define Octree_H

#include <iostream>
#include <vector>
#include <fstream>
#include <bitset>

#include "include/math/Vector3.h"
#include "OctreeNode.h"
#include "OcTreeKey.h"
#include <opencv2/core/core.hpp>

#include "MapPoint.h"

#define OCTOMAP_ERROR_STR(args)   std::cerr << "ERROR: " << args << std::endl
#define OCTOMAP_DEBUG(...)        fprintf(stderr, __VA_ARGS__), fflush(stderr)
#define OCTOMAP_WARNING_STR(args) std::cerr << "WARNING: " << args << std::endl

typedef octomath::Vector3 point3d;

namespace ORB_SLAM2 {

    class MapPoint;

    class Octree{
    public:
        ORB_SLAM2::OctreeNode* root;

        const unsigned int tree_depth;
        const unsigned int tree_max_val;
        double resolution;
        double resolution_factor; 
  
        size_t tree_size;
        bool size_changed;

        point3d tree_center;

        double max_value[3];
        double min_value[3];
        std::vector<double> sizeLookupTable;

        const std::string binaryFileHeader = "# Octomap OcTree binary file";
        const std::string fileHeader = "# Octomap OcTree file";

        public:
		Octree(double in_resolution) :
        root(NULL), tree_depth(16), tree_max_val(32768),
        resolution(in_resolution), tree_size(0)
        {
            init();
        };

        void init()
        {

            this->setResolution(this->resolution);
            for (unsigned i = 0; i< 3; i++){
                max_value[i] = -(std::numeric_limits<double>::max( ));
                min_value[i] = std::numeric_limits<double>::max( );
            }
            size_changed = true;
        }

        void setResolution(double r) 
        {
            resolution = r;
            resolution_factor = 1. / resolution;

            tree_center(0) = tree_center(1) = tree_center(2)
                = (float) (((double) tree_max_val) / resolution_factor);

            // init node size lookup table:
            sizeLookupTable.resize(tree_depth+1);
            for(unsigned i = 0; i <= tree_depth; ++i){
            sizeLookupTable[i] = resolution * double(1 << (tree_depth - i));
            }

            size_changed = true;
        }

        OctreeNode* updateNode(const OcTreeKey& key, MapPoint* _MapPoint) 
        {
            this->search(key);

            bool createdRoot = false;
            if (this->root == NULL){
                this->root = new OctreeNode();
                this->tree_size++;
                createdRoot = true;
            }

            return updateNodeRecurs(this->root, createdRoot, key, 0, _MapPoint);
        }

        OctreeNode* updateNode(const float xw, const float yw, const float zw, MapPoint* _MapPoint)
        {       
            octomath::Vector3 point(xw, yw, zw);
            OcTreeKey key;
            if (!this->coordToKeyChecked(point, key))
                return NULL;

            return updateNode(key, _MapPoint);
        }

        bool DeletePoint(const OcTreeKey& key, MapPoint* _MapPoint) 
        {
            OctreeNode* leaf = this->search(key);
            if(leaf == NULL || this->root == NULL)
            {
                cout << "delete point error" << endl;
                return false;
            }

            return DeletePointRecurs(this->root, key, 0, _MapPoint);
        }

        bool DeletePoint(const float xw, const float yw, const float zw, MapPoint* _MapPoint)
        {           
            octomath::Vector3 point(xw, yw, zw);
            OcTreeKey key;
            if (!this->coordToKeyChecked(point, key))
                return NULL;

            return DeletePoint(key, _MapPoint);
        }

        bool coordToKeyChecked(const point3d& point, OcTreeKey& key) const
        {
            for (unsigned int i=0;i<3;i++) {
                if (!coordToKeyChecked( point(i), key[i])){
                    return false;
                }
            }
            return true;
        }

        bool coordToKeyChecked(double coordinate, key_type& keyval) const 
        {
            // scale to resolution and shift center for tree_max_val
            int scaled_coord =  ((int) floor(resolution_factor * coordinate)) + tree_max_val;

            // keyval within range of tree?
            if (( scaled_coord >= 0) && (((unsigned int) scaled_coord) < (2*tree_max_val))) {
                keyval = scaled_coord;
                return true;
            }
            return false;
        }

        OctreeNode* updateNodeRecurs(OctreeNode* node, bool node_just_created, const OcTreeKey& key, unsigned int depth, MapPoint* _MapPoint) 
        {
            bool created_node = false;

            assert(node);

            // follow down to last level
            if (depth < this->tree_depth) {
                unsigned int pos = computeChildIdx(key, this->tree_depth -1 - depth);
                if (!this->nodeChildExists(node, pos)) {
                    // child does not exist, but maybe it's a pruned node?
                    if (!this->nodeHasChildren(node) && !node_just_created ) {
                        // current node does not have children AND it is not a new node
                        // -> expand pruned node
                        this->expandNode(node);
                    }
                    else {
                        // not a pruned node, create requested child
                        this->createNodeChild(node, pos);
                        created_node = true;
                    }
                }

                OctreeNode* retval = updateNodeRecurs(this->getNodeChild(node, pos), created_node, key, depth+1, _MapPoint);

                return retval;
            }
            // at last level, update node, end of recursion
            else {
                node->mspMapPoints.insert(_MapPoint);
                node->value = node->mspMapPoints.size();
                UpdateColor(node);
                return node;
            }
        }

        bool DeletePointRecurs(OctreeNode* node, const OcTreeKey& key, unsigned int depth, MapPoint* _MapPoint) 
        {
            assert(node);

            if(depth >= tree_depth)
            {
                std::set<MapPoint*>::iterator it;
                it = node->mspMapPoints.find(_MapPoint);
                if(it != node->mspMapPoints.end())
                {
                    //node->mspMapPoints.erase(it);
                }

                if(node->mspMapPoints.size() == 0)
                    return true;
                else
                {
                    node->value = node->mspMapPoints.size();
                    UpdateColor(node);
                    return false; 
                }       
            }

            unsigned int pos = computeChildIdx(key, this->tree_depth-1-depth);

            bool deleteChild = DeletePointRecurs(getNodeChild(node, pos), key, depth+1, _MapPoint);

            if (deleteChild){

                this->deleteNodeChild(node, pos);

                if (!nodeHasChildren(node))
                {
                    node->children = NULL;
                    return true;
                }
            }

            return false;
        }

        void UpdateColor(OctreeNode* node)
        {
            int color = node->value / 20 * (0xFF00 - 0x00FF) + 0x00FF;
            int r = (color & 0xFF00) / 0xFF;
            int g = color & 0x00FF;
            int b = 0;
            node->setColor(r,g,b);
        }


        OctreeNode* search (const OcTreeKey& key, unsigned int depth = 0) const 
        {
            assert(depth <= tree_depth);
            if (root == NULL)
                return NULL;

            if (depth == 0)
                depth = tree_depth;

            // generate appropriate key_at_depth for queried depth
            OcTreeKey key_at_depth = key;
            if (depth != tree_depth)
                key_at_depth = adjustKeyAtDepth(key, depth);

            OctreeNode* curNode (root);

            int diff = tree_depth - depth;

            // follow nodes down to requested level (for diff = 0 it's the last level)
            for (int i=(tree_depth-1); i>=diff; --i) {
                unsigned int pos = computeChildIdx(key_at_depth, i);
                if (nodeChildExists(curNode, pos)) {
                // cast needed: (nodes need to ensure it's the right pointer)
                curNode = getNodeChild(curNode, pos);
                } else {
                    // we expected a child but did not get it
                    // is the current node a leaf already?
                    if (!nodeHasChildren(curNode)) { // TODO similar check to nodeChildExists?
                        return curNode;
                    } else {
                    // it is not, search failed
                    return NULL;
                    }
                }
            } // end for
            return curNode;
        };

        OctreeNode* search (const point3d& value, unsigned int depth = 0) const 
        {
            OcTreeKey key;
            if (!coordToKeyChecked(value, key)){
                //OCTOMAP_ERROR_STR("Error in search: ["<< value <<"] is out of OcTree bounds!");
                return NULL;
            }
            else 
            {
                return this->search(key, depth);
            }
        }

        bool pruneNode(OctreeNode* node)
        {

            if (!isNodeCollapsible(node))
                return false;

            // set value to children's values (all assumed equal)
            node->copyData(*(getNodeChild(node, 0)));

            // delete children (known to be leafs at this point!)
            for (unsigned int i=0;i<8;i++) {
                deleteNodeChild(node, i);
            }
            delete[] node->children;
            node->children = NULL;

            return true;
        }

        void deleteNodeChild(OctreeNode* node, unsigned int childIdx)
        {
            assert((childIdx < 8) && (node->children != NULL));
            assert(node->children[childIdx] != NULL);
            delete static_cast<OctreeNode*>(node->children[childIdx]); // TODO delete check if empty
            node->children[childIdx] = NULL;

            tree_size--;
            size_changed = true;
        }

        bool isNodeCollapsible(const OctreeNode* node) const
        {
            // all children must exist, must not have children of
            // their own and have the same occupancy probability
            if (!nodeChildExists(node, 0))
                return false;

            const OctreeNode* firstChild = getNodeChild(node, 0);
            if (nodeHasChildren(firstChild))
                return false;

            for (unsigned int i = 1; i<8; i++) {
                // comparison via getChild so that casts of derived classes ensure
                // that the right == operator gets called
                if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || !(*(getNodeChild(node, i)) == *(firstChild)))
                    return false;
            }

            return true;
        }

        void expandNode(OctreeNode* node)
        {
            assert(!nodeHasChildren(node));

            for (unsigned int k=0; k<8; k++) {
                createNodeChild(node, k);
                //OctreeNode* newNode = createNodeChild(node, k);
                //newNode->copyData(*node);
            }
        }

        OctreeNode* createNodeChild(OctreeNode* node, unsigned int childIdx)
        {
            assert(childIdx < 8);
            if (node->children == NULL) {
                allocNodeChildren(node);
            }
            assert (node->children[childIdx] == NULL);
            OctreeNode* newNode = new OctreeNode();
            node->children[childIdx] = static_cast<OctreeNode*>(newNode);

            tree_size++;
            size_changed = true;

            return newNode;
        }

        void allocNodeChildren(OctreeNode* node)
        {
            // TODO NODE*
            node->children = new OctreeNode*[8];
            for (unsigned int i=0; i<8; i++) {
            node->children[i] = NULL;
            }
        }

        OcTreeKey adjustKeyAtDepth(const OcTreeKey& key, unsigned int depth) const
        {
            if (depth == tree_depth)
                return key;

            assert(depth <= tree_depth);

            return OcTreeKey(adjustKeyAtDepth(key[0], depth), adjustKeyAtDepth(key[1], depth), adjustKeyAtDepth(key[2], depth));
        };

        key_type adjustKeyAtDepth(key_type key, unsigned int depth) const
        {
            unsigned int diff = tree_depth - depth;

            if(diff == 0)
                return key;
            else
                return (((key-tree_max_val) >> diff) << diff) + (1 << (diff-1)) + tree_max_val;
        };

        bool nodeChildExists(const OctreeNode* node, unsigned int childIdx) const
        {
            assert(childIdx < 8);
            if ((node->children != NULL) && (node->children[childIdx] != NULL))
                return true;
            else
                return false;
        };

        OctreeNode* getNodeChild(const OctreeNode* node, unsigned int childIdx) const
        {
            assert((childIdx < 8) && (node->children != NULL));
            assert(node->children[childIdx] != NULL);

            return static_cast<OctreeNode*>(node->children[childIdx]);
        };

        bool nodeHasChildren(const OctreeNode* node) const 
        {
            if (node->children == NULL)
                return false;

            for (unsigned int i = 0; i<8; i++){
                if (node->children[i] != NULL)
                    return true;
            }
            return false;
        };

        bool isNodeOccupied(const OctreeNode* Node) const
        {
            return (Node->value >= 0);
        }

        bool writeBinary(const std::string& filename)
        {
            std::ofstream binary_outfile( filename.c_str(), std::ios_base::binary);

            if (!binary_outfile.is_open()){
                OCTOMAP_ERROR_STR("Filestream to "<< filename << " not open, nothing written.");
                return false;
            }
            return writeBinary(binary_outfile);
        };

        bool writeBinary(std::ostream &s)
        {
            // convert to max likelihood first, this makes efficient pruning on binary data possible
            //this->toMaxLikelihood();
            //this->prune();
            return writeBinaryConst(s);
        }

        bool writeBinaryConst(std::ostream &s) const
        {
            // write new header first:
            s << binaryFileHeader <<"\n# (feel free to add / change comments, but leave the first line as it is!)\n#\n";
            s << "id " << "ColorOcTree" << std::endl;
            s << "size "<< tree_size << std::endl;
            s << "res " << resolution << std::endl;
            s << "data" << std::endl;

            writeBinaryData(s);

            if (s.good()){
                OCTOMAP_DEBUG(" done.\n");
                return true;
            } else {
                OCTOMAP_WARNING_STR("Output stream not \"good\" after writing tree");
                return false;
            }
        }

        std::ostream& writeBinaryData(std::ostream &s) const
        {
            OCTOMAP_DEBUG("Writing %zu nodes to output stream...", tree_size);
            if (this->root)
                this->writeBinaryNode(s, this->root);
            return s;
        }

        std::ostream& writeBinaryNode(std::ostream &s, const OctreeNode* node) const
        {
            assert(node);

            // 2 bits for each children, 8 children per node -> 16 bits
            std::bitset<8> child1to4;
            std::bitset<8> child5to8;

            // 10 : child is free node
            // 01 : child is occupied node
            // 00 : child is unkown node
            // 11 : child has children


            // speedup: only set bits to 1, rest is init with 0 anyway,
            //          can be one logic expression per bit

            for (unsigned int i=0; i<4; i++) {
                if (this->nodeChildExists(node, i)) {
                    const OctreeNode* child = this->getNodeChild(node, i);
                    if      (this->nodeHasChildren(child))  { child1to4[i*2] = 1; child1to4[i*2+1] = 1; }
                    else if (this->isNodeOccupied(child)) { child1to4[i*2] = 0; child1to4[i*2+1] = 1; }
                        else                            { child1to4[i*2] = 1; child1to4[i*2+1] = 0; }
                }
                else {
                    child1to4[i*2] = 0; child1to4[i*2+1] = 0;
                }
            }

            for (unsigned int i=0; i<4; i++) {
                if (this->nodeChildExists(node, i+4)) {
                    const OctreeNode* child = this->getNodeChild(node, i+4);
                    if      (this->nodeHasChildren(child))  { child5to8[i*2] = 1; child5to8[i*2+1] = 1; }
                    else if (this->isNodeOccupied(child)) { child5to8[i*2] = 0; child5to8[i*2+1] = 1; }
                        else                            { child5to8[i*2] = 1; child5to8[i*2+1] = 0; }
                }
                else {
                child5to8[i*2] = 0; child5to8[i*2+1] = 0;
                }
            }
            //     std::cout << "wrote: "
            //        << child1to4.to_string<char,std::char_traits<char>,std::allocator<char> >() << " "
            //        << child5to8.to_string<char,std::char_traits<char>,std::allocator<char> >() << std::endl;

            char child1to4_char = (char) child1to4.to_ulong();
            char child5to8_char = (char) child5to8.to_ulong();

            s.write((char*)&child1to4_char, sizeof(char));
            s.write((char*)&child5to8_char, sizeof(char));

            // write children's children
            for (unsigned int i=0; i<8; i++) {
                if (this->nodeChildExists(node, i)) {
                    const OctreeNode* child = this->getNodeChild(node, i);
                    if (this->nodeHasChildren(child)) {
                        writeBinaryNode(s, child);
                    }
                }
            }
            return s;
        }

        bool write(const std::string& filename) const
        {
            std::ofstream file(filename.c_str(), std::ios_base::out | std::ios_base::binary);

            if (!file.is_open()){
                OCTOMAP_ERROR_STR("Filestream to "<< filename << " not open, nothing written.");
                return false;
            } else {
                // TODO: check is_good of finished stream, return
                write(file);
                file.close();
            }
            return true;
        }

        bool write(std::ostream &s) const
        {
            s << fileHeader <<"\n# (feel free to add / change comments, but leave the first line as it is!)\n#\n";
            s << "id " << "ColorOcTree" << std::endl;
            s << "size "<< tree_size << std::endl;
            s << "res " << resolution << std::endl;
            s << "data" << std::endl;

            // write the actual data:
            writeData(s);
            // TODO: ret.val, checks stream?
            return true;
        }

        std::ostream& writeData(std::ostream &s) const
        {
            if (root)
                writeNodesRecurs(root, s);
            return s;
        }

        std::ostream& writeNodesRecurs(const OctreeNode* node, std::ostream &s) const
        {
            node->writeData(s);

            // 1 bit for each children; 0: empty, 1: allocated
            std::bitset<8> children;
            for (unsigned int i=0; i<8; i++) {
                if (nodeChildExists(node, i))
                    children[i] = 1;
                else
                    children[i] = 0;
            }

            char children_char = (char) children.to_ulong();
            s.write((char*)&children_char, sizeof(char));

            // recursively write children
            for (unsigned int i=0; i<8; i++) {
                if (children[i] == 1) {
                    this->writeNodesRecurs(getNodeChild(node, i), s);
                }
            }
            return s;
        }

    };
}

#endif