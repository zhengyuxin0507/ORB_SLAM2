#ifndef OctreeNode_H
#define OctreeNode_H

#include <iostream>
#include "assert.h"
#include <limits>
#include <set>
#include "MapPoint.h"

typedef unsigned char uint8_t;

namespace ORB_SLAM2 {

    class OctreeNode
    {

    class Color 
    {
    public:
        Color() : r(255), g(255), b(255) {}
        Color(uint8_t _r, uint8_t _g, uint8_t _b)  : r(_r), g(_g), b(_b) {}

        inline bool operator== (const Color &other) const {
            return (r==other.r && g==other.g && b==other.b);
        }

        inline bool operator!= (const Color &other) const {
            return (r!=other.r || g!=other.g || b!=other.b);
        }

        uint8_t r, g, b;
    };

    public:
    OctreeNode():children(NULL), value(0.0f)
    {
    };

    OctreeNode(const OctreeNode& rhs):children(NULL), value(rhs.value)
    {
        if (rhs.children != NULL){
            allocChildren();
        for (unsigned i = 0; i<8; ++i){
            if (rhs.children[i] != NULL)
            children[i] = new OctreeNode(*(static_cast<OctreeNode*>(rhs.children[i])));
            }
        }
    }

    ~OctreeNode()
    {
        assert(children == NULL);
    };


    bool operator== (const OctreeNode& rhs) const
    {
        return rhs.value == value;
    }

    void updateOccupancyChildren() 
    {
      this->setLogOdds(this->getMaxChildLogOdds());  // conservative
    }

    void setLogOdds(float l) { value = l; }

    float getMaxChildLogOdds() const
    {
        float max = -std::numeric_limits<float>::max();
    
        if (children !=NULL){
            for (unsigned int i=0; i<8; i++) {
                if (children[i] != NULL) {
                float l = static_cast<OctreeNode*>(children[i])->value;
                if (l > max)
                    max = l;
                }
            }
        }
    return max;
    }

    inline void setColor(uint8_t r, uint8_t g, uint8_t b) {
      this->color = Color(r,g,b); 
    }

    std::ostream& writeData(std::ostream &s) const 
    {
        s.write((const char*) &value, sizeof(value)); // occupancy
        s.write((const char*) &color, sizeof(Color)); // color
        return s;
    }

    vector<MapPoint*> GetAllMapPoints()
    {
        return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
    }

    public:

        std::set< MapPoint* > mspMapPoints;

        Color color;

        OctreeNode** children;

        float value;

        void allocChildren()
        {
            children = new OctreeNode*[8];
            for (unsigned int i=0; i<8; i++) {
            children[i] = NULL;
            }
        };

        void copyData(const OctreeNode& from)
        {
            value = from.value;     
        }
    };
}

#endif