#ifndef OCTOMAP_TEXTURE_OCTREE_H
#define OCTOMAP_TEXTURE_OCTREE_H


#include <iostream>
#include <array>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

namespace octomap {
  
  class Face {
  public:
    Face() : value(255), nObs(0) {}
    Face(unsigned char _value, unsigned int _nObs)
      : value(_value), nObs(_nObs) {}
    inline bool operator== (const Face &other) const {
      return (value == other.value && nObs == other.nObs);
    }
    inline bool operator!= (const Face &other) const {
      return (value!=other.value || nObs!=other.nObs);
    }
    inline void addObservation (unsigned char _v) {
      value = ((nObs*value) + _v)/(nObs+1);
      nObs++;
    }
    unsigned char value; // mean intensity value
    unsigned int nObs; // number of observations in mean
  } ;

  enum FaceEnum {xplus=0, xminus, yplus, yminus, zplus, zminus};

  // node definition
  class TextureOcTreeNode : public OcTreeNode {    
  public:
    TextureOcTreeNode() : OcTreeNode() {}

    TextureOcTreeNode(const TextureOcTreeNode& rhs) : OcTreeNode(rhs), faces(rhs.faces) {}

    bool operator==(const TextureOcTreeNode& rhs) const{
      return (rhs.value == value && rhs.faces == faces); // TODO might need to redefine this so faces checks each one
    }
    
    // children
    inline TextureOcTreeNode* getChild(unsigned int i) {
      return static_cast<TextureOcTreeNode*> (OcTreeNode::getChild(i));
    }
    inline const TextureOcTreeNode* getChild(unsigned int i) const {
      return static_cast<const TextureOcTreeNode*> (OcTreeNode::getChild(i));
    }

    bool createChild(unsigned int i) {
      if (children == NULL) allocChildren();
      children[i] = new TextureOcTreeNode();
      return true;
    }

    bool pruneNode();
    void expandNode();
    
    inline unsigned char getFace(FaceEnum fe) const { return faces.at(fe).value; }
    inline void addObservation(FaceEnum fe, unsigned char val) { faces.at(fe).addObservation(val); }
    inline void setFace(FaceEnum fe, Face f) { faces.at(fe) = f; }

    // has any color been integrated? (pure white is very unlikely...)
    inline bool observed(FaceEnum fe) const { 
      return (bool) faces.at(fe).nObs;
    }

    void updateTextureFromChildren();

    Face getAverageChildTexture() const;
  
    // file I/O
    std::istream& readValue (std::istream &s);
    std::ostream& writeValue(std::ostream &s) const;
    
  protected:
    std::array<Face, 6> faces;
  };


  // tree definition
  class TextureOcTree : public OccupancyOcTreeBase <TextureOcTreeNode> {

  public:
    /// Default constructor, sets resolution of leafs
    TextureOcTree(double resolution) : OccupancyOcTreeBase<TextureOcTreeNode>(resolution) {};  
      
    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    TextureOcTree* create() const {return new TextureOcTree(resolution); }

    std::string getTreeType() const {return "TextureOcTree";}
   
    // set node texture at given key or coordinate. Replaces previous texture.
    TextureOcTreeNode* setNodeTexture(const OcTreeKey& key, const FaceEnum& fe, const Face& f);  

    TextureOcTreeNode* setNodeTexture(const float& x, const float& y, 
                                 const float& z, const FaceEnum& fe, const Face& f) {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return setNodeTexture(key,fe,f);
    }

    // add a new texture measurement to one of the faces
    TextureOcTreeNode* integrateFaceObservation(const OcTreeKey& key, const FaceEnum& fe, const unsigned char& val);
    TextureOcTreeNode* integrateFaceObservation(const float& x, const float& y, const float& z, 
                                                const FaceEnum& fe, const unsigned char& val) {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return integrateFaceObservation(key, fe, val);
    }

    // update inner nodes, sets color to average child color
    void updateInnerOccupancy();

    // uses gnuplot to plot a RGB histogram in EPS format
    void writeTextureHistogram(std::string filename);
    
  protected:
    void updateInnerOccupancyRecurs(TextureOcTreeNode* node, unsigned int depth);

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once
     */
    class StaticMemberInitializer{
       public:
         StaticMemberInitializer() {
           TextureOcTree* tree = new TextureOcTree(0.1);
           AbstractOcTree::registerTreeType(tree);
         }
    };
    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer textureOcTreeMemberInit;

  };

  //! user friendly output in format (xplus xminus yplus yminus zplus zminus)
  std::ostream& operator<<(std::ostream& out, Face const& f);

} // end namespace

#endif
