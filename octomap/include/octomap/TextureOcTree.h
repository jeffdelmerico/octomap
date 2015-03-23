#ifndef OCTOMAP_TEXTURE_OCTREE_H
#define OCTOMAP_TEXTURE_OCTREE_H


#include <iostream>
#include <array>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeStereo.h>

namespace octomap {
  
  // A few global utility functions
  enum class FaceEnum {xplus=0, xminus, yplus, yminus, zplus, zminus};
  inline FaceEnum operator++( FaceEnum& f ) { 
    return f = (FaceEnum)(std::underlying_type<FaceEnum>::type(f) + 1); 
  }

  // user friendly output in format 
  class Face;
  class TextureOcTreeNode;
  std::ostream& operator<<(std::ostream& out, Face const& f);
  std::ostream& operator<<(std::ostream& out, TextureOcTreeNode* n);


  class Face {
  public:
    Face() : value(0), nObs(0), fe(FaceEnum::xplus) {}
    Face(unsigned char _value, unsigned int _nObs, FaceEnum _fe)
      : value(_value), nObs(_nObs), fe(_fe) {}
    inline bool operator== (const Face &other) const {
      return (value == other.value && nObs == other.nObs && fe == other.fe);
    }
    inline bool operator!= (const Face &other) const {
      return (value!=other.value || nObs!=other.nObs || fe!=other.fe);
    }
    inline void addObservation (unsigned char _v) {
      value = ((nObs*value) + _v)/(nObs+1);
      nObs++;
    }
    unsigned char value; // mean intensity value
    unsigned int nObs; // number of observations in mean
    FaceEnum fe; // Which face does this represent
  } ;

  // node definition
  class TextureOcTreeNode : public OcTreeNode {    
  public:
    TextureOcTreeNode() : OcTreeNode() 
    {
      FaceEnum fe = FaceEnum::xplus;
      for(auto f : faces)
      {
        f = Face(0,0,fe);
        ++fe;
      }
    }

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
    
    inline Face getFace(FaceEnum fe) const { return faces.at((int) fe); }
    inline unsigned char getFaceValue(FaceEnum fe) const { return faces.at((int) fe).value; }
    inline unsigned char getFaceObservations(FaceEnum fe) const { return faces.at((int) fe).nObs; }
    inline void addObservation(FaceEnum fe, unsigned char val) { faces.at((int) fe).addObservation(val); }
    inline void setFace(FaceEnum fe, Face f) { faces.at((int) fe) = f; }

    // have any observations been integrated?
    inline bool observed(FaceEnum fe) const { 
      return (bool) faces.at((int) fe).nObs;
    }

    void updateTextureChildren();

    Face getAverageChildTexture(FaceEnum fe) const;
  
    // file I/O
    std::istream& readValue (std::istream &s);
    std::ostream& writeValue(std::ostream &s) const;
    
  protected:
    std::array<Face, 6> faces;
  };


  // tree definition
  class TextureOcTree : public OccupancyOcTreeStereo <TextureOcTreeNode> {

  public:
    /// Default constructor, sets resolution of leafs
    TextureOcTree(double resolution, double coeff, double max_range) 
      : OccupancyOcTreeStereo<TextureOcTreeNode>(resolution,coeff,max_range) {};  
    virtual ~TextureOcTree() {};
      
    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    TextureOcTree* create() const {return new TextureOcTree(resolution,coefficient,maximum_range); }

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

    // Look up a face texture
    unsigned char getNodeTexture(const OcTreeKey& key, const FaceEnum& fe) const {
      return search(key)->getFaceValue(fe);
    }
    unsigned char getNodeTexture(const float& x, const float& y, const float& z, const FaceEnum& fe) const {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return 0;
      return getNodeTexture(key, fe);
    }

    // Print texture at a node to a stream
    void printNodeTexture(const OcTreeKey& key) const {
      TextureOcTreeNode* n = this->search(key);
      if (n)
        std::cout << (TextureOcTreeNode *) n;
      else
        std::cout << "This voxel has not been observed." << std::endl;
    }
    void printNodeTexture(const float& x, const float& y, const float& z) const {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return;
      std::cout << "Texture at (" << x << ',' << y << ',' << z << ')' << std::endl;
      printNodeTexture(key);
    }

    // update inner nodes, sets texture to average child texture
    void updateInnerOccupancy();

    // uses gnuplot to plot a RGB histogram in EPS format
    void writeTextureHistogram(std::string filename);

    void insertTexturePoint(const octomap::point3d& point,
                            const unsigned char& intensity,
                            const octomap::point3d& sensor_origin);

    // Insert a cloud of points with intensities
    void insertPointCloud(const Pointcloud& scan, 
                          const std::vector<unsigned char>& intensities,
                          const octomap::point3d& sensor_origin,
                          const octomath::Vector3& sensor_orientation, 
                          double maxrange = -1.0f, bool lazy_eval = false, 
                          bool discretize = false);
    
  protected:
    void updateInnerOccupancyRecurs(TextureOcTreeNode* node, unsigned int depth);

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once
     */
    class StaticMemberInitializer{
       public:
         StaticMemberInitializer() {
           TextureOcTree* tree = new TextureOcTree(0.1,10.0,15.0);
           AbstractOcTree::registerTreeType(tree);
         }
    };
    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer textureOcTreeMemberInit;

  };

} // end namespace

#endif
