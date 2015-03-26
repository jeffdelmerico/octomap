#include <octomap/TextureOcTree.h>

namespace octomap {

  // node implementation  --------------------------------------
  std::ostream& TextureOcTreeNode::writeValue (std::ostream &s) const {
    // 1 bit for each children; 0: empty, 1: allocated
    std::bitset<8> children;
    for (unsigned int i=0; i<8; i++) {
      if (childExists(i)) children[i] = 1;
      else                children[i] = 0;
    }
    char children_char = (char) children.to_ulong();
    
    // write node data
    s.write((const char*) &value, sizeof(value)); // occupancy
    // write face data
    for (auto it = faces.begin(); it!= faces.end(); it++)
      s.write((const char*) it, sizeof(Face)); // Faces
    s.write((char*)&children_char, sizeof(char)); // child existence

    // write existing children
    for (unsigned int i=0; i<8; ++i) 
      if (children[i] == 1) this->getChild(i)->writeValue(s);    
    return s;
  }

  std::istream& TextureOcTreeNode::readValue (std::istream &s) {
    // read node data
    char children_char;
    s.read((char*) &value, sizeof(value)); // occupancy
    // write face data
    for (auto it = faces.begin(); it!= faces.end(); it++)
      s.read((char*) it, sizeof(Face)); // Faces
    s.read((char*)&children_char, sizeof(char)); // child existence

    // read existing children
    std::bitset<8> children ((unsigned long long) children_char);
    for (unsigned int i=0; i<8; i++) {
      if (children[i] == 1){
        createChild(i);
        getChild(i)->readValue(s);
      }
    }
    return s;
  }

  Face TextureOcTreeNode::getAverageChildTexture(FaceEnum fe) const {
    int val(0);
    int obs(0);
    for (int i=0; i<8; i++) {
      if (childExists(i) && getChild(i)->observed(fe)) {
        unsigned int nObs = getChild(i)->getFaceObservations(fe);
        val += (getChild(i)->getFaceValue(fe) * nObs);
        obs += nObs;
      }
    }
    if (obs < 1)
    {
      val = 0;
      obs = 0;
    }
    else 
    {
      val = floor((float) val/(float) obs);
    }
    return Face(val,obs,fe);
  }

  void TextureOcTreeNode::updateTextureChildren() {      
    for (auto f : faces)
      if (f.nObs > 0) f = getAverageChildTexture(f.fe);
  }
  

  // pruning =============

  bool TextureOcTreeNode::pruneNode() {
    // checks for equal occupancy only, texture ignored
    if (!this->collapsible()) return false;
    // set occupancy value 
    setLogOdds(getChild(0)->getLogOdds());
    // set face textures to average child face textures 
    updateTextureChildren();
    // delete children
    for (unsigned int i=0;i<8;i++) {
      delete children[i];
    }
    delete[] children;
    children = NULL;
    return true;
  }

  void TextureOcTreeNode::expandNode() {
    assert(!hasChildren());
    for (unsigned int k=0; k<8; k++) {
      createChild(k);
      children[k]->setValue(value);
      int i = 0;
      for (auto f : faces) 
      {
        getChild(k)->setFace(static_cast<FaceEnum>(i),f);
        i++;
      }
    }
  }


  // tree implementation  --------------------------------------

  TextureOcTreeNode* TextureOcTree::setNodeTexture(const OcTreeKey& key, const FaceEnum& fe, 
                                                   const Face& f) {
    TextureOcTreeNode* n = search (key);
    if (n != 0) {
      n->setFace(fe, f); 
    }
    return n;
  }

  TextureOcTreeNode* TextureOcTree::integrateFaceObservation(const OcTreeKey& key, const FaceEnum& fe, 
                                                             const unsigned char& val) {
    TextureOcTreeNode* n = search(key);
    if (n!=0) {
      n->addObservation(fe, val);
    }
    return n;
  }

  void TextureOcTree::updateInnerOccupancy() {
    this->updateInnerOccupancyRecurs(this->root, 0);
  }

  void TextureOcTree::updateInnerOccupancyRecurs(TextureOcTreeNode* node, unsigned int depth) {
    // only recurse and update for inner nodes:
    if (node->hasChildren()){
      // return early for last level:
      if (depth < this->tree_depth){
        for (unsigned int i=0; i<8; i++) {
          if (node->childExists(i)) {
            updateInnerOccupancyRecurs(node->getChild(i), depth+1);
          }
        }
      }
      node->updateOccupancyChildren();
      node->updateTextureChildren();
    }
  }

  void TextureOcTree::writeTextureHistogram(std::string filename) {

#ifdef _MSC_VER
    fprintf(stderr, "The color histogram uses gnuplot, this is not supported under windows.\n");
#else
    // build histograms for face textures
    std::vector<int> histogram_xplus (256,0);
    std::vector<int> histogram_xminus (256,0);
    std::vector<int> histogram_yplus (256,0);
    std::vector<int> histogram_yminus (256,0);
    std::vector<int> histogram_zplus (256,0);
    std::vector<int> histogram_zminus (256,0);
    for(TextureOcTree::tree_iterator it = this->begin_tree(),
          end=this->end_tree(); it!= end; ++it) {
      if (!it.isLeaf() || !this->isNodeOccupied(*it)) continue;

      ++histogram_xplus[it->getFaceValue(FaceEnum::xplus)];
      ++histogram_xminus[it->getFaceValue(FaceEnum::xminus)];
      ++histogram_yplus[it->getFaceValue(FaceEnum::yplus)];
      ++histogram_yminus[it->getFaceValue(FaceEnum::yminus)];
      ++histogram_zplus[it->getFaceValue(FaceEnum::zplus)];
      ++histogram_zminus[it->getFaceValue(FaceEnum::zminus)];
    }
    // plot data
    FILE *gui = popen("gnuplot ", "w");
    fprintf(gui, "set term postscript eps enhanced color\n");
    fprintf(gui, "set output \"%s\"\n", filename.c_str());
    fprintf(gui, "plot [-1:256] ");
    fprintf(gui, "'-' w filledcurve lt 1 lc 1 tit \"+x\",");
    fprintf(gui, "'-' w filledcurve lt 1 lc 2 tit \"-x\",");
    fprintf(gui, "'-' w filledcurve lt 1 lc 3 tit \"+y\",");
    fprintf(gui, "'-' w filledcurve lt 1 lc 4 tit \"-y\",");
    fprintf(gui, "'-' w filledcurve lt 1 lc 5 tit \"+z\",");
    fprintf(gui, "'-' w filledcurve lt 1 lc 6 tit \"-z\",");
    fprintf(gui, "'-' w l lt 1 lc 1 tit \"\",");
    fprintf(gui, "'-' w l lt 1 lc 2 tit \"\",");
    fprintf(gui, "'-' w l lt 1 lc 3 tit \"\",");
    fprintf(gui, "'-' w l lt 1 lc 4 tit \"\",");
    fprintf(gui, "'-' w l lt 1 lc 5 tit \"\",");
    fprintf(gui, "'-' w l lt 1 lc 6 tit \"\"\n");

    // Here we omit the zero bin, since this is the default value and will be
    // large relative to the actually observed texture frequencies 
    for (int i=1; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_xplus[i]);    
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
    for (int i=1; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_xminus[i]);    
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
    for (int i=1; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_yplus[i]);    
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
    for (int i=1; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_yminus[i]);    
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
    for (int i=1; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_zplus[i]);    
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
    for (int i=1; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_zminus[i]);    
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");

    for (int i=1; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_xplus[i]);    
    fprintf(gui, "e\n");
    for (int i=1; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_xminus[i]);    
    fprintf(gui, "e\n");
    for (int i=1; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_yplus[i]);    
    fprintf(gui, "e\n");
    for (int i=1; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_yminus[i]);    
    fprintf(gui, "e\n");
    for (int i=1; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_zplus[i]);    
    fprintf(gui, "e\n");
    for (int i=1; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_zminus[i]);    
    fprintf(gui, "e\n");
    fflush(gui);
#endif
  }


  // point insertion  --------------------------------------

  void TextureOcTree::insertTexturePoint(const octomap::point3d& point,
                                         const unsigned char& intensity,
                                         const octomap::point3d& sensor_origin) {
    
    // Compute unit vector in the ray direction
    const octomath::Vector3 direction = (point - sensor_origin).normalize();
    
    // Compute key for leaf voxel that point lies inside
    OcTreeKey key;
    if (!this->coordToKeyChecked(point, key)) return;
    
    // Compute intersection point with voxel    
    point3d intersection;
    point3d center = this->keyToCoord(key);
    bool success = this->getRayIntersection(sensor_origin,direction,center,intersection);

    // Add observation to side it hits first
    if(success)
    {
      float dmin = std::numeric_limits<float>::max();
      FaceEnum fe;
      float d = 0.0;
      
      // x+
      d = fabs((center.x() + resolution/2.0) - intersection.x());
      if( d < dmin )
      {
        dmin = d;
        fe = FaceEnum::xplus;
      }
      // x-
      d = fabs((center.x() - resolution/2.0) - intersection.x());
      if( d < dmin )
      {
        dmin = d;
        fe = FaceEnum::xminus;
      }
      // y+
      d = fabs((center.y() + resolution/2.0) - intersection.y());
      if( d < dmin )
      {
        dmin = d;
        fe = FaceEnum::yplus;
      }
      // y-
      d = fabs((center.y() - resolution/2.0) - intersection.y());
      if( d < dmin )
      {
        dmin = d;
        fe = FaceEnum::yminus;
      }
      // z+
      d = fabs((center.z() + resolution/2.0) - intersection.z());
      if( d < dmin )
      {
        dmin = d;
        fe = FaceEnum::zplus;
      }
      // z-
      d = fabs((center.z() - resolution/2.0) - intersection.z());
      if( d < dmin )
      {
        dmin = d;
        fe = FaceEnum::zminus;
      }
      
      integrateFaceObservation(key,fe,intensity);
    }
  }

  void TextureOcTree::insertPointCloud(const Pointcloud& scan, 
                        const std::vector<unsigned char>& intensities,
                        const octomap::point3d& sensor_origin,
                        const octomath::Vector3& sensor_orientation, 
                        double maxrange, bool lazy_eval, bool discretize) {

    // First update occupancies
    OccupancyOcTreeStereo::insertPointCloud(scan,sensor_origin,sensor_orientation,maxrange,lazy_eval,discretize);
    // For each point, compute the face that it intersects for the voxel it resides in
    // and update that face's texture from the intensity at that point

    // First, verify that we have the same number of intensities as points
    if (scan.size() != intensities.size())
    {
      std::cerr << "Could not insert the point cloud: " << scan.size() << " points and " << intensities.size() << " intensities." << std::endl;
      return;
    }
    else if(scan.size() < 1)
      return;

    // Next process each point
#ifdef _OPENMP
    omp_set_num_threads(this->keyrays.size());
    #pragma omp parallel for
#endif
    for (int i = 0; i < (int)scan.size(); ++i) {
      const point3d& p = scan[i];
      insertTexturePoint(p,intensities.at(i),sensor_origin);
    }
  }
    

  std::ostream& operator<<(std::ostream& out, Face const& f) {
    return out << '(' << (unsigned int)f.value << ' ' << (unsigned int)f.nObs << ')';
  }

  std::ostream& operator<<(std::ostream& out, TextureOcTreeNode* n) {
    return out << "+x : " << n->getFace(FaceEnum::xplus) << std::endl
               << "-x : " << n->getFace(FaceEnum::xminus) << std::endl 
               << "+y : " << n->getFace(FaceEnum::yplus) << std::endl
               << "-y : " << n->getFace(FaceEnum::yminus) << std::endl
               << "+z : " << n->getFace(FaceEnum::zplus) << std::endl
               << "-z : " << n->getFace(FaceEnum::zminus) << std::endl;
  }

  TextureOcTree::StaticMemberInitializer TextureOcTree::textureOcTreeMemberInit;
} // end namespace

