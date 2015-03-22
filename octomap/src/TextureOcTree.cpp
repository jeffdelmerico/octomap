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

  /*
  ColorOcTreeNode::Color ColorOcTreeNode::getAverageChildColor() const {
    int mr(0), mg(0), mb(0);
    int c(0);
    for (int i=0; i<8; i++) {
      if (childExists(i) && getChild(i)->isColorSet()) {
        mr += getChild(i)->getColor().r;
        mg += getChild(i)->getColor().g;
        mb += getChild(i)->getColor().b;
        ++c;
      }
    }
    if (c) {
      mr /= c;
      mg /= c;
      mb /= c;
      return Color((unsigned char) mr, (unsigned char) mg, (unsigned char) mb);
    }
    else { // no child had a color other than white
      return Color(255, 255, 255);
    }
  }
  */

  void TextureOcTreeNode::updateTextureFromChildren() {      
    for (auto it = faces.begin(); it != faces.end(); it++)
      if (it->nObs > 0) *it = getAverageChildTexture();
  }
  

  // pruning =============

  bool TextureOcTreeNode::pruneNode() {
    // checks for equal occupancy only, texture ignored
    if (!this->collapsible()) return false;
    // set occupancy value 
    setLogOdds(getChild(0)->getLogOdds());
    // set face textures to average child face textures 
    updateTextureFromChildren();
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
      for (auto i :W 
        getChild(k)->setFace(i,faces.at(i));
    }
  }


  /*
  // tree implementation  --------------------------------------

  ColorOcTreeNode* ColorOcTree::setNodeColor(const OcTreeKey& key, 
                                             const unsigned char& r, 
                                             const unsigned char& g, 
                                             const unsigned char& b) {
    ColorOcTreeNode* n = search (key);
    if (n != 0) {
      n->setColor(r, g, b); 
    }
    return n;
  }

  ColorOcTreeNode* ColorOcTree::averageNodeColor(const OcTreeKey& key, 
                                                 const unsigned char& r, 
                                                 const unsigned char& g, 
                                                 const unsigned char& b) {
    ColorOcTreeNode* n = search (key);
    if (n != 0) {
      if (n->isColorSet()) {
        ColorOcTreeNode::Color prev_color = n->getColor();
        n->setColor((prev_color.r + r)/2, (prev_color.g + g)/2, (prev_color.b + b)/2); 
      }
      else {
        n->setColor(r, g, b);
      }
    }
    return n;
  }

  ColorOcTreeNode* ColorOcTree::integrateNodeColor(const OcTreeKey& key, 
                                                   const unsigned char& r, 
                                                   const unsigned char& g, 
                                                   const unsigned char& b) {
    ColorOcTreeNode* n = search (key);
    if (n != 0) {
      if (n->isColorSet()) {
        ColorOcTreeNode::Color prev_color = n->getColor();
        double node_prob = n->getOccupancy();
        unsigned char new_r = (unsigned char) ((double) prev_color.r * node_prob 
                                               +  (double) r * (0.99-node_prob));
        unsigned char new_g = (unsigned char) ((double) prev_color.g * node_prob 
                                               +  (double) g * (0.99-node_prob));
        unsigned char new_b = (unsigned char) ((double) prev_color.b * node_prob 
                                               +  (double) b * (0.99-node_prob));
        n->setColor(new_r, new_g, new_b); 
      }
      else {
        n->setColor(r, g, b);
      }
    }
    return n;
  }
  
  
  void ColorOcTree::updateInnerOccupancy() {
    this->updateInnerOccupancyRecurs(this->root, 0);
  }

  void ColorOcTree::updateInnerOccupancyRecurs(ColorOcTreeNode* node, unsigned int depth) {
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
      node->updateColorChildren();
    }
  }

  void ColorOcTree::writeColorHistogram(std::string filename) {

#ifdef _MSC_VER
    fprintf(stderr, "The color histogram uses gnuplot, this is not supported under windows.\n");
#else
    // build RGB histogram
    std::vector<int> histogram_r (256,0);
    std::vector<int> histogram_g (256,0);
    std::vector<int> histogram_b (256,0);
    for(ColorOcTree::tree_iterator it = this->begin_tree(),
          end=this->end_tree(); it!= end; ++it) {
      if (!it.isLeaf() || !this->isNodeOccupied(*it)) continue;
      ColorOcTreeNode::Color& c = it->getColor();
      ++histogram_r[c.r];
      ++histogram_g[c.g];
      ++histogram_b[c.b];
    }
    // plot data
    FILE *gui = popen("gnuplot ", "w");
    fprintf(gui, "set term postscript eps enhanced color\n");
    fprintf(gui, "set output \"%s\"\n", filename.c_str());
    fprintf(gui, "plot [-1:256] ");
    fprintf(gui,"'-' w filledcurve lt 1 lc 1 tit \"r\",");
    fprintf(gui, "'-' w filledcurve lt 1 lc 2 tit \"g\",");
    fprintf(gui, "'-' w filledcurve lt 1 lc 3 tit \"b\",");
    fprintf(gui, "'-' w l lt 1 lc 1 tit \"\",");
    fprintf(gui, "'-' w l lt 1 lc 2 tit \"\",");
    fprintf(gui, "'-' w l lt 1 lc 3 tit \"\"\n");

    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_r[i]);    
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_g[i]);    
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_b[i]);    
    fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_r[i]);    
    fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_g[i]);    
    fprintf(gui, "e\n");
    for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_b[i]);    
    fprintf(gui, "e\n");
    fflush(gui);
#endif
  }

  std::ostream& operator<<(std::ostream& out, ColorOcTreeNode::Color const& c) {
    return out << '(' << (unsigned int)c.r << ' ' << (unsigned int)c.g << ' ' << (unsigned int)c.b << ')';
  }


  ColorOcTree::StaticMemberInitializer ColorOcTree::colorOcTreeMemberInit;
  */
} // end namespace

