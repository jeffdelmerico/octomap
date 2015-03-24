/*
 * This file is part of OctoMap - An Efficient Probabilistic 3D Mapping
 * Framework Based on Octrees
 * http://octomap.github.io
 *
 * Copyright (c) 2009-2014, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved. License for the viewer octovis: GNU GPL v2
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <octovis/TextureOcTreeDrawer.h>

namespace octomap {

  TextureOcTreeDrawer::TextureOcTreeDrawer() 
    : OcTreeDrawer() {
  }

  TextureOcTreeDrawer::~TextureOcTreeDrawer() {
  }

  void TextureOcTreeDrawer::setOcTree(const AbstractOcTree& tree_pnt,
                                    const octomap::pose6d& origin_,
                                    int map_id_) {

    const TextureOcTree& tree = ((const TextureOcTree&) tree_pnt);

    this->map_id = map_id_;

    // save origin used during cube generation
    this->initial_origin = octomap::pose6d(octomap::point3d(0,0,0), origin_.rot());
    // origin is in global coords
    this->origin = origin_;
    
    // maximum size to prevent crashes on large maps: (should be checked in a better way than a constant)
    bool showAll = (tree.size() < 5 * 1e6);
    bool uses_origin = ( (origin_.rot().x() != 0.) && (origin_.rot().y() != 0.)
                         && (origin_.rot().z() != 0.) && (origin_.rot().u() != 1.) );

    // walk the tree one to find the number of nodes in each category
    // (this is used to set up the OpenGL arrays)
    // TODO: this step may be left out, if we maintained the GLArrays in std::vectors instead...
    unsigned int cnt_occupied(0);
    unsigned int cnt_free(0);
    for(TextureOcTree::tree_iterator it = tree.begin_tree(this->m_max_tree_depth),
          end=tree.end_tree(); it!= end; ++it) {
      if (it.isLeaf()) { 
        if (tree.isNodeOccupied(*it)){ // occupied voxels
          ++cnt_occupied;
        }
        else if (showAll) { // freespace voxels
          ++cnt_free;
        }
      }        
    }    
    // setup GL arrays for cube quads and cube colors
    initGLArrays(cnt_occupied      , m_occupiedSize     , &m_occupiedArray    , &m_occupiedColorArray);
    initGLArrays(cnt_free          , m_freeSize         , &m_freeArray        , NULL);

    std::vector<octomath::Vector3> cube_template;
    initCubeTemplate(origin, cube_template);

    unsigned int idx_occupied(0);
    unsigned int idx_free(0);
    unsigned int texture_idx_occupied(0);

    m_grid_voxels.clear();
    OcTreeVolume voxel; // current voxel, possibly transformed 
    for(TextureOcTree::tree_iterator it = tree.begin_tree(this->m_max_tree_depth),
          end=tree.end_tree(); it!= end; ++it) {

      if (it.isLeaf()) { // voxels for leaf nodes
        if (uses_origin) 
          voxel = OcTreeVolume(origin.rot().rotate(it.getCoordinate()), it.getSize());
        else 
          voxel = OcTreeVolume(it.getCoordinate(), it.getSize());
        
        if (tree.isNodeOccupied(*it)){ // occupied voxels
          unsigned int intensity_sum = 0;
          unsigned int obs = 0;
          for (auto i=0; i<6; i++) {
            intensity_sum += (it->getFaceValue((FaceEnum) i) * it-> getFaceObservations((FaceEnum) i));
            obs += it->getFaceObservations((FaceEnum) i);
          }
          unsigned char intensity = floor((float) intensity_sum/(float) obs);
          idx_occupied = generateCube(voxel, cube_template, idx_occupied, &m_occupiedArray);
          texture_idx_occupied = setCubeTexture (intensity, intensity, intensity, 
                                                (unsigned char)(it->getOccupancy() * 255.),
                                                texture_idx_occupied, &m_occupiedColorArray);
        }
        else if (showAll) { // freespace voxels
          idx_free = generateCube(voxel, cube_template, idx_free, &m_freeArray);
        }

        // grid structure voxel
        if (showAll) m_grid_voxels.push_back(voxel);        
      }
      
      else { // inner node voxels (for grid structure only)
        if (showAll) {
          if (uses_origin)
            voxel = OcTreeVolume(origin.rot().rotate(it.getCoordinate()), it.getSize());
          else
            voxel = OcTreeVolume(it.getCoordinate(), it.getSize());
          m_grid_voxels.push_back(voxel);
        }
      }      
    } // end for all voxels

    m_octree_grid_vis_initialized = false;

    if(m_drawOcTreeGrid)
      initOctreeGridVis();    
  }

  /*
  void TextureOcTreeDrawer::initGLArrays(const unsigned int& num_cubes,
                                  unsigned int& glArraySize,
                                  GLfloat*** glArray, GLfloat** glColorArray) {

    clearCubes(glArray, glArraySize, glColorArray);

    // store size of GL arrays for drawing
    glArraySize = num_cubes * 4 * 3;

    // allocate cube arrays, 6 quads per cube
    *glArray = new GLfloat* [6];
    for (unsigned i = 0; i<6; ++i){
      (*glArray)[i] = new GLfloat[glArraySize];
    }
    // setup quad color array, if given
    if (glColorArray != NULL)
    {
      glColorArray = new GLfloat* [6];
      for (unsigned i = 0; i<6; i++){
        glColorArray[i] = new GLfloat[glArraySize * 4 *4];
      }
    }
  }

  void TextureOcTreeDrawer::clearCubes(GLfloat*** glArray,
                                unsigned int& glArraySize,
                                GLfloat** glColorArray) {

    if (glArraySize != 0) {
      for (unsigned i = 0; i < 6; ++i) {
        delete[] (*glArray)[i];
      }
      delete[] *glArray;
      *glArray = NULL;
      glArraySize = 0;
    }
    if (glColorArray != NULL && *glColorArray != NULL && glArraySize != 0) {
      for (unsigned i = 0; i < 6; ++i) {
        delete[] glColorArray[i];
      }
      delete[] glColorArray;
    }
  }
  */

  unsigned int TextureOcTreeDrawer::setCubeTexture(const unsigned char& r,
                                                   const unsigned char& g,
                                                   const unsigned char& b,
                                                   const unsigned char& a,
                                                   const unsigned int& current_array_idx,
                                                   GLfloat** glColorArray) {

    if (glColorArray == NULL) return current_array_idx;
    unsigned int colorIdx = current_array_idx;
    // set color for next 4 vertices (=one quad)
    for (int k = 0; k < 4; ++k) {
      (*glColorArray)[colorIdx    ] = (double) r/255.;
      (*glColorArray)[colorIdx + 1] = (double) g/255.;
      (*glColorArray)[colorIdx + 2] = (double) b/255.;
      (*glColorArray)[colorIdx + 3] = (double) a/255.;
      colorIdx += 4;
    }  
    /*
    for (int face = 0; face < 6; ++face)
    {
      for (int k = 0; k < 4; ++k) {
        glColorArray[face][colorIdx    ] = (double) r/255.;
        glColorArray[face][colorIdx + 1] = (double) g/255.;
        glColorArray[face][colorIdx + 2] = (double) b/255.;
        glColorArray[face][colorIdx + 3] = (double) a/255.;
        colorIdx += 4;
      }  
    }
    */
    return colorIdx;
  }
  
  /*
  void TextureOcTreeDrawer::drawCubes(GLfloat** cubeArray, unsigned int cubeArraySize,
                               GLfloat** cubeColorArray) const {
    if (cubeArraySize == 0 || cubeArray == NULL){
      std::cerr << "Warning: GLfloat array to draw cubes appears to be empty, nothing drawn.\n";
      return;
    }

    // save current color
    GLfloat* curcol = new GLfloat[4];
    glGetFloatv(GL_CURRENT_COLOR, curcol);

    // enable color pointer when heightColorMode is enabled:

    if ((m_colorMode == CM_COLOR_HEIGHT || m_colorMode == CM_GRAY_HEIGHT) && (cubeColorArray != NULL)){
      glEnableClientState(GL_COLOR_ARRAY);
      // top surfaces:
      glNormal3f(0.0f, 1.0f, 0.0f);
      glVertexPointer(3, GL_FLOAT, 0, cubeArray[0]);
      glColorPointer(4, GL_FLOAT, 0, cubeColorArray[0]);
      glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
      // bottom surfaces:
      glNormal3f(0.0f, -1.0f, 0.0f);
      glVertexPointer(3, GL_FLOAT, 0, cubeArray[1]);
      glColorPointer(4, GL_FLOAT, 0, cubeColorArray[1]);
      glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
      // right surfaces:
      glNormal3f(1.0f, 0.0f, 0.0f);
      glVertexPointer(3, GL_FLOAT, 0, cubeArray[2]);
      glColorPointer(4, GL_FLOAT, 0, cubeColorArray[2]);
      glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
      // left surfaces:
      glNormal3f(-1.0f, 0.0f, 0.0f);
      glVertexPointer(3, GL_FLOAT, 0, cubeArray[3]);
      glColorPointer(4, GL_FLOAT, 0, cubeColorArray[3]);
      glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
      // back surfaces:
      glNormal3f(0.0f, 0.0f, -1.0f);
      glVertexPointer(3, GL_FLOAT, 0, cubeArray[4]);
      glColorPointer(4, GL_FLOAT, 0, cubeColorArray[4]);
      glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
      // front surfaces:
      glNormal3f(0.0f, 0.0f, 1.0f);
      glVertexPointer(3, GL_FLOAT, 0, cubeArray[5]);
      glColorPointer(4, GL_FLOAT, 0, cubeColorArray[5]);
      glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
    }
    else {
      // top surfaces:
      glNormal3f(0.0f, 1.0f, 0.0f);
      glVertexPointer(3, GL_FLOAT, 0, cubeArray[0]);
      glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
      // bottom surfaces:
      glNormal3f(0.0f, -1.0f, 0.0f);
      glVertexPointer(3, GL_FLOAT, 0, cubeArray[1]);
      glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
      // right surfaces:
      glNormal3f(1.0f, 0.0f, 0.0f);
      glVertexPointer(3, GL_FLOAT, 0, cubeArray[2]);
      glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
      // left surfaces:
      glNormal3f(-1.0f, 0.0f, 0.0f);
      glVertexPointer(3, GL_FLOAT, 0, cubeArray[3]);
      glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
      // back surfaces:
      glNormal3f(0.0f, 0.0f, -1.0f);
      glVertexPointer(3, GL_FLOAT, 0, cubeArray[4]);
      glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
      // front surfaces:
      glNormal3f(0.0f, 0.0f, 1.0f);
      glVertexPointer(3, GL_FLOAT, 0, cubeArray[5]);
      glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
    }

    if ((m_colorMode == CM_COLOR_HEIGHT || m_colorMode == CM_GRAY_HEIGHT)
        && (cubeColorArray != NULL)){
      glDisableClientState(GL_COLOR_ARRAY);
    }

    // draw bounding linies of cubes in printout:
    if (m_colorMode == CM_PRINTOUT){
      glDisable(GL_LIGHTING);
      glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);
      glEnable (GL_LINE_SMOOTH);
      glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);   // Draw Polygons only as Wireframes
      glLineWidth(2.0f);
      glColor3f(0.0f, 0.0f, 0.0f);
      glCullFace(GL_FRONT_AND_BACK);        // Don't draw any Polygons faces
      //glDepthFunc (GL_LEQUAL);

      // top meshes:
      glNormal3f(0.0f, 1.0f, 0.0f);
      glVertexPointer(3, GL_FLOAT, 0, cubeArray[0]);
      glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
      // bottom meshes:
      glNormal3f(0.0f, -1.0f, 0.0f);
      glVertexPointer(3, GL_FLOAT, 0, cubeArray[1]);
      glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
      // right meshes:
      glNormal3f(1.0f, 0.0f, 0.0f);
      glVertexPointer(3, GL_FLOAT, 0, cubeArray[2]);
      glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
      // left meshes:
      glNormal3f(-1.0f, 0.0f, 0.0f);
      glVertexPointer(3, GL_FLOAT, 0, cubeArray[3]);
      glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);

      // restore defaults:
      glCullFace(GL_BACK);
      //glDepthFunc(GL_LESS);
      glDisable(GL_LINE_SMOOTH);
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      glEnable(GL_LIGHTING);
    }
    // reset color
    glColor4fv(curcol);
    delete[] curcol;
  }
  */

} // end namespace
