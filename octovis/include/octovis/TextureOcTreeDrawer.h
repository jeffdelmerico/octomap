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

#ifndef OCTOVIS_TEXTURE_OCTREEDRAWER_H_
#define OCTOVIS_TEXTURE_OCTREEDRAWER_H_

#include <octovis/OcTreeDrawer.h>
#include <octomap/TextureOcTree.h>

namespace octomap {

  class TextureOcTreeDrawer : public OcTreeDrawer {
  public:
    TextureOcTreeDrawer();
    virtual ~TextureOcTreeDrawer();

    virtual void setOcTree(const AbstractOcTree& tree_pnt, const pose6d& origin, int map_id_);

  protected:
    /*
    //! setup OpenGL arrays
    void initGLArrays(const unsigned int& num_cubes, unsigned int& glArraySize,
                       GLfloat*** glArray, GLfloat** glColorArray);

    //! clear OpenGL visualization
    void clearCubes(GLfloat*** glArray, unsigned int& glArraySize,
                    GLfloat** glColorArray = NULL);
    */

    virtual unsigned int setCubeTexture(const unsigned char& r,
                                        const unsigned char& g,
                                        const unsigned char& b,
                                        const unsigned char& a,
                                        const unsigned int& current_array_idx,
                                        GLfloat** glColorArray);

    /*
    void drawCubes(GLfloat** cubeArray, unsigned int cubeArraySize,
        GLfloat** cubeColorArray = NULL) const;
    */

    };

} // end namespace

#endif
