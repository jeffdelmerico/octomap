#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

using namespace std;
using namespace octomap;


int main(int argc, char** argv) {

  //##############################################################     

  TextureOcTree tree (0.1, 10.0, 15.0);
  float x(1.0f), y(1.0f), z(1.0f);
  float d(1.0f);
  //point3d origin (0.0f, 0.0f, 0.0f);
  
  // Add an occupancy observation
  TextureOcTreeNode* n = tree.updateNode(x,y,z, true, d); 
  // Print texture from node pointer
  cout << n << endl;

  // Set some textures for the faces of that voxel
  Face f1(127, 1, FaceEnum::xplus);
  Face f2(255, 2, FaceEnum::yplus);
  tree.setNodeTexture(x,y,z,FaceEnum::xplus,f1);
  tree.setNodeTexture(x,y,z,FaceEnum::yplus,f2);
  // Print texture from tree lookup
  tree.printNodeTexture(x,y,z);

  // Integrate some new texture observations
  tree.integrateFaceObservation(x,y,z,FaceEnum::xplus,200);
  tree.integrateFaceObservation(x,y,z,FaceEnum::yplus,100);
  tree.integrateFaceObservation(x,y,z,FaceEnum::zplus,200);
  tree.integrateFaceObservation(x,y,z,FaceEnum::zplus,100);
  tree.printNodeTexture(x,y,z);

  // Write histogram plot
  tree.writeTextureHistogram("texture_histogram.eps");

  /*
  cout << "generating spherical scan at " << origin << " ..." << endl;
  point3d point_on_surface (4.01f, 0.01f, 0.01f);

  Pointcloud cloud;

  for (int i=-100; i<101; i++) {
    for (int j=-100; j<101; j++) {
      point3d rotated = point_on_surface;
      rotated.rotate_IP(0, DEG2RAD(i*0.5), DEG2RAD(j*0.5));
      cloud.push_back(rotated);
    }
  } 
  */

  /*
  cout << "generating planar scan at " << origin << " ..." << endl;

  Pointcloud cloud;

  for (int i=-100; i<101; i++) {
    for (int j=-100; j<101; j++) {
      point3d p(3.0f + 0.03*j,0.03*i,-1.0);
      cloud.push_back(p);
    }
  }
  */

  /*
  octomath::Vector3 orientation(1.0, 2.0, 0.0);

  tree.insertPointCloud(cloud, origin, orientation);

  cout << "done." << endl;
  cout << "writing to spherical_scan.bt..." << endl;
  tree.writeBinary("spherical_scan.bt");
  */

  /*
  cout << "writing to planar_scan.bt..." << endl;
  tree.writeBinary("planar_scan.bt");
  */

}
