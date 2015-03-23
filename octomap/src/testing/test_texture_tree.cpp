#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include <octomap/octomap_timing.h>

using namespace std;
using namespace octomap;


double timediff(const timeval& start, const timeval& stop){
  return (stop.tv_sec - start.tv_sec) + 1.0e-6 *(stop.tv_usec - start.tv_usec);
}

int main(int argc, char** argv) {

  //##############################################################     

  TextureOcTree tree (0.1, 10.0, 15.0);
  float x(1.0f), y(1.0f), z(1.0f);
  float d(1.0f);
  point3d origin (-1.0f, 0.0f, 0.0f);
  
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

  cout << "generating planar scan at " << origin << " ..." << endl;

  Pointcloud cloud;
  vector<unsigned char> intensities;

  for (int i=-100; i<101; i++) {
    for (int j=-100; j<101; j++) {
      point3d p(1.0, 1e-4+0.03*j, 1e-4+0.03*i);
      cloud.push_back(p);
      unsigned char intensity = 100 + floor((i+j)/2.0f);
      intensities.push_back(intensity);
    }
  }

  octomath::Vector3 orientation(1.0, 0.0, 0.0);

  // Set up timer
  timeval start;
  timeval stop;
  double insert_time = 0.0;
  gettimeofday(&start, NULL);  // start timer
  tree.insertPointCloud(cloud, intensities, origin, orientation);
  gettimeofday(&stop, NULL);  // stop timer
  insert_time = timediff(start, stop);
  std::cout << "Inserting " << cloud.size() << " points took " << insert_time << " seconds \n========================\n\n";

  // Query a few more textures
  tree.printNodeTexture(1.0,1.0,1.0);
  tree.printNodeTexture(1.0,-1.0,1.0);
  tree.printNodeTexture(1.0,1.0,-1.0);
  tree.printNodeTexture(1.0,-1.0,-1.0);
  tree.printNodeTexture(-1.0,-1.0,-1.0);
  tree.printNodeTexture(1.0,3.0,3.0);
  tree.printNodeTexture(1.0,-3.0,-3.0);
  tree.printNodeTexture(1.0,0.05,0.05);
  tree.printNodeTexture(1.0,-0.05,-0.05);

  /*
  cout << "done." << endl;
  cout << "writing to spherical_scan.bt..." << endl;
  tree.writeBinary("spherical_scan.bt");
  */

  cout << "writing to planar_scan.bt..." << endl;
  tree.writeBinary("planar_scan.bt");

}
