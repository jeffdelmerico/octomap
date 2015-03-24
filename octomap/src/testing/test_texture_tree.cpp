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

  cout << "generating spherical scan at " << origin << " ..." << endl;
  point3d point_on_surface (4.01f, 0.01f, 0.01f);

  Pointcloud cloud;
  vector<unsigned char> intensities;

  for (int i=-100; i<101; i++) {
    for (int j=-100; j<101; j++) {
      point3d rotated = point_on_surface;
      rotated.rotate_IP(0, DEG2RAD(i*0.5), DEG2RAD(j*0.5));
      cloud.push_back(rotated);
      unsigned char intensity = 100 + floor((i+j)/2.0f);
      intensities.push_back(intensity);
    }
  } 

  cout << "generating planar scan at " << origin << " ..." << endl;

  /*
  Pointcloud cloud;
  vector<unsigned char> intensities;

  for (int i=-100; i<101; i++) {
    for (int j=-100; j<101; j++) {
      point3d p(3.0, 1e-4+0.03*j, 1e-4+0.03*i);
      cloud.push_back(p);
      unsigned char intensity = 100 + floor((i+j)/2.0f);
      intensities.push_back(intensity);
    }
  }
  */

  octomath::Vector3 orientation(1.0, 0.0, 0.0);

  // Set up timer
  timeval start;
  timeval stop;
  double texture_insert_time = 0.0;
  gettimeofday(&start, NULL);  // start timer
  tree.insertPointCloud(cloud, intensities, origin, orientation);
  tree.toMaxLikelihood();
  tree.prune();
  gettimeofday(&stop, NULL);  // stop timer
  texture_insert_time = timediff(start, stop);
  std::cout << "[TextureOcTree:] Inserting " << cloud.size() << " points took " << texture_insert_time << " seconds \n========================\n\n";

  // Write histogram plot
  tree.writeTextureHistogram("texture_histogram.eps");

  // Query a few more textures
  /*
  tree.printNodeTexture(1.0,1.0,1.0);
  tree.printNodeTexture(1.0,-1.0,1.0);
  tree.printNodeTexture(1.0,1.0,-1.0);
  tree.printNodeTexture(1.0,-1.0,-1.0);
  tree.printNodeTexture(-1.0,-1.0,-1.0);
  tree.printNodeTexture(1.0,3.0,3.0);
  tree.printNodeTexture(1.0,-3.0,-3.0);
  tree.printNodeTexture(1.0,0.05,0.05);
  tree.printNodeTexture(1.0,-0.05,-0.05);
  */

  /*
  cout << "done." << endl;
  cout << "writing to spherical_scan.bt..." << endl;
  tree.writeBinary("spherical_scan.bt");
  */

  cout << "writing to texture_scan.bt..." << endl;
  tree.writeBinary("texture_scan.bt");

  // Test timing for stereo OcTree
  OcTreeStereo stereoTree (0.1, 10.0, 15.0);
  double stereo_insert_time = 0.0;
  gettimeofday(&start, NULL);  // start timer
  stereoTree.insertPointCloud(cloud, origin, orientation);
  stereoTree.toMaxLikelihood();
  stereoTree.prune();
  gettimeofday(&stop, NULL);  // stop timer
  stereo_insert_time = timediff(start, stop);
  std::cout << "[OcTreeStereo:] Inserting " << cloud.size() << " points took " << stereo_insert_time << " seconds \n========================\n\n";
  //cout << "writing to stereo_scan.bt..." << endl;
  //stereoTree.writeBinary("stereo_scan.bt");

  // Test timing for regular OcTree
  OcTree regularTree(0.1);
  double regular_insert_time = 0.0;
  gettimeofday(&start, NULL);  // start timer
  regularTree.insertPointCloud(cloud, origin);
  regularTree.toMaxLikelihood();
  regularTree.prune();
  gettimeofday(&stop, NULL);  // stop timer
  regular_insert_time = timediff(start, stop);
  std::cout << "[OcTree:] Inserting " << cloud.size() << " points took " << regular_insert_time << " seconds \n========================\n\n";
  //cout << "writing to regular_scan.bt..." << endl;
  //regularTree.writeBinary("regular_scan.bt");

  // Test accesses - iterate over all leaf nodes and check for occupancy (and texture)
  double texture_access_time = 0.0;
  unsigned int occupied_count = 0;
  unsigned int total_count = 0;
  double jnk = 0.0;
  unsigned char jnktxt = 0;
  gettimeofday(&start, NULL);  // start timer
  for(TextureOcTree::iterator it = tree.begin(tree.getTreeDepth()); it!= tree.end(); ++it){
    total_count++;
    if(tree.isNodeOccupied(*it))
    {
      jnk = it->getOccupancy();
      occupied_count++;
      for(int face = 0; face < 6; face++)
        jnktxt = it->getFaceValue((FaceEnum) face);
    }
  }
  gettimeofday(&stop, NULL);  // stop timer
  texture_access_time = timediff(start, stop);
  std::cout << "[TextureOcTree:] querying " << total_count << " voxels (" << occupied_count << " occupied) took " << texture_access_time << " seconds \n========================\n\n";

  double stereo_access_time = 0.0;
  occupied_count = 0;
  total_count = 0;
  gettimeofday(&start, NULL);  // start timer
  for(OcTreeStereo::iterator it = stereoTree.begin(stereoTree.getTreeDepth()); it!= stereoTree.end(); ++it){
    total_count++;
    if(stereoTree.isNodeOccupied(*it))
    {
      jnk = it->getOccupancy();
      occupied_count++;
    }
  }
  gettimeofday(&stop, NULL);  // stop timer
  stereo_access_time = timediff(start, stop);
  std::cout << "[OcTreeStereo:] querying " << total_count << " voxels (" << occupied_count << " occupied) took " << stereo_access_time << " seconds \n========================\n\n";

  double regular_access_time = 0.0;
  occupied_count = 0;
  total_count = 0;
  gettimeofday(&start, NULL);  // start timer
  for(OcTree::iterator it = regularTree.begin(regularTree.getTreeDepth()); it!= regularTree.end(); ++it){
    total_count++;
    if(regularTree.isNodeOccupied(*it))
    {
      jnk = it->getOccupancy();
      occupied_count++;
    }
  }
  gettimeofday(&stop, NULL);  // stop timer
  regular_access_time = timediff(start, stop);
  std::cout << "[OcTree:] querying " << total_count << " voxels (" << occupied_count << " occupied) took " << regular_access_time << " seconds \n========================\n\n";

}
