#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

using namespace std;
using namespace octomap;


int main(int argc, char** argv) {

  //##############################################################     

  TextureOcTree tree (0.1, 10.0, 15.0);
  point3d origin (0.0f, 0.0f, 0.0f);
  Face f1(127, 1);
  Face f2(255, 2);
  tree.setNodeTexture(1.0,1.0,1.0,FaceEnum::xplus,f1);
  tree.setNodeTexture(1.0,1.0,1.0,FaceEnum::yplus,f2);


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
