
#include <stdio.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

using namespace std;
using namespace octomap;



int main(int argc, char** argv) {


  //##############################################################     

  bool use_stereo = true;
  OcTree tree (0.1);
  tree.setOccupancyThres(0.55);
  tree.setStereoSensorModel(use_stereo);
  tree.setStereoErrorCoeff(8.887); // = ( resolution * stddev_disp) / (2*sqrt(2) * b * f)

  cout << "Using stereo sensor model? " << (tree.getStereoSensorModel() ? "true" : "false" ) << endl;
  cout << "Stereo error coefficient: " << tree.getStereoErrorCoeff() << endl;

  point3d origin (0.01f, 0.01f, 0.02f);
  point3d point_on_surface (4.01f, 0.01f, 0.01f);

  /*
  cout << "generating spherical scan at " << origin << " ..." << endl;

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

  for (int i=-100; i<101; i++) {
    for (int j=-100; j<101; j++) {
      point3d p(1.5f,0.03*i,0.03*j);
      cloud.push_back(p);
    }
  }


  // insert in global coordinates:
  if (use_stereo)
    tree.insertPointCloudStereo(cloud, origin);
  else
    tree.insertPointCloud(cloud, origin);

  cout << "done." << endl;
  /*
  cout << "writing to spherical_scan.bt..." << endl;
  tree.writeBinary("spherical_scan.bt");
  */

  cout << "writing to planar_scan.bt..." << endl;
  tree.writeBinary("planar_scan.bt");


}
