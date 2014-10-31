SET (octomap_stereo_SRCS
  AbstractOcTree.cpp
  AbstractOccupancyOcTree.cpp
  Pointcloud.cpp
  ScanGraph.cpp
  CountingOcTree.cpp
  OcTree.cpp
  OcTreeNode.cpp
  OcTreeStamped.cpp
  OcTreeLUT.cpp
  ColorOcTree.cpp
  )

# dynamic and static libs, see CMake FAQ:
ADD_LIBRARY( octomap_stereo SHARED ${octomap_stereo_SRCS})
set_target_properties( octomap_stereo PROPERTIES
  VERSION ${OCTOMAP_STEREO_VERSION}
  SOVERSION ${OCTOMAP_STEREO_SOVERSION}
)
ADD_LIBRARY( octomap_stereo-static STATIC ${octomap_stereo_SRCS})
SET_TARGET_PROPERTIES(octomap_stereo-static PROPERTIES OUTPUT_NAME "octomap_stereo") 

TARGET_LINK_LIBRARIES(octomap_stereo octomath_stereo)

#ADD_SUBDIRECTORY( testing )
#
#ADD_EXECUTABLE(graph2tree graph2tree.cpp)
#TARGET_LINK_LIBRARIES(graph2tree octomap_stereo)
#
#ADD_EXECUTABLE(log2graph log2graph.cpp)
#TARGET_LINK_LIBRARIES(log2graph octomap_stereo)
#
#ADD_EXECUTABLE(binvox2bt binvox2bt.cpp)
#TARGET_LINK_LIBRARIES(binvox2bt octomap_stereo)
#
#ADD_EXECUTABLE(bt2vrml bt2vrml.cpp)
#TARGET_LINK_LIBRARIES(bt2vrml octomap_stereo)
#
#ADD_EXECUTABLE(edit_octree edit_octree.cpp)
#TARGET_LINK_LIBRARIES(edit_octree octomap_stereo)
#
#ADD_EXECUTABLE(convert_octree convert_octree.cpp)
#TARGET_LINK_LIBRARIES(convert_octree octomap_stereo)
#
#ADD_EXECUTABLE(eval_octree_accuracy eval_octree_accuracy.cpp)
#TARGET_LINK_LIBRARIES(eval_octree_accuracy octomap_stereo)
#
#ADD_EXECUTABLE(compare_octrees compare_octrees.cpp)
#TARGET_LINK_LIBRARIES(compare_octrees octomap_stereo)
#
#ADD_EXECUTABLE(simple_example simple_example.cpp)
#TARGET_LINK_LIBRARIES(simple_example octomap_stereo)
#
#ADD_EXECUTABLE(normals_example normals_example.cpp)
#TARGET_LINK_LIBRARIES(normals_example octomap_stereo)
#
#ADD_EXECUTABLE(intersection_example intersection_example.cpp)
#TARGET_LINK_LIBRARIES(intersection_example octomap_stereo)


# installation:
# store all header files to install:
# file(GLOB octomap_stereo_impl_HDRS *.h *.hxx *.hpp)
# install(FILES ${octomap_stereo_impl_HDRS} DESTINATION include/octomap_stereo)

install(TARGETS 
	octomap_stereo
	octomap_stereo-static 
	#graph2tree
	#log2graph
	#binvox2bt
	#bt2vrml
	#edit_octree
	#convert_octree
	#eval_octree_accuracy
	#compare_octrees
	${INSTALL_TARGETS_DEFAULT_ARGS}
)

