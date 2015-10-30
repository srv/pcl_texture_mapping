#include <pcl_texture_mapping/texture_mapper.h>

using namespace pcl;
using namespace std;
namespace fs = boost::filesystem;

TextureMapper::TextureMapper(ros::NodeHandle nh, ros::NodeHandle nhp)
{
  string cloud_filename, work_dir;
  nhp.param("cloud_filename", cloud_filename, string(""));
  nhp.param("work_dir", work_dir, string(""));
  nhp.param("focal_length", focal_length_, 0.0);
  nhp.param("image_height", image_height_, 768);
  nhp.param("image_width", image_width_, 1024);

  process(cloud_filename, work_dir);

  std::cout << "Finished!" << std::endl;
}

PointCloudRGB::Ptr TextureMapper::filterCloud(PointCloudRGB::Ptr in_cloud, float voxel_size)
{
  // Remove nans
  vector<int> indices;
  PointCloudRGB::Ptr cloud(new PointCloudRGB);
  removeNaNFromPointCloud(*in_cloud, *cloud, indices);
  indices.clear();

  // Voxel grid filter (used as x-y surface extraction. Note that leaf in z is very big)
  // ApproximateVoxelGrid<PointRGB> grid;
  // grid.setLeafSize(voxel_size, voxel_size, 0.05f);
  // grid.setDownsampleAllData(true);
  // grid.setInputCloud(cloud);
  // grid.filter(*cloud);

  // Remove isolated points
  // RadiusOutlierRemoval<PointRGB> outrem;
  // outrem.setInputCloud(cloud);
  // outrem.setRadiusSearch(0.4);
  // outrem.setMinNeighborsInRadius(10);
  // outrem.filter(*cloud);
  // StatisticalOutlierRemoval<PointRGB> sor;
  // sor.setInputCloud(cloud);
  // sor.setMeanK(30);
  // sor.setStddevMulThresh(1.0);
  // sor.filter(*cloud);

  return cloud;
}

void TextureMapper::greedyTriangulation(PointCloudRGB::Ptr cloud,
                                        PolygonMesh& triangles)
{
  // Convert pointcloud to XYZ
  PointCloudXYZ::Ptr cloud_xyz(new PointCloudXYZ);
  cloud_xyz->points.resize(cloud->size());
  for (size_t i = 0; i < cloud_xyz->points.size(); i++) {
      cloud_xyz->points[i].x = cloud->points[i].x;
      cloud_xyz->points[i].y = cloud->points[i].y;
      cloud_xyz->points[i].z = cloud->points[i].z;
  }

  // Normal estimation*
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
  search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
  tree->setInputCloud(cloud_xyz);
  n.setInputCloud(cloud_xyz);
  n.setSearchMethod(tree);
  n.setKSearch(20);
  n.compute(*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
  concatenateFields(*cloud_xyz, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  search::KdTree<PointNormal>::Ptr tree2(new search::KdTree<PointNormal>);
  tree2->setInputCloud(cloud_with_normals);

  // Initialize objects
  GreedyProjectionTriangulation<PointNormal> gp3;

  // Set the maximum distance between connected points(maximum edge length)
  gp3.setSearchRadius(0.05);

  // Set typical values for the parameters
  gp3.setMu(100.0);  // 2.5
  gp3.setMaximumNearestNeighbors(300);  // 100
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(true);

  // Get result
  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(tree2);
  gp3.reconstruct(triangles);

  // Additional vertex information
  vector<int> parts = gp3.getPartIDs();
  vector<int> states = gp3.getPointStates();
}

void TextureMapper::transformTFToEigen(const tf::Transform &t, Eigen::Affine3f &e)
{
  for(int i=0; i<3; i++)
  {
    e.matrix()(i,3) = t.getOrigin()[i];
    for(int j=0; j<3; j++)
    {
      e.matrix()(i,j) = t.getBasis()[i][j];
    }
  }
  // Fill in identity in last row
  for (int col = 0 ; col < 3; col ++)
    e.matrix()(3, col) = 0;
  e.matrix()(3,3) = 1;
}

/** \brief Save a textureMesh object to obj file */
int TextureMapper::saveOBJFile (const string &file_name,
                 const TextureMesh &tex_mesh, unsigned precision)
{
  if (tex_mesh.cloud.data.empty ())
  {
    PCL_ERROR ("[io::saveOBJFile] Input point cloud has no data!\n");
    return (-1);
  }

  // Open file
  ofstream fs;
  fs.precision (precision);
  fs.open (file_name.c_str ());

  // Define material file
  string mtl_file_name = file_name.substr (0, file_name.find_last_of (".")) + ".mtl";
  // Strip path for "mtllib" command
  string mtl_file_name_nopath = mtl_file_name;
  mtl_file_name_nopath.erase (0, mtl_file_name.find_last_of ('/') + 1);

  /* Write 3D information */
  // number of points
  int nr_points  = tex_mesh.cloud.width * tex_mesh.cloud.height;
  int point_size = tex_mesh.cloud.data.size () / nr_points;

  // mesh size
  int nr_meshes = tex_mesh.tex_polygons.size ();
  // number of faces for header
  int nr_faces = 0;
  for (int m = 0; m < nr_meshes; ++m)
    nr_faces += tex_mesh.tex_polygons[m].size ();

  // Write the header information
  fs << "####" << endl;
  fs << "# OBJ dataFile simple version. File name: " << file_name << endl;
  fs << "# Vertices: " << nr_points << endl;
  fs << "# Faces: " <<nr_faces << endl;
  fs << "# Material information:" << endl;
  fs << "mtllib " << mtl_file_name_nopath << endl;
  fs << "####" << endl;

  // Write vertex coordinates
  fs << "# Vertices" << endl;
  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    // "v" just be written one
    bool v_written = false;
    for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
    {
      int count = tex_mesh.cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      // adding vertex
      if ((tex_mesh.cloud.fields[d].datatype == PCLPointField::FLOAT32) && (
                tex_mesh.cloud.fields[d].name == "x" ||
                tex_mesh.cloud.fields[d].name == "y" ||
                tex_mesh.cloud.fields[d].name == "z"))
      {
        if (!v_written)
        {
            // write vertices beginning with v
            fs << "v ";
            v_written = true;
        }
        float value;
        memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
            break;
        fs << " ";
      }
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[io::saveOBJFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
    fs << endl;
  }
  fs << "# "<< nr_points <<" vertices" << endl;

  // Write vertex normals
  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    // "vn" just be written one
    bool v_written = false;
    for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
    {
      int count = tex_mesh.cloud.fields[d].count;
      if (count == 0)
      count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      // adding vertex
      if ((tex_mesh.cloud.fields[d].datatype == PCLPointField::FLOAT32) && (
      tex_mesh.cloud.fields[d].name == "normal_x" ||
      tex_mesh.cloud.fields[d].name == "normal_y" ||
      tex_mesh.cloud.fields[d].name == "normal_z"))
      {
        if (!v_written)
        {
          // write vertices beginning with vn
          fs << "vn ";
          v_written = true;
        }
        float value;
        memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
          break;
        fs << " ";
      }
    }
    if (xyz != 3)
    {
    PCL_ERROR ("[io::saveOBJFile] Input point cloud has no normals!\n");
    return (-2);
    }
    fs << endl;
  }
  // Write vertex texture with "vt" (adding latter)

  for (int m = 0; m < nr_meshes; ++m)
  {
    if(tex_mesh.tex_coordinates.size() == 0)
      continue;

    PCL_INFO ("%lu vertex textures in submesh %d\n", tex_mesh.tex_coordinates[m].size (), m);
    fs << "# " << tex_mesh.tex_coordinates[m].size() << " vertex textures in submesh " << m <<  endl;
    for (size_t i = 0; i < tex_mesh.tex_coordinates[m].size (); ++i)
    {
      fs << "vt ";
      fs <<  tex_mesh.tex_coordinates[m][i][0] << " " << tex_mesh.tex_coordinates[m][i][1] << endl;
    }
  }

  int f_idx = 0;

  // int idx_vt =0;
  PCL_INFO ("Writting faces...\n");
  for (int m = 0; m < nr_meshes; ++m)
  {
    if (m > 0)
      f_idx += tex_mesh.tex_polygons[m-1].size ();

    if(tex_mesh.tex_materials.size() !=0)
    {
      fs << "# The material will be used for mesh " << m << endl;
      //TODO pbl here with multi texture and unseen faces
      fs << "usemtl " <<  tex_mesh.tex_materials[m].tex_name << endl;
      fs << "# Faces" << endl;
    }
    for (size_t i = 0; i < tex_mesh.tex_polygons[m].size(); ++i)
    {
      // Write faces with "f"
      fs << "f";
      size_t j = 0;
      // There's one UV per vertex per face, i.e., the same vertex can have
      // different UV depending on the face.
      for (j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size (); ++j)
      {
        unsigned int idx = tex_mesh.tex_polygons[m][i].vertices[j] + 1;
        fs << " " << idx
        << "/" << 3*(i+f_idx) +j+1
        << "/" << idx; // vertex index in obj file format starting with 1
      }
      fs << endl;
    }
    PCL_INFO ("%lu faces in mesh %d \n", tex_mesh.tex_polygons[m].size () , m);
    fs << "# "<< tex_mesh.tex_polygons[m].size() << " faces in mesh " << m << endl;
  }
  fs << "# End of File";

  // Close obj file
  PCL_INFO ("Closing obj file\n");
  fs.close ();

  /* Write material defination for OBJ file*/
  // Open file
  PCL_INFO ("Writing material files\n");
  //dont do it if no material to write
  if(tex_mesh.tex_materials.size() ==0)
    return (0);

  ofstream m_fs;
  m_fs.precision (precision);
  m_fs.open (mtl_file_name.c_str ());

  // default
  m_fs << "#" << endl;
  m_fs << "# Wavefront material file" << endl;
  m_fs << "#" << endl;
  for(int m = 0; m < nr_meshes; ++m)
  {
    m_fs << "newmtl " << tex_mesh.tex_materials[m].tex_name << endl;
    m_fs << "Ka "<< tex_mesh.tex_materials[m].tex_Ka.r << " " << tex_mesh.tex_materials[m].tex_Ka.g << " " << tex_mesh.tex_materials[m].tex_Ka.b << endl; // defines the ambient color of the material to be (r,g,b).
    m_fs << "Kd "<< tex_mesh.tex_materials[m].tex_Kd.r << " " << tex_mesh.tex_materials[m].tex_Kd.g << " " << tex_mesh.tex_materials[m].tex_Kd.b << endl; // defines the diffuse color of the material to be (r,g,b).
    m_fs << "Ks "<< tex_mesh.tex_materials[m].tex_Ks.r << " " << tex_mesh.tex_materials[m].tex_Ks.g << " " << tex_mesh.tex_materials[m].tex_Ks.b << endl; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
    m_fs << "d " << tex_mesh.tex_materials[m].tex_d << endl; // defines the transparency of the material to be alpha.
    m_fs << "Ns "<< tex_mesh.tex_materials[m].tex_Ns  << endl; // defines the shininess of the material to be s.
    m_fs << "illum "<< tex_mesh.tex_materials[m].tex_illum << endl; // denotes the illumination model used by the material.
    // illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
    // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
    m_fs << "map_Kd " << tex_mesh.tex_materials[m].tex_file << endl;
    m_fs << "###" << endl;
  }
  m_fs.close ();
  return (0);
}

void TextureMapper::mapTexture(PointCloudXYZ::Ptr cloud,
                const PolygonMesh& triangles,
                const vector< pair<string, tf::Transform> > &img_poses,
                TextureMesh& mesh)
{
  // Create the TextureMesh object that will contain our UV-mapped mesh
  // TextureMesh mesh;
  mesh.cloud = triangles.cloud;
  vector<Vertices> polygon_1;

  // push faces into the texturemesh object
  polygon_1.resize(triangles.polygons.size());
  for(size_t i =0; i < triangles.polygons.size(); ++i)
  {
    polygon_1[i] = triangles.polygons[i];
  }
  mesh.tex_polygons.push_back(polygon_1);

  // Load textures and cameras poses and intrinsics
  ROS_INFO("\nLoading textures and camera poses...\n");
  texture_mapping::CameraVector cams_vector;
  for (size_t i = 0; i < img_poses.size(); i++)
  {
    TextureMapping<PointXYZ>::Camera cam;
    Eigen::Affine3f transform;
    tf::Transform T = img_poses[i].second;
    transformTFToEigen(T, transform);
    cam.pose = transform;
    cam.focal_length = focal_length_;
    cam.height = image_height_;
    cam.width = image_width_;
    cam.texture_file = img_poses[i].first;
    cams_vector.push_back(cam);
  }
  ROS_INFO("\tLoaded %lu textures.\n", cams_vector.size());

  //showCameras(cams_vector, cloud);

  // Create materials for each texture(and one extra for occluded faces)
  mesh.tex_materials.resize(cams_vector.size() + 1);
  for(int i = 0 ; i <= cams_vector.size() ; ++i)
  {
    TexMaterial mesh_material;
    mesh_material.tex_Ka.r = 0.2f;
    mesh_material.tex_Ka.g = 0.2f;
    mesh_material.tex_Ka.b = 0.2f;

    mesh_material.tex_Kd.r = 0.8f;
    mesh_material.tex_Kd.g = 0.8f;
    mesh_material.tex_Kd.b = 0.8f;

    mesh_material.tex_Ks.r = 1.0f;
    mesh_material.tex_Ks.g = 1.0f;
    mesh_material.tex_Ks.b = 1.0f;

    mesh_material.tex_d = 1.0f;
    mesh_material.tex_Ns = 75.0f;
    mesh_material.tex_illum = 2;

    stringstream tex_name;
    tex_name << "material_" << i;
    tex_name >> mesh_material.tex_name;

    if(i < cams_vector.size()) {
      mesh_material.tex_file = cams_vector[i].texture_file;
    } else {
      mesh_material.tex_file = "occluded.jpg";
      mesh_material.tex_d = 0.0f;
    }
    mesh.tex_materials[i] = mesh_material;
  }

  std::vector<std::vector< Eigen::Vector2f> > previous, next;
  previous = mesh.tex_coordinates;

  ROS_INFO_STREAM("PREVIOUS: " << previous.size());

  // Sort faces
  ROS_INFO("\nSorting faces by cameras...\n");
  // TextureMapping object that will perform the sort
  TextureMapping<PointXYZ> tm;
  tm.textureMeshwithMultipleCameras(mesh, cams_vector);

  next = mesh.tex_coordinates;
  ROS_INFO_STREAM("NEXT: " << next[1][1000](0) << ", " << next[1][1000](1));


  ROS_INFO("Sorting faces by cameras done.\n");
  for (size_t i = 0 ; i <= cams_vector.size() ; ++i)
  {
    ROS_INFO("\tSub mesh %lu contains %lu faces and %lu UV coordinates.\n", i, mesh.tex_polygons[i].size(), mesh.tex_coordinates[i].size());
  }

  // compute normals for the mesh
  ROS_INFO("\nEstimating normals...\n");
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
  search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ> ());
  tree->setInputCloud(cloud);
  n.setInputCloud(cloud);
  n.setSearchMethod(tree);
  n.setKSearch(20);
  n.compute(*normals);

  // Concatenate XYZ and normal fields
  PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
  concatenateFields(*cloud, *normals, *cloud_with_normals);
  ROS_INFO("...Done.\n");

  toPCLPointCloud2(*cloud_with_normals, mesh.cloud);

  ROS_INFO("\nSaving mesh to textured_mesh.obj\n");

  saveOBJFile("textured_mesh.obj", mesh, 5);
}


tf::Transform TextureMapper::readPoses(string work_dir, int cloud_id, tf::Transform& cloud_pose)
{
  string graph_file = work_dir + "/graph_vertices.txt";

  string line;
  bool found = false;
  ifstream file(graph_file.c_str());
  while (getline(file, line))
  {
    int i = 0;
    string value;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    istringstream ss(line);
    while(getline(ss, value, ','))
    {
      if (i == 1)
      {
        if (value == boost::lexical_cast<string>(cloud_id))
          found = true;
      }
      else
      {
        if (found)
        {
          if (i == 2)
            x = boost::lexical_cast<double>(value);
          else if (i == 3)
            y = boost::lexical_cast<double>(value);
          else if (i == 4)
            z = boost::lexical_cast<double>(value);
          else if (i == 5)
            qx = boost::lexical_cast<double>(value);
          else if (i == 6)
            qy = boost::lexical_cast<double>(value);
          else if (i == 7)
            qz = boost::lexical_cast<double>(value);
          else if (i == 8)
            qw = boost::lexical_cast<double>(value);
          break;
        }
      }
      i++;
    }

    if (found)
    {
      // Build the tf
      tf::Vector3 t(x, y, z);
      tf::Quaternion q(qx, qy, qz, qw);
      tf::Transform transf(q, t);
      cloud_pose = transf;
    }
  }

  tf::Vector3 t(0.06, 0, 0);
  tf::Quaternion q(0, 0, 0, 1);
  // tf::Matrix3x3 m;
  // m.setRPY(1.57, 0, 1.57);
  tf::Transform transf(q, t);
  cloud_pose = transf;
}

/** \brief Display a 3D representation showing the a cloud and a list of camera with their 6DOf poses */
void TextureMapper::showCameras (pcl::texture_mapping::CameraVector cams, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

  // visualization object
  pcl::visualization::PCLVisualizer visu ("cameras");

  // add a visual for each camera at the correct pose
  for(int i = 0 ; i < cams.size () ; ++i)
  {
    // read current camera
    pcl::TextureMapping<pcl::PointXYZ>::Camera cam = cams[i];
    double focal = cam.focal_length;
    double height = cam.height;
    double width = cam.width;

    // create a 5-point visual for each camera
    pcl::PointXYZ p1, p2, p3, p4, p5;
    p1.x=0; p1.y=0; p1.z=0;
    double angleX = RAD2DEG (2.0 * atan (width / (2.0*focal)));
    double angleY = RAD2DEG (2.0 * atan (height / (2.0*focal)));
    double dist = 0.75;
    double minX, minY, maxX, maxY;
    maxX = dist*tan (atan (width / (2.0*focal)));
    minX = -maxX;
    maxY = dist*tan (atan (height / (2.0*focal)));
    minY = -maxY;
    p2.x=minX; p2.y=minY; p2.z=dist;
    p3.x=maxX; p3.y=minY; p3.z=dist;
    p4.x=maxX; p4.y=maxY; p4.z=dist;
    p5.x=minX; p5.y=maxY; p5.z=dist;
    p1=pcl::transformPoint (p1, cam.pose);
    p2=pcl::transformPoint (p2, cam.pose);
    p3=pcl::transformPoint (p3, cam.pose);
    p4=pcl::transformPoint (p4, cam.pose);
    p5=pcl::transformPoint (p5, cam.pose);
    std::stringstream ss;
    ss << "Cam #" << i+1;
    visu.addText3D(ss.str (), p1, 0.1, 1.0, 1.0, 1.0, ss.str ());

    ss.str ("");
    ss << "camera_" << i << "line1";
    visu.addLine (p1, p2,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line2";
    visu.addLine (p1, p3,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line3";
    visu.addLine (p1, p4,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line4";
    visu.addLine (p1, p5,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line5";
    visu.addLine (p2, p5,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line6";
    visu.addLine (p5, p4,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line7";
    visu.addLine (p4, p3,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line8";
    visu.addLine (p3, p2,ss.str ());
  }

  // add a coordinate system
  visu.addCoordinateSystem (0.25);

  // add the mesh's cloud (colored on Z axis)
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler (cloud, "z");
  visu.addPointCloud (cloud, color_handler, "cloud");

  // reset camera
  visu.resetCamera ();

  // wait for user input
  visu.spin ();
}

void TextureMapper::process(string cloud_filename, string work_dir)
{
  ROS_INFO("\nLoading cloud from file %s...\n", cloud_filename.c_str());
  PointCloudRGB::Ptr cloud(new PointCloud<PointRGB>);
  PCDReader reader;
  reader.read (cloud_filename, *cloud);
  // Eigen::Affine3d tf_eigen;
  // tf::Vector3 t(0, 0, 0);
  // tf::Matrix3x3 m;
  // m.setRPY(0,0,0);
  // tf::Transform transf(m, t);
  // transformTFToEigen(tf_tf, tf_eigen);
  // pcl::transformPointCloud(*cloud, *cloud, tf_eigen);
  ROS_INFO("\tInput cloud contains %lu vertices\n", cloud->points.size());

  ROS_INFO("\nLoading poses from file %sgraph_vertices.txt ...\n", work_dir.c_str());
  vector< pair<string, tf::Transform> > img_poses;
  tf::Transform img_pose;
  std::string image_name("/home/miquel/Dropbox/UIB/workspace/indigo/src/stereo_slam/keyframes/00000.jpg");
  readPoses(work_dir, 0, img_pose);
  img_poses.push_back(std::make_pair(image_name, img_pose));
  ROS_INFO("\tLoaded %lu poses\n", img_poses.size());

  ROS_INFO("\nFiltering cloud...\n");
  PointCloudRGB::Ptr cloud_filtered;
  cloud_filtered = filterCloud(cloud, 0.02f);
  ROS_INFO("\tFiltered cloud contains %lu vertices\n",
           cloud_filtered->points.size());

  ROS_INFO("\nTriangulating cloud...\n");
  PolygonMesh triangles;
  greedyTriangulation(cloud_filtered, triangles);
  ROS_INFO("\tMesh contains %lu faces and %lu vertices\n",
           triangles.polygons.size(),
           cloud_filtered->points.size());

  ROS_INFO("\nTexturing mesh...\n");
  TextureMesh mesh;
  PointCloudXYZ::Ptr cloud_filtered_xyz(new PointCloudXYZ);
  copyPointCloud(*cloud_filtered, *cloud_filtered_xyz);
  mapTexture(cloud_filtered_xyz, triangles, img_poses, mesh);

  // visualization::PCLVisualizer view;      // Just Modify here
  // view.addTextureMesh(mesh);
  // view.spin();
}
