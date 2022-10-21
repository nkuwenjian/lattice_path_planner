#include "lattice_path_planner/utils.h"
#include "lattice_path_planner/sbpl_bfs_2d.h"
#include "lattice_path_planner/sbpl_exception.h"

namespace lattice_path_planner
{
// poses start at 0,0 continuous domain with half-bin less to account for 0,0 discrete domain start
BoundingBox2 get_2d_motion_cells(const std::vector<sbpl_2Dpt_t>& polygon, const std::vector<sbpl_xy_theta_pt_t>& poses, 
  std::vector<sbpl_2Dcell_t>* cells, double res)
{
  if (poses[0].x != 0 || poses[0].y != 0) {
    throw SBPL_Exception("ERROR: Poses should start at 0,0 continuous domain");
  }

  //the bounding box of the polygon
  BoundingBox2 boundingbox;

  sbpl_xy_theta_pt_t temppose;
  sbpl_2Dpt_t offset;
  offset.x = DISCXY2CONT(0, res);
  offset.y = DISCXY2CONT(0, res);

  // can't find any motion cells if there are no poses
  if (poses.empty()) {
    return boundingbox;
  }

  //get first footprint set
  std::set<sbpl_2Dcell_t> first_cell_set;
  temppose.x = poses[0].x + offset.x;
  temppose.y = poses[0].y + offset.y;
  temppose.theta = poses[0].theta;
  boundingbox = get_2d_footprint_cells(polygon, &first_cell_set, temppose, res);

  //duplicate first footprint set into motion set
  std::set<sbpl_2Dcell_t> cell_set = first_cell_set;

  //call get footprint on the rest of the points
  for (unsigned int i = 1; i < poses.size(); i++) {
    temppose.x = poses[i].x + offset.x;
    temppose.y = poses[i].y + offset.y;
    temppose.theta = poses[i].theta;
    boundingbox.Add(get_2d_footprint_cells(polygon, &cell_set, temppose, res));
  }

  //convert the motion set to a vector but don't include the cells in the first footprint set
  cells->reserve(cell_set.size() - first_cell_set.size());
  for (std::set<sbpl_2Dcell_t>::iterator it = cell_set.begin(); it != cell_set.end(); it++) {
    if (first_cell_set.find(*it) == first_cell_set.end()) {
      cells->push_back(*it);
    }
  }

  return boundingbox;
}

// This function is inefficient and should be avoided if possible (you should
// use overloaded functions that uses a set for the cells)!
// pose w.r.t 0,0 discrete domain start
void get_2d_footprint_cells(std::vector<sbpl_2Dpt_t> polygon, std::vector<sbpl_2Dcell_t>* cells, 
  sbpl_xy_theta_pt_t pose, double res)
{
  int x = CONTXY2DISC(pose.x, res);
  int y = CONTXY2DISC(pose.y, res);
  if (pose.x != DISCXY2CONT(x, res) || pose.y != DISCXY2CONT(y, res)) {
    ROS_INFO("x=%d, y=%d, pose.x=%f, pose.y=%f\n", x, y, pose.x, pose.y);
    throw SBPL_Exception("ERROR: Pose should be w.r.t 0,0 discrete domain start");
  }

  std::set<sbpl_2Dcell_t> cell_set;
  for (unsigned int i = 0; i < cells->size(); i++)
    cell_set.insert(cells->at(i));
  get_2d_footprint_cells(polygon, &cell_set, pose, res);
  cells->clear();
  cells->reserve(cell_set.size());

  for (std::set<sbpl_2Dcell_t>::iterator it = cell_set.begin(); it != cell_set.end(); it++) {
    cells->push_back(*it);
  }
}

// pose w.r.t 0,0 discrete domain start
BoundingBox2 get_2d_footprint_cells(const std::vector<sbpl_2Dpt_t>& polygon, std::set<sbpl_2Dcell_t>* cells, 
  const sbpl_xy_theta_pt_t& pose, double res)
{
  //the bounding box of the polygon
  BoundingBox2 boundingbox;

  sbpl_2Dpt_t sourcepose;
  sourcepose.x = DISCXY2CONT(0, res);
  sourcepose.y = DISCXY2CONT(0, res);

  //special case for point robot
  if (polygon.size() <= 1) {
    sbpl_2Dcell_t cell;
    cell.x = CONTXY2DISC(pose.x, res);
    cell.y = CONTXY2DISC(pose.y, res);

    cells->insert(cell);
    boundingbox.SetMinimum(sbpl_2Dcell_t());
    boundingbox.SetMaximum(sbpl_2Dcell_t());
    return boundingbox;
  }

  //run bressenham line algorithm around the polygon (add them to the cells set)
  //while doing that find the min and max (x,y) and the average x and y
  double cth = cos(pose.theta);
  double sth = sin(pose.theta);

  std::vector<sbpl_2Dcell_t > disc_polygon;
  disc_polygon.reserve(polygon.size() + 1);
  int minx = INFINITECOST;
  int maxx = -INFINITECOST;
  int miny = INFINITECOST;
  int maxy = -INFINITECOST;

  //find the bounding box of the polygon
  for (unsigned int i = 0; i < polygon.size(); i++) {
    sbpl_2Dcell_t p;
    p.x = CONTXY2DISC(cth * polygon[i].x - sth * polygon[i].y + pose.x, res);
    p.y = CONTXY2DISC(sth * polygon[i].x + cth * polygon[i].y + pose.y, res);      
    disc_polygon.push_back(p);
    if (p.x < minx) minx = p.x;
    if (p.x > maxx) maxx = p.x;
    if (p.y < miny) miny = p.y;
    if (p.y > maxy) maxy = p.y;

    //compute the bounding box
    p.x = CONTXY2DISC(cth * polygon[i].x - sth * polygon[i].y + sourcepose.x, res);
    p.y = CONTXY2DISC(sth * polygon[i].x + cth * polygon[i].y + sourcepose.y, res);
    boundingbox.Add(p);
  }
  disc_polygon.push_back(disc_polygon.front());

  //make a grid big enough for the footprint
  int sizex = (maxx - minx + 1) + 2;
  int sizey = (maxy - miny + 1) + 2;
  int** grid = new int*[sizex];
  for (int i = 0; i < sizex; i++) {
    grid[i] = new int[sizey];
    for (int j = 0; j < sizey; j++)
      grid[i][j] = 0;
  }

  //plot line points on the grid
  for (unsigned int i = 1; i < disc_polygon.size(); i++) {
    int x0 = disc_polygon[i - 1].x - minx + 1;
    int y0 = disc_polygon[i - 1].y - miny + 1;
    int x1 = disc_polygon[i].x - minx + 1;
    int y1 = disc_polygon[i].y - miny + 1;

    //bressenham (add the line cells to the set and to a vector)
    bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);
    if (steep) {
      int temp = x0;
      x0 = y0;
      y0 = temp;
      temp = x1;
      x1 = y1;
      y1 = temp;
    }
    if (x0 > x1) {
      int temp = x0;
      x0 = x1;
      x1 = temp;
      temp = y0;
      y0 = y1;
      y1 = temp;
    }
    int deltax = x1 - x0;
    int deltay = std::abs(y1 - y0);
    int error = deltax / 2;
    int ystep = (y0 < y1 ? 1 : -1);
    int y = y0;
    for (int x = x0; x <= x1; x++) {
      if (steep) {
        grid[y][x] = 1;
        cells->insert(sbpl_2Dcell_t(y - 1 + minx, x - 1 + miny));
      }
      else {
        grid[x][y] = 1;
        cells->insert(sbpl_2Dcell_t(x - 1 + minx, y - 1 + miny));
      }
      int last_error = error;
      error -= deltay;
      if (error < 0 && x != x1) {
        //make sure we can't have a diagonal line (the 8-connected bfs will leak through)

        int tempy = y;
        int tempx = x;
        if (last_error < -error)
          tempy += ystep;
        else
          tempx += 1;
        if (steep) {
          grid[tempy][tempx] = 1;
          cells->insert(sbpl_2Dcell_t(tempy - 1 + minx, tempx - 1 + miny));
        }
        else {
          grid[tempx][tempy] = 1;
          cells->insert(sbpl_2Dcell_t(tempx - 1 + minx, tempy - 1 + miny));
        }

        y += ystep;
        error += deltax;
      }
    }
  }

  //run a 2d bfs from the average (x,y)
  sbpl_bfs_2d bfs(sizex, sizey, 1);
  bfs.compute_distance_from_point(grid, 0, 0);

  for (int i = 0; i < sizex; i++)
    delete[] grid[i];
  delete[] grid;

  //add all cells expanded to the cells set
  for (int i = 1; i < sizex - 1; i++) {
    for (int j = 1; j < sizey - 1; j++) {
      if (bfs.get_distance(i, j) < 0) cells->insert(sbpl_2Dcell_t(i - 1 + minx, j - 1 + miny));
    }
  }

  return boundingbox;
}

}  // namespace lattice_path_planner