/******************************************************************************
 * Copyright (c) 2022, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "lattice_path_planner/common/footprint_helper.h"

#include <limits>

#include "lattice_path_planner/common/breadth_first_search.h"

namespace lattice_path_planner {
namespace common {

// poses start at 0,0 continuous domain with half-bin less to account for 0,0
// discrete domain start
XYCellBounds FootprintHelper::GetMotionXYCells(
    const std::vector<XYPoint>& polygon, const std::vector<XYThetaPoint>& poses,
    std::vector<XYCell>* cells, double res) {
  // the bounding box of the polygon
  XYCellBounds xybounds;

  // can't find any motion cells if there are no poses
  if (poses.empty()) {
    return xybounds;
  }

  CHECK_EQ(poses[0].x(), 0.0) << "poses should start at 0,0 continuous domain";
  CHECK_EQ(poses[0].y(), 0.0) << "poses should start at 0,0 continuous domain";

  XYThetaPoint tmp;
  XYPoint offset(DiscXY2Cont(0, res), DiscXY2Cont(0, res));

  // get first footprint set
  std::set<XYCell> first_cell_set;
  tmp.set_x(poses[0].x() + offset.x());
  tmp.set_y(poses[0].y() + offset.y());
  tmp.set_theta(poses[0].theta());
  xybounds = GetFootprintXYCells(polygon, &first_cell_set, tmp, res);

  // duplicate first footprint set into motion set
  std::set<XYCell> cell_set = first_cell_set;

  // call get footprint on the rest of the points
  for (size_t i = 1; i < poses.size(); i++) {
    tmp.set_x(poses[i].x() + offset.x());
    tmp.set_y(poses[i].y() + offset.y());
    tmp.set_theta(poses[i].theta());
    xybounds.Add(GetFootprintXYCells(polygon, &cell_set, tmp, res));
  }

  // convert the motion set to a vector but don't include the cells in the first
  // footprint set
  cells->reserve(cell_set.size() - first_cell_set.size());
  for (const XYCell& cell : cell_set) {
    if (first_cell_set.find(cell) == first_cell_set.end()) {
      cells->push_back(cell);
    }
  }

  return xybounds;
}

// This function is inefficient and should be avoided if possible (you should
// use overloaded functions that uses a set for the cells)!
// pose w.r.t 0,0 discrete domain start
void FootprintHelper::GetFootprintXYCells(const std::vector<XYPoint>& polygon,
                                          std::vector<XYCell>* cells,
                                          const XYThetaPoint& pose,
                                          double res) {
  int x = ContXY2Disc(pose.x(), res);
  int y = ContXY2Disc(pose.y(), res);
  CHECK_EQ(pose.x(), DiscXY2Cont(x, res))
      << "pose should be w.r.t 0,0 discrete domain start";
  CHECK_EQ(pose.y(), DiscXY2Cont(y, res))
      << "pose should be w.r.t 0,0 discrete domain start";

  std::set<XYCell> cell_set;
  for (const XYCell& cell : *cells) {
    cell_set.insert(cell);
  }
  GetFootprintXYCells(polygon, &cell_set, pose, res);
  cells->clear();
  cells->reserve(cell_set.size());

  for (const XYCell& cell : cell_set) {
    cells->push_back(cell);
  }
}

// pose w.r.t 0,0 discrete domain start
XYCellBounds FootprintHelper::GetFootprintXYCells(
    const std::vector<XYPoint>& polygon, std::set<XYCell>* cells,
    const XYThetaPoint& pose, double res) {
  // the bounding box of the polygon
  XYCellBounds xybounds;

  // special case for point robot
  if (polygon.size() <= 1) {
    cells->insert({ContXY2Disc(pose.x(), res), ContXY2Disc(pose.y(), res)});
    xybounds.set_minimum(XYCell());
    xybounds.set_maximum(XYCell());
    return xybounds;
  }

  XYPoint sourcepose(DiscXY2Cont(0, res), DiscXY2Cont(0, res));

  // run bressenham line algorithm around the polygon (add them to the cells
  // set) while doing that find the min and max (x,y) and the average x and y
  double cth = cos(pose.theta());
  double sth = sin(pose.theta());

  std::vector<XYCell> disc_polygon;
  disc_polygon.reserve(polygon.size() + 1);
  int minx = std::numeric_limits<int>::max();
  int maxx = -std::numeric_limits<int>::max();
  int miny = std::numeric_limits<int>::max();
  int maxy = -std::numeric_limits<int>::max();

  // find the bounding box of the polygon
  for (const XYPoint& point : polygon) {
    XYCell p;
    p.set_x(ContXY2Disc(cth * point.x() - sth * point.y() + pose.x(), res));
    p.set_y(ContXY2Disc(sth * point.x() + cth * point.y() + pose.y(), res));
    disc_polygon.push_back(p);
    if (p.x() < minx) {
      minx = p.x();
    }
    if (p.x() > maxx) {
      maxx = p.x();
    }
    if (p.y() < miny) {
      miny = p.y();
    }
    if (p.y() > maxy) {
      maxy = p.y();
    }

    // compute the bounding box
    p.set_x(
        ContXY2Disc(cth * point.x() - sth * point.y() + sourcepose.x(), res));
    p.set_y(
        ContXY2Disc(sth * point.x() + cth * point.y() + sourcepose.y(), res));
    xybounds.Add(p);
  }
  disc_polygon.push_back(disc_polygon.front());

  // make a grid big enough for the footprint
  int sizex = (maxx - minx + 1) + 2;
  int sizey = (maxy - miny + 1) + 2;
  std::vector<std::vector<int>> grid;
  grid.resize(sizex);
  for (int i = 0; i < sizex; i++) {
    grid[i].resize(sizey);
    for (int j = 0; j < sizey; j++) {
      grid[i][j] = 0;
    }
  }

  // plot line points on the grid
  for (size_t i = 1; i < disc_polygon.size(); i++) {
    int x0 = disc_polygon[i - 1].x() - minx + 1;
    int y0 = disc_polygon[i - 1].y() - miny + 1;
    int x1 = disc_polygon[i].x() - minx + 1;
    int y1 = disc_polygon[i].y() - miny + 1;

    // bressenham (add the line cells to the set and to a vector)
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
        cells->insert({y - 1 + minx, x - 1 + miny});
      } else {
        grid[x][y] = 1;
        cells->insert({x - 1 + minx, y - 1 + miny});
      }
      int last_error = error;
      error -= deltay;
      if (error < 0 && x != x1) {
        // make sure we can't have a diagonal line (the 8-connected bfs will
        // leak through)

        int tempy = y;
        int tempx = x;
        if (last_error < -error) {
          tempy += ystep;
        } else {
          tempx += 1;
        }
        if (steep) {
          grid[tempy][tempx] = 1;
          cells->insert({tempy - 1 + minx, tempx - 1 + miny});
        } else {
          grid[tempx][tempy] = 1;
          cells->insert({tempx - 1 + minx, tempy - 1 + miny});
        }

        y += ystep;
        error += deltax;
      }
    }
  }

  // run a 2d bfs from the average (x,y)
  BreadthFirstSearch bfs(sizex, sizey, 1);
  bfs.ComputeDistanceFromPoint(grid, 0, 0);

  // add all cells expanded to the cells set
  for (int i = 1; i < sizex - 1; i++) {
    for (int j = 1; j < sizey - 1; j++) {
      if (bfs.GetDistance(i, j) < 0) {
        cells->insert({i - 1 + minx, j - 1 + miny});
      }
    }
  }

  return xybounds;
}

}  // namespace common
}  // namespace lattice_path_planner
