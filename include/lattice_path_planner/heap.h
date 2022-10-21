#ifndef LATTICE_PATH_PLANNER_HEAP_H
#define LATTICE_PATH_PLANNER_HEAP_H

#include <cstdio>
#include "lattice_path_planner/utils.h"

#define HEAPSIZE 20000000
#define HEAPSIZE_INIT 5000

namespace lattice_path_planner
{
class SearchStateBase
{
public:
  SearchStateBase() {}
  virtual ~SearchStateBase() {}
public:
  int heap_index;
};

struct HeapElement
{
  SearchStateBase* element;
  int key;
};

class Heap
{
public:
  Heap();
  Heap(int initial_size);
  ~Heap();

public:
  bool emptyHeap() const;
  void makeEmptyHeap();
  void insertHeap(SearchStateBase* search_state, int key);
  void updateHeap(SearchStateBase* search_state, int new_key);
  int getMinKey() const;
  SearchStateBase* deleteMinHeap();

private:
  void percolateUp(int hole, HeapElement temp);
  void percolateDown(int hole, HeapElement temp);
  void percolateUpOrDown(int hole, HeapElement temp);
  void sizeCheck();
  void growHeap();

public:
  int heap_size;
  int heap_capacity;
  HeapElement* heap_elements;
};

}  // namespace lattice_path_planner

#endif  // LATTICE_PATH_PLANNER_HEAP_H