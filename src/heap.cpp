#include "lattice_path_planner/heap.h"
#include "lattice_path_planner/sbpl_exception.h"

namespace lattice_path_planner
{
Heap::Heap()
{
  heap_size = 0;
  heap_capacity = HEAPSIZE_INIT;
  heap_elements = new HeapElement[heap_capacity];
}

Heap::Heap(int initial_size)
{
  heap_size = 0;
  heap_capacity = initial_size;
  heap_elements = new HeapElement[heap_capacity];
}

Heap::~Heap()
{
  for (int i = 1; i <= heap_size; i++)
    heap_elements[i].element->heap_index = 0;

  delete[] heap_elements;
}

bool Heap::emptyHeap() const
{
  return heap_size == 0;
}

void Heap::makeEmptyHeap()
{
  for (int i = 1; i <= heap_size; i++)
    heap_elements[i].element->heap_index = 0;

  heap_size = 0;
}

void Heap::percolateDown(int hole, HeapElement temp)
{
  if (heap_size != 0)
  {
    int child;
    for (; 2 * hole <= heap_size; hole = child)
    {
      child = 2 * hole;
      if (child != heap_size && heap_elements[child + 1].key < heap_elements[child].key)
        child++;

      if (heap_elements[child].key < temp.key)
      {
        heap_elements[hole] = heap_elements[child];
        heap_elements[hole].element->heap_index = hole;
      }
      else
        break;
    }

    heap_elements[hole] = temp;
    heap_elements[hole].element->heap_index = hole;
  }
}

void Heap::percolateUp(int hole, HeapElement temp)
{
  if (heap_size != 0)
  {
    for (; hole > 1 && temp.key < heap_elements[hole / 2].key; hole /= 2)
    {
      heap_elements[hole] = heap_elements[hole / 2];
      heap_elements[hole].element->heap_index = hole;
    }

    heap_elements[hole] = temp;
    heap_elements[hole].element->heap_index = hole;
  }
}

void Heap::percolateUpOrDown(int hole, HeapElement temp)
{
  if (heap_size != 0)
  {
    if (hole > 1 && temp.key < heap_elements[hole / 2].key)
      percolateUp(hole, temp);
    else
      percolateDown(hole, temp);
  }
}

void Heap::sizeCheck()
{
  if (heap_size + 1 == HEAPSIZE)
    throw SBPL_Exception("ERROR: The heap is full");
  else if (heap_size + 1 == heap_capacity)
    growHeap();
}

void Heap::growHeap()
{
  heap_capacity *= 2;
  if (heap_capacity > HEAPSIZE)
    heap_capacity = HEAPSIZE;

  HeapElement* newHeap = new HeapElement[heap_capacity];
  for (int i = 1; i <= heap_size; i++)
    newHeap[i] = heap_elements[i];

  delete[] heap_elements;
  heap_elements = newHeap;
}

void Heap::insertHeap(SearchStateBase* search_state, int key)
{
  sizeCheck();

  if (search_state->heap_index != 0)
    throw SBPL_Exception("ERROR: The element is already in the heap");

  HeapElement temp;
  temp.element = search_state;
  temp.key = key;

  percolateUp(++heap_size, temp);
}

int Heap::getMinKey() const
{
  if (heap_size == 0)
    return INFINITECOST;
  else
    return heap_elements[1].key;
}

SearchStateBase* Heap::deleteMinHeap()
{
  if (emptyHeap())
    throw SBPL_Exception("ERROR: The heap is empty");

  SearchStateBase* temp = heap_elements[1].element;
  temp->heap_index = 0;

  percolateDown(1, heap_elements[heap_size--]);
  return temp;
}

void Heap::updateHeap(SearchStateBase* search_state, int new_key)
{
  if (search_state->heap_index == 0)
    throw SBPL_Exception("ERROR: The element is not in the heap");

  if (heap_elements[search_state->heap_index].key != new_key)
  {
    heap_elements[search_state->heap_index].key = new_key;
    percolateUpOrDown(search_state->heap_index, heap_elements[search_state->heap_index]);
  }
}

}  // namespace lattice_path_planner