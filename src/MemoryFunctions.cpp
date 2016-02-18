//
//  GENERAL DESCRIPTION: Functions used to create shared memory spaces.
//
//  Notes :
//
//
//  DATE            NAME           REASON
//*******************************************************************************
//
//*******************************************************************************
#define _OPT_PROTOS

#include "MemoryFunctions.h"


// Shared Memory Key Assignments
key_t ImageBufferKey = 5001;
key_t StatusInformationKey = 5002;


// Shared Memory Pointer Assignments
struct ImageBufferType *ImageBufferptr;
struct StatusInformationType *StatusInformationptr;


//******************************************************************************
//	Create Shared Memory Segment
//																			                                       
//  This function creates a shared memory segment for use by all processes which
//  attach to the segment.
//
//******************************************************************************
void* CreateSharedMemorySegment (key_t key, int32u segmentsize, int32u permissions)
{
  int32s sharedmemoryid;
  void *memoryseg;
  
  sharedmemoryid = shmget(key, segmentsize, permissions | IPC_CREAT);
  
  memoryseg = shmat(sharedmemoryid, (void *) 0, 0);
 
  ASSERT(memoryseg != (void*)-1);
  
  return memoryseg;
}


//******************************************************************************
//	Create Shared Memory
//																			                                       
//	This function creates all shared memory or attaches to
//  the shared memory so the module has access to the shared memory.
//
//******************************************************************************
void CreateSharedMemory ()
{
  // Create Pointers to Global Shared Memory Segments
  ImageBufferptr = (ImageBuffer*)CreateSharedMemorySegment (ImageBufferkey, sizeof(struct ImageBufferType ), 0666);

  StatusInformationptr = (StatusInformationType*)CreateSharedMemorySegment (StatusInformationkey, sizeof(struct StatusInformationType), 0666);
}