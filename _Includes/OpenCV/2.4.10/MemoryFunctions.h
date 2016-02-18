//******************************************************************************
//
//  GENERAL DESCRIPTION: Function prototypes for functions used to create shared
//                       memory spaces.
//
//*******************************************************************************
//  DATE            NAME           REASON
//*******************************************************************************
//
//
//*******************************************************************************

#ifndef MEMORYFUNCTIONS_H_
#define MEMORYFUNCTIONS_H_


// Included Files
#include <sys/types.h>
#include <sys/shm.h>
#include <sys/ipc.h>


// Shared Memory Key Assignments
extern key_t ImageBufferKey = 5001;
extern key_t StatusInformationKey = 5002;

struct StatusInformationType {
  // what the RoboRIO needs to tell the thread
  double GyroReadingsFromRoboRIO;

  // what the image thread needs to send to RoboRIO
  int    TargetX;
  int    TargetY;
  double TargetDistance;
  double GyroReadingToFaceTarget;
};

struct ImageBufferType {
  // I don't know what this is, I don't think it can be a Mat
  // because that might have a struct that then points to some memory
  // for the actual pixels.
  int Pixels[320][240]; // how to get data into a fixed size memory area?
}

// Pointers for Shared Memory Data Structures
extern struct ImageBufferType *ImageBufferptr;
extern struct StatusInformationType *StatusInformationptr;


//******************************************************************************
//	Create Shared Memory
//																			                                        
//	This function creates all shared memory or attaches to
//  the shared memory so the module has access to the shared memory.
//
//******************************************************************************
void CreateSharedMemory ();

#endif /* MEMORYFUNCTIONS_H_ */
