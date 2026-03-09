#ifndef __FrameQueue_h__
#define __FrameQueue_h__

#include <streams.h>
#include <list>


class FrameQueue
{
public:
  FrameQueue(void) {}
  ~FrameQueue();

  mutable CCritSec m_csFrame2Queue;
  std::list<IMediaSample*> q;

  void AddFrame(IMediaSample*pFrame);
  void Erase(void);
  IMediaSample *PeekFrame(void) const;
  IMediaSample *PeekFrame(REFERENCE_TIME &tStart, REFERENCE_TIME &tStop);
  void PopFrame(void);
};

#endif