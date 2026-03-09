#include "stdafx.h"

#include "FrameQueue.h"


FrameQueue::~FrameQueue()
{
  while(!q.empty())
  {
    IMediaSample *pFrame = q.front();
    q.pop_front();
    if(pFrame) pFrame->Release();
  }
}


void FrameQueue::Erase(void)
{
IMediaSample *pFrame;
  while(!q.empty())
  {
    {		// locked scope
      CAutoLock lck(&m_csFrame2Queue);
      pFrame = q.front();
      q.pop_front();
    }
    if(pFrame) pFrame->Release();
  }
}

void FrameQueue::AddFrame(IMediaSample*pFrame)
{
  if(pFrame)
  {
    pFrame->AddRef();
    CAutoLock lck(&m_csFrame2Queue);
    q.push_back(pFrame);
  }
}


IMediaSample *FrameQueue::PeekFrame(void) const
{
CAutoLock lck(&m_csFrame2Queue);
  if(q.empty()) return NULL;
  IMediaSample *IS = q.front();
  if(IS) IS->AddRef();
return IS;
}


IMediaSample *FrameQueue::PeekFrame(REFERENCE_TIME &tStart, REFERENCE_TIME &tStop)
{
//LONGLONG tStart2, tStop2;
REFERENCE_TIME TimeStart2, TimeEnd2;
IMediaSample *IS = NULL;
CAutoLock lck(&m_csFrame2Queue);

  while(!q.empty())
  {
    std::list<IMediaSample*>::iterator it = q.begin();
    IS = *it;
    if(IS!=NULL)
    {
      it++;
      if(it==q.end()) 
      {
        q.pop_front();
        return(IS);		// IS is the last element in queue.
      }
      //IS->GetMediaTime(&tStart2,&tStop2);
      IS->GetTime(&TimeStart2,&TimeEnd2);
      if(TimeStart2>=tStart || TimeEnd2>=tStart)
      {
        q.pop_front();
        return(IS);
      }
    }
    q.pop_front();
  }

  if(IS) IS->AddRef();
return IS;
}


void FrameQueue::PopFrame(void)
{
  IMediaSample *pFrame;
  {
    CAutoLock lck(&m_csFrame2Queue);
    if(q.empty()) return;
    pFrame = q.front();
    q.pop_front();
  }
  if(pFrame) pFrame->Release();		// It is better to unlock queue here.
}

