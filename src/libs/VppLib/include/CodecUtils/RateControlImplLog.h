/** @file

MODULE			  : RateControlImplLog

TAG				    : RCIL

FILE NAME		  : RateControlImplLog.h

DESCRIPTION		: A class to hold a natural log function  model for the frame buffer rate rate control to
				        match an average rate stream.

COPYRIGHT		  : (c)CSIR 2007-2017 all rights resevered

LICENSE			  : Software License Agreement (BSD License)

RESTRICTIONS	: Redistribution and use in source and binary forms, with or without 
				        modification, are permitted provided that the following conditions 
				        are met:

								* Redistributions of source code must retain the above copyright notice, 
								this list of conditions and the following disclaimer.
								* Redistributions in binary form must reproduce the above copyright notice, 
								this list of conditions and the following disclaimer in the documentation 
								and/or other materials provided with the distribution.
								* Neither the name of the CSIR nor the names of its contributors may be used 
								to endorse or promote products derived from this software without specific 
								prior written permission.

								THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
								"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
								LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
								A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
								CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
								EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
								PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
								PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
								LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
								NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
								SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

===========================================================================
*/
#ifndef _RATECONTROLIMPLLOG_H
#define _RATECONTROLIMPLLOG_H

#pragma once

#include  <math.h>
#include  "IRateControl.h"
#include  "MeasurementTable.h"

//#define RCIL_DUMP_RATECNTL  1
#define RCIL_DUMP_FILENAME "C:/Users/KFerguson/Documents/Excel/VideoEvaluation/RateControlLog_"	///< File name head only

/*
---------------------------------------------------------------------------
	Class definition.
---------------------------------------------------------------------------
*/
class RateControlImplLog : public IRateControl
{
public:
  RateControlImplLog(void) 
  { _a = 0.2; _b = 1.4; _numOfFrames = 0; 
    _rateUpper = 24.0; _rateLower = 0.0001; _distUpper = 67108864.0; _distLower = 256.0;
  }
  RateControlImplLog(double modelparam_a, double modelparam_b) 
  { _a = modelparam_a; _b = modelparam_b; _numOfFrames = 0; 
    _rateUpper = 24.0; _rateLower = 0.0001; _distUpper = 67108864.0; _distLower = 256.0;
  }
  virtual ~RateControlImplLog() { Destroy(); }

/// IRateControl Interface methods.
public:
  int	  Create(int numOfFrames);
  void	Destroy(void);
  int   GetFameBufferLength(void) { return(_numOfFrames); }
  void  SetRDLimits(double rateUpper, double rateLower, double distUpper, double distLower) 
        { _rateUpper = rateUpper; _rateLower = rateLower; _distUpper = distUpper; _distLower = distLower; }

  /// Pre-encoding: Apply the rate control model to predict the distortion from the target coeff rate.
  int PredictDistortion(double targetAvgRate, double rateLimit);
  /// Post-encoding: Add samples to the fifo buffers and hold the sample that is discarded.
  void StoreMeasurements(double rate, double coeffrate, double distortion, double mse, double mae);
  /// Return average rate across the buffer as the performance measure.
  double GetPerformanceMeasure(void)  { return(_modelFit); }

  /// Is the data in the fifos valid.
  int ValidData(void) 
    { if(!_distortionSamples.Empty() && !_meanDiffSamples.Empty() && !_modelRateSamples.Empty() && !_buffRateSamples.Empty()) return(1); return(0); }
  /// Was the prediction calculation out of bounds?
  bool OutOfBounds(void) { return(_outOfBoundsFlag); }
  /// Who is the culprit?
  bool UpperDistortionOverflow(void)  { return(_upperDistortionOverflowFlag); }
  bool LowerDistortionOverflow(void)  { return(_lowerDistortionOverflowFlag); }
  bool UpperRateOverflow(void)        { return(_upperRateOverflowFlag); }
  bool LowerRateOverflow(void)        { return(_lowerRateOverflowFlag); }
  /// Reset the fifos to empty.
  void Reset(void)
  { _modelMismatchRateSamples.MarkAsEmpty(); _modelPredRateSamples.MarkAsEmpty(); _distortionSamples.MarkAsEmpty(); 
    _modelRateSamples.MarkAsEmpty(); _buffRateSamples.MarkAsEmpty(); _meanDiffSamples.MarkAsEmpty();  }
  /// In this implementation only the mean abs err is used to indicate the degree of scene change. It is immediately reset to zero after use.
  void SignalSceneChange(double mse, double mae) { _mseSignal = mae; }

  /// Utility functions.
  void Dump(const char* filename);  ///< Requires RCIL_DUMP_RATECNTL to be set.
  void Dump(void);                  ///< Requires RCIL_DUMP_FILENAME to be defined.

/// Further implementation specific public methods.
public:

/// Private methods.
protected:
  /// These following group of methods must be called in this exact order because they operate
  /// on members that are interdependant.
  void PutMeanDiff(double meandiff, bool initialise) 
    { if(meandiff < 1.0) meandiff = 1.0;
      PutLinearMeasure(_meanDiffSamples, meandiff, &_prevMeanDiff, initialise); }
  void PutDistortion(double distortion, bool initialise) ///< Must only be called after a call to PutMeanDiff(). 
    { if(distortion < 1.0) distortion = 1.0;
      PutLogMeasure(_distortionSamples, ((_meanDiffSamples.GetBuffer())[0])/distortion, &_prevDistortion, initialise); }
  void PutModelRate(double rate, bool initialise) 
    { PutLinearMeasure(_modelRateSamples, rate, &_prevModelRate, initialise); }
  void PutBuffRate(double rate, bool initialise) 
    { PutLinearMeasure(_buffRateSamples, rate, &_prevBuffRate, initialise); }
  void PutPredRate(double rate, bool initialise) ///< Must only be called after PutDistortion().  
    { 
      double predRate = _a*(_distortionSamples.GetBuffer())[0] + _b;
      PutLinearMeasure(_modelPredRateSamples, predRate, &_prevModelPredRate, initialise);
      double mismatch = fabs(rate - predRate);
      PutLinearMeasure(_modelMismatchRateSamples, mismatch, &_prevModelMismatchRate, initialise);
    }

  double GetMostRecentModelRate(void) { return((_modelRateSamples.GetBuffer())[0]); }
  double GetMostRecentBuffRate(void) { return((_buffRateSamples.GetBuffer())[0]); }
  double GetMostRecentMeanDiff(void) { return((_meanDiffSamples.GetBuffer())[0]); }

  /// Model equations.
  double  ModelDistortion(double rate, double a, double b, double var) { return(var*exp((b - rate)/a)); }
  double  ModelRate(double distortion, double a, double b, double var) { return((a*log(var/distortion)) + b);}
  double  ModelMeanDifference(void);
  void    ModelInitialise(void);
  void    ModelUpdate(void);

  /// Model fit R^2 support.
  double RSqrLinRevOrder(double* samples, int length, double mlin, double clin);

/// Constant members.
public:

/// Common members.
protected:
	int		_numOfFrames;

  /// Clip the values of the rate and distortion to these limits.
  double    _rateUpper;
  double    _rateLower;
  double    _distUpper;
  double    _distLower;

  Fifo      _meanDiffSamples;     ///< Mean differnce between samples of frames.
  double    _prevMeanDiff;
  Fifo      _buffRateSamples;     ///< Stream rate including headers of frames.
  double    _prevBuffRate;
  Fifo      _modelRateSamples;    ///< Model rate is the stream rate.
  double    _prevModelRate;
  Fifo      _distortionSamples;   ///< Dmax for the frames.
  double    _prevDistortion;
  /// Signal when the predicted values are not valid due to out of range errors. Use
  /// this signal to prevent contributions to the model data.
  bool      _outOfBoundsFlag;
  bool      _upperDistortionOverflowFlag;
  bool      _lowerDistortionOverflowFlag;
  bool      _upperRateOverflowFlag;
  bool      _lowerRateOverflowFlag;

  /// To update the constants, _a and _b, Cramer's Rule is used to solve the linear equation 
  /// for a least mean squares criterion curve fit. Running sums are used.
  double    _Sxy;
  double    _Sx2;
  double    _Sx;
  double    _Sy;
  double    _a; ///< Log model constants.
  double    _b;

  /// The prediction of Mean Diff requires a least mean squares criterion curve fit
  /// to a linear extrapolation model with constants a1 and a2. A long term and a
  /// a short term model are used.
  double    _SEi;   ///< Intermediate persistant values for Cramer's Rule calculation.
  double    _SEi2;
  double    _SEy;
  double    _SEiy;
  double    _a1;    ///< Linear model constants.
  double    _a2;
  double    _R2;    ///< Statistical R^2 value to compare performance of long term vs short term fit.
  /// Short term samples. No. of samples is fixed to the most recent in the mean diff sample fifo.
  double    _SEi_s;
  double    _SEi2_s;
  double    _SEy_s;
  double    _SEiy_s;
  double    _a1_s;
  double    _a2_s;
  double    _R2_s;

  /// Buffer rate control for average bit rate.
  double    _Sbuff;

  /// Measure the performance by the model accuracy.
  double    _modelFit;
  Fifo      _modelPredRateSamples;      ///< Model predicted rate is that defined by _a and _b.
  double    _prevModelPredRate;
  Fifo      _modelMismatchRateSamples;  ///< Abs diff between actual rate and model predicted rate.
  double    _prevModelMismatchRate;
  double    _Spr;                       ///< Fifo sum of pred model rate;
  double    _Smr;                       ///< Fifo sum of mismatch rate;

  /// Signalling parameters.
  double    _mseSignal;

  MeasurementTable  _RCTable;
  int               _RCTableLen;
  int               _RCTablePos;
#ifdef RCIL_DUMP_RATECNTL
  double            _predMD;
  double            _MD;
  double            _predDmax;
  double            _predRate;
  double            _rate;
#endif
};// end class RateControlImplLog.

#endif	// _RATECONTROLIMPLLOG_H
