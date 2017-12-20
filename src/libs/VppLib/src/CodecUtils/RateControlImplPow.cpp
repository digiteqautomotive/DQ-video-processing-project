/** @file

MODULE				: RateControlImplPow

TAG						: RCIP

FILE NAME			: RateControlImplPow.cpp

DESCRIPTION		: A class to hold a power function  model for the frame buffer rate rate control to
                match an average rate stream.

COPYRIGHT			: (c)CSIR 2007-2016 all rights resevered

LICENSE				: Software License Agreement (BSD License)

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

=========================================================================================
*/
#ifdef _WINDOWS
#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
#include <windows.h>
#else
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#endif

#include <sstream>
#include <stdlib.h>
#include <string.h>
#include "RateControlImplPow.h"

/*
---------------------------------------------------------------------------
Constants
---------------------------------------------------------------------------
*/
#define RCIP_NUM_SHORT_TERM_SAMPLES 4

/*
---------------------------------------------------------------------------
	Construction and destruction.
---------------------------------------------------------------------------
*/
int RateControlImplPow::Create(int numOfFrames)
{
  /// Clean up first.
  Destroy();

  if( (!_meanDiffSamples.Create(numOfFrames))       || (!_modelRateSamples.Create(numOfFrames))  || 
      (!_buffRateSamples.Create(numOfFrames))       || (!_distortionSamples.Create(numOfFrames)) ||
      (!_modelPredRateSamples.Create(numOfFrames))  || (!_modelMismatchRateSamples.Create(numOfFrames)) ) 
    return(0);

  _numOfFrames = numOfFrames; 
  _meanDiffSamples.Clear(); 
  _modelRateSamples.Clear(); 
  _buffRateSamples.Clear(); 
  _distortionSamples.Clear();
  _modelPredRateSamples.Clear();
  _modelMismatchRateSamples.Clear();

  _prevMeanDiff           = 0.0;
  _prevModelRate          = 0.0;
  _prevBuffRate           = 0.0;
  _prevDistortion         = 0.0;
  _prevModelPredRate      = 0.0;
  _prevModelMismatchRate  = 0.0;

  _outOfBoundsFlag    = false;

  _modelFit = 0.0;
  _Spr      = 0.0;
  _Smr      = 0.0;

  _Sx   = 0.0;
  _Sy   = 0.0;
  _Sxy  = 0.0;
  _Sx2  = 0.0;

  /// Long term mean difference
  _SEi  = (double)(numOfFrames*(numOfFrames+1))/2.0;
  _SEi2 = (double)(numOfFrames*(numOfFrames+1)*((2*numOfFrames)+1))/6.0;
  _SEy  = 0.0;
  _SEiy = 0.0;
  _a1   = 1.0;
  _a2   = 0.0;
  _R2   = 1.0;
  /// Short term
  _SEi_s  = (double)(RCIP_NUM_SHORT_TERM_SAMPLES*(RCIP_NUM_SHORT_TERM_SAMPLES + 1)) / 2.0;
  _SEi2_s = (double)(RCIP_NUM_SHORT_TERM_SAMPLES*(RCIP_NUM_SHORT_TERM_SAMPLES + 1)*((2 * RCIP_NUM_SHORT_TERM_SAMPLES) + 1)) / 6.0;
  _SEy_s  = 0.0;
  _SEiy_s = 0.0;
  _a1_s   = 1.0;
  _a2_s   = 0.0;
  _R2_s   = 1.0;

  _Sbuff = 0.0;

  _mseSignal = 0.0;

  _RCTableLen = 0;
  _RCTablePos = 0;

#ifdef RCIP_DUMP_RATECNTL
  _RCTableLen = 500;
	_RCTable.Create(8, _RCTableLen);

  _RCTable.SetHeading(0, "Frame");
  _RCTable.SetDataType(0, MeasurementTable::INT);
  _RCTable.SetHeading(1, "Dmax");
  _RCTable.SetDataType(1, MeasurementTable::INT);
  _RCTable.SetHeading(2, "Rate");
  _RCTable.SetDataType(2, MeasurementTable::DOUBLE);
  _RCTable.SetHeading(3, "Pred Rate");
  _RCTable.SetDataType(3, MeasurementTable::DOUBLE);
  _RCTable.SetHeading(4, "Pred Mean Diff");
  _RCTable.SetDataType(4, MeasurementTable::DOUBLE);
  _RCTable.SetHeading(5, "Mean Diff");
  _RCTable.SetDataType(5, MeasurementTable::DOUBLE);
  _RCTable.SetHeading(6, "RD a");
  _RCTable.SetDataType(6, MeasurementTable::DOUBLE);
  _RCTable.SetHeading(7, "RD b");
  _RCTable.SetDataType(7, MeasurementTable::DOUBLE);

  int i;
  char charBuffer[8];
  for(i = 0; i < 16; i++)
  {
    _RCTable.SetHeading(i+8, (const char*)_itoa(i, charBuffer, 10) );
    _RCTable.SetDataType(i+8, MeasurementTable::DOUBLE);
    _RCTable.SetHeading(i+24, (const char*)_itoa(i, charBuffer, 10) );
    _RCTable.SetDataType(i+24, MeasurementTable::DOUBLE);
  }//end for i...

  _predMD           = 0.0;
  _MD               = 0.0;
  _predDmax         = 0.0;
  _predRate         = 0.0;
  _rate             = 0.0;
#endif

  return(1);
}//end Create.

void RateControlImplPow::Destroy(void)
{
  _RCTable.Destroy();
  _RCTableLen = 0;
  _RCTablePos = 0;

  _modelMismatchRateSamples.Destroy(); 
  _modelPredRateSamples.Destroy(); 
  _distortionSamples.Destroy(); 
  _modelRateSamples.Destroy(); 
  _buffRateSamples.Destroy(); 
  _meanDiffSamples.Destroy();

  _numOfFrames = 0;

}//end Destroy.

void RateControlImplPow::Dump(const char* filename)
{
  if(_RCTablePos > 0)
    _RCTable.Save(filename, ",", 1);
  _RCTablePos = 0;
}//end Dump.

void RateControlImplPow::Dump(void)
{
  std::ostringstream filename;
  filename << RCIP_DUMP_FILENAME;  ///< Head.
  filename << ".csv";              ///< Tail
  Dump(filename.str().c_str());
}//end Dump.

/*
---------------------------------------------------------------------------
	Public Interface Methods.
---------------------------------------------------------------------------
*/

///---------------------- Post-Encoding Measurements --------------------------

/// Add samples to the fifo buffers and hold the sample that is discarded. The
/// last discarded value is needed in the running sum update process.

/** Store model measurements after encoding a frame.
All rate measurements are in bpp. This implementation does not use the mae parameter.
@param rate       : Total rate of the frame including headers.
@param coeffrate  : Not used in this implementation.
@param distortion : Implementation specific distortion of the frame.
@param mse        : Implementation specific signal mean square difference between pels in the frame.
@param mae        : Implementation specific signal mean absolute difference between pels in the frame.
@return           : none.
*/
void RateControlImplPow::StoreMeasurements(double rate, double coeffrate, double distortion, double mse, double mae)
{ 
  bool initialisationRequired = (bool)(!ValidData());  ///< Status prior to storing new measurement values.

#ifdef RCIP_DUMP_RATECNTL
  _MD   = mse;
  _rate = rate;

  /// This method is the last to be called after processing a frame. Therefore dump here prior to updating the params.
  if(_RCTablePos < _RCTableLen)
  {
    _RCTable.WriteItem(0, _RCTablePos, _RCTablePos);            ///< Frame
    _RCTable.WriteItem(1, _RCTablePos, (int)(0.5 + _predDmax)); ///< Dmax
    _RCTable.WriteItem(2, _RCTablePos, _predRate);              ///< Required Rate
    _RCTable.WriteItem(3, _RCTablePos, rate);                   ///< Actual Rate
    _RCTable.WriteItem(4, _RCTablePos, _predMD);                ///< Pred MSE
    _RCTable.WriteItem(5, _RCTablePos, mse);                    ///< Actual MSE
    _RCTable.WriteItem(6, _RCTablePos, _a);                     ///< Model parameter a
    _RCTable.WriteItem(7, _RCTablePos, _b);                     ///< Model parameter b

    /// These data points were used to find _a and _b used in the prediction.
    for(int ti = 0; ti < 16; ti++)
    {
      _RCTable.WriteItem(ti+8, _RCTablePos, (_distortionSamples.GetBuffer())[ti]);  ///< Model ln(mse/dmax) data points
      _RCTable.WriteItem(ti+24, _RCTablePos, (_modelRateSamples.GetBuffer())[ti]);  ///< Model rate data points points
    }//end for i...

    _RCTablePos++;
  }//end if _RCTablePos... 
#endif

  /// Store all new measurements. 
  PutMeanDiff(mse, initialisationRequired);      ///<  NB: Mean diff MUST be stored first as the others use its value.
  PutDistortion(distortion, initialisationRequired);  ///< Internally uses meandiff
  PutModelRate(rate, initialisationRequired);         ///< ln(r) to use in the curve fitting process.
  PutBuffRate(rate, initialisationRequired);          ///< Actual rate produced in the encoding process.
  PutPredRate(rate, initialisationRequired);          ///< Both the predicted rate and the mismatch rate.

  if (!initialisationRequired) ///< Update the running sums associated with the model. 
    ModelUpdate();
  else  ///< Indicates that there was no valid data in the model before storing these first samples.
        /// Initialise the fifo buffers for the R-D model, Mean Diff and Model Diff prediction models.
    ModelInitialise();

}//end StoreMeasurements.

///---------------------- Pre-Encoding Predictions --------------------------

/** Predict the distortion for the next frame from the desired average rate.
The distortion measure is predicted from the coeff rate (total rate - header rate) using
an appropriate R-D model. The header rate and the signal mean diff are predicted from the
previous measured frame data using linear extrapolation.
@param  targetAvgRate : Total targeted average rate (including headers).
@param  rateLimit     : Upper limit to predicted rate.
@return               : Predicted distortion.
*/
int RateControlImplPow::PredictDistortion(double targetAvgRate, double rateLimit)
{
  _outOfBoundsFlag              = false;
  _upperDistortionOverflowFlag  = false;
  _lowerDistortionOverflowFlag  = false;
  _upperRateOverflowFlag        = false;
  _lowerRateOverflowFlag        = false;

  /// Default settings for the non-valid data case.
  double targetRate      = targetAvgRate;
  double targetRateLimit = rateLimit;

  ///------------------ Rate Prediction --------------------------------------
  if(ValidData())
  {
    /// Buffer averaging model: What must the target rate for the
    /// next N frames be to make the average over the rate buffer equal
    /// to the average target rate specified in the parameter?

    /// N past frames and recover in k frames: target rate = target avg rate*(N+k)/k - total buff/k.
    int numRecoverFrames = _numOfFrames;  ///< Make recovery same as past num of frames until further notice.
    targetRate = ((targetAvgRate * (double)(_numOfFrames + numRecoverFrames))/(double)numRecoverFrames) - (_Sbuff/(double)numRecoverFrames);

  }//end if ValidData...

  /// Keep the target rate within the upper rate limit.
  if(targetRate > targetRateLimit)
    targetRate = targetRateLimit;

  /// No less than the coeff floor rate.
  if(targetRate < 0.00)
    targetRate = _rateLower;

  ///--------------- Distortion Prediction -----------------------------------------

  /// Predict Mean Diff with linear extrapolation model. Select between a long term or
  /// short term prediction based on the best statistical R^2 fit with the model. Override
  /// the prediction if a scene change was signalled externally.
  double predMD = ModelMeanDifference();

  /// Model function solved for distortion.
  double distd = ModelDistortion(targetRate, _a, _b, predMD);

  /// Limit a decrease change in distortion (implying limiting an increase in rate) to dd = prevd - 0.25*prevd.
  double prevd = (_meanDiffSamples.GetBuffer())[0]*(exp((_distortionSamples.GetBuffer())[0]));
  double limit = prevd - (0.25*prevd);
  if((distd < limit)&&(prevd < _distUpper))
  {
    distd = limit;
    targetRate = ModelRate(distd, _a, _b, predMD);
  }//end if distd...

  /// Cannot be greater than 16x16x(256^2). In response, converge
  /// towards the upper distortion limit by halving from the previous 
  /// frame distortion.
  if (distd > _distUpper)
  {
    distd = prevd + (abs(_distUpper - prevd)*0.5);
    _outOfBoundsFlag = true;
    _upperDistortionOverflowFlag = true;
  }//end if distd...
   /// Cannot be less than 16x16x(2^2). Converge to lower distortion limit.
  if (distd < _distLower)
  {
    distd = prevd - (abs(prevd - _distLower)*0.5);
    _outOfBoundsFlag = true;
    _lowerDistortionOverflowFlag = true;
  }//end if distd...

   /// Update the target rate with changes in Dmax boundaries.
  if(_outOfBoundsFlag)
    targetRate = ModelRate(distd, _a, _b, predMD);

#ifdef RCIP_DUMP_RATECNTL
  _predRate = targetRate;
  _predMD   = predMD;
  _predDmax = distd;
#endif

  return((int)(distd + 0.5));
}//end PredictDistortion.

 /*
 ----------------------------------------------------------------------------------------------------
 Private Methods
 ----------------------------------------------------------------------------------------------------
 */

 /** Calculate R^2 from a sample list.
 Determine the R^2 statistical model fit within an array of samples. Only
 calculate for the length specified from the list and do it in reverse order.
 @param  samples : Sample list of values
 @param  length  : Length from the value list to use.
 @param  mlin    : Linear model slope.
 @param  clin    : Linear model y-intercept.
 @return         : R^2.
 */
double RateControlImplPow::RSqrLinRevOrder(double* samples, int length, double mlin, double clin)
{
  double mean = 0.0;
  for (int i = 0; i < length; i++)
    mean += samples[i];
  mean = mean / length;

  double SStot = 0.0;
  double SSres = 0.0;
  for (int i = 0; i < length; i++)
  {
    double x = samples[i];
    SStot += (x - mean)*(x - mean);
    SSres += (x - (mlin*(length - i) + clin))*(x - (mlin*(length - i) + clin));
  }//end for i...

  double R2 = 1.0;
  if (SStot != 0.0)
    R2 = 1.0 - (SSres / SStot);

  return(R2);
}//end RSqrLinRevOrder.

 /** Predict the mean difference between pels in the frame.
 Predict the frame Mean Diff with linear extrapolation model. Select between a long term or
 short term prediction based on the best statistical R^2 fit with the model. Override the
 prediction if a scene change was signalled externally and reset the signal to zero.
 @return : Predicted mean difference.
 */
double RateControlImplPow::ModelMeanDifference(void)
{
  double predMD;
  if (_mseSignal == 0.0)
  {
    if (_R2_s > _R2)  ///< Short term is closer to 1.0 than the long term (better fit).
      predMD = (_a1_s*(double)(RCIP_NUM_SHORT_TERM_SAMPLES + 1)) + _a2_s; ///< Mean diff samples are in reverse order.
    else
      predMD = (_a1*(double)(_numOfFrames + 1)) + _a2;
    /// Can never be negative. Use last value if prediction is negative.
    if (predMD < 0.0)
      predMD = (_meanDiffSamples.GetBuffer())[0];
  }//end if _mseSignal...
  else
  {
    predMD = _mseSignal;
    _mseSignal = 0.0;  ///< Clear the signal.
  }//end else...

  return(predMD);
}//end ModelMeanDifference.

 /** Initialise all R-D model buffers and persistant member variables.
 Initialise the fifo buffers for the R-D model, Mean Diff and Model prediction members. The
 fifo buffers must have some valid data in them and the model parameters must be reset prior
 to calling this method. The values that are updated on every StoreMeasurement() call are
 initialised here.
 @return : None.
 */
void RateControlImplPow::ModelInitialise(void)
{
  /// Initialise the fifo buffers for the R-D model, Mean Diff and Model Diff prediction models.
  _Sx   = 0;
  _Sy   = 0;
  _Sxy  = 0;        ///< Power R-D.
  _Sx2  = 0;

  _modelFit = 0.0;  ///< Performance measures.
  _Spr      = 0.0;
  _Smr      = 0.0;

  _SEy  = 0.0;      ///< Linear Mean Diff.
  _SEiy   = 0.0;
  _SEy_s  = 0.0;
  _SEiy_s = 0.0;

  _Sbuff = 0.0;     ///< Frame bit buffer.

  for (int i = 0; i < _numOfFrames; i++)
  {
    ///< Power R-D.
    double x = (_distortionSamples.GetBuffer())[i];
    double y = (_modelRateSamples.GetBuffer())[i];

    double x2 = x*x;
    _Sx += x;
    _Sy += y;
    _Sxy += x*y;
    _Sx2 += x2;

    _Spr += (_modelPredRateSamples.GetBuffer())[i];
    _Smr += (_modelMismatchRateSamples.GetBuffer())[i];

    ///< Linear Mean Diff
    x = (_meanDiffSamples.GetBuffer())[i];
    /// _SEi and _SEi2 are constants.
    _SEy += x;
    /// The sample time series are in reverse order in the fifo. x[fifo(0)] is x[_numOfFrames-1].
    /// The independent axis of the model describes samples at [1..N] where 1 is the oldest and N 
    /// from the last frame. Extrapolation will predict x[N+1].
    _SEiy += (_numOfFrames - i)*x; ///< The sample time series are in reverse order in the fifo.

                                   /// Frame bit buffer rate. 
    _Sbuff += (_buffRateSamples.GetBuffer())[i];

  }//end for i...
   /// Long term R sqr.
  _R2 = RSqrLinRevOrder(_meanDiffSamples.GetBuffer(), _numOfFrames, _a1, _a2);

  for (int i = 0; i < RCIP_NUM_SHORT_TERM_SAMPLES; i++)
  {
    double x = (_meanDiffSamples.GetBuffer())[i];
    _SEy_s += x;
    _SEiy_s += (RCIP_NUM_SHORT_TERM_SAMPLES - i)*x;
  }//end for i...
   /// Short term R sqr.
  _R2_s = RSqrLinRevOrder(_meanDiffSamples.GetBuffer(), RCIP_NUM_SHORT_TERM_SAMPLES, _a1_s, _a2_s);

}//end ModelInitialise.

/** Update all R-D model parameters.
Update the model parameters for the R-D model (ln(Dmax/md), ln(r)), Mean Diff and the running
sums. The latest data must have been loaded into the fifo buffers prior to calling this
method. The values are updated on every StoreMeasurement() call.
@return : None.
*/
void RateControlImplPow::ModelUpdate(void)
{
  /// Update (ln(Dmax/md), ln(r)) running sums.
  double x = (_distortionSamples.GetBuffer())[0];
  double y = (_modelRateSamples.GetBuffer())[0];

  double x2 = x*x;
  double px2 = _prevDistortion*_prevDistortion;
  _Sx = UpdateRunningSum(_prevDistortion, x, _Sx);
  _Sy = UpdateRunningSum(_prevModelRate, y, _Sy);
  _Sxy = UpdateRunningSum(_prevDistortion*_prevModelRate, x*y, _Sxy);
  _Sx2 = UpdateRunningSum(px2, x2, _Sx2);

  _Spr = UpdateRunningSum(_prevModelPredRate, (_modelPredRateSamples.GetBuffer())[0], _Spr);
  _Smr = UpdateRunningSum(_prevModelMismatchRate, (_modelMismatchRateSamples.GetBuffer())[0], _Smr);

  _modelFit = _Smr / _Spr;

  /// Determine new R-D model parameters.
//  double tempValForSlopeBoundaryCheck = 1.0 / (log(_distUpper / (_meanDiffSamples.GetBuffer())[0]));
//  double tempValForRateCheck          = log(_rateLower);
  /// Improvements on the current parameters applied to the objective function. Find the Cramer's Rule 
  /// solution and test it against boundary conditions. If the solution is out of bounds then use a gradient
  /// decsent algorithm to modify the current parameters. Choose only those solutions that improve on the
  /// objective function.

  /// Cramer's Solution and objective function (aa,bb) with current objective function (_a,_b)
  double aa = 0.0;
  double bb = 0.0;
  CramersRuleSoln(_Sx2, _Sx, _Sx, (double)_numOfFrames, _Sxy, _Sy, &bb, &aa);
  double logaa = aa;
  aa = exp(aa);
  double loga = log(_a);
  double currObjFunc    = ObjectiveFunction(_numOfFrames, log(_a), _b);
  double cramerObjFunc  = ObjectiveFunction(_numOfFrames, logaa, bb);
   /// Outside boubndary conditions.
//  bool cramerDisqualified = (aa < 24.0) || (aa > 100000.0) || (bb < -3.0) || (bb > -0.3);
//  bool cramerDisqualified = (aa <= 0.0) || (bb >= 0.0);// || (bb > ((tempValForRateCheck - logaa)*tempValForSlopeBoundaryCheck));
//  bool cramerDisqualified = !WithinModelBounds(aa, bb);

  /// Choose best improved solution.
  bool cramerInBounds = WithinModelBounds(aa, bb);
  if (cramerInBounds && (cramerObjFunc <= currObjFunc) )
  {
    _a = aa;
    _b = bb;
  }//end if cramerInBounds...
  else ///< Either out of bounds or no improvement on current solution.
  {
    /// Try gradient descent for a better solution.
    /// Determine the gradient vector at current parameters (_a,_b).
    double a, b;
    double gradObjFunc = GradientDescent(&a, &b);
    bool   gradInBounds = WithinModelBounds(a, b);

    if (gradInBounds && (gradObjFunc <= currObjFunc) )
    {
      _a = a;
      _b = b;
    }// end if gradInBounds...

    /// All other conditions leave _a and _b unchanged.
  }//end else...

/*
  if (!cramerDisqualified && (cramerObjFunc <= currObjFunc))
  {
    _a = aa;
    _b = bb;
  }//end if !cramerDisqualified...
  else ///< use gradient descent change if Cramer's soln is out of bounds.
  {
    /// Determine the gradient vector at current parameters (_a,_b).
    double a, b;
    double gradObjFunc = GradientDescent(&a, &b);

    if ( (gradObjFunc <= currObjFunc) && (a > 0) && (b < 0) && (b <= ((tempValForRateCheck - log(a))*tempValForSlopeBoundaryCheck)) )
//    if (gradObjFunc <= currObjFunc)
      {
      _a = a;
      _b = b;
    }//end if gradObjFunc...
  }//end else...
*/
   /// Update mean diff running sums.
  _SEy = UpdateRunningSum(_prevMeanDiff, (_meanDiffSamples.GetBuffer())[0], _SEy);
  _SEiy = 0.0;
  for (int i = 0; i < _numOfFrames; i++)
    _SEiy += (_numOfFrames - i)*(_meanDiffSamples.GetBuffer())[i]; ///< The sample time series are in reverse order in the fifo.
  /// Update linear constants. Cramer's Rule.
  CramersRuleSoln(_SEi2, _SEi, _SEi, (double)_numOfFrames, _SEiy, _SEy, &_a1, &_a2);

  /// Update short term mean differnce members.
  _SEy_s = 0.0;
  _SEiy_s = 0.0;
  for (int i = 0; i < RCIP_NUM_SHORT_TERM_SAMPLES; i++)
  {
    _SEiy_s += (RCIP_NUM_SHORT_TERM_SAMPLES - i)*(_meanDiffSamples.GetBuffer())[i]; ///< The sample time series are in reverse order in the fifo.
    _SEy_s += (_meanDiffSamples.GetBuffer())[i];
  }//end for i...
   /// Update linear constants. Cramer's Rule.
  CramersRuleSoln(_SEi2_s, _SEi_s, _SEi_s, (double)RCIP_NUM_SHORT_TERM_SAMPLES, _SEiy_s, _SEy_s, &_a1_s, &_a2_s);

  /// Determine R sqr.
  _R2 = RSqrLinRevOrder(_meanDiffSamples.GetBuffer(), _numOfFrames, _a1, _a2);
  _R2_s = RSqrLinRevOrder(_meanDiffSamples.GetBuffer(), RCIP_NUM_SHORT_TERM_SAMPLES, _a1_s, _a2_s);

  /// Update total buffer rate running sums.
  _Sbuff = UpdateRunningSum(_prevBuffRate, (_buffRateSamples.GetBuffer())[0], _Sbuff);

}//end ModelUpdate.

/** Determine new model parameters through gradient descent..
The input model parameters are the current (_a,_b). Find another new set (a,b) using gadient 
descent for an improved objective function.
@param a : Return model parameter a
@param b : Return model parameter b
@return  : New objective function value.
*/
double RateControlImplPow::GradientDescent(double* a, double* b)
{
  double la, lb;

  /// Determine the gradient vector at current parameters (_a,_b).
  double dsdb = 2.0*((_b*_Sx2) + (log(_a)*_Sx) - _Sxy);
  double dsda = 2.0*((_b*_Sx) + (log(_a)*(double)_numOfFrames) - _Sy) / _a;

  /// Start with obj function value slightly larger than the current obj function value.
  double gradObjFunc = ObjectiveFunction(_numOfFrames, log(_a), _b) + 1.0;

  /// Continuously halve gamma to find the best new (la, lb) parameters.
  double  gamma      = 1.0;
  int     cnt        = 0;
  bool    goingDown  = true;
  while ((cnt < 26) && (goingDown))
  {
    lb = _b - (gamma * dsdb);
    la = _a - (gamma * dsda);
    double lclGradObjFunc = ObjectiveFunction(_numOfFrames, log(la), lb);

    /// If not descending then back up to previous a & b and get out.
    if (cnt && (lclGradObjFunc >= gradObjFunc))
    {
      goingDown = false;
      gamma = gamma*2.0;  ///< Reverse back.
      lb = _b - (gamma * dsdb);
      la = _a - (gamma * dsda);
    }//end if lclGradObjFunc...
    else
    {
      gradObjFunc = lclGradObjFunc;
      gamma = gamma / 2.0;
    }//end else...

    cnt++;
  }//end while cnt...

  *a = la;
  *b = lb;
  return(gradObjFunc);
}//end GradientDescent.
