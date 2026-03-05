#include "TogglerProperties.h"
#include "TogglerFilter.h"


CUnknown * WINAPI TogglerProperties::CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr)
{
  TogglerProperties *pNewObject = new TogglerProperties(pUnk);
  if(pHr) *pHr = (pNewObject==NULL) ? E_OUTOFMEMORY : S_OK;
  return pNewObject;
}


TogglerProperties::TogglerProperties(IUnknown *pUnk) :
FilterPropertiesBase(NAME("Toggler Properties"), pUnk, IDD_TOGGLE_DIALOG, IDD_TOGGLE_CAPTION)
{
}


HRESULT TogglerProperties::ReadSettings(void)
{
  initialiseControls();

  HRESULT hr = setEditTextFromIntFilterParameter(FILTER_PARAM_TARGET_HEIGHT, IDC_EDIT_TARGET_HEIGHT);
  if (FAILED(hr)) return hr;   
  hr = setEditTextFromIntFilterParameter(FILTER_PARAM_TARGET_WIDTH, IDC_EDIT_TARGET_WIDTH);
  if(FAILED(hr)) return hr;

  setEditTextFromIntFilterParameter(FILTER_PARAM_ACT_WIDTH, IDC_EDIT_ACT_WIDTH);
  setEditTextFromIntFilterParameter(FILTER_PARAM_ACT_HEIGHT, IDC_EDIT_ACT_HEIGHT);

  hr = setCheckBoxFromBoolFilterParameter(FILTER_THROTTLE, IDC_CHECK_THROTTLE);  
  if(FAILED(hr)) return hr;
  hr = setCheckBoxFromBoolFilterParameter(FILTER_DUPLICATE_FRAMES, IDC_CHECK_DUPLICATE_FRAMES);  
  if(FAILED(hr)) return hr;

  hr = setCheckBoxFromBoolFilterParameter(FILTER_PARAM_ORIENTATION, IDC_CHECK_INVERT);
  if(FAILED(hr)) return hr;
  hr = setCheckBoxFromBoolFilterParameter(FILTER_PARAM_CHOOSE_INPUT,IDC_CHECK_TOGGLE_INPUT);
  if(FAILED(hr)) return hr;

  hr = setEditTextFromIntFilterParameter(FILTER_PARAM_DURATION, IDC_EDIT_TOGGLE_DURATION);
  return hr;
}


HRESULT TogglerProperties::OnApplyChanges(void)
{
  HRESULT hr = setIntFilterParameterFromEditText(FILTER_PARAM_TARGET_WIDTH, IDC_EDIT_TARGET_WIDTH);
  if(FAILED(hr)) return hr;
  hr = setIntFilterParameterFromEditText(FILTER_PARAM_TARGET_HEIGHT, IDC_EDIT_TARGET_HEIGHT);
  if(FAILED(hr)) return hr;

  hr = setBoolFilterParameterFromCheckBox(FILTER_THROTTLE, IDC_CHECK_THROTTLE);
  if(FAILED(hr)) return hr;
  hr = setBoolFilterParameterFromCheckBox(FILTER_DUPLICATE_FRAMES, IDC_CHECK_DUPLICATE_FRAMES);
  if(FAILED(hr)) return hr;

  hr = setBoolFilterParameterFromCheckBox(FILTER_PARAM_ORIENTATION, IDC_CHECK_INVERT);
  if(FAILED(hr)) return hr;
  hr = setBoolFilterParameterFromCheckBox(FILTER_PARAM_CHOOSE_INPUT, IDC_CHECK_TOGGLE_INPUT);
  if(FAILED(hr)) return hr;

  hr = setIntFilterParameterFromEditText(FILTER_PARAM_DURATION, IDC_EDIT_TOGGLE_DURATION);
  return hr;
}


void TogglerProperties::initialiseControls()
{
  InitCommonControls();
  const short upper = SHRT_MAX;
  setSpinBoxRange(IDC_SPIN1, 0, upper);
  setSpinBoxRange(IDC_SPIN2, 0, upper);
  //setSpinBoxRange(IDC_SPIN_DURATION, 0, upper);
}
