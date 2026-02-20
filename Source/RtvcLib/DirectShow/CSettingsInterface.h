/** @file

MODULE				: DirectShow

FILE NAME			: CSettingsInterface.h

DESCRIPTION			: The implementation of the COM ISettingsInterface
					DirectShow filters inheriting this implementation should implement the initParameters method in which 
					the addParameter method should be called for each member variable that is to be managed by this class.
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008-2017, CSIR
Copyright (c) 2025-2026, Digiteq automotive, programmed by Jaroslav Fojtik	All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of the CSIR nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

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
===========================================================================*/
#pragma once

#include "SettingsInterface.h"
#include "CSettingsInterface.h"

// STL
#include <string>
#include <list>


enum RTVC_PARAM_TYPES
{
	VPP_STRING,
	VPP_INT,
	VPP_UINT,
	VPP_BOOL,
	VPP_DOUBLE
};


class ParameterBase
{
public:
	ParameterBase(const char *NewName, const OLECHAR *NewOleName, bool NewReadOnly=false);
        ~ParameterBase();

	std::string	Name;
        OLECHAR		*OleName;
	unsigned	Hash;
	bool		ReadOnly;

	virtual bool Validate(void)	{return true;}
	virtual void RevertToDefaultValue(void) = 0;
        virtual void RevertToPreviousValue(void) = 0;
	virtual std::string ToStr(void) const = 0;
        virtual bool FromStr(const char *str) = 0;
        virtual size_t Size(void) const = 0;
	virtual RTVC_PARAM_TYPES Type(void) const = 0;
        virtual HRESULT Save(IStream *pStm) const = 0;
        virtual HRESULT Load(IStream *pStm) = 0;
        virtual void getVariant(VARIANT &var) const = 0;
        virtual bool setVariant(const VARIANT &var) = 0;
};


class ParameterInt: public ParameterBase
{
public:
	ParameterInt(const char *NewName, const OLECHAR *NewOleName, int *ValueAddr, int IniDefaultValue, bool ReadOnly=false);
	int	*pValue;
	int	DefaultValue;
	int	PreviousValue;
	
	virtual std::string ToStr(void) const;
        virtual bool FromStr(const char *str);

	virtual RTVC_PARAM_TYPES Type(void) const {return VPP_INT;}
	virtual size_t Size(void) const {return sizeof(int);}
        virtual void RevertToDefaultValue(void) {*pValue=DefaultValue;}
        virtual void RevertToPreviousValue(void) {*pValue=PreviousValue;}
        virtual HRESULT Save(IStream *pStm) const {return pStm->Write(pValue,sizeof(*pValue),NULL);};
	virtual HRESULT Load(IStream *pStm) {return pStm->Read(pValue,sizeof(*pValue),NULL);};
        virtual void getVariant(VARIANT &var) const;
        virtual bool setVariant(const VARIANT &var);
};


class ParameterUnsigned: public ParameterBase
{
public:
	ParameterUnsigned(const char *Name, const OLECHAR *NewOleName, unsigned *ValueAddr, unsigned IniDefaultValue, bool ReadOnly=false);
	unsigned *pValue;
	unsigned DefaultValue;
	unsigned PreviousValue;

	virtual std::string ToStr(void) const;
        virtual bool FromStr(const char *str);
	
	virtual RTVC_PARAM_TYPES Type(void) const {return VPP_UINT;}
        virtual size_t Size(void) const {return sizeof(unsigned);}
	virtual void RevertToDefaultValue(void) {*pValue=DefaultValue;}
	virtual void RevertToPreviousValue(void) {*pValue=PreviousValue;}        
        virtual HRESULT Save(IStream *pStm) const {return pStm->Write(pValue,sizeof(*pValue),NULL);};
	virtual HRESULT Load(IStream *pStm) {return pStm->Read(pValue,sizeof(*pValue),NULL);};
        virtual void getVariant(VARIANT &var) const;
        virtual bool setVariant(const VARIANT &var);
};


class ParameterDouble: public ParameterBase
{
public:
	ParameterDouble(const char *NewName, const OLECHAR *NewOleName, double *ValueAddr, double IniDefaultValue, double NewReadOnly=false);
	double *pValue;
	double DefaultValue;
	double PreviousValue;

	virtual std::string ToStr(void) const;
        virtual bool FromStr(const char *str);

	virtual RTVC_PARAM_TYPES Type(void) const {return VPP_DOUBLE;}
        virtual size_t Size(void) const {return sizeof(double);}
        virtual void RevertToPreviousValue(void) {*pValue=PreviousValue;}
	virtual void RevertToDefaultValue(void) {*pValue=DefaultValue;}
        virtual HRESULT Save(IStream *pStm) const {return pStm->Write(pValue,sizeof(*pValue),NULL);};
	virtual HRESULT Load(IStream *pStm) {return pStm->Read(pValue,sizeof(*pValue),NULL);};
        virtual void getVariant(VARIANT &var) const;
        virtual bool setVariant(const VARIANT &var);
};


class ParameterBool: public ParameterBase
{
public:
	ParameterBool(const char *Name, const OLECHAR *NewOleName, bool *ValueAddr, bool IniDefaultValue, bool ReadOnly=false);
	bool *pValue;
	bool DefaultValue;
	bool PreviousValue;

	virtual std::string ToStr(void) const;
        virtual bool FromStr(const char *str);

	virtual RTVC_PARAM_TYPES Type(void) const {return VPP_BOOL;}
        virtual size_t Size(void) const {return 1;}
	virtual void RevertToDefaultValue(void) {*pValue=DefaultValue;}
        virtual void RevertToPreviousValue(void) {*pValue=PreviousValue;}
        virtual HRESULT Save(IStream *pStm) const {return pStm->Write(pValue,1,NULL);};
	virtual HRESULT Load(IStream *pStm);
        virtual void getVariant(VARIANT &var) const;
        virtual bool setVariant(const VARIANT &var);
};


class ParameterString: public ParameterBase
{
public:
	ParameterString(const char *Name, const OLECHAR *NewOleName, std::string *ValueAddr, std::string IniDefaultValue, bool ReadOnly=false);
	std::string *pValue;
	std::string DefaultValue;
	std::string PreviousValue;

	virtual std::string ToStr(void) const;
        virtual bool FromStr(const char *str);

	virtual RTVC_PARAM_TYPES Type(void) const {return VPP_STRING;}
        virtual size_t Size(void) const {return pValue->length()+1;}
	virtual void RevertToDefaultValue(void) {*pValue=DefaultValue;}
        virtual void RevertToPreviousValue(void) {*pValue=PreviousValue;}
        virtual HRESULT Save(IStream *pStm) const;
	virtual HRESULT Load(IStream *pStm);
        virtual void getVariant(VARIANT &var) const;
        virtual bool setVariant(const VARIANT &var);
};



////////////////////////////////////////////////////////////////////////////////////////////////



/** \ingroup DirectShowLib
 * ISettingsInterface implementation. */
class CSettingsInterface: public ISettingsInterface, public IPersistStream, public IPersistPropertyBag, public IPropertyBag
{
public:
	CSettingsInterface(void);
	virtual ~CSettingsInterface(void);

	/// This helper method reverts a member variables value back to the initial value and should be used e.g. if any of the postconditions of 
	/// SetParameter are unfulfilled. This is usually the case if other conditions exist upon which the success of SetParameter depends.
	/// One way to use this method is to override SetParameter and call the base class method. Then check the postcondition and if that has not been met,
	/// Call revertParameter
	/// @param[in] szParamName The name of the parameter that should be reverted to its previous value
	virtual bool revertParameter(const char* szParamName);

	std::string GetParameter(const char* szParamName, bool *Ok=NULL);

		//---------------- Overridden from ISettingsInterface ----------------------
	/// This method returns the value of the requested parameter.
	/// @return This method returns S_OK on success and E_FAIL on failure.
	/// @param[in] szParamName The name of the parameter
	/// @param[in] nBufferSize The size of the passed in buffer szValue
	/// @param[out] szValue The buffer to store the result in
	/// @param[out] pLength The length of the result
	STDMETHODIMP GetParameter(const char* szParamName, int nBufferSize, char* szValue, int* pLength);
	/// Overridden from ISettingsInterface
	/// This method sets the value of the requested parameter provided the parameter exists
	/// @return This method returns S_OK on success and E_FAIL on failure
	/// @param[in] szParamName The name of the parameter
	/// @param[in] szValue The value the parameter should be set to
	STDMETHODIMP SetParameter(const char* szParamName, const char* szValue);
	/// Overridden from ISettingsInterface
	/// This method iterates over all parameters and constructs a string containing all parameters current settings.
	/// @return This method returns S_OK on success and E_FAIL on failure e.g. if the passed in buffer is too small. 
	/// @param[out] szResult A buffer for the result of the method
	/// @param[in] nSize The size of the passed in buffer
	STDMETHODIMP GetParameterSettings(char* szResult, int nSize);

		//----------------- IPersistStream------------------
        virtual STDMETHODIMP GetSizeMax(ULARGE_INTEGER *pcbSize);
	virtual STDMETHODIMP Save(IStream *pStm, BOOL fClearDirty);
	virtual STDMETHODIMP IsDirty(void);
	virtual STDMETHODIMP Load(IStream *pStm);

		//----------------- IPersistPropertyBag------------------
	virtual STDMETHODIMP InitNew(void);
        virtual HRESULT STDMETHODCALLTYPE Load(__RPC__in_opt IPropertyBag *pPropBag, __RPC__in_opt IErrorLog *pErrorLog);
        virtual HRESULT STDMETHODCALLTYPE Save(__RPC__in_opt IPropertyBag *pPropBag, BOOL fClearDirty, BOOL fSaveAllProperties);

		//----------------- IPropertyBag------------------
	virtual HRESULT STDMETHODCALLTYPE Read(LPCOLESTR pszPropName, VARIANT *pVar, IErrorLog *pErrorLog);
	virtual HRESULT STDMETHODCALLTYPE Write(LPCOLESTR pszPropName, VARIANT *pVar);

protected:
	// This method needs to be implemented by child classes and called in the child class constructor
	// It takes care of mapping the parameters to member variables
	// Call the addParameter method from this method where <br>
	// szParamName: is the name of the parameter<br>
	// pAddr is a pointer to the member variable which will store the value<br>
	// bReadOnly will ensure that the param value cannot be changed<br>
	// vAllowedValues: If there are any items in this vector, only those will be deemed as valid values
	// virtual void initParameters() = 0; 

	/// The addParameter methods should be called from inside initParameters at start up
	/// @param[in] sParamName The name of the parameter that should be reverted to its previous value
	/// @param[in] pAddr A pointer to the member variable that will store the value
	/// @param[in] nDefaultValue An optional default value for the member variable
	/// @param[in] bReadOnly Readonly parameters cannot be set using the SetParameter method
	/// @param[in] vAllowedValues A vector of allowed values. If the size of this vector > 0 only the values in the vector are considered valid
	void addParameter(const char *sParamName, int* pAddr, int nDefaultValue, bool bReadOnly=false);  //, std::vector<int> vAllowedValues = std::vector<int>());
        void addParameter(const char *sParamName, const OLECHAR *oleName, int* pAddr, int nDefaultValue, bool bReadOnly=false); //, std::vector<int> vAllowedValues)
	/// The addParameter methods should be called from inside initParameters at start up
	/// @param[in] sParamName The name of the parameter that should be reverted to its previous value
	/// @param[in] pAddr A pointer to the member variable that will store the value
	/// @param[in] uiDefaultValue An optional default value for the member variable
	/// @param[in] bReadOnly Readonly parameters cannot be set using the SetParameter method
	/// @param[in] vAllowedValues A vector of allowed values. If the size of this vector > 0 only the values in the vector are considered valid
	void addParameter(const char *sParamName, unsigned* pAddr, unsigned uiDefaultValue, bool bReadOnly=false); //, std::vector<unsigned> vAllowedValues = std::vector<unsigned>());
	void addParameter(const char *sParamName, const OLECHAR *oleName, unsigned* pAddr, unsigned uiDefaultValue, bool bReadOnly=false);
	/// The addParameter methods should be called from inside initParameters at start up
	/// @param[in] sParamName The name of the parameter that should be reverted to its previous value
	/// @param[in] pAddr A pointer to the member variable that will store the value
	/// @param[in] szDefaultValue An optional default value for the member variable
	/// @param[in] bReadOnly Readonly parameters cannot be set using the SetParameter method
	/// @param[in] vAllowedValues A vector of allowed values. If the size of this vector > 0 only the values in the vector are considered valid
	void addParameter(const char *sParamName, std::string* pAddr, std::string szDefaultValue, bool bReadOnly=false); //, std::vector<std::string> vAllowedValues = std::vector<std::string>());
        void addParameter(const char *sParamName, const OLECHAR *oleName, std::string* pAddr, std::string szDefaultValue, bool bReadOnly=false);
	/// The addParameter methods should be called from inside initParameters at start up
	/// @param[in] sParamName The name of the parameter that should be reverted to its previous value
	/// @param[in] pAddr A pointer to the member variable that will store the value
	/// @param[in] bDefaultValue An optional default value for the member variable
	/// @param[in] bReadOnly Readonly parameters cannot be set using the SetParameter method
	/// @param[in] vAllowedValues A vector of allowed values. If the size of this vector > 0 only the values in the vector are considered valid
	void addParameter(const char *sParamName, bool* pAddr, bool bDefaultValue, bool bReadOnly=false); //, std::vector<bool> vAllowedValues = std::vector<bool>());
        void addParameter(const char *sParamName, const OLECHAR *oleName, bool* pAddr, bool bDefaultValue, bool bReadOnly=false);
	/// The addParameter methods should be called from inside initParameters at start up
	/// @param[in] sParamName The name of the parameter that should be reverted to its previous value
	/// @param[in] pAddr A pointer to the member variable that will store the value
	/// @param[in] dDefaultValue An optional default value for the member variable
	/// @param[in] bReadOnly Readonly parameters cannot be set using the SetParameter method
	/// @param[in] vAllowedValues A vector of allowed values. If the size of this vector > 0 only the values in the vector are considered valid
	void addParameter(const char *sParamName, double* pAddr, double dDefaultValue, bool bReadOnly=false); //, std::vector<double> vAllowedValues = std::vector<double>());
        void addParameter(const char *sParamName, const OLECHAR *oleName, double* pAddr, double dDefaultValue, bool bReadOnly=false);

	bool m_DirtyProps;

private:
	std::list<ParameterBase*> m_vParams;       
};

