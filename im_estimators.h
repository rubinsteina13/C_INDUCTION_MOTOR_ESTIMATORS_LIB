/**
  ***********************************************************************************
  * @file    im_estimators.h
  * @author  Serhii Yatsenko [royalroad1995@gmail.com]
  * @version V1.0
  * @date    May-2020
  * @brief   This file contains the type definition of data structures and function
  *			 		 prototypes for implementation the induction motor (IM) rotor flux and
  *			 		 speed estimators.
  ***********************************************************************************
  * @license
  *
  * MIT License
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  * SOFTWARE.
  *
  ***********************************************************************************
  */

/* Define to prevent recursive inclusion ------------------------------------------*/
#ifndef __IM_ESTIMATORS_H__
#define __IM_ESTIMATORS_H__

#ifdef __cplusplus
	extern "C" {
#endif

/* Includes -----------------------------------------------------------------------*/
#include "fp_pid.h"	// P/I/D controllers library
#include <math.h>	

/* Exported types -----------------------------------------------------------------*/

/** 
  * @brief	"IM parameters Module" data structure
  */
typedef struct sIMparams
{
// Inputs:
	float fDt;														// Discretization time, Sec
	float fNpP;														// Count of pole pairs
	float fRs;														// Stator resistance, Ohm
	float fRr;														// Rotor resistance, Ohm
	float fLs;														// Stator inductance, H
	float fLr;														// Rotor inductance, H
	float fLm;														// Magnetizing inductance, H
// Internal variables:
	float f1divTr;												// fRr/fLr
	float f1divKr;												// fLr/fLm
	float fSigLs;													// (1 - (fLm^2)/(fLs*fLr)) * fLs
// Functions:
	void  (*m_init)(struct sIMparams*);		// Pointer to Init() function
} tIMparams;

/** 
  * @brief	"IM sensorless stator back-EMF observer Module" data structure
  */
typedef struct sIMstatObs
{
// Inputs:
	float		fIsAl;												// Stator current Alpha, A
	float		fIsBe;												// Stator current Beta, A
	float		fUsAl;												// Stator voltage Alpha, Volts
	float		fUsBe;												// Stator voltage Beta, Volts
// Internal variables:
	float		fPrevIsAl;										// Previous value of stator current
																				// Alpha, A
	float		fPrevIsBe;										// Previous value of stator current
																				// Beta, A
// Outputs:
	float		fEsAl;												// Stator back-EMF Alpha, Volts
	float		fEsBe;												// Stator back-EMF Beta, Volts
// Functions:
	void		(*m_calc)(struct sIMstatObs*,	// Pointer to estimator function
								 tIMparams*);	
} tIMstatObs;

/** 
  * @brief	"IM sensored rotor flux & back-EMF observer Module" data structure
  */
typedef struct sIMrotObs
{
// Inputs:
	float		fIsAl;												// Stator current Alpha, A
	float		fIsBe;												// Stator current Beta, A
	float		fWrE;													// Rotor electrical speed, Rad/Sec
// Internal variables:
	float		fPrevErAl;										// Previous value of rotor back-EMF
																				// Alpha, Volts
	float		fPrevErBe;										// Previous value of rotor back-EMF
																				// Beta, Volts
	float		fPrevFrAl;										// Previous value of rotor flux
																				// Alpha, Wb
	float		fPrevFrBe;										// Previous value of rotor flux
																				// Beta, Wb
// Outputs:
	float		fFrAl;												// Rotor flux Alpha, Wb
	float		fFrBe;												// Rotor flux Beta, Wb
	float		fErAl;												// Rotor back-EMF Alpha, Volts
	float		fErBe;												// Rotor back-EMF Beta, Volts
// Functions:
	void		(*m_calc)(struct sIMrotObs*,	// Pointer to estimator function
								 tIMparams*);	
} tIMrotObs;

/** 
  * @brief	"IM sensorless rotor speed & flux observer Module" data structure
  */
typedef struct sIMspeedObs
{
// Inputs:
	float		fUsAl;												// Stator voltage Alpha, Volts
	float		fUsBe;												// Stator voltage Beta, Volts
	float		fIsAl;												// Stator current Alpha, A
	float		fIsBe;												// Stator current Beta, A
// Internal variables:
	tIMstatObs	sIMstatObs;								// Stator observer data structure
	tIMrotObs	sIMrotObs;									// Rotor observer data structure
	tPI			sPI;													// PI-controller data structure
// Outputs:
	float		fWrE;													// Rotor electrical speed, Rad/Sec
	float		fFrAng;												// Rotor flux angle, Rad
	float		fFrMagn;											// Rotor flux magnitude, Wb
// Functions:
	void		(*m_calc)(struct sIMspeedObs*,// Pointer to estimator function
								 tIMparams*);
} tIMspeedObs;

/* Exported constants -------------------------------------------------------------*/

/** 
  * @brief	Initialization constant with defaults for "tIMparams" user variables
  */
#define IM_PARAMS_DEFAULTS {						\
	.fDt				= 1.0f,										\
	.fNpP				= 0.0f,										\
	.fRs				= 0.0f,										\
	.fRr				= 0.0f,										\
	.fLs				= 0.0f,										\
	.fLr				= 0.0f,										\
	.fLm				= 0.0f,										\
	.f1divTr		= 0.0f,										\
	.f1divKr		= 0.0f,										\
	.fSigLs			= 0.0f,										\
	.m_init			= tIMparams_init					\
}

/** 
  * @brief	Initialization constant with defaults for "tIMstatObs" user variables
  */
#define IM_STAT_OBS_DEFAULTS {					\
	.fIsAl			= 0.0f,										\
	.fIsBe			= 0.0f,										\
	.fUsAl			= 0.0f,										\
	.fUsBe			= 0.0f,										\
	.fPrevIsAl	= 0.0f,										\
	.fPrevIsBe	= 0.0f,										\
	.fEsAl			= 0.0f,										\
	.fEsBe			= 0.0f,										\
	.m_calc			= tIMstatObs_calc					\
}

/** 
  * @brief	Initialization constant with defaults for "tIMrotObs" user variables
  */
#define IM_ROT_OBS_DEFAULTS {						\
	.fIsAl			= 0.0f,										\
	.fIsBe			= 0.0f,										\
	.fWrE				= 0.0f,										\
	.fPrevErAl	= 0.0f,										\
	.fPrevErBe	= 0.0f,										\
	.fPrevFrAl	= 0.0f,										\
	.fPrevFrBe	= 0.0f,										\
	.fFrAl			= 0.0f,										\
	.fFrBe			= 0.0f,										\
	.fErAl			= 0.0f,										\
	.fErBe			= 0.0f,										\
	.m_calc			= tIMrotObs_calc					\
}

/** 
  * @brief	Initialization constant with defaults for "tIMspeedObs" user variables
  */
#define IM_SPEED_OBS_DEFAULTS {					\
	.fUsAl			= 0.0f,										\
	.fUsBe			= 0.0f,										\
	.fIsAl			= 0.0f,										\
	.fIsBe			= 0.0f,										\
	.sIMstatObs	= IM_STAT_OBS_DEFAULTS,		\
	.sIMrotObs	= IM_ROT_OBS_DEFAULTS,		\
	.sPI				= PI_DEFAULTS,						\
	.fWrE				= 0.0f,										\
	.fFrAng			= 0.0f,										\
	.fFrMagn		= 0.0f,										\
	.m_calc			= tIMspeedObs_calc				\
}

/* Exported macro -----------------------------------------------------------------*/
/* Exported functions -------------------------------------------------------------*/

/* IM parameters initialization function prototype *********************************/
void tIMparams_init(tIMparams*);

/* IM stator back-EMF observer function prototype **********************************/
void tIMstatObs_calc(tIMstatObs*, tIMparams*);

/* IM rotor back-EMF and flux observer function prototype **************************/
void tIMrotObs_calc(tIMrotObs*, tIMparams*);

/* IM rotor speed and flux observer function prototype *****************************/
void tIMspeedObs_calc(tIMspeedObs*, tIMparams*);

#ifdef __cplusplus
}
#endif

#endif /* __IM_ESTIMATORS_H__ */

/*********************************** END OF FILE ***********************************/
