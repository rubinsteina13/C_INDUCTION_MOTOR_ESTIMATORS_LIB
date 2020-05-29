/**
  ***********************************************************************************
  * @file    fp_pid.h
  * @author  Serhii Yatsenko [royalroad1995@gmail.com]
  * @version V1.0
  * @date    May-2020
  * @brief   This file contains the type definition of data structures and function
  *	     prototypes for implementation the floating point P/I/D controllers.
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
#ifndef __FP_PID_H__
#define __FP_PID_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes -----------------------------------------------------------------------*/
/* Exported types -----------------------------------------------------------------*/

/** 
  * @brief "Floating point P Controller Module" data structure
  */ 
typedef struct sP
{
// Inputs:
	float fIn;			// Controller's input
	float fUpOutLim;		// Controller's output upper limit
	float fLowOutLim;		// Controller's output lower limit
	float fKp;			// Proportional coefficient value
// Outputs:
	float fOut;			// Controller's output
// Functions:
	void  (*m_calc)(struct sP*);	// Pointer to controller's out calculator
	void  (*m_rst)(struct sP*);	// Pointer to controller's reset function
} tP;

/** 
  * @brief "Floating point PI Controller Module" data structure
  */ 
typedef struct sPI
{
// Inputs:
	float fDtSec;			// Discretization time, Sec
	float fIn;			// Controller's input
	float fKp;			// Proportional coefficient value
	float fKi;			// Integral coefficient value
	float fUpOutLim;		// Controller's output upper limit
	float fLowOutLim;		// Controller's output lower limit
// Internal variables:	
	float fPout;			// Proportional link's output
	float fIout;			// Integral link's output
	float fIprevIn;			// Integral link's previous input
	float fIprevOut;		// Integral link's previous output
// Outputs:
	float fOut;			// Controller's output
// Functions:
	void  (*m_calc)(struct sPI*);	// Pointer to controller's out calculator
	void  (*m_rst)(struct sPI*);	// Pointer to controller's reset function
} tPI;

/** 
  * @brief "Floating point PD Controller Module" data structure
  */ 
typedef struct sPD
{
// Inputs:
	float fDtSec;			// Discretization time, Sec
	float fIn;			// Controller's input
	float fKp;			// Proportional coefficient value
	float fKd;			// Derivative coefficient value
	float fUpOutLim;		// Controller's output upper limit
	float fLowOutLim;		// Controller's output lower limit
// Internal variables:	
	float fPout;			// Proportional link's output
	float fDout;			// Derivative link's output
	float fDprevIn;			// Derivative link's previous input
	float fDprevOut;		// Derivative link's previous output
// Outputs:
	float fOut;			// Controller's output
// Functions:
	void  (*m_calc)(struct sPD*);	// Pointer to controller's out calculator
	void  (*m_rst)(struct sPD*);	// Pointer to controller's reset function
} tPD;

/** 
  * @brief "Floating point PID Controller Module" data structure
  */ 
typedef struct sPID
{
// Inputs:
	float fDtSec;			// Discretization time, Sec
	float fIn;			// Controller's input
	float fKp;			// Proportional coefficient value
	float fKi;			// Integral coefficient value
	float fKd;			// Derivative coefficient value
	float fUpOutLim;		// Controller's output upper limit
	float fLowOutLim;		// Controller's output lower limit
// Internal variables:
	float fPout;			// Proportional link's output
	float fIout;			// Integral link's output
	float fDout;			// Derivative link's output
	float fIprevIn;			// Integral link's previous input
	float fIprevOut;		// Integral link's previous output
	float fDprevIn;			// Derivative link's previous input
	float fDprevOut;		// Derivative link's previous output
// Outputs:
	float fOut;			// Controller's output
// Functions:
	void  (*m_calc)(struct sPID*);	// Pointer to controller's out calculator
	void  (*m_rst)(struct sPID*);	// Pointer to controller's reset function
} tPID;

/* Exported constants -------------------------------------------------------------*/

/** 
  * @brief Initialization constant with defaults for user variables with "tP" type
  */
#define P_DEFAULTS {			\
	.fIn		= 0.0f,		\
	.fOut		= 0.0f,		\
	.fKp		= 0.0f,		\
	.fUpOutLim	= 0.0f,		\
	.fLowOutLim	= 0.0f,		\
	.m_calc		= tP_calc,	\
	.m_rst		= tP_rst	\
}

/** 
  * @brief Initialization constant with defaults for user variables with "tPI" type
  */
#define PI_DEFAULTS {			\
	.fDtSec		= 1.0f,		\
	.fIn		= 0.0f,		\
	.fOut		= 0.0f,		\
	.fKp		= 0.0f,		\
	.fKi		= 0.0f,		\
	.fUpOutLim	= 0.0f,		\
	.fLowOutLim	= 0.0f,		\
	.fPout		= 0.0f,		\
	.fIout		= 0.0f,		\
	.fIprevIn	= 0.0f,		\
	.fIprevOut	= 0.0f,		\
	.m_calc		= tPI_calc,	\
	.m_rst		= tPI_rst	\
}

/** 
  * @brief Initialization constant with defaults for user variables with "tPD" type
  */
#define PD_DEFAULTS {			\
	.fDtSec		= 1.0f,		\
	.fIn		= 0.0f,		\
	.fOut		= 0.0f,		\
	.fKp		= 0.0f,		\
	.fKd		= 0.0f,		\
	.fUpOutLim	= 0.0f,		\
	.fLowOutLim	= 0.0f,		\
	.fPout		= 0.0f,		\
	.fDout		= 0.0f,		\
	.fDprevIn	= 0.0f,		\
	.fDprevOut	= 0.0f,		\
	.m_calc		= tPD_calc,	\
	.m_rst		= tPD_rst	\
}

/** 
  * @brief Initialization constant with defaults for user variables with "tPID" type
  */
#define PID_DEFAULTS {			\
	.fDtSec		= 1.0f,		\
	.fIn		= 0.0f,		\
	.fOut		= 0.0f,		\
	.fKp		= 0.0f,		\
	.fKi		= 0.0f,		\
	.fKd		= 0.0f,		\
	.fUpOutLim	= 0.0f,		\
	.fLowOutLim	= 0.0f,		\
	.fPout		= 0.0f,		\
	.fIout		= 0.0f,		\
	.fDout		= 0.0f,		\
	.fIprevIn	= 0.0f,		\
	.fIprevOut	= 0.0f,		\
	.fDprevIn	= 0.0f,		\
	.fDprevOut	= 0.0f,		\
	.m_calc		= tPID_calc,	\
	.m_rst		= tPID_rst	\
}
	
/* Exported macro -----------------------------------------------------------------*/
/* Exported functions -------------------------------------------------------------*/

/* P controller's output calculation function prototype ****************************/
void tP_calc(tP*);

/* Reset the internal variables of P cnotroller ************************************/
void tP_rst(tP*);

/* PI controller's output calculation function prototype ***************************/
void tPI_calc(tPI*);

/* Reset the internal variables of PI cnotroller ***********************************/
void tPI_rst(tPI*);

/* PD controller's output calculation function prototype ***************************/
void tPD_calc(tPD*);

/* Reset the internal variables of PD cnotroller ***********************************/
void tPD_rst(tPD*);

/* PID controller's output calculation function prototype **************************/
void tPID_calc(tPID*);

/* Reset the internal variables of PID cnotroller **********************************/
void tPID_rst(tPID*);

#ifdef __cplusplus
}
#endif

#endif /* __FP_PID_H__ */

/*********************************** END OF FILE ***********************************/
