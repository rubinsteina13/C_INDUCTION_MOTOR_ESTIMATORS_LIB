/**
  ***********************************************************************************
  * @file    fp_pid.c
  * @author  Serhii Yatsenko [royalroad1995@gmail.com]
  * @version V1.0
  * @date    May-2020
  * @brief   This file provides firmware function for implementation the following
  *	     types of floating point controllers:
  *		+ proportional (P) controller;
  *		+ proportional–integral (PI) controller;
  *		+ proportional–derivative (PD) controller;
  *		+ proportional–integral–derivative (PID) controller.
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

/* Includes -----------------------------------------------------------------------*/
#include "fp_pid.h"

/* Private typedef ----------------------------------------------------------------*/
/* Private define -----------------------------------------------------------------*/
/* Private constants --------------------------------------------------------------*/
/* Private macro ------------------------------------------------------------------*/
/* Private variables --------------------------------------------------------------*/
/* Private function prototypes ----------------------------------------------------*/
/* Private functions --------------------------------------------------------------*/

/**
  * @brief  Calculate and update the P-controller output.
  * @param  ptP: pointer to user data structure with type "ptP".               
  * @retval None
  */
void tP_calc(tP* ptP)
{
	float fPreOut = ptP->fIn * ptP->fKp;
	
	if(fPreOut > ptP->fUpOutLim) fPreOut = ptP->fUpOutLim;
	if(fPreOut < ptP->fLowOutLim) fPreOut = ptP->fLowOutLim;
	
	ptP->fOut = fPreOut;
}

/**
  * @brief  Reset the internal variables of P-controller to defaults.
  * @param  ptP: pointer to user data structure with type "ptP".               
  * @retval None
  */
void tP_rst(tP* ptP)
{
	ptP->fIn = 0.0f;
	ptP->fOut = 0.0f;
}

/**
  * @brief  Calculate and update the PI-controller output.
  * @param  ptPI: pointer to user data structure with type "ptPI".               
  * @retval None
  */
void tPI_calc(tPI* ptPI)
{
	float fPreOut;
	
	ptPI->fPout = ptPI->fIn * ptPI->fKp;
	
	ptPI->fIout = ptPI->fIprevOut + 0.5f*ptPI->fDtSec*(
			ptPI->fPout*ptPI->fKi + ptPI->fIprevIn);
	ptPI->fIprevIn = ptPI->fPout;
	ptPI->fIprevOut = ptPI->fIout;
	
	fPreOut = ptPI->fPout + ptPI->fIout;
	
	if(fPreOut > ptPI->fUpOutLim) fPreOut = ptPI->fUpOutLim;
	if(fPreOut < ptPI->fLowOutLim) fPreOut = ptPI->fLowOutLim;
	
	ptPI->fOut = fPreOut;
}

/**
  * @brief  Reset the internal variables of PI-controller to defaults.
  * @param  ptPI: pointer to user data structure with type "ptPI".               
  * @retval None
  */
void tPI_rst(tPI* ptPI)
{
	ptPI->fIn = 0.0f;
	ptPI->fIout = 0.0f;
	ptPI->fIprevIn = 0.0f;
	ptPI->fIprevOut = 0.0f;
	ptPI->fOut = 0.0f;
	ptPI->fPout = 0.0f;
}

/**
  * @brief  Calculate and update the PD-controller output.
  * @param  ptPD: pointer to user data structure with type "ptPD".               
  * @retval None
  */
void tPD_calc(tPD* ptPD)
{
	float fPreOut;
	
	ptPD->fPout = ptPD->fIn * ptPD->fKp;
	
	ptPD->fDout = (ptPD->fPout*ptPD->fKd - ptPD->fDprevIn)/ptPD->fDtSec;
	ptPD->fDprevIn = ptPD->fPout;
	ptPD->fDprevOut = ptPD->fDout;
	
	fPreOut = ptPD->fPout + ptPD->fDout;
	
	if(fPreOut > ptPD->fUpOutLim) fPreOut = ptPD->fUpOutLim;
	if(fPreOut < ptPD->fLowOutLim) fPreOut = ptPD->fLowOutLim;
	
	ptPD->fOut = fPreOut;
}

/**
  * @brief  Reset the internal variables of PD-controller to defaults.
  * @param  ptPD: pointer to user data structure with type "ptPD".               
  * @retval None
  */
void tPD_rst(tPD* ptPD)
{
	ptPD->fDout = 0.0f;
	ptPD->fDprevIn = 0.0f;
	ptPD->fDprevOut = 0.0f;
	ptPD->fIn = 0.0f;
	ptPD->fOut = 0.0f;
	ptPD->fPout = 0.0f;
}

/**
  * @brief  Calculate and update the PID-controller output.
  * @param  ptPID: pointer to user data structure with type "tPID".               
  * @retval None
  */
void tPID_calc(tPID* ptPID)
{
	float fPreOut;
	
	ptPID->fPout = ptPID->fIn * ptPID->fKp;
	
	ptPID->fIout = ptPID->fIprevOut + 0.5f*ptPID->fDtSec*(
			ptPID->fPout*ptPID->fKi + ptPID->fIprevIn);
	ptPID->fIprevIn = ptPID->fPout;
	ptPID->fIprevOut = ptPID->fIout;
	
	ptPID->fDout = (ptPID->fPout*ptPID->fKd - ptPID->fDprevIn)/ptPID->fDtSec;
	ptPID->fDprevIn = ptPID->fPout;
	ptPID->fDprevOut = ptPID->fDout;
	
	fPreOut = ptPID->fPout + ptPID->fIout + ptPID->fDout;
	
	if(fPreOut > ptPID->fUpOutLim) fPreOut = ptPID->fUpOutLim;
	if(fPreOut < ptPID->fLowOutLim) fPreOut = ptPID->fLowOutLim;
	
	ptPID->fOut = fPreOut;
}

/**
  * @brief  Reset the internal variables of PID-controller to defaults.
  * @param  ptPID: pointer to user data structure with type "tPID".               
  * @retval None
  */
void tPID_rst(tPID* ptPID)
{
	ptPID->fDout = 0.0f;
	ptPID->fDprevIn = 0.0f;
	ptPID->fDprevOut = 0.0f;
	ptPID->fIn = 0.0f;
	ptPID->fIout = 0.0f;
	ptPID->fIprevIn = 0.0f;
	ptPID->fIprevOut = 0.0f;
	ptPID->fOut = 0.0f;
	ptPID->fPout = 0.0f;
}

/*********************************** END OF FILE ***********************************/
