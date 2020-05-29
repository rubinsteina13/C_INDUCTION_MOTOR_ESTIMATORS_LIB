/**
  ***********************************************************************************
  * @file    im_estimators.c
  * @author  Serhii Yatsenko [royalroad1995@gmail.com]
  * @version V1.0
  * @date    May-2020
  * @brief   This file provides firmware function for implementation the following
  *			 		 types induction motor estimators:
  *						+	sensored rotor flux (angle, magnitude) and back-EMF observer; 
  *						+	sensorless stator back-EMF observer; 
  *						+	sensorless rotor speed and flux (angle, magnitude) observer.
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
#include "im_estimators.h"

/* Private typedef ----------------------------------------------------------------*/
/* Private define -----------------------------------------------------------------*/
/* Private constants --------------------------------------------------------------*/
/* Private macro ------------------------------------------------------------------*/
/* Private variables --------------------------------------------------------------*/
/* Private function prototypes ----------------------------------------------------*/
/* Private functions --------------------------------------------------------------*/

/**
  * @brief  Initialize the induction motor parameters.
  * @param  ptIMparams: pointer to user data structure with type "tIMparams".               
  * @retval None
  */
void tIMparams_init(tIMparams* ptIMparams)
{
	ptIMparams->f1divTr = ptIMparams->fRr/ptIMparams->fLr;
	ptIMparams->f1divKr = ptIMparams->fLr/ptIMparams->fLm;
	ptIMparams->fSigLs	= (1.0f - ptIMparams->fLm*ptIMparams->fLm/
											(ptIMparams->fLs*ptIMparams->fLr)) * ptIMparams->fLs;
}

/**
  * @brief  IM stator back-EMF observer calculation function
  * @param  ptIMstatObs: pointer to user data structure with type "tIMstatObs",
  *			ptIMparams: pointer to user data structure with type "tIMparams".               
  * @retval None
  */
void tIMstatObs_calc(tIMstatObs* ptIMstatObs, tIMparams* ptIMparams)
{
	float fDiffIsAl, fDiffIsBe;
	
	fDiffIsAl = (ptIMstatObs->fIsAl - ptIMstatObs->fPrevIsAl)/ptIMparams->fDt;
							ptIMstatObs->fPrevIsAl = ptIMstatObs->fIsAl;
	
	fDiffIsBe = (ptIMstatObs->fIsBe - ptIMstatObs->fPrevIsBe)/ptIMparams->fDt;
							ptIMstatObs->fPrevIsBe = ptIMstatObs->fIsBe;
	
	ptIMstatObs->fEsAl = (ptIMstatObs->fUsAl - ptIMparams->fRs*ptIMstatObs->fIsAl - 
											ptIMparams->fSigLs*fDiffIsAl)*ptIMparams->f1divKr;
						
	ptIMstatObs->fEsBe = (ptIMstatObs->fUsBe - ptIMparams->fRs*ptIMstatObs->fIsBe -
											ptIMparams->fSigLs*fDiffIsBe)*ptIMparams->f1divKr;
}

/**
  * @brief  IM rotor back-EMF and flux observer calculation function
  * @param  ptIMrotObs: pointer to user data structure with type "tIMrotObs",
  *			ptIMparams: pointer to user data structure with type "tIMparams".               
  * @retval None
  */
void tIMrotObs_calc(tIMrotObs* ptIMrotObs, tIMparams* ptIMparams)
{
	ptIMrotObs->fErAl = (ptIMrotObs->fIsAl*ptIMparams->fLm - ptIMrotObs->fFrAl)*
											ptIMparams->f1divTr - ptIMrotObs->fWrE*ptIMrotObs->fFrBe;
	
	ptIMrotObs->fFrAl = ptIMrotObs->fPrevFrAl + 0.5f*ptIMparams->fDt*(
										 ptIMrotObs->fErAl + ptIMrotObs->fPrevErAl);
	ptIMrotObs->fPrevErAl = ptIMrotObs->fErAl;
	ptIMrotObs->fPrevFrAl = ptIMrotObs->fFrAl;
						
	ptIMrotObs->fErBe = (ptIMrotObs->fIsBe*ptIMparams->fLm - ptIMrotObs->fFrBe)*
											ptIMparams->f1divTr + ptIMrotObs->fWrE*ptIMrotObs->fFrAl;

	ptIMrotObs->fFrBe = ptIMrotObs->fPrevFrBe + 0.5f*ptIMparams->fDt*(
										 ptIMrotObs->fErBe + ptIMrotObs->fPrevErBe);
	ptIMrotObs->fPrevErBe = ptIMrotObs->fErBe;
	ptIMrotObs->fPrevFrBe = ptIMrotObs->fFrBe;
}

/**
  * @brief  IM rotor speed and flux observer calculation function
  * @param  ptIMspeedObs: pointer to user data structure with type "tIMspeedObs",
  *			ptIMparams: pointer to user data structure with type "tIMparams".               
  * @retval None
  */
void tIMspeedObs_calc(tIMspeedObs* ptIMspeedObs, tIMparams* ptIMparams)
{
	ptIMspeedObs->sIMstatObs.fUsAl = ptIMspeedObs->fUsAl;
	ptIMspeedObs->sIMstatObs.fUsBe = ptIMspeedObs->fUsBe;
	ptIMspeedObs->sIMstatObs.fIsAl = ptIMspeedObs->fIsAl;
	ptIMspeedObs->sIMstatObs.fIsBe = ptIMspeedObs->fIsBe;
	
	ptIMspeedObs->sIMrotObs.fIsAl = ptIMspeedObs->fIsAl;
	ptIMspeedObs->sIMrotObs.fIsBe = ptIMspeedObs->fIsBe;
	ptIMspeedObs->sIMrotObs.fWrE = ptIMspeedObs->fWrE;
	
	ptIMspeedObs->sIMstatObs.m_calc(&ptIMspeedObs->sIMstatObs, ptIMparams);
	ptIMspeedObs->sIMrotObs.m_calc(&ptIMspeedObs->sIMrotObs, ptIMparams);
	
	
	ptIMspeedObs->sPI.fIn = ptIMspeedObs->fIsAl*(ptIMspeedObs->sIMstatObs.fEsBe - 
												 ptIMspeedObs->sIMrotObs.fErBe) - ptIMspeedObs->fIsBe*(
												 ptIMspeedObs->sIMstatObs.fEsAl - 
												 ptIMspeedObs->sIMrotObs.fErAl);
	
	ptIMspeedObs->sPI.m_calc(&ptIMspeedObs->sPI);
	
	ptIMspeedObs->fWrE = ptIMspeedObs->sPI.fOut;
	
	ptIMspeedObs->fFrAng = atan2f(ptIMspeedObs->sIMrotObs.fFrBe,
															ptIMspeedObs->sIMrotObs.fFrAl);
									
	ptIMspeedObs->fFrMagn = hypotf(ptIMspeedObs->sIMrotObs.fFrBe,
																ptIMspeedObs->sIMrotObs.fFrAl);
}

/*********************************** END OF FILE ***********************************/
