# C-library with implementation of the Induction Motor rotor speed anf flux estimators (observers)

* This embedded C-library provides the various types of [Induction Motor](https://en.wikipedia.org/wiki/Induction_motor) estimators to implementation the [FOC](https://en.wikipedia.org/wiki/Vector_control_(motor)) control systems:
	* Sensored rotor flux (angle, magnitude) and back-EMF observer
	* Sensorless stator back-EMF observer
	* Sensorless rotor speed and flux (angle, magnitude) observer

* Project structure
	* README.md - current file
	* LICENSE - file with license description
	* [fp_pid.h](https://github.com/rubinsteina13/C_PID_CONTROLLERS_LIB/blob/master/fp_pid.h) - C-header file with user data types and function prototypes ([P/I/D library](https://github.com/rubinsteina13/C_PID_CONTROLLERS_LIB))
  * [fp_pid.c](https://github.com/rubinsteina13/C_PID_CONTROLLERS_LIB/blob/master/fp_pid.c) - C-source file with firmware functions ([P/I/D library](https://github.com/rubinsteina13/C_PID_CONTROLLERS_LIB))
  * im_estimators.h - C-header file with user data types and function prototypes (Induction Motor estimators library)
  * im_estimators.c - C-source file with firmware functions (Induction Motor estimators library)

# HowToUse (example)

* Example 1 - Stator back-EMF observer

		#include "im_estimators.h"
		
		// Measurable values of stator voltages and currents:
		float IsAl, IsBe, UsAl, UsBe;
		
		// Observable value of stator back-EMF:
		float EsAl, EsBe;
		
		// 1st step: create and initialize the global variables of user data structures
		tIMparams IMparams = IM_PARAMS_DEFAULTS;
		tIMstatObs sIMstatObs = IM_STAT_OBS_DEFAULTS;
		
		// 2nd step: do some settings
		IMparams.fDt = 0.0001f;         // set the discretization (sample) time
		IMparams.fNpP = 2.0f;           // set the count of stator pole pairs
		IMparams.fRr = 4.516f;          // set the rotor resistance constant
		IMparams.fRs = 50.0f;           // set the stator resistance constant
		IMparams.fLr = 0.143f;          // set the rotor inductance constant
		IMparams.fLs = 0.143f;          // set the stator inductance constant
		IMparams.fLm = 0.14f;           // set the magnetizing inductance constant
		IMparams.m_init(&IMparams);     // call the initialization function of induction motor parameters
		
		// 3rd step: Next code must be executed every time with IMparams.fDt period when 
		// new calculation of Stator back-EMF values is needed
		sIMstatObs.fIsAl = IsAl;        // update the stator current Alpha
		sIMstatObs.fIsBe = IsBe;        // update the stator current Beta
		sIMstatObs.fUsAl = UsAl;        // update the stator voltage Alpha
		sIMstatObs.fUsBe = UsBe;        // update the stator voltage Beta
		sIMstatObs.m_calc(&sIMstatObs, &IMparams); // call the Stator back-EMF observer function
		EsAl = sIMstatObs.fEsAl;        // observed stator back-EMF voltage Alpha
		EsBe = sIMstatObs.fEsBe;        // observed stator back-EMF voltage Beta

* Example 2 - Rotor flux and back-EMF observer

		#include "im_estimators.h"
		
		// Measurable values of stator currents:
		float IsAl, IsBe;
		
		// Measurable value of rotor mechanical speed (Rad/Sec):
		float Wr;
		
		// Observable values of rotor back-EMF and flux:
		float ErAl, ErBe, Fang, Fmag;
		
		// 1st step: create and initialize the global variables of user data structures
		tIMparams IMparams = IM_PARAMS_DEFAULTS;
		tIMrotObs IMrotObs = IM_ROT_OBS_DEFAULTS;
		
		// 2nd step: do some settings
		IMparams.fDt = 0.0001f;         // set the discretization (sample) time
		IMparams.fNpP = 2.0f;           // set the count of stator pole pairs
		IMparams.fRr = 4.516f;          // set the rotor resistance constant
		IMparams.fRs = 50.0f;           // set the stator resistance constant
		IMparams.fLr = 0.143f;          // set the rotor inductance constant
		IMparams.fLs = 0.143f;          // set the stator inductance constant
		IMparams.fLm = 0.14f;           // set the magnetizing inductance constant
		IMparams.m_init(&IMparams);     // call the initialization function of induction motor parameters
		
		// 3rd step: Next code must be executed every time with IMparams.fDt period when 
		// new calculation of rotor back-EMF and flux values is needed
		IMrotObs.fIsAl = IsAl;          // update the stator current Alpha
		IMrotObs.fIsBe = IsBe;          // update the stator current Beta
		IMrotObs.fWrE = Wr*IMparams.fNpP; // update the rotor electrical speed value
		IMrotObs.m_calc(&IMrotObs, &IMparams); // call the rotor back-EMF and flux observer function
		ErAl = IMrotObs.fErAl;          // observed rotor back-EMF voltage Alpha
		ErBe = IMrotObs.fErBe;          // observed rotor back-EMF voltage Beta
		Fang = atan2f(IMrotObs.fFrAl, IMrotObs.fFrBe); // observed rotor flux angle
		Fmag = hypotf(IMrotObs.fFrAl, IMrotObs.fFrBe); // observed rotor flux magnitude

* Example 3 - Rotor speed and flux observer

		// Measurable values of stator voltages and currents:
		float IsAl, IsBe, UsAl, UsBe;
		
		// Observable value of rotot mechanical speed (Rad/Sec):
		float Wr;
		
		// Observable values of rotor flux:
		float Fang, Fmag;
		
		// 1st step: create and initialize the global variables of user data structures
		tIMparams IMparams = IM_PARAMS_DEFAULTS;
		tIMspeedObs sIMspeedObs = IM_SPEED_OBS_DEFAULTS;
		
		// 2nd step: do some settings
		IMparams.fDt = 0.0001f;         // set the discretization (sample) time
		IMparams.fNpP = 2.0f;           // set the count of stator pole pairs
		IMparams.fRr = 4.516f;          // set the rotor resistance constant
		IMparams.fRs = 50.0f;           // set the stator resistance constant
		IMparams.fLr = 0.143f;          // set the rotor inductance constant
		IMparams.fLs = 0.143f;          // set the stator inductance constant
		IMparams.fLm = 0.14f;           // set the magnetizing inductance constant
		IMparams.m_init(&IMparams);     // call the initialization function of induction motor parameters
		// configure the speed observer adapter based on PI-controller:
		sIMspeedObs.sPI.fDtSec = IMparams.fDt; // set the discretization (sample) time for PI-controller
		sIMspeedObs.sPI.fKp = 0.1f;     // set the proportional coefficient of PI-controller
		sIMspeedObs.sPI.fKp = 0.01f;    // set the integral coefficient of PI-controller
		sIMspeedObs.sPI.fUpOutLim = 300.0f; // set the PI-controller's output upper limit (Max rotor electrical speed value)
		sIMspeedObs.sPI.fUpOutLim = -300.0f;// set the PI-controller's output lower limit (Min rotor electrical speed value)
		
		// 3rd step: Next code must be executed every time with IMparams.fDt period when 
		// new calculation of rotor speed and flux values is needed
		sIMspeedObs.fIsAl = IsAl;       // update the stator current Alpha
		sIMspeedObs.fIsBe = IsBe;       // update the stator current Beta
		sIMspeedObs.fUsAl = UsAl;       // update the stator voltage Alpha
		sIMspeedObs.fUsBe = UsBe;       // update the stator voltage Beta
		sIMspeedObs.m_calc(&sIMspeedObs, &IMparams); // call the rotor speed and flux observer function
		Wr = sIMspeedObs.fWrE/IMparams.fNpP; // observed rotor mechanical speed
		Fang = sIMspeedObs.fFrAng;      // observed rotor flux angle
		Fmag = sIMspeedObs.fFrMagn;     // observed rotor flux magnitude

# License
  
[MIT](./LICENSE "License Description")
