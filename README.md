# C-library with implementation of the Induction Motor rotor speed anf flux estimators

* This embedded C-library provides the various types of [Induction Motor](https://en.wikipedia.org/wiki/Induction_motor) estimators to implementation the [FOC](https://en.wikipedia.org/wiki/Vector_control_(motor)) control systems:
	* Sensored rotor flux (angle, magnitude) and back-EMF observer
	* Sensorless stator back-EMF observer
	* Sensorless rotor speed and flux (angle, magnitude) observer

* Project structure
	* README.md - current file
	* LICENSE - file with license description
	* fp_pid.h - C-header file with user data types and function prototypes (P/I/D library)
  * fp_pid.c - C-source file with firmware functions (P/I/D library)
  * im_estimators.h - C-header file with user data types and function prototypes (Induction Motor estimators library)
  * im_estimators.c - C-source file with firmware functions (Induction Motor estimators library)

# HowToUse (example)

* Example 1 - Stator back-EMF observer

		//

* Example 2 - Rotor flux and back-EMF observer

		//

* Example 3 - Rotor speed and flux observer

		//

# License
  
[MIT](./LICENSE "License Description")
