#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project endeffector.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	16
#define CUST_BYTE_NUM_IN	16
#define TOT_BYTE_NUM_ROUND_OUT	16
#define TOT_BYTE_NUM_ROUND_IN	16


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
		double      x_axis_actual;
		double      y_axis_actual;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		double      x_axis_expected;
		double      y_axis_expected;
	}Cust;
} PROCBUFFER_IN;

#endif