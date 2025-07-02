#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project endeffector.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	40
#define CUST_BYTE_NUM_IN	16
#define TOT_BYTE_NUM_ROUND_OUT	40
#define TOT_BYTE_NUM_ROUND_IN	16


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
		double      velocity_1;
		double      velocity_2;
		double      roll;
		double      pitch;
		double      yaw;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		double      velocity_1;
		double      velocity_2;
	}Cust;
} PROCBUFFER_IN;

#endif
