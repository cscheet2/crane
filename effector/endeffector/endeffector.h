#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project endeffector.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	32
#define CUST_BYTE_NUM_IN	32
#define TOT_BYTE_NUM_ROUND_OUT	32
#define TOT_BYTE_NUM_ROUND_IN	32


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
		double      roll_position;
		double      pitch_position;
		double      yaw_position;
		double      Kp;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		double      roll_position;
		double      pitch_position;
		double      yaw_position;
		double      z_length;
	}Cust;
} PROCBUFFER_IN;

#endif